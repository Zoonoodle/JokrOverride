#include "subsystems/lift.hpp"

#include "config.hpp"

#include <algorithm>
#include <cmath>

namespace subsystems {

Lift::Lift()
    : motors_(config::LIFT_PORTS, pros::v5::MotorGears::green),  // TODO(team): confirm cartridge
      pid_(config::LIFT_KP, config::LIFT_KI, config::LIFT_KD, config::LIFT_WINDUP_RANGE, true),
      settle_(config::LIFT_SETTLE_DEG, config::LIFT_SETTLE_MS) {
    motors_.set_encoder_units_all(pros::v5::MotorEncoderUnits::degrees);
    // HOLD keeps the arm from dropping if the loop ever outputs 0.
    motors_.set_brake_mode_all(pros::v5::MotorBrake::hold);
    if (config::LIFT_ROTATION_PORT != 0) rotation_ = std::make_unique<pros::Rotation>(config::LIFT_ROTATION_PORT);
}

void Lift::zero() {
    motors_.tare_position_all();
    if (rotation_) rotation_->reset_position();
    target_ = 0.0;
    pid_.reset();
    settle_.reset();
}

double Lift::getPosition() const {
    // Rotation sensor sits on the lift pivot, so it already reads lift degrees (centidegrees / 100).
    if (rotation_) return rotation_->get_position() / 100.0;
    // Motor encoders read motor degrees; divide by the gear ratio to get lift degrees.
    return motors_.get_position() / config::LIFT_GEAR_RATIO;
}

void Lift::setTarget(double degrees) {
    const double clamped = std::clamp(degrees, config::lift_preset::DOWN, config::lift_preset::MAX);
    if (clamped == target_) return;
    target_ = clamped;
    pid_.reset();     // old integral/derivative belong to the old target
    settle_.reset();  // a new target is not settled yet
    settled_ = false;
}

void Lift::manual(double input) { manualInput_ = std::clamp(input, -1.0, 1.0); }

void Lift::update() {
    const double position = getPosition();

    // 1) Driver override: raw power, but never past the top stop or below the bottom.
    if (std::fabs(manualInput_) > 0.05) {
        double mv = manualInput_ * config::LIFT_MANUAL_MV;
        if (position >= config::lift_preset::MAX && mv > 0) mv = 0;
        if (position <= config::lift_preset::DOWN && mv < 0) mv = 0;
        motors_.move_voltage(static_cast<int>(mv + config::LIFT_KG));
        wasManual_ = true;
        settled_ = false;
        return;
    }
    // Driver just let go: hold here instead of snapping back to the old preset.
    if (wasManual_) {
        wasManual_ = false;
        setTarget(position);
    }

    // 2) PID + gravity feedforward.
    const float error = static_cast<float>(target_ - position);
    float output = pid_.update(error) + config::LIFT_KG;

    // Resting on the hard stop at the bottom: don't fight the frame, let it sit.
    if (target_ <= config::lift_preset::DOWN && position <= config::lift_preset::DOWN + 1.0) output = 0;

    // 3) Output limit (motors only accept -12000..12000 mV).
    output = std::clamp(output, -config::LIFT_MAX_MV, config::LIFT_MAX_MV);
    motors_.move_voltage(static_cast<int>(output));

    // 4) Settle check: within the band for long enough.
    settled_ = settle_.update(error);
}

void Lift::startTask() {
    if (task_) return;
    task_ = std::make_unique<pros::Task>([this] {
        while (true) {
            update();
            pros::delay(10);
        }
    });
}

}  // namespace subsystems
