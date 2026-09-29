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
    if (config::LIFT_ROTATION_PORT != 0) {
        rotation_ = std::make_unique<pros::Rotation>(config::LIFT_ROTATION_PORT);
        rotation_->set_reversed(config::LIFT_ROTATION_REVERSED);
    }
}

void Lift::zero() {
    motors_.tare_position_all();
    if (rotation_) rotation_->reset_position();
    goal_ = target_ = 0.0;
    speed_ = 0.0;
    faulted_ = false;
    pushStart_ = 0;
    pid_.reset();
    settle_.reset();
}

double Lift::getMotorPosition() const {
    // Motor encoders read motor degrees; divide by the gear ratio to get lift degrees.
    return motors_.get_position() / config::LIFT_GEAR_RATIO;
}

double Lift::getSensorPosition() const {
    if (!rotation_) return 0.0;
    // centidegrees -> sensor degrees -> lift degrees
    return rotation_->get_position() / 100.0 / config::LIFT_ROTATION_RATIO;
}

double Lift::getTemperature() const {
    double hottest = 0.0;
    for (double t : motors_.get_temperature_all()) hottest = std::max(hottest, t);
    return hottest;
}

double Lift::getPosition() const { return rotation_ ? getSensorPosition() : getMotorPosition(); }

void Lift::setTarget(double degrees, double speedDegPerSec) {
    const double clamped = std::clamp(degrees, config::lift_preset::DOWN, config::lift_preset::MAX);
    if (clamped == goal_ && speedDegPerSec == speed_) return;
    goal_ = clamped;
    speed_ = speedDegPerSec;
    // A slow move starts from where the lift actually is, not from the old setpoint.
    target_ = speed_ > 0 ? getPosition() : goal_;
    pid_.reset();     // old integral/derivative belong to the old target
    settle_.reset();  // a new target is not settled yet
    settled_ = false;
}

void Lift::manual(double input) { manualInput_ = std::clamp(input, -1.0, 1.0); }

void Lift::update() {
    const std::uint32_t now = pros::millis();
    const double dt = lastUpdate_ == 0 ? 0.01 : (now - lastUpdate_) / 1000.0;
    lastUpdate_ = now;
    const double position = getPosition();

    // 1) Driver override: raw power, but never past the top stop or below the bottom.
    //    Also the way out of a fault: manual doesn't trust the PID.
    if (std::fabs(manualInput_) > 0.05) {
        double mv = manualInput_ * config::LIFT_MANUAL_MV;
        if (position >= config::lift_preset::MAX && mv > 0) mv = 0;
        if (position <= config::lift_preset::DOWN && mv < 0) mv = 0;
        motors_.move_voltage(static_cast<int>(mv + config::LIFT_KG));
        wasManual_ = true;
        settled_ = false;
        pushStart_ = 0;
        return;
    }
    // Driver just let go: hold here instead of snapping back to the old preset.
    if (wasManual_) {
        wasManual_ = false;
        faulted_ = false;  // driver took over; give the PID another chance
        setTarget(position);
    }

    if (faulted_) {
        motors_.brake();
        settled_ = false;
        return;
    }

    // 2) Walk the setpoint toward the goal for speed-limited moves.
    if (speed_ > 0 && target_ != goal_) {
        const double step = speed_ * dt;
        if (std::fabs(goal_ - target_) <= step) target_ = goal_;
        else target_ += goal_ > target_ ? step : -step;
    }

    // 3) PID + gravity feedforward.
    const float error = static_cast<float>(target_ - position);
    float output = pid_.update(error) + config::LIFT_KG;

    // Resting on the hard stop at the bottom: don't fight the frame, let it sit.
    if (goal_ <= config::lift_preset::DOWN && position <= config::lift_preset::DOWN + 1.0) output = 0;

    // 4) Output limit (motors only accept -12000..12000 mV).
    output = std::clamp(output, -config::LIFT_MAX_MV, config::LIFT_MAX_MV);

    // 5) Runaway guard: pushing hard one way for a while but moving the other way = direction mismatch.
    const int dir = output >= config::LIFT_FAULT_MV ? 1 : output <= -config::LIFT_FAULT_MV ? -1 : 0;
    if (dir == 0 || dir != pushDir_) {
        pushDir_ = dir;
        pushStart_ = dir == 0 ? 0 : now;
        pushStartPos_ = position;
    } else if (now - pushStart_ >= static_cast<std::uint32_t>(config::LIFT_FAULT_MS)) {
        if ((position - pushStartPos_) * dir < -config::LIFT_FAULT_DEG) {
            faulted_ = true;
            motors_.brake();
            return;
        }
        pushStart_ = now;  // moving the right way (or stalled): start a new window
        pushStartPos_ = position;
    }

    motors_.move_voltage(static_cast<int>(output));

    // 6) Settle check: at the final goal and within the band for long enough.
    if (target_ != goal_) {
        settle_.reset();  // still walking toward the goal
        settled_ = false;
    } else {
        settled_ = settle_.update(error);
    }
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
