#include "subsystems/lift.hpp"

#include "config.hpp"

#include <algorithm>
#include <cmath>

namespace subsystems {

Lift::Lift()
    : motors_(config::LIFT_PORTS, pros::v5::MotorGears::red),  // both lift motors are red (100 RPM) cartridges
      pid_(config::LIFT_KP, config::LIFT_KI, config::LIFT_KD, config::LIFT_WINDUP_RANGE, true),
      settle_(config::LIFT_SETTLE_DEG, config::LIFT_SETTLE_MS) {
    motors_.set_encoder_units_all(pros::v5::MotorEncoderUnits::degrees);
    // HOLD keeps the arm from dropping under its own weight whenever update() brakes (idle, or the
    // driver just let go of L1/L2).
    motors_.set_brake_mode_all(pros::v5::MotorBrake::hold);
    if (config::LIFT_ROTATION_PORT != 0) {
        rotation_ = std::make_unique<pros::Rotation>(config::LIFT_ROTATION_PORT);
        rotation_->set_reversed(config::LIFT_ROTATION_REVERSED);
    }
}

void Lift::resetPid() {
    pid_.reset();
    shadowPrevError_ = 0.0;  // kept in lockstep with pid_'s internal prevError, see update()
}

void Lift::zero() {
    // Motor encoders are only ever a fallback position source (getPosition() prefers the rotation
    // sensor whenever one is fitted, which it is -- see getPosition() below) and are only read for
    // the brain's diagnostic ROT/MOT cross-check, so taring them here is harmless.
    motors_.tare_position_all();

    // Absolute zero, NOT a startup reset: the rotation sensor's own persistent zero point has been
    // calibrated (on the brain's device screen, outside this code) to the physical bottom of the
    // lift, so get_angle() is already a true absolute 0..360 sensor-degree reading from that zero
    // and survives power cycles on its own -- it does NOT need, and must NOT get, a software reset
    // here. Resetting it (the old rotation_->reset_position()/set_position(0)) would throw that
    // calibration away and make every height relative to wherever the arm happened to be sitting at
    // THIS particular boot instead (e.g. a little high from band tension with the motors unpowered) --
    // exactly the bug this avoids.
    //
    // What we do instead: seed the multi-turn position counter (what get_position() tracks) from the
    // current absolute angle, folded to the signed sensor-degree range closest to 0, (-180, 180]: a
    // lift resting a hair low (angle ~359.9 deg) seeds as ~-0.1 deg, not +359.9. From then on
    // get_position() keeps counting turns correctly from that seed as the arm moves, through
    // getSensorPosition()'s /LIFT_ROTATION_RATIO conversion to arm degrees.
    //
    // This is only unambiguous if the lift powers on within +-45 ARM degrees (+-180 sensor degrees --
    // one quarter of the 4:1-geared sensor's single revolution) of the true bottom: true by design
    // (DOWN/0 is the documented power-on position, see config::lift_preset) but NOT verified here --
    // a lift powered on further from the bottom than that would seed to the wrong turn count. set_reversed()
    // was already applied in the constructor (it runs long before this, see Lift::Lift()), so it
    // affects get_angle() here and every later get_position() read the same consistent way.
    if (rotation_) {
        std::int32_t angleCentideg = rotation_->get_angle();  // 0..35999 centidegrees, absolute, persists
        if (angleCentideg > 18000) angleCentideg -= 36000;    // fold into (-18000, 18000] centidegrees
        startupAngleDeg_ = angleCentideg / 100.0;             // kept for the brain screen only
        rotation_->set_position(angleCentideg);               // seed the counter -- NOT reset_position()
    }
    active_ = false;
    goal_ = target_ = 0.0;
    speed_ = 0.0;
    faulted_ = false;
    blocked_ = false;
    pushStart_ = 0;
    stallStart_ = 0;
    resetPid();
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

double Lift::getRawAngle() const { return rotation_ ? rotation_->get_angle() / 100.0 : 0.0; }

double Lift::getTemperature() const {
    double hottest = 0.0;
    for (double t : motors_.get_temperature_all()) hottest = std::max(hottest, t);
    return hottest;
}

double Lift::getCurrentDraw() const {
    double highest = 0.0;
    for (std::int32_t c : motors_.get_current_draw_all()) highest = std::max(highest, static_cast<double>(c));
    return highest;
}

double Lift::getPosition() const { return rotation_ ? getSensorPosition() : getMotorPosition(); }

void Lift::setTarget(double degrees, double speedDegPerSec) {
    const double clamped = std::clamp(degrees, config::lift_preset::DOWN, config::lift_preset::MAX);
    if (active_ && clamped == goal_ && speedDegPerSec == speed_) return;
    active_ = true;
    blocked_ = false;
    stallStart_ = 0;
    goal_ = clamped;
    speed_ = speedDegPerSec;
    // A slow move starts from where the lift actually is, not from the old setpoint.
    target_ = speed_ > 0 ? getPosition() : goal_;
    resetPid();       // old integral/derivative belong to the old target
    settle_.reset();  // a new target is not settled yet
    settled_ = false;
}

void Lift::moveTo(double degrees, int timeoutMs, bool waitUntilSettled) {
    setTarget(degrees);
    if (!waitUntilSettled) return;
    const std::uint32_t start = pros::millis();
    while (!isSettled() && !isBlocked() && !isFaulted()) {
        if (timeoutMs > 0 && pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) break;
        pros::delay(10);
    }
}

void Lift::cancel() {
    active_ = false;
    speed_ = 0.0;
    blocked_ = false;
    stallStart_ = 0;
    settled_ = false;
    resetPid();
    settle_.reset();
}

const char* Lift::activePresetName() const {
    if (!active_) return "none";
    namespace preset = config::lift_preset;
    if (goal_ == preset::DOWN) return "DOWN";
    if (goal_ == preset::INTAKE) return "INTAKE";
    if (goal_ == preset::MATCHLOAD) return "MATCHLOAD";
    if (goal_ == preset::PIVOT_CLEAR) return "PIVOT_CLEAR";
    if (goal_ == preset::MAX) return "MAX";
    static const char* const kLevelNames[preset::STACK_LEVEL_COUNT] = {"STACK 1", "STACK 2", "STACK 3", "STACK 4",
                                                                        "STACK 5"};
    for (int i = 0; i < preset::STACK_LEVEL_COUNT; ++i)
        if (goal_ == preset::STACK_LEVELS[i]) return kLevelNames[i];
    return "custom";
}

void Lift::driverPower(double input) { manualPower_ = std::clamp(input, -1.0, 1.0); }

void Lift::update() {
    const std::uint32_t now = pros::millis();
    const double dt = lastUpdate_ == 0 ? 0.01 : (now - lastUpdate_) / 1000.0;
    lastUpdate_ = now;
    const double position = getPosition();

    // 1) Driver manual override (D-pad Up/Down): full send, no feedforward -- same feel as plain
    //    motor power. This is the ONLY thing allowed to preempt the PID, and it always wins: cancel
    //    whatever target was active so nothing "fights" the driver, and so release doesn't jump
    //    back to it either (see class comment).
    if (std::fabs(manualPower_) > 0.02) {
        if (active_) cancel();
        wasManual_ = true;
        const double mv = std::clamp(manualPower_, -1.0, 1.0) * config::LIFT_MAX_MV;
        motors_.move_voltage(static_cast<int>(mv));
        settled_ = false;
        pushStart_ = 0;
        lastError_ = 0.0;
        lastOutput_ = mv;
        lastP_ = lastI_ = lastD_ = 0.0;
        return;
    }
    if (wasManual_) {
        wasManual_ = false;
        faulted_ = false;  // driver took manual control; give the PID a clean slate next time
    }

    // 2) No commanded height: brake and hold right here. This is the state at power-on (nothing
    //    moves until something explicitly calls setTarget()/moveTo()) and right after the driver
    //    lets go of the manual override (no old target resumes).
    if (!active_) {
        motors_.brake();
        settled_ = false;
        lastError_ = 0.0;
        lastOutput_ = 0.0;
        lastP_ = lastI_ = lastD_ = 0.0;
        pushStart_ = 0;
        stallStart_ = 0;
        blocked_ = false;
        return;
    }

    if (faulted_) {
        motors_.brake();
        settled_ = false;
        return;
    }

    // 3) Final stretch down to the bottom: once inside the slow zone, (re)target a slow walk to 0
    //    instead of whatever jump/speed was active, so the PID is actively holding the arm back
    //    against gravity for the rest of the way down, not coasting under a capped voltage (a
    //    voltage cap alone can't slow a falling arm -- only a PID chasing a slow, receding target
    //    can, because it will push back / brake whenever the arm tries to fall faster than that).
    //    This re-engages on its own every tick, so even a repeated moveTo(DOWN) mid-descent only
    //    glitches for a single 10 ms tick before the slow walk picks back up from wherever it is.
    if (goal_ <= config::lift_preset::DOWN && position <= config::lift_preset::DOWN + config::LIFT_SLOW_ZONE_DEG &&
        speed_ != config::LIFT_SLOW_SPEED_DPS) {
        speed_ = config::LIFT_SLOW_SPEED_DPS;
        target_ = position;  // start the slow walk from right here, not from the old setpoint
    }

    // 4) Walk the setpoint toward the goal for speed-limited moves (including the slow zone above).
    if (speed_ > 0 && target_ != goal_) {
        const double step = speed_ * dt;
        if (std::fabs(goal_ - target_) <= step) target_ = goal_;
        else target_ += goal_ > target_ ? step : -step;
    }

    // 5) PID + gravity feedforward. The P and D terms below are computed the same way lemlib::PID
    //    computes them internally (see include/lemlib/pid.hpp: update() takes no dt, so it can't be
    //    doing any time-based scaling -- kP*error and kD*(error-prevError), one call per tick, same
    //    as here). The I term is the exact residual against the real output, so P+I+D+kG always
    //    equals the actual output regardless, even if that assumption is ever slightly off. This is
    //    for the tuning screen only; it never feeds back into control.
    const float error = static_cast<float>(target_ - position);
    const float pTerm = config::LIFT_KP * error;
    const float dTerm = config::LIFT_KD * (error - static_cast<float>(shadowPrevError_));
    shadowPrevError_ = error;
    const float pidOut = pid_.update(error);
    const float iTerm = pidOut - pTerm - dTerm;
    float output = pidOut + config::LIFT_KG;

    // Going to the bottom: never let the gravity feedforward push UP (that's what made the lift bounce
    // at startup, 17ab865). Once resting near the hard stop, hold still instead of fighting the frame.
    // The slow-zone walk above (step 3) is what actually controls descent speed near the bottom now --
    // this just caps the direction, it does not force a minimum downward push the way it used to.
    if (goal_ <= config::lift_preset::DOWN) {
        if (position <= config::lift_preset::DOWN + config::LIFT_REST_BAND_DEG) {
            motors_.brake();
            settled_ = true;
            pushStart_ = 0;
            resetPid();
            lastError_ = error;
            lastOutput_ = 0.0;
            lastP_ = lastI_ = lastD_ = 0.0;
            return;
        }
        output = std::min(output, 0.0f);
    }

    // 6) Output limit (motors only accept -12000..12000 mV).
    output = std::clamp(output, -config::LIFT_MAX_MV, config::LIFT_MAX_MV);

    lastError_ = error;
    lastOutput_ = output;
    lastP_ = pTerm;
    lastI_ = iTerm;
    lastD_ = dTerm;

    // 7) Runaway guard: pushing hard one way for a while but moving the other way = direction mismatch.
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

    // 8) Stall guard: pushing hard toward a target it can't reach (past the real top, or jammed).
    //    Instead of grinding the motors, give up on that target and hold where it is.
    if (std::fabs(error) > 2 * config::LIFT_SETTLE_DEG && std::fabs(output) >= config::LIFT_STALL_MV) {
        if (stallStart_ == 0) {
            stallStart_ = now;
            stallStartPos_ = position;
        } else if (std::fabs(position - stallStartPos_) >= config::LIFT_STALL_DEG) {
            stallStart_ = now;  // still making progress
            stallStartPos_ = position;
        } else if (now - stallStart_ >= static_cast<std::uint32_t>(config::LIFT_STALL_MS)) {
            setTarget(position);
            blocked_ = true;  // cleared by the next new target
            stallStart_ = 0;
            return;
        }
    } else {
        stallStart_ = 0;
    }

    motors_.move_voltage(static_cast<int>(output));

    // 9) Settle check: at the final goal and within the band for long enough.
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
