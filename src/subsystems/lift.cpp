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
    // A trip to the bottom is ALWAYS profiled, even when the caller asked for a jump (speed 0, e.g.
    // the B stow button or a bare auton moveTo(DOWN)). A setpoint that jumps straight to 0 means the
    // arm drops at whatever voltage the PID will give it (~150 arm deg/s wide open) and only starts
    // being slowed once it is already inside the last LIFT_SLOW_ZONE_DEG -- far too late to stop in.
    // Filling in the cruise here instead means the setpoint, and therefore the arm, is walking down
    // at a known speed for the whole trip. The resolved speed (not the raw argument) is what the
    // "same move, ignore it" check below compares, so a repeated moveTo(DOWN) mid-descent really is
    // a no-op and can't restart the profile from wherever the arm has got to.
    const double resolved = clamped <= config::lift_preset::DOWN && speedDegPerSec <= 0.0
                                ? config::LIFT_LOWER_SPEED_DEG_PER_S
                                : speedDegPerSec;
    if (active_ && clamped == goal_ && resolved == speed_) return;
    active_ = true;
    blocked_ = false;
    stallStart_ = 0;
    goal_ = clamped;
    speed_ = resolved;
    // A speed-limited move starts from where the lift actually is, not from the old setpoint.
    target_ = speed_ > 0 ? getPosition() : goal_;
    descentStart_ = target_;  // anchor for the ramp-up in update(), see LIFT_LOWER_RAMP_DEG
    descentFloor_ = 0.0;      // a brand-new move never inherits a stale pivot floor, see setDescentFloor()
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

    // 3) How fast the setpoint is allowed to move this tick. Normally that is just the commanded
    //    speed_ (setTarget() already guarantees a nonzero one for any trip to the bottom, so the
    //    whole descent is profiled from wherever the arm was when the move started). The final
    //    stretch is slower: once the SETPOINT -- not the arm -- is within LIFT_SLOW_ZONE_DEG of 0,
    //    it creeps the rest of the way at LIFT_SLOW_SPEED_DPS. Testing the setpoint is the point:
    //    the arm is tracking it, so it has already been decelerating since before it entered the
    //    zone, instead of showing up at the zone boundary at full speed with 20 deg left to stop in.
    //    Only this speed changes at the handover -- target_ is never snapped back to the live
    //    position mid-descent, because that would throw away exactly the lag the PID is correcting.
    double walkSpeed = speed_;
    if (goal_ <= config::lift_preset::DOWN) {
        // Ramp-up: don't start a descent at full cruise -- climb linearly from
        // LIFT_LOWER_START_SPEED_DEG_PER_S up to cruise (speed_) over the first LIFT_LOWER_RAMP_DEG of
        // travel from wherever this move began (descentStart_, set once in setTarget()). Measured on
        // the SETPOINT, same reasoning as the slow zone below: the arm is tracking it, so it leaves
        // the top smoothly instead of snapping to cruise speed the instant the move is commanded.
        if (config::LIFT_LOWER_RAMP_DEG > 0.0) {
            const double traveled = std::fabs(descentStart_ - target_);
            if (traveled < config::LIFT_LOWER_RAMP_DEG) {
                const double t = traveled / config::LIFT_LOWER_RAMP_DEG;
                walkSpeed = config::LIFT_LOWER_START_SPEED_DEG_PER_S +
                            t * (speed_ - config::LIFT_LOWER_START_SPEED_DEG_PER_S);
            }
        }
        if (target_ <= config::lift_preset::DOWN + config::LIFT_SLOW_ZONE_DEG)
            walkSpeed = std::min(walkSpeed, config::LIFT_SLOW_SPEED_DPS);
    }

    // 4) Walk the setpoint toward the goal for speed-limited moves (including the ramp-up and slow
    //    walk above), then hold it at descentFloor_ if a caller has raised one (Scoring does, while
    //    the pivot it owns is still retracting -- see setDescentFloor()). This only ever pauses the
    //    LAST bit of an already-moving profile: the floor starts at 0 (no effect) and callers raise
    //    it only once they need it, so it can delay arrival but can never turn into a stop-then-hop.
    if (walkSpeed > 0 && target_ != goal_) {
        const double step = walkSpeed * dt;
        if (std::fabs(goal_ - target_) <= step) target_ = goal_;
        else target_ += goal_ > target_ ? step : -step;
    }
    if (goal_ <= config::lift_preset::DOWN) target_ = std::max(target_, descentFloor_);

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

    // Going to the bottom: inside the rest band, stop controlling altogether and just brake-hold.
    // That is the one place the gravity feedforward must not be allowed to push UP -- an arm already
    // sitting on (or held a hair above, by the rubber bands) the hard stop with kG under it is what
    // made the lift bounce at startup (17ab865), and there is nothing left to control down there
    // anyway. ABOVE the rest band the output is deliberately NOT clamped downward: pushing up is how
    // the loop brakes an arm that is falling faster than the setpoint is walking, and the old
    // `output = std::min(output, 0)` here made that impossible, so the whole descent could only ever
    // coast -- it arrived at the frame at whatever speed gravity had given it. Which way the bands
    // and gravity net out at a given height is not known, so the loop is simply allowed to push
    // either way and let the error decide.
    if (goal_ <= config::lift_preset::DOWN && position <= config::lift_preset::DOWN + config::LIFT_REST_BAND_DEG) {
        motors_.brake();
        settled_ = true;
        pushStart_ = 0;
        resetPid();
        lastError_ = error;
        lastOutput_ = 0.0;
        lastP_ = lastI_ = lastD_ = 0.0;
        return;
    }

    // 6) Output limit (motors only accept -12000..12000 mV).
    output = std::clamp(output, -config::LIFT_MAX_MV, config::LIFT_MAX_MV);

    lastError_ = error;
    lastOutput_ = output;
    lastP_ = pTerm;
    lastI_ = iTerm;
    lastD_ = dTerm;

    // 7) Runaway guard: the failure it exists for is a direction mismatch -- rotation sensor or
    //    LIFT_PORTS reversed, so the loop pushes the wrong way, the error grows, and it pushes
    //    harder until the lift drives itself into a hard stop. The tell is the ERROR, not the
    //    motion: push hard in the direction that should shrink the error, and watch whether the
    //    error shrinks. It must not be "pushing one way while the arm moves the other", which is
    //    what this used to test -- braking a descending arm is exactly that pattern (output up,
    //    arm still travelling down) and it is the correct thing to be doing, so a controlled
    //    descent kept tripping a guard meant for a wiring fault.
    const int dir = std::fabs(output) >= config::LIFT_FAULT_MV && output * error > 0.0f ? (output > 0 ? 1 : -1) : 0;
    if (dir == 0 || dir != pushDir_) {
        pushDir_ = dir;
        pushStart_ = dir == 0 ? 0 : now;
        pushStartErr_ = std::fabs(error);
    } else if (now - pushStart_ >= static_cast<std::uint32_t>(config::LIFT_FAULT_MS)) {
        if (std::fabs(error) - pushStartErr_ > config::LIFT_FAULT_DEG) {
            faulted_ = true;
            motors_.brake();
            return;
        }
        pushStart_ = now;  // error shrinking or holding: start a new window
        pushStartErr_ = std::fabs(error);
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
