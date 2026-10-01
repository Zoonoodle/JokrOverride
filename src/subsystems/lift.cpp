#include "subsystems/lift.hpp"

#include "config.hpp"
#include "pros/error.h"

#include <algorithm>
#include <cmath>

namespace subsystems {

namespace {
/** Fold a sensor angle in centidegrees (0..35999) into the signed range closest to zero,
 *  (-18000, 18000]. A lift resting a hair low reads ~35990 and folds to ~-10, not +35990. */
std::int32_t foldCentideg(std::int32_t centideg) {
    if (centideg > 18000) centideg -= 36000;
    return centideg;
}
}  // namespace

Lift::Lift()
    : motors_(config::LIFT_PORTS, pros::v5::MotorGears::red),  // both lift motors are red (100 RPM) cartridges
      pid_(config::LIFT_KP, config::LIFT_KI, config::LIFT_KD, config::LIFT_WINDUP_RANGE, true) {
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

    // Absolute-angle SEED, not a startup reset: the rotation sensor's own persistent zero point has
    // been calibrated (on the brain's device screen, outside this code) to the physical bottom of the
    // lift, so get_angle() is already a true absolute 0..360 sensor-degree reading from that zero and
    // survives power cycles on its own -- it does NOT need, and must NOT get, a software reset HERE.
    // Resetting it here would throw that calibration away and make every height relative to wherever
    // the arm happened to be sitting at THIS particular boot instead (e.g. a little high from band
    // tension with the motors unpowered) -- exactly the bug this avoids. zero() also commands no
    // motion of any kind, which is what keeps "nothing moves at startup" true of initialize().
    //
    // What we do instead: seed the multi-turn position counter (what get_position() tracks) from the
    // current absolute angle, folded to the signed sensor-degree range closest to 0, (-180, 180].
    //
    // LIMIT, and why homing exists: this is only unambiguous if the lift powers on within +-45 ARM
    // degrees (+-180 sensor degrees -- one quarter of the 4:1-geared sensor's single revolution) of
    // the true bottom. An arm that powered on further up than that seeds to the wrong turn count and
    // every preset is then off by a whole sensor revolution. That is the real failure the user sees as
    // "our arm always starts up too much". requestHome() fixes it properly by driving the arm down to
    // its hard stop at the start of the first ENABLED mode and set_position(0)-ing THERE -- which is a
    // different thing from resetting the sensor here, and is the only place in this class allowed to
    // do it (see updateHoming()). set_reversed() was already applied in the constructor, so it affects
    // get_angle() here and every later get_position() read the same consistent way.
    if (rotation_) {
        const std::int32_t angleCentideg = foldCentideg(rotation_->get_angle());
        startupAngleDeg_ = angleCentideg / 100.0;  // kept for the brain screen only
        rotation_->set_position(angleCentideg);    // seed the counter -- NOT reset_position()
    }
    active_ = false;
    goal_ = target_ = 0.0;
    profVel_ = 0.0;
    precise_ = false;
    faulted_ = false;
    blocked_ = false;
    holding_ = false;
    settled_ = false;
    inBandSince_ = 0;
    pushStart_ = 0;
    stallStart_ = 0;
    home_ = Home::IDLE;
    homeRequested_ = false;
    queuedTarget_ = false;
    resetPid();
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

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------
void Lift::requestHome() {
    // Only ever from IDLE: a second request after a run has homed, failed or been cancelled is a
    // no-op, which is what makes "auton then opcontrol" home exactly once per program run.
    if (home_ != Home::IDLE) return;
    if (!rotation_) {
        home_ = Home::SKIPPED;  // nothing to home against; do NOT move the arm blind
        return;
    }
    homeRequested_ = true;
    home_ = Home::RUNNING;
    homeStart_ = pros::millis();
    homeStillSince_ = 0;
    homeStopAngleDeg_ = 0.0;
    // Homing owns the motors while it runs, so no PID target may be active underneath it.
    active_ = false;
    holding_ = false;
    settled_ = false;
    inBandSince_ = 0;
    resetPid();
}

void Lift::updateHoming(const std::uint32_t now) {
    // A sensor that stopped answering mid-home: stop pushing rather than drive down blind.
    if (!rotation_) {
        home_ = Home::SKIPPED;
        motors_.brake();
        startQueuedTarget();
        return;
    }
    const std::int32_t rawCentideg = rotation_->get_position();
    if (rawCentideg == PROS_ERR) {
        home_ = Home::FAILED;
        motors_.brake();
        startQueuedTarget();
        return;
    }
    const double rawSensorDeg = rawCentideg / 100.0;  // SENSOR degrees, the unit LIFT_HOME_STILL_DEG is in
    const std::uint32_t elapsed = now - homeStart_;

    // Timeout: stop pushing, keep zero()'s seed, say so on the brain.
    if (elapsed >= static_cast<std::uint32_t>(config::LIFT_HOME_TIMEOUT_MS)) {
        homeStopAngleDeg_ = foldCentideg(rotation_->get_angle()) / 100.0;
        home_ = Home::FAILED;
        motors_.brake();
        startQueuedTarget();
        return;
    }

    // "Stopped" = the raw sensor moved less than LIFT_HOME_STILL_DEG across a whole
    // LIFT_HOME_STILL_MS window. Any movement bigger than that re-opens the window, so small wobble
    // at the stop still counts as stopped (which is what the user described: 358-359-0-1 and settling).
    // Only start looking after LIFT_HOME_MIN_DRIVE_MS, so a sticky start isn't read as the bottom.
    bool stopped = false;
    if (elapsed >= static_cast<std::uint32_t>(config::LIFT_HOME_MIN_DRIVE_MS)) {
        if (homeStillSince_ == 0 || std::fabs(rawSensorDeg - homeStillRef_) > config::LIFT_HOME_STILL_DEG) {
            homeStillSince_ = now;
            homeStillRef_ = rawSensorDeg;
        } else if (now - homeStillSince_ >= static_cast<std::uint32_t>(config::LIFT_HOME_STILL_MS)) {
            stopped = true;
        }
    }

    if (!stopped) {
        // Steady, moderate downward drive. NOT full speed: it ends against the frame.
        motors_.move_voltage(static_cast<int>(config::LIFT_HOME_MV));
        lastError_ = 0.0;
        lastOutput_ = config::LIFT_HOME_MV;
        lastP_ = lastI_ = lastD_ = 0.0;
        return;
    }

    // Stopped. Stop pushing first -- whatever we decide next, the motors must not stay stalled
    // against the frame.
    motors_.brake();
    lastOutput_ = 0.0;
    const std::int32_t angleCentideg = foldCentideg(rotation_->get_angle());
    homeStopAngleDeg_ = angleCentideg / 100.0;

    // Plausibility: the sensor's own calibrated absolute zero IS the bottom of the lift, so if we
    // really are at the hard stop the folded absolute angle must be near 0. If it isn't, the arm is
    // jammed on something partway down -- do NOT zero there, that would bake the jam into every
    // preset. Keep zero()'s seed and report it instead.
    if (std::fabs(homeStopAngleDeg_) <= config::LIFT_HOME_ZERO_WINDOW_DEG) {
        rotation_->set_position(0);  // the ONE place in this class that may zero the sensor -- see zero()
        home_ = Home::DONE;
    } else {
        home_ = Home::FAILED;
    }
    if (home_ == Home::DONE) {
        // Hold at the bottom exactly like a normal arrival at DOWN, so the loop has an owner and the
        // motors are not left stalled against the frame. The rest-band branch in update() (position is
        // 0 now, we just zeroed it) turns this straight into a plain brake-hold.
        active_ = true;
        goal_ = 0.0;
        target_ = getPosition();
        profVel_ = 0.0;
        precise_ = false;
        descentFloor_ = 0.0;
        holding_ = false;
        inBandSince_ = 0;
        resetPid();
    } else {
        // FAILED: the arm stopped somewhere that is not the bottom, so it is jammed on something.
        // Leave NO target active -- update()'s "nothing commanded" branch just brake-holds where it
        // is. Commanding DOWN here instead would push straight back into whatever it is caught on
        // (and, with the seed un-zeroed, from a position the code doesn't really know).
        active_ = false;
    }
    startQueuedTarget();  // a button pressed during homing still does what the driver asked
}

void Lift::startQueuedTarget() {
    if (!queuedTarget_) return;
    queuedTarget_ = false;
    setTarget(queuedGoal_, queuedPrecise_);
}

const char* Lift::homeStateName() const {
    switch (home_) {
        case Home::IDLE: return "";
        case Home::RUNNING: return "HOMING";
        case Home::DONE: return "HOMED";
        case Home::FAILED: return "HOME FAIL";
        case Home::CANCELLED: return "HOME CANCELLED";
        case Home::SKIPPED: return "HOME SKIPPED";
    }
    return "";
}

// ---------------------------------------------------------------------------
// Targets
// ---------------------------------------------------------------------------
void Lift::setTarget(double degrees, bool precise) {
    const double clamped = std::clamp(degrees, config::lift_preset::DOWN, config::lift_preset::MAX);
    // INTAKE is always precise, wherever it is commanded from, so no call site can forget: it is the
    // one height that has to physically line up with the intake handoff (see LIFT_PRECISE_* in
    // config.hpp). Everything else trades a few degrees of accuracy for a stop that doesn't hunt.
    const bool wantPrecise = precise || std::fabs(clamped - config::lift_preset::INTAKE) < 0.5;

    // Homing owns the motors until it finishes. Queue the target rather than dropping it (so a button
    // pressed during homing still does what the driver asked) and rather than letting it abort homing
    // (so the arm still gets a trustworthy zero).
    if (home_ == Home::RUNNING) {
        queuedTarget_ = true;
        queuedGoal_ = clamped;
        queuedPrecise_ = wantPrecise;
        return;
    }

    if (active_ && clamped == goal_ && wantPrecise == precise_) return;  // same move, already running
    active_ = true;
    blocked_ = false;
    // A deliberate new command clears a latched runaway fault: the guard's job is to stop the motion
    // that was happening, not to brick every button until someone wiggles the D-pad. A real wiring
    // fault re-faults within LIFT_FAULT_MS, so nothing is lost.
    faulted_ = false;
    stallStart_ = 0;
    pushStart_ = 0;
    goal_ = clamped;
    precise_ = wantPrecise;
    // The profile always starts from where the arm actually is, never from a stale setpoint.
    target_ = getPosition();
    profVel_ = 0.0;
    descentFloor_ = 0.0;  // a brand-new move never inherits a stale pivot floor, see setDescentFloor()
    holding_ = false;
    settled_ = false;
    inBandSince_ = 0;
    resetPid();  // old integral/derivative belong to the old target
}

void Lift::moveTo(double degrees, int timeoutMs, bool waitUntilSettled) {
    setTarget(degrees);
    if (!waitUntilSettled) return;
    const std::uint32_t start = pros::millis();
    // isHoming() is part of the wait on purpose: a blocking auton move asked for during homing is
    // queued, so "not settled yet" is the truth until homing ends and the queued move finishes.
    while (isHoming() || (!isSettled() && !isBlocked() && !isFaulted())) {
        if (timeoutMs > 0 && pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) break;
        pros::delay(10);
    }
}

void Lift::cancel() {
    active_ = false;
    blocked_ = false;
    stallStart_ = 0;
    settled_ = false;
    holding_ = false;
    inBandSince_ = 0;
    profVel_ = 0.0;
    queuedTarget_ = false;
    resetPid();
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

// ---------------------------------------------------------------------------
// The one control loop
// ---------------------------------------------------------------------------
void Lift::update() {
    const std::uint32_t now = pros::millis();
    const double dt = lastUpdate_ == 0 ? 0.01 : (now - lastUpdate_) / 1000.0;
    lastUpdate_ = now;
    const double position = getPosition();

    // 1) Driver manual override (D-pad Up/Down): full send, no feedforward -- same feel as plain
    //    motor power. This is the ONLY thing allowed to preempt everything else, and it always wins:
    //    cancel whatever target was active so nothing "fights" the driver, cancel homing (and don't
    //    retry it this run -- the arm is wherever the driver put it, and zero()'s seed stands), and
    //    make sure release doesn't jump back to an old target either (see class comment).
    if (std::fabs(manualPower_) > 0.02) {
        if (active_) cancel();
        if (home_ == Home::RUNNING) {
            home_ = Home::CANCELLED;
            queuedTarget_ = false;
        }
        wasManual_ = true;
        const double mv = std::clamp(manualPower_, -1.0, 1.0) * config::LIFT_MAX_MV;
        motors_.move_voltage(static_cast<int>(mv));
        settled_ = false;
        holding_ = false;
        inBandSince_ = 0;
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

    // 2) Homing: find the hard stop and zero there. Runs as a mode in this same task, so it is still
    //    this one function writing the motors, and nothing anywhere blocks on it. The stall and
    //    runaway guards are not reached while homing -- deliberately: homing's whole job is to drive
    //    the arm into a hard stop and notice that it stopped, which is exactly what those guards
    //    exist to prevent in normal operation. updateHoming() has its own timeout instead.
    if (home_ == Home::RUNNING) {
        updateHoming(now);
        return;
    }

    // 3) No commanded height: brake and hold right here. This is the state at power-on (nothing
    //    moves until something explicitly calls setTarget()/moveTo()) and right after the driver
    //    lets go of the manual override (no old target resumes).
    if (!active_) {
        motors_.brake();
        settled_ = false;
        holding_ = false;
        inBandSince_ = 0;
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

    // 4) Going to DOWN: inside the rest band, stop controlling altogether and just brake-hold. That
    //    is the one place the gravity feedforward must not be allowed to push UP -- an arm already
    //    sitting on (or held a hair above, by the rubber bands) the hard stop with kG under it is what
    //    made the lift bounce at startup (17ab865), and there is nothing left to control down there.
    if (goal_ <= config::lift_preset::DOWN && position <= config::lift_preset::DOWN + config::LIFT_REST_BAND_DEG) {
        motors_.brake();
        target_ = goal_;
        profVel_ = 0.0;
        holding_ = true;
        settled_ = true;
        pushStart_ = 0;
        stallStart_ = 0;
        resetPid();
        lastError_ = goal_ - position;
        lastOutput_ = 0.0;
        lastP_ = lastI_ = lastD_ = 0.0;
        return;
    }

    // 5) Arrival band, with hysteresis. Inside tolerance (for LIFT_HOLD_MS) the loop stops chasing the
    //    target and holds; it only goes back to chasing once the arm has drifted LIFT_HOLD_RELEASE_DEG
    //    FURTHER than the tolerance. This -- not a gain -- is what stops the hunting the old
    //    high-gain-chase-forever loop did. The band is only considered once the profile has actually
    //    finished, so flying through it at speed mid-move can never be mistaken for arriving.
    const double gap = goal_ - position;
    if (target_ == goal_) {
        if (std::fabs(gap) <= tolerance() + (holding_ ? config::LIFT_HOLD_RELEASE_DEG : 0.0)) {
            if (inBandSince_ == 0) inBandSince_ = now;
            if (now - inBandSince_ >= static_cast<std::uint32_t>(config::LIFT_HOLD_MS)) holding_ = true;
        } else {
            inBandSince_ = 0;
            holding_ = false;
        }
    } else {
        inBandSince_ = 0;
        holding_ = false;
    }

    if (holding_) {
        // Gentle hold: gravity feedforward plus a low-gain, low-authority P term. A precise goal keeps
        // the full loop gain instead, because its band is only LIFT_PRECISE_TOLERANCE_DEG wide and it
        // really does have to close that last degree. Capped under LIFT_FAULT_MV and LIFT_STALL_MV, so
        // a hold can never look like a runaway or a stall to the guards below.
        const float holdKp = precise_ ? config::LIFT_KP : config::LIFT_HOLD_KP;
        const float holdMax = precise_ ? config::LIFT_MAX_MV : config::LIFT_HOLD_MAX_MV;
        float output = config::LIFT_KG + holdKp * static_cast<float>(gap);
        output = std::clamp(output, -holdMax, holdMax);
        motors_.move_voltage(static_cast<int>(output));
        settled_ = true;
        pushStart_ = 0;
        stallStart_ = 0;
        resetPid();
        lastError_ = gap;
        lastOutput_ = output;
        lastP_ = holdKp * static_cast<float>(gap);
        lastI_ = lastD_ = 0.0;
        return;
    }
    settled_ = false;

    // 6) Trapezoidal profile on the setpoint. vStop is the fastest the profile may be going and still
    //    stop in the distance that is left (v = sqrt(2*a*d)) -- that is the whole deceleration, known
    //    in advance instead of discovered by overshooting. The start-speed floor is itself capped by
    //    vStop, so it can make a move begin instantly without ever stopping it ending cleanly.
    const double dir = goal_ >= target_ ? 1.0 : -1.0;
    const double remaining = std::fabs(goal_ - target_);
    double vMax = config::LIFT_MAX_SPEED_DEG_PER_S;
    if (precise_ && remaining <= config::LIFT_PRECISE_ZONE_DEG)
        vMax = std::min(vMax, config::LIFT_PRECISE_SPEED_DEG_PER_S);
    const double vStop = std::sqrt(2.0 * config::LIFT_DECEL_DEG_PER_S2 * remaining);
    double v = std::min(profVel_ + config::LIFT_ACCEL_DEG_PER_S2 * dt, vMax);
    v = std::min(v, vStop);
    // The start floor is capped by BOTH limits, so it can make a move begin instantly without ever
    // overriding the deceleration (vStop) or a precise goal's slow final approach (vMax).
    v = std::max(v, std::min({config::LIFT_START_SPEED_DEG_PER_S, vStop, vMax}));

    const double prevTarget = target_;
    const double step = v * dt;
    if (step >= remaining) target_ = goal_;
    else target_ += dir * step;

    // 6b) THE LEASH: never let the setpoint get more than LIFT_MAX_LEAD_DEG ahead of the arm. If the
    //     arm can't keep up, the setpoint waits for it. This is what bounds the tracking error -- and
    //     therefore the PID's whole contribution -- no matter how far the goal is, so there is never
    //     any stored error for the arm to overshoot with. It is also why the runaway guard below
    //     watches the distance to the GOAL rather than this error.
    if (dir > 0) target_ = std::min(target_, std::min(goal_, position + config::LIFT_MAX_LEAD_DEG));
    else target_ = std::max(target_, std::max(goal_, position - config::LIFT_MAX_LEAD_DEG));
    // Scoring's pivot-retract floor, for a descent only (see setDescentFloor()). It can delay arrival
    // but never turns the profile into a stop-then-hop.
    if (headingToBottom()) target_ = std::max(target_, descentFloor_);

    // 6c) The profile's speed is read back from how far the setpoint ACTUALLY moved, so the leash, the
    //     floor and arrival at the goal all feed back into it automatically: when the leash is binding
    //     the profile paces the arm instead of running away from it.
    //     Only movement in the DIRECTION OF TRAVEL counts. The leash and the floor are both clamps, so
    //     they can also drag the setpoint BACKWARDS in one tick -- if the arm sags while the profile is
    //     heading up, the leash follows it down. Feeding that backwards jump into the feedforward would
    //     command a big DOWNWARD voltage on the exact tick we are trying to go up, which is the
    //     opposite of what the leash is for. So a backwards tick contributes zero: the feedforward goes
    //     quiet, the PID (which sees the error) deals with it, and the profile restarts from
    //     LIFT_START_SPEED_DEG_PER_S next tick. Clamped to v so a floor change can't spike either.
    const double applied = dt > 0.0 ? (target_ - prevTarget) / dt : 0.0;
    const double advance = std::clamp(dir > 0 ? std::max(applied, 0.0) : std::min(applied, 0.0), -v, v);
    profVel_ = std::fabs(advance);
    const float vff = static_cast<float>(config::LIFT_KV * advance);

    // 7) PID + gravity feedforward + velocity feedforward. The P and D terms below are computed the
    //    same way lemlib::PID computes them internally (see include/lemlib/pid.hpp: update() takes no
    //    dt, so it can't be doing any time-based scaling -- kP*error and kD*(error-prevError), one
    //    call per tick, same as here). The I term is the exact residual against the real PID output,
    //    so the telemetry always adds up. This is for the tuning screen only; it never feeds back.
    const float error = static_cast<float>(target_ - position);
    const float pTerm = config::LIFT_KP * error;
    const float dTerm = config::LIFT_KD * (error - static_cast<float>(shadowPrevError_));
    shadowPrevError_ = error;
    const float pidOut = pid_.update(error);
    const float iTerm = pidOut - pTerm - dTerm;
    float output = pidOut + config::LIFT_KG + vff;

    // 8) Output limit (motors only accept -12000..12000 mV).
    output = std::clamp(output, -config::LIFT_MAX_MV, config::LIFT_MAX_MV);

    lastError_ = error;
    lastOutput_ = output;
    lastP_ = pTerm;
    lastI_ = iTerm;
    lastD_ = dTerm;

    // 9) Runaway guard: the failure it exists for is a direction mismatch -- rotation sensor or
    //    LIFT_PORTS reversed, so the loop pushes the wrong way, the arm goes further from the goal,
    //    and it pushes harder until the lift drives itself into a hard stop. The tell is the DISTANCE
    //    TO THE GOAL, not the motion and not the tracking error: push hard in the direction that
    //    should close that distance, and watch whether it closes. It must not be "pushing one way
    //    while the arm moves the other", which is what this used to test -- braking a descending arm
    //    is exactly that pattern and is the correct thing to be doing. And it must not be the
    //    tracking error either, because the leash in 6b deliberately keeps that small by dragging the
    //    setpoint along with the arm, which a reversed sensor would hide behind.
    const int dir9 = std::fabs(output) >= config::LIFT_FAULT_MV && output * static_cast<float>(gap) > 0.0f
                         ? (output > 0 ? 1 : -1)
                         : 0;
    if (dir9 == 0 || dir9 != pushDir_) {
        pushDir_ = dir9;
        pushStart_ = dir9 == 0 ? 0 : now;
        pushStartGap_ = std::fabs(gap);
    } else if (now - pushStart_ >= static_cast<std::uint32_t>(config::LIFT_FAULT_MS)) {
        if (std::fabs(gap) - pushStartGap_ > config::LIFT_FAULT_DEG) {
            faulted_ = true;
            motors_.brake();
            return;
        }
        pushStart_ = now;  // gap shrinking or holding: start a new window
        pushStartGap_ = std::fabs(gap);
    }

    // 10) Stall guard: the profile still has somewhere to go but the arm isn't going there (preset
    //     above the real top, or jammed). Instead of grinding, give up on that target and hold.
    //     The test is "profile unfinished AND not moving", NOT the old "output high AND not moving",
    //     because the leash in 6b makes a high output impossible during a jam: the setpoint stops
    //     dead against the arm, so the feedforward goes to zero and the error is pinned at
    //     LIFT_MAX_LEAD_DEG -- which is only ~3000 mV going up, and a few hundred going down, where
    //     gravity feedforward and the error nearly cancel. A voltage threshold would simply never
    //     fire, and the lift would push quietly at a jam forever. "The profile wants to move and the
    //     arm doesn't" is the real condition, works in both directions, and cannot false-trip: the
    //     slowest the profile ever walks is LIFT_PRECISE_SPEED_DEG_PER_S, which still covers 17 deg
    //     in a LIFT_STALL_MS window, and a full-speed move covers ~1.4 deg per single tick.
    //     The one exception is Scoring's pivot floor deliberately holding the setpoint still: that is
    //     "not moving" on purpose and only lasts PIVOT_TIME_MS, so it is excluded outright.
    const bool floorHolding =
        descentFloor_ > 0.0 && headingToBottom() && target_ <= descentFloor_ + 1e-9;
    if (target_ != goal_ && !floorHolding) {
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
