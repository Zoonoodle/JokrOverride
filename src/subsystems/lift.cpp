#include "subsystems/lift.hpp"

#include "config.hpp"
#include "pros/error.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace subsystems {

namespace {
/** Fold a sensor angle in centidegrees (0..35999) into the signed range closest to zero,
 *  (-18000, 18000]. A lift resting a hair low reads ~35990 and folds to ~-10, not +35990. */
std::int32_t foldCentideg(std::int32_t centideg) {
    if (centideg > 18000) centideg -= 36000;
    return centideg;
}

std::int16_t toI16(double v) {
    return static_cast<std::int16_t>(std::clamp(v, -32000.0, 32000.0));
}
}  // namespace

Lift::Lift() : motors_(config::LIFT_PORTS, pros::v5::MotorGears::red) {  // both lift motors are red (100 RPM) cartridges
    motors_.set_encoder_units_all(pros::v5::MotorEncoderUnits::degrees);
    // HOLD is what every brake() in this file means: the firmware's own position hold. It is the HOLD
    // phase of a move, the idle state, and what catches the arm when the driver lets go of the D-pad.
    motors_.set_brake_mode_all(pros::v5::MotorBrake::hold);
    if (config::LIFT_ROTATION_PORT != 0) {
        rotation_ = std::make_unique<pros::Rotation>(config::LIFT_ROTATION_PORT);
        rotation_->set_reversed(config::LIFT_ROTATION_REVERSED);
    }
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
        rotation_->set_position(angleCentideg);  // seed the counter -- NOT reset_position()
    }
    active_ = false;
    goal_ = 0.0;
    precise_ = descent_ = false;
    phase_ = Phase::HOLD;
    holdGoal_ = 0.0;
    velI_ = 0.0;
    gravityMv_ = 0.0;
    descendHeld_ = descendOwned_ = descendRequest_ = false;
    seatHot_ = false;
    seatStillSince_ = 0;
    restWakes_ = 0;
    parkMv_ = 0.0;
    parkTimeout_ = false;
    holdSettleSince_ = 0;
    score_ = returnFast_ = scoreStuck_ = releasedByTravel_ = releasedByFloor_ = false;
    scoredAbove_ = scoreCycle_ = returnRequest_ = false;
    grab_ = gripWanted_ = gripAcked_ = false;
    clawReleaseWanted_ = clawReleased_ = false;
    contact_.reset();
    contactTrigger_ = ContactDetector::Trigger::NONE;
    unstickTries_ = 0;
    faulted_ = false;
    blocked_ = false;
    stallStart_ = 0;
    havePrevPosition_ = false;
    velocity_ = 0.0;
    home_ = Home::IDLE;
    homeRequested_ = false;
    queuedTarget_ = false;
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
    // Homing owns the motors while it runs, so no target may be active underneath it.
    active_ = false;
    phase_ = Phase::HOLD;
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
        lastOutput_ = 0.0;
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
        havePrevPosition_ = false;  // the position just jumped to 0 by decree; don't read that as a velocity
    } else {
        home_ = Home::FAILED;
    }
    if (home_ == Home::DONE) {
        // Homing just proved the arm is physically on its hard stop, which is exactly what SEAT means,
        // so adopt a DOWN target already in SEAT: the loop has an owner, the arm stays pressed down
        // instead of drifting back up, and arrival at DOWN (isSettled()) is true from here. The seat's
        // own temperature guard applies, same as after an L2.
        active_ = true;
        goal_ = config::lift_preset::DOWN;
        precise_ = false;
        descent_ = true;
        descentTopPct_ = config::LIFT_DESCENT_TOP_PCT;
        descentMidPct_ = config::LIFT_DESCENT_MID_PCT;
        descentStart_ = getPosition();
        descentMidDeg_ = config::LIFT_DESCENT_SLOW_ZONE_DEG;
        blocked_ = false;
        seatHot_ = false;
        enterSeat(now);
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
    // One-shot: did this call come from descendManual() (R2 in clamped mode)? Consumed either way, so
    // a queued or ignored request can't leak into a later, unrelated move.
    const bool fromR2 = descendRequest_;
    descendRequest_ = false;
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

    // A genuinely new command: the motors must not still be coasting from REST, the move timeout's
    // clock restarts, and REST's creep counter starts fresh.
    wakeMotors();
    moveStart_ = pros::millis();
    restWakes_ = 0;
    parkTimeout_ = false;
    seatStillSince_ = 0;

    if (active_ && clamped == goal_ && wantPrecise == precise_ && !faulted_ && !blocked_) return;  // same move, already running

    // Is this a RETARGET of a move that is already flying, rather than a brand-new move? A position
    // move already in DRIVE / BRAKE / APPROACH, not faulted or blocked, with the same precision, and a
    // new goal that is still a position move: then the ONLY things that have to change are the goal
    // and the runaway guard's reference. Phase, the velocity loop's integral, the approach clock and
    // the stall window are all left exactly as they are, and update()'s per-tick phase choice picks up
    // the new distance on the very next tick. That is what makes two or three quick L1 taps merge into
    // one continuous rise instead of each tap restarting the move from scratch.
    // A deliberate new height overrides a score sequence outright (and clears its latched reports),
    // exactly as it clears a fault or a stall: the guards' and the macro's job is to stop what was
    // happening, not to brick the next command.
    score_ = false;
    clawReleaseWanted_ = false;
    scoreStuck_ = false;
    releasedByTravel_ = false;
    releasedByFloor_ = false;
    returnFast_ = false;
    // A grab push is a command like any other: a new height replaces it.
    grab_ = gripWanted_ = gripAcked_ = false;
    // Leaving the two-press score cycle -- unless this IS press 2's run home, which startScoreReturn()
    // marks with the one-shot below. Everything else (L1, R1, B, a manual nudge) means the driver has
    // moved on, so the autoclamp may arm again.
    const bool fromReturn = returnRequest_;
    returnRequest_ = false;
    scoredAbove_ = false;
    if (!fromReturn) scoreCycle_ = false;

    // "Flying" = really in the middle of a position move, so moving the goal alone is enough (this is
    // what merges rapid L1 taps). Settled states -- HOLD, PARK, REST, SEAT -- and every special mode
    // must start a fresh move instead, or a retarget from a parked arm would never leave PARK.
    const bool flying = active_ && !descent_ && !faulted_ && !blocked_ &&
                        (phase_ == Phase::DRIVE || phase_ == Phase::BRAKE || phase_ == Phase::APPROACH);
    const bool newIsDescent = !wantPrecise && clamped <= config::LIFT_BOTTOM_GOAL_DEG;
    if (flying && wantPrecise == precise_ && !newIsDescent) {
        goal_ = clamped;
        minDist_ = std::fabs(goal_ - getPosition());
        return;
    }

    active_ = true;
    descendOwned_ = fromR2;
    // A deliberate new command clears a latched runaway fault or a stall: the guards' job is to stop
    // the motion that was happening, not to brick every button until someone wiggles the D-pad. A real
    // wiring fault or jam simply re-trips on the next DRIVE, so nothing is lost.
    faulted_ = false;
    blocked_ = false;
    goal_ = clamped;
    precise_ = wantPrecise;
    // A non-precise goal at the bottom is not a position move at all: it is the banded descent that
    // ends SEATED on the hard stop (L2, B, auton DOWN, R2). INTAKE is numerically inside
    // LIFT_BOTTOM_GOAL_DEG but is always precise, so R1 keeps its own exact approach and never seats.
    descent_ = !precise_ && goal_ <= config::LIFT_BOTTOM_GOAL_DEG;
    const std::uint32_t now = pros::millis();
    const double position = getPosition();
    if (descent_) {
        // R2's descent has its own, gentler top band; L2/B/auton use the config bands.
        const double top = descendOwned_ ? config::R_LIFT_DOWN_PCT : config::LIFT_DESCENT_TOP_PCT;
        const double mid = descendOwned_ ? config::R_LIFT_DOWN_PCT : config::LIFT_DESCENT_MID_PCT;
        enterDescend(now, position, top, mid);
        return;
    }
    // Already inside this goal's band: arrived, no motion at all (so a level re-tap never twitches).
    if (std::fabs(goal_ - position) <= arrivalTolerance()) enterHold(now, goal_);
    else enterDrive(now, position);
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
    wakeMotors();  // never leave the motors coasting once nothing is commanding the lift
    active_ = false;
    blocked_ = false;
    stallStart_ = 0;
    velI_ = 0.0;
    phase_ = Phase::HOLD;
    queuedTarget_ = false;
    descendOwned_ = false;
    descendRequest_ = false;
    // A cancel abandons a score sequence outright, including an outstanding "open the jaws" request:
    // the driver has taken over, and the claw must stay exactly where it physically is.
    score_ = false;
    clawReleaseWanted_ = false;
    returnFast_ = false;
    grab_ = gripWanted_ = gripAcked_ = false;
    scoredAbove_ = false;
    scoreCycle_ = false;
    returnRequest_ = false;
}

const char* Lift::activePresetName() const {
    if (!active_) return "none";
    namespace preset = config::lift_preset;
    if (goal_ == preset::DOWN) return "DOWN";
    if (goal_ == preset::INTAKE) return "INTAKE";
    if (goal_ == preset::MATCHLOAD) return "MATCHLOAD";
    if (goal_ == preset::MAX) return "MAX";
    static const char* const kLevelNames[preset::STACK_LEVEL_COUNT] = {"STACK 1", "STACK 2", "STACK 3", "STACK 4",
                                                                        "STACK 5"};
    for (int i = 0; i < preset::STACK_LEVEL_COUNT; ++i)
        if (goal_ == preset::STACK_LEVELS[i]) return kLevelNames[i];
    return "custom";
}

const char* Lift::phaseName() const {
    if (!active_) return "";
    switch (phase_) {
        case Phase::DRIVE: return "DRIVE";
        case Phase::BRAKE: return "BRAKE";
        case Phase::APPROACH: return "APPROACH";
        case Phase::HOLD:
            // The latched hold L2 press 1 ends in: the arm is parked L2_CLEAR_RISE_DEG above the
            // stack with the jaws open, waiting for a second L2 to run home and seat.
            return scoredAbove_ ? "SCORED UP" : "HOLD";
        case Phase::PARK:
            if (scoredAbove_) return "SCORED UP";
            return parkTimeout_ ? "PARK(t/o)" : "PARK";
        case Phase::REST: return "REST";
        case Phase::DESCEND: return returnFast_ ? "RETURN" : "DESCEND";
        case Phase::FINAL: return "FINAL";
        case Phase::SEAT: return seatHot_ ? "SEAT(hot)" : (seatShort_ ? "SEAT(short)" : "SEAT");
        case Phase::PRESS: return "PRESS";
        case Phase::RELEASE:
            // Which signal fired is the first thing worth knowing when tuning, so it goes on the line.
            // REL(32) means the detector never fired at all and L2_AUTO_RELEASE_DEG's travel rule
            // forced it; REL(floor) means the start was too low for that travel to ever complete and
            // L2_RELEASE_FLOOR_DEG forced it instead.
            if (releasedByTravel_) return "REL(32)";
            if (releasedByFloor_) return "REL(floor)";
            switch (contactTrigger_) {
                case ContactDetector::Trigger::STALL: return "REL(stall)";
                case ContactDetector::Trigger::LAG: return "REL(lag)";
                case ContactDetector::Trigger::EFFORT: return "REL(eff)";
                case ContactDetector::Trigger::SPEED: return "REL(spd)";
                case ContactDetector::Trigger::NONE: break;
            }
            return "REL";
        case Phase::UNSTICK: return "UNSTK";
        case Phase::GRAB: return "GRAB";
        case Phase::POSTPUSH: return "PUSH";
    }
    return "";
}

void Lift::driverPower(double input) { manualPower_ = std::clamp(input, -1.0, 1.0); }

void Lift::descendManual(bool held) {
    if (!held) {
        descendHeld_ = false;
        return;
    }
    if (descendHeld_) return;  // already running; update() keeps it going
    // Rising edge: ask for the ordinary trip to the bottom, flagged as R2's so setTarget() gives it
    // R_LIFT_DOWN_PCT and so letting go of the button ends it (see update()).
    descendHeld_ = true;
    descendRequest_ = true;
    setTarget(config::lift_preset::DOWN);
}

// ---------------------------------------------------------------------------
// Phase helpers
// ---------------------------------------------------------------------------
double Lift::arrivalTolerance() const {
    return precise_ ? config::LIFT_PRECISE_TOLERANCE_DEG : config::LIFT_TOLERANCE_DEG;
}

double Lift::holdDeadband() const {
    // INTAKE (precise) keeps correcting right in; everything else settles inside a band the driver is
    // happy with and is then left alone -- see "Settling instead of hunting" in config.hpp.
    return precise_ ? config::LIFT_HOLD_PRECISE_DEADBAND_DEG : config::LIFT_HOLD_DEADBAND_DEG;
}

void Lift::wakeMotors() {
    if (!coasting_) return;
    // Back to the firmware hold, which is what brake() means everywhere else in this file.
    motors_.set_brake_mode_all(pros::v5::MotorBrake::hold);
    coasting_ = false;
}

double Lift::speedProfile(double dist, double dir) const {
    // Three limits, all unsigned deg/s; the smallest wins at any distance.
    //   cap   -- the cruise speed cap for this direction.
    //   curve -- the stopping-distance relation v = sqrt(2 a d), aiming to reach the final-approach
    //            speed (not zero) LIFT_APPROACH_ZONE_DEG before the goal.
    //   close -- the final approach: a position P loop in speed units, clamped between the crawl's
    //            floor and ceiling, so the arm walks the last few degrees in at a controlled speed and
    //            is practically stopped when the hold takes it. max(curve, close) is what makes
    //            `close` only bite near the end.
    // The FLOOR (LIFT_APPROACH_MIN_DEG_PER_S) is deliberate: kP * dist on its own decays
    // exponentially, so the arm approaches the goal ever more slowly and the last couple of degrees
    // dominate the move's duration. The floor makes the tail a constant walk-in instead, and
    // LIFT_STOPPED_DEG_PER_S sits above the floor so arrival still latches (see config.hpp).
    const double cap = config::LIFT_FREE_SPEED_DEG_PER_S *
                       (dir >= 0.0 ? config::LIFT_UP_SPEED_PCT : config::LIFT_DOWN_SPEED_PCT) / 100.0;
    const double curve =
        std::sqrt(2.0 * config::LIFT_DECEL_DEG_PER_S2 * std::max(dist - config::LIFT_APPROACH_ZONE_DEG, 0.0));
    const double close = std::clamp(config::LIFT_APPROACH_KP_PER_S * dist,
                                    config::LIFT_APPROACH_MIN_DEG_PER_S, config::LIFT_APPROACH_DEG_PER_S);
    return std::min(cap, std::max(curve, close));
}

double Lift::descentSpeed(double position) const {
    // Which band of the descent the arm is in, read off its POSITION (see config.hpp).
    double pct;
    if (position <= config::LIFT_DESCENT_SLOW_ZONE_DEG) pct = config::LIFT_DESCENT_SLOW_PCT;
    else if (position <= descentMidDeg_) pct = descentMidPct_;
    else pct = descentTopPct_;
    return config::LIFT_FREE_SPEED_DEG_PER_S * pct / 100.0;
}

void Lift::enterDrive(std::uint32_t now, double position) {
    phase_ = Phase::DRIVE;
    phaseStart_ = now;
    minDist_ = std::fabs(goal_ - position);
    stallStart_ = 0;
    // A brand-new move starts the velocity loop from scratch; enterBrake() primes it properly at the
    // DRIVE -> BRAKE handover. (A RETARGET of a move already in flight does NOT come through here --
    // see setTarget() -- precisely so that it keeps the integral it has already learned.)
    velI_ = 0.0;
}

void Lift::enterBrake(std::uint32_t now) {
    phase_ = Phase::BRAKE;
    phaseStart_ = now;
    stallStart_ = 0;
    // Seed the velocity loop's integral with the steady-state voltage for what the arm is doing right
    // now: the learned gravity voltage (from the last hold) plus the back-EMF of the current speed.
    // That is what the integral would settle at anyway, so the loop starts on the curve instead of
    // having to integrate its way onto it.
    //
    // What this replaces, and why it mattered: the old seed was dir0 * LIFT_MAX_MV - kv * velocity,
    // i.e. "whatever of the FULL DRIVE voltage is not being eaten by back-EMF". That is only true if
    // the measured velocity is the arm's true velocity -- and the velocity estimate LAGS. A measurement
    // 30 deg/s low turned the seed into +2400 mV of UPWARD bias injected at the exact moment the loop
    // was supposed to start braking. That was a direct cause of L1 overshooting. The seed below has
    // the opposite sensitivity: a velocity read low makes it seed LOW, so the loop brakes slightly
    // harder and stops slightly short, which the final approach then walks in. Lag can only cost
    // accuracy here, never overshoot.
    const double kv = config::LIFT_MAX_MV / config::LIFT_FREE_SPEED_DEG_PER_S;  // mV per deg/s
    velI_ = std::clamp(gravityMv_ + kv * velocity_, -static_cast<double>(config::LIFT_VEL_MAX_MV),
                       static_cast<double>(config::LIFT_VEL_MAX_MV));
}

void Lift::enterHold(std::uint32_t now, double holdAt) {
    wakeMotors();
    phase_ = Phase::HOLD;
    phaseStart_ = now;
    stallStart_ = 0;
    holdSettleSince_ = 0;
    parkTimeout_ = false;
    holdGoal_ = holdAt;
    // Hand the velocity loop's integral over to the hold: after the slow final approach it is the
    // voltage this arm needs at this angle, so the hold starts already knowing it and never has to sag
    // first and then catch up (that sag-and-catch-up was the old design's limit cycle). Take the
    // back-EMF of the arrival speed back out of it first -- the integral was sustaining the crawl as
    // well as fighting gravity, and carrying that extra into a hold that wants ZERO speed would make
    // the arm creep past the level before the KI term unlearned it.
    const double kv = config::LIFT_MAX_MV / config::LIFT_FREE_SPEED_DEG_PER_S;  // mV per deg/s
    gravityMv_ = std::clamp(velI_ - kv * velocity_, -static_cast<double>(config::LIFT_HOLD_MAX_MV),
                            static_cast<double>(config::LIFT_HOLD_MAX_MV));
    velI_ = 0.0;
}

void Lift::enterDescend(std::uint32_t now, double position, double topPct, double midPct) {
    phase_ = Phase::DESCEND;
    phaseStart_ = now;
    descentTopPct_ = topPct;
    descentMidPct_ = midPct;
    descentStart_ = position;
    // "The middle" is a fraction of THIS descent's fast travel, not a fixed height, so a drop from
    // level 1 and a drop from level 5 both get a top band and a middle band.
    descentMidDeg_ = config::LIFT_DESCENT_SLOW_ZONE_DEG +
                     config::LIFT_DESCENT_MID_FRACTION *
                         std::max(position - config::LIFT_DESCENT_SLOW_ZONE_DEG, 0.0);
    descentRef_ = std::max(-velocity_, 0.0);  // ramp on from whatever the arm is already doing
    stallStart_ = 0;
    velI_ = gravityMv_;
    seatHot_ = false;
    seatTempTick_ = 0;
}

void Lift::enterFinal(std::uint32_t now) {
    phase_ = Phase::FINAL;
    phaseStart_ = now;  // FINAL's own clock -- it forces for exactly LIFT_FINAL_FORCE_MS from here
    stallStart_ = 0;
    descentRef_ = std::max(-velocity_, 0.0);  // ramp on from whatever DESCEND already had it doing
    velI_ = std::clamp(gravityMv_, -static_cast<double>(config::LIFT_FINAL_MAX_MV),
                       static_cast<double>(config::LIFT_FINAL_MAX_MV));
}

void Lift::enterSeat(std::uint32_t now, bool shortSeat) {
    // Whatever brought the arm here, it is now on its hard stop with nothing in the jaws that a score
    // could re-grab: the two-press score cycle (and its autoclamp inhibit) is over.
    scoreCycle_ = false;
    phase_ = Phase::SEAT;
    phaseStart_ = now;
    stallStart_ = 0;
    descentRef_ = 0.0;
    velI_ = 0.0;
    gravityMv_ = 0.0;  // the frame is holding the arm now, not the motors
    seatTempTick_ = 0;
    seatShort_ = shortSeat;
    seatStillSince_ = 0;  // REST's "has it actually stopped moving" window starts here
}

void Lift::enterPark(std::uint32_t now, double parkAt, bool timedOut) {
    // SETTLED. Freeze whatever voltage the hold had learned for this height and just keep pushing
    // exactly that -- no P, no D, no integral update, no feedback of any kind. A constant has no loop,
    // so it cannot hunt. The arm stays where it is because that voltage is, by construction, what was
    // holding it a moment ago.
    wakeMotors();
    phase_ = Phase::PARK;
    phaseStart_ = now;
    stallStart_ = 0;
    parkGoal_ = parkAt;
    holdGoal_ = parkAt;
    parkTimeout_ = timedOut;
    parkMv_ = std::clamp(gravityMv_, -static_cast<double>(config::LIFT_HOLD_MAX_MV),
                         static_cast<double>(config::LIFT_HOLD_MAX_MV));
    velI_ = 0.0;
    holdSettleSince_ = 0;
}

void Lift::enterRest(std::uint32_t now) {
    // SETTLED AT THE BOTTOM: motors loose. Brake mode COAST plus 0 V draws nothing at all -- unlike
    // brake(), which with MotorBrake::hold is a firmware position loop. The frame holds the arm.
    phase_ = Phase::REST;
    phaseStart_ = now;
    stallStart_ = 0;
    velI_ = 0.0;
    gravityMv_ = 0.0;  // nothing is being held up; don't carry a stale voltage into the next move
    if (!coasting_) {
        motors_.set_brake_mode_all(pros::v5::MotorBrake::coast);
        coasting_ = true;
    }
    motors_.move_voltage(0);
    lastOutput_ = 0.0;
    lastVRef_ = 0.0;
}

bool Lift::updatePark(std::uint32_t now, double position) {
    // The only way out from in here is a real drift away from where it parked -- and not before
    // LIFT_PARK_COOLDOWN_MS, so PARK and HOLD cannot ping-pong. Anything smaller is exactly the
    // hunting PARK exists to stop.
    if (std::fabs(position - parkGoal_) > config::LIFT_REENGAGE_DEG &&
        now - phaseStart_ >= static_cast<std::uint32_t>(config::LIFT_PARK_COOLDOWN_MS)) {
        enterHold(now, parkGoal_);
        return false;
    }
    motors_.move_voltage(static_cast<int>(parkMv_));
    lastOutput_ = parkMv_;
    lastVRef_ = 0.0;
    lastError_ = parkGoal_ - position;
    trace(now, 'K', position, parkMv_, std::fabs(velocity_) > 2.0);
    return true;
}

bool Lift::stalled(std::uint32_t now, double position) {
    if (stallStart_ == 0 || std::fabs(position - stallStartPos_) >= config::LIFT_STALL_DEG) {
        stallStart_ = now;  // window (re)opens: either just started, or the arm did move
        stallStartPos_ = position;
        return false;
    }
    return now - stallStart_ >= static_cast<std::uint32_t>(config::LIFT_STALL_MS);
}

double Lift::runVelocityLoop(double vRef, double dt, double upMaxMv, double downMaxMv) {
    const double err = vRef - velocity_;
    double out = config::LIFT_VEL_KP * err + velI_;
    // The limits are per-call, and ASYMMETRIC, because L2's press needs a much tighter cap on how hard
    // it may push DOWN (L2_PRESS_MAX_MV, the crush limit -- the integral winds up against whatever is
    // resisting, so without it the press would keep raising the push until something moved, i.e.
    // flatten the stack) while still needing full UPWARD authority: holding a loaded six-bar back to
    // only 90 deg/s at a high angle can easily cost more than the press cap, and capping that side too
    // would let the arm outrun its own commanded descent and land hard.
    if (out > upMaxMv) out = upMaxMv;
    else if (out < -downMaxMv) out = -downMaxMv;
    else velI_ = std::clamp(velI_ + config::LIFT_VEL_KI * err * dt, -downMaxMv, upMaxMv);  // unsaturated only
    motors_.move_voltage(static_cast<int>(out));
    lastOutput_ = out;
    return out;
}

void Lift::runHold(std::uint32_t now, double dt, double position) {
    // A damped position hold. The KD term is the whole point: it opposes whatever speed the arm still
    // has when the hold takes over, so the six-bar's stored energy is absorbed instead of handed back.
    // The firmware brake-hold this replaces had no damping term we could set, which is why a slightly
    // fast arrival rang up and down for half a second.
    const double band = holdDeadband();
    const double err = holdGoal_ - position;
    const bool inBand = std::fabs(err) <= band;
    double p = 0.0;
    if (err > band) p = config::LIFT_HOLD_KP * (err - band);
    else if (err < -band) p = config::LIFT_HOLD_KP * (err + band);
    double out = p - config::LIFT_HOLD_KD * velocity_ + gravityMv_;
    const double sat = config::LIFT_HOLD_MAX_MV;
    if (out > sat) out = sat;
    else if (out < -sat) out = -sat;
    // FREEZE THE INTEGRAL INSIDE THE BAND. Letting it keep winding against stiction while the P term
    // is deliberately silent is precisely the limit cycle this is meant to end: it winds until the arm
    // breaks free, overshoots, and then has to unwind. Inside the band the hold is just the gravity
    // voltage it has already learned.
    else if (!inBand) gravityMv_ = std::clamp(gravityMv_ + config::LIFT_HOLD_KI * err * dt, -sat, sat);
    motors_.move_voltage(static_cast<int>(out));
    lastOutput_ = out;
    lastVRef_ = 0.0;
    // Keep tracing while the arm is still settling, so a bounce would be visible in the trace.
    trace(now, 'H', position, out, std::fabs(velocity_) > 2.0);

    // ...and then STOP CORRECTING. Either the arm has been inside the band and slow long enough to
    // call it settled, or the hold has simply had its LIFT_SETTLE_TIMEOUT_MS -- whichever comes first,
    // PARK freezes this voltage and takes the loop out of the picture entirely.
    if (inBand && std::fabs(velocity_) <= config::LIFT_STOPPED_DEG_PER_S) {
        if (holdSettleSince_ == 0) holdSettleSince_ = now;
    } else {
        holdSettleSince_ = 0;
    }
    const bool confirmed = holdSettleSince_ != 0 &&
                           now - holdSettleSince_ >= static_cast<std::uint32_t>(config::LIFT_PARK_CONFIRM_MS);
    const bool heldLongEnough =
        now - phaseStart_ >= static_cast<std::uint32_t>(config::LIFT_SETTLE_TIMEOUT_MS);
    if (confirmed || heldLongEnough) enterPark(now, holdGoal_, false);
}

// ---------------------------------------------------------------------------
// L2 scoring, PRESS 1: PRESS -> RELEASE -> (UNSTICK) -> POSTPUSH -> rise + hold
// (press 2 is startScoreReturn(), further down)
// ---------------------------------------------------------------------------
void Lift::startScore() {
    // Owns the motors from here as an active command with DOWN as its nominal goal, so everything
    // that asks "where is the lift heading" still gets a sensible answer and isSettled() is false
    // until the arm is actually seated at the bottom.
    active_ = true;
    goal_ = config::lift_preset::DOWN;
    precise_ = false;
    descent_ = false;
    faulted_ = false;
    blocked_ = false;
    score_ = true;
    returnFast_ = false;
    scoreStuck_ = false;
    scoredAbove_ = false;
    scoreCycle_ = true;  // ...until press 2 seats the arm: the autoclamp is inhibited throughout
    grab_ = gripWanted_ = gripAcked_ = false;
    releasedByTravel_ = false;
    releasedByFloor_ = false;
    clawReleaseWanted_ = false;
    clawReleased_ = false;
    contact_.reset();
    contactTrigger_ = ContactDetector::Trigger::NONE;
    freeSince_ = 0;
    stuckSince_ = 0;
    unstickTries_ = 0;
    pressRef_ = std::max(-velocity_, 0.0);  // ramp on from whatever the arm is already doing
    velI_ = gravityMv_;
    phase_ = Phase::PRESS;
    phaseStart_ = pros::millis();
    stallStart_ = 0;
    descentStart_ = getPosition();  // the runaway guard's reference for the whole sequence
    scoreStartPos_ = descentStart_;  // FIXED for the whole score -- the origin for L2_AUTO_RELEASE_DEG's
                                     // relative travel rule. Unlike descentStart_, UNSTICK never moves it.
}

void Lift::clawReleaseDone() {
    if (!clawReleaseWanted_ || clawReleased_) return;
    clawReleased_ = true;
    clawReleasedAt_ = pros::millis();
}

// ---------------------------------------------------------------------------
// THE GRAB: a constant downward push while the jaws close (see lift.hpp)
// ---------------------------------------------------------------------------
void Lift::startGrab() {
    // Homing owns the motors and is finding the true zero: do NOT push against it. The caller sees
    // isGrabbing() == false and falls back to the ordinary close-then-rise (see Scoring::update()).
    if (home_ == Home::RUNNING) return;
    const std::uint32_t now = pros::millis();
    active_ = true;
    goal_ = getPosition();  // nominal, so "where is it heading" has a sane answer
    precise_ = false;
    descent_ = false;
    descendOwned_ = false;
    descendRequest_ = false;
    score_ = false;
    returnFast_ = false;
    scoreStuck_ = false;
    scoredAbove_ = false;
    clawReleaseWanted_ = false;
    faulted_ = false;
    blocked_ = false;
    grab_ = true;
    gripWanted_ = false;
    gripAcked_ = false;
    grabStart_ = now;
    velI_ = 0.0;
    phase_ = Phase::GRAB;
    phaseStart_ = now;
    stallStart_ = 0;
}

void Lift::clawGripDone() {
    if (!gripWanted_ || gripAcked_) return;
    gripAcked_ = true;
    gripAckedAt_ = pros::millis();
}

void Lift::updateGrab(std::uint32_t now, double dt, double position) {
    const std::uint32_t t = now - grabStart_;
    lastError_ = 0.0;
    lastVRef_ = 0.0;

    // Ask for the jaws partway through the push, so they close around the pin while the claw is
    // pressed down onto it rather than above it.
    if (!gripWanted_ && t >= static_cast<std::uint32_t>(config::L1_GRAB_CLOSE_AT_MS)) gripWanted_ = true;

    // Stop pushing only when the push has had its time AND the jaws have had theirs. The arm must
    // never pull up out of a claw that is still physically closing -- the same interlock CLAMPING
    // enforces. L1_GRAB_MAX_MS is the safety net for "nobody ever acknowledged" (no Scoring tick).
    const bool pushedLongEnough = t >= static_cast<std::uint32_t>(config::L1_GRAB_PUSH_MS);
    const bool jawsShut = gripAcked_ &&
                          now - gripAckedAt_ >= static_cast<std::uint32_t>(config::CLAW_CLOSE_TIME_MS);
    const bool timedOut = t >= static_cast<std::uint32_t>(config::L1_GRAB_MAX_MS);
    if ((pushedLongEnough && jawsShut) || timedOut) {
        grab_ = false;
        gripWanted_ = false;
        // HOLD right here. Scoring's L1 path commands the rise on its next tick; auton::grab() simply
        // does not, and the arm stays exactly where the push left it, actively held.
        goal_ = position;
        enterHold(now, position);
        runHold(now, dt, position);
        return;
    }

    const double mv = -config::LIFT_MAX_MV * config::L1_GRAB_PUSH_PCT / 100.0;
    motors_.move_voltage(static_cast<int>(mv));
    lastOutput_ = mv;
    trace(now, 'G', position, mv, true);
}

// ---------------------------------------------------------------------------
// L2 press 1's ending, and press 2
// ---------------------------------------------------------------------------
void Lift::endScoreClear(std::uint32_t now, double position) {
    // PRESS 1 IS OVER. The pin is off and the jaws are clear of it, so rise L2_CLEAR_RISE_DEG above
    // wherever the press actually stopped and hold there, latched, until a second L2 (or anything
    // else) says otherwise -- the driver's "drive away with the claw above the pins".
    score_ = false;
    clawReleaseWanted_ = false;
    returnFast_ = false;
    descent_ = false;
    scoredAbove_ = true;
    scoreCycle_ = true;  // still inside the cycle: the autoclamp stays inhibited until press 2 seats
    active_ = true;
    precise_ = false;
    blocked_ = false;
    goal_ = std::min(position + config::L2_CLEAR_RISE_DEG, config::lift_preset::MAX);
    // An ordinary position move does the rise: DRIVE -> BRAKE -> APPROACH -> HOLD, already tuned.
    enterDrive(now, position);
}

void Lift::startScoreReturn() {
    const std::uint32_t now = pros::millis();
    const double position = getPosition();
    returnRequest_ = true;  // one-shot: tells setTarget() this is press 2, so the cycle flag survives
    descendRequest_ = false;
    returnFast_ = true;     // brain/trace: RETURN
    setTarget(config::lift_preset::DOWN);
    // setTarget() picked the config descent bands; press 2 is the FAST run home, so re-enter with
    // L2_RETURN_SPEED_PCT for both fast bands (the slow zone, FINAL and SEAT are untouched).
    if (descent_) enterDescend(now, position, config::L2_RETURN_SPEED_PCT, config::L2_RETURN_SPEED_PCT);
    returnFast_ = true;     // enterDescend() doesn't touch it, but setTarget() cleared it above
}

void Lift::updateScore(std::uint32_t now, double dt, double position) {
    lastError_ = goal_ - position;
    const bool unsticking = phase_ == Phase::UNSTICK;
    const double commanded = config::LIFT_FREE_SPEED_DEG_PER_S *
                             (unsticking ? config::L2_UNSTICK_SPEED_PCT : config::L2_SCORE_SPEED_PCT) / 100.0;

    // Ramp the speed reference, same mechanism as the descent's band changes.
    const double maxStep = config::LIFT_DESCENT_RAMP_DEG_PER_S2 * dt;
    pressRef_ += std::clamp(commanded - pressRef_, -maxStep, maxStep);
    lastVRef_ = unsticking ? pressRef_ : -pressRef_;  // up while unsticking, down otherwise

    // The arm's DESCENT speed, unsigned: an arm moving up is not descending slowly, it is stopped or
    // worse, so it floors at 0.
    const double descendSpeed = std::max(-velocity_, 0.0);
    TraceAux aux{};

    // Runaway guard, for the pressing phases only: we are commanding DOWN, so the arm must not climb
    // away from where the sequence started. (UNSTICK climbs on purpose, so it is exempt, and it moves
    // descentStart_ up with it when it hands back.)
    if (!unsticking && position > descentStart_ + config::LIFT_FAULT_DEG) {
        faulted_ = true;
        score_ = false;
        clawReleaseWanted_ = false;
        motors_.brake();
        lastOutput_ = 0.0;
        trace(now, 'F', position, 0.0, true);
        return;
    }

    // --- UNSTICK: a short hop upward with the jaws already open, then press again. -------------
    if (unsticking) {
        const bool there = position >= unstickGoal_;
        const bool timedOut = now - phaseStart_ >= static_cast<std::uint32_t>(config::L2_UNSTICK_TIMEOUT_MS);
        if (there || timedOut) {
            // Back to pressing down and re-judging. The claw is open, so this is a RELEASE-phase
            // press: the arm getting moving again means free, staying stopped means try again or give up.
            phase_ = Phase::RELEASE;
            phaseStart_ = now;
            pressRef_ = 0.0;
            freeSince_ = 0;
            stuckSince_ = 0;
            descentStart_ = position;
            clawReleasedAt_ = now;  // restart the "jaws have had time to open" clock from here
            velI_ = gravityMv_;
            lastVRef_ = 0.0;  // this one tick commands a standstill; the ramp takes over next tick
        } else {
            const double mv = runVelocityLoop(lastVRef_, dt, config::LIFT_VEL_MAX_MV, config::LIFT_VEL_MAX_MV);
            trace(now, 'U', position, mv, true);
            return;
        }
    }

    if (phase_ == Phase::PRESS) {
        // --- Has the pin landed? subsystems::ContactDetector decides (pure, host-tested -- see its
        // header and test/contact_detector_test.cpp). It is fed the PREVIOUS tick's output, which is
        // what lastOutput_ still holds here: the effort signal compares the voltage the loop actually
        // sent while descending against the voltage a free descent needed, and this tick's has not
        // been computed yet.
        const ContactDetector::Config cfg{
            config::L2_CONTACT_SETTLE_MS,        config::L2_CONTACT_IGNORE_MS,
            config::L2_CONTACT_MS,               config::L2_CONTACT_BASELINE_TAU_MS,
            config::L2_CONTACT_OUTPUT_TAU_MS,    config::L2_CONTACT_LAG_DEG,
            config::L2_CONTACT_LAG_LEAK_MS,      config::L2_CONTACT_EFFORT_MV,
            config::L2_CONTACT_SPEED_FRAC,       config::L2_CONTACT_FREEZE_FRAC,
            config::L2_CONTACT_STALL_DEG_PER_S,  config::L2_CONTACT_ARM_TRAVEL_DEG,
            config::L2_CONTACT_NOSTART_MS,
        };
        const ContactDetector::Trigger fired = contact_.step(dt, descendSpeed, lastOutput_, cfg);
        aux.ratioPct = static_cast<int>(contact_.speedRatio() * 100.0);
        aux.lag10 = static_cast<int>(contact_.lagDeg() * 10.0);
        aux.effortMv = static_cast<int>(contact_.effortMv());
        aux.trigger = ContactDetector::triggerCode(contact_.firedTrigger());

        // GUARANTEED RELEASE, BY RELATIVE TRAVEL: the detector gets first say (checked above, including
        // on this very tick), but if it has not fired once the arm has travelled L2_AUTO_RELEASE_DEG
        // DOWN FROM scoreStartPos_ (fixed at the start of this score, not the live position), release
        // anyway -- there is no more "press to the bottom with nothing released" outcome. This is
        // RELATIVE travel, not an absolute reading: a start too low for that much travel to ever happen
        // before the floor (level 1, 31.0 deg) falls back to the same L2_RELEASE_FLOOR_DEG that
        // RELEASE's own safety net below uses, so it still gets real sensing time rather than releasing
        // on the very first tick.
        const double traveled = scoreStartPos_ - position;
        const bool travelRelease =
            fired == ContactDetector::Trigger::NONE && traveled >= config::L2_AUTO_RELEASE_DEG;
        const bool floorRelease = fired == ContactDetector::Trigger::NONE && !travelRelease &&
                                  position <= config::L2_RELEASE_FLOOR_DEG;
        if (fired != ContactDetector::Trigger::NONE || travelRelease || floorRelease) {
            // CONTACT (detected, or forced by travel/floor) this tick. Ask Scoring for the jaws (it
            // owns the claw) and keep pushing down so they slide off. Scoring polls needsClawRelease()
            // in the same 10 ms driver loop, so the cylinder is commanded within a tick -- no handshake
            // to wait on.
            clawReleaseWanted_ = true;
            clawReleased_ = false;
            contactTrigger_ = fired;  // NONE for either forced path; see releasedByTravel_/releasedByFloor_
            releasedByTravel_ = travelRelease;
            releasedByFloor_ = floorRelease;
            phase_ = Phase::RELEASE;
            phaseStart_ = now;
            freeSince_ = 0;
            stuckSince_ = 0;
            aux.trigger = travelRelease ? 5 : (floorRelease ? 6 : ContactDetector::triggerCode(fired));
        }
    } else {  // Phase::RELEASE or Phase::POSTPUSH
        // Carry the detector's last readings so the r lines show what fired and what it saw.
        aux.ratioPct = static_cast<int>(contact_.speedRatio() * 100.0);
        aux.lag10 = static_cast<int>(contact_.lagDeg() * 10.0);
        aux.effortMv = static_cast<int>(contact_.effortMv());
        aux.trigger = releasedByTravel_ ? 5 : (releasedByFloor_ ? 6 : ContactDetector::triggerCode(contactTrigger_));

        // POSTPUSH has already decided the arm is free -- it is only running out its timer below, so
        // none of the release judging applies to it any more.
        if (phase_ == Phase::POSTPUSH) {
            if (now - phaseStart_ >= static_cast<std::uint32_t>(config::L2_POST_RELEASE_PUSH_MS)) {
                endScoreClear(now, position);
                return;
            }
            const double mv =
                runVelocityLoop(lastVRef_, dt, config::LIFT_VEL_MAX_MV, config::L2_PRESS_MAX_MV);
            trace(now, 'p', position, mv, true, aux);
            return;
        }

        // Don't judge anything until the jaws have actually had time to open. If nobody ever
        // acknowledges the request (no Scoring::update() running, e.g. a bare auton), carry on after
        // L2_RELEASE_ACK_MS rather than pressing down forever.
        if (!clawReleased_ && now - phaseStart_ >= static_cast<std::uint32_t>(config::L2_RELEASE_ACK_MS)) {
            clawReleased_ = true;
            clawReleasedAt_ = now;
        }
        const bool jawsOpen = clawReleased_ &&
                              now - clawReleasedAt_ >= static_cast<std::uint32_t>(config::CLAW_OPEN_TIME_MS);
        // With the jaws open the arm either moves or it does not -- nothing subtle to detect, so this
        // is a plain speed test rather than the full detector.
        if (descendSpeed >= config::L2_CONTACT_STALL_DEG_PER_S) {
            if (freeSince_ == 0) freeSince_ = now;
            stuckSince_ = 0;
        } else {
            freeSince_ = 0;
            if (stuckSince_ == 0) stuckSince_ = now;
        }
        if (clawReleased_ && position <= config::L2_RELEASE_FLOOR_DEG) {
            // Safety floor for THIS phase (separate job from the travel/floor release rules in PRESS,
            // same constant on purpose -- see config.hpp): the jaws haven't resolved free-or-stuck yet
            // but the arm is nearly at the bottom anyway -- stop treating this as a press and hand over
            // to the fast run-home descent right here. Gated on clawReleased_ (acknowledged, or
            // L2_RELEASE_ACK_MS has elapsed) so a PRESS-side floor release landing exactly ON this same
            // height can never race Scoring out of ever seeing needsClawRelease() -- endScore() clears
            // clawReleaseWanted_, and without this gate that could happen before Scoring's own 10 ms
            // loop ever polled it.
            // The jaws are open and we are as low as this press should ever go: skip the extra
            // post-release push (there is nowhere left to push) and go straight to the clear rise.
            endScoreClear(now, position);
            return;
        }
        if (jawsOpen && freeSince_ != 0 &&
            now - freeSince_ >= static_cast<std::uint32_t>(config::L2_FREE_MS)) {
            // THE CLAW CAME OFF THE PIN and the arm is running freely again. Keep pressing down for
            // L2_POST_RELEASE_PUSH_MS so the jaws slide fully clear of it, then rise and hold. Note
            // this is only reached from the FREED check -- a claw still hooked on the pin goes to
            // UNSTICK below instead, so the rise can never drag a pin up with it.
            phase_ = Phase::POSTPUSH;
            phaseStart_ = now;
            // fall through to the press output below, now as POSTPUSH
        }
        if (jawsOpen && stuckSince_ != 0 && now - stuckSince_ >= static_cast<std::uint32_t>(config::L2_STUCK_MS)) {
            // Still stopped with the jaws open: the claw is hooked on the pin.
            if (unstickTries_ < config::L2_MAX_UNSTICK_TRIES) {
                ++unstickTries_;
                phase_ = Phase::UNSTICK;
                phaseStart_ = now;
                pressRef_ = std::max(velocity_, 0.0);
                unstickGoal_ = std::min(position + config::L2_UNSTICK_UP_DEG, config::lift_preset::MAX);
                freeSince_ = 0;
                stuckSince_ = 0;
                velI_ = gravityMv_;
                const double mv = runVelocityLoop(pressRef_, dt, config::LIFT_VEL_MAX_MV, config::LIFT_VEL_MAX_MV);
                trace(now, 'U', position, mv, true, aux);
                return;
            }
            // Out of tries. Stop, hold where it is, and report it -- never keep grinding.
            scoreStuck_ = true;
            score_ = false;
            clawReleaseWanted_ = false;
            blocked_ = true;  // so nothing mistakes this for an arrival at DOWN
            enterHold(now, position);
            runHold(now, dt, position);
            return;
        }
    }

    if (phase_ == Phase::POSTPUSH &&
        now - phaseStart_ >= static_cast<std::uint32_t>(config::L2_POST_RELEASE_PUSH_MS)) {
        endScoreClear(now, position);
        return;
    }

    // Pressing down, with the push capped so the loop cannot crush the stack while the arm slows.
    const double mv = runVelocityLoop(lastVRef_, dt, config::LIFT_VEL_MAX_MV, config::L2_PRESS_MAX_MV);
    trace(now, phase_ == Phase::PRESS ? 'P' : (phase_ == Phase::POSTPUSH ? 'p' : 'r'), position, mv,
          true, aux);
}

// ---------------------------------------------------------------------------
// A trip to the bottom: DESCEND -> SEAT
// ---------------------------------------------------------------------------
void Lift::updateDescent(std::uint32_t now, double dt, double position) {
    lastError_ = goal_ - position;

    // --- phase transitions first, so the output below always matches the phase ---
    if ((phase_ == Phase::SEAT || phase_ == Phase::REST) &&
        position > config::LIFT_FINAL_START_DEG + config::LIFT_SEAT_UNSEAT_DEG) {
        wakeMotors();
        enterDescend(now, position, descentTopPct_, descentMidPct_);  // knocked off the stop: walk it back down
    }
    // REST's own, tighter guard: the motors are LOOSE down here, and this lift's rubber bands pull UP,
    // so the arm can creep off its stop without anything knocking it. Rising past LIFT_REST_WAKE_DEG
    // re-seats it with FINAL's real authority and then rests again -- but only LIFT_REST_MAX_WAKES
    // times, after which it stays SEATed (pressed down at LIFT_SEAT_MV) rather than pulsing forever.
    if (phase_ == Phase::REST && position > config::LIFT_REST_WAKE_DEG) {
        if (restWakes_ < config::LIFT_REST_MAX_WAKES) {
            ++restWakes_;
            wakeMotors();
            enterFinal(now);
        }
    }
    if (phase_ == Phase::DESCEND || phase_ == Phase::FINAL) {
        // Runaway guard, for the whole trip down (DESCEND or FINAL): we are commanding DOWN, so the
        // arm must not climb. A reversed rotation sensor or a reversed LIFT_PORTS sign shows up here
        // within ~60 ms.
        if (position > descentStart_ + config::LIFT_FAULT_DEG) {
            faulted_ = true;
            motors_.brake();
            lastOutput_ = 0.0;
            lastVRef_ = 0.0;
            trace(now, 'F', position, 0.0, true);
            return;
        }
    }
    if (phase_ == Phase::DESCEND) {
        if (position <= config::LIFT_FINAL_START_DEG) {
            // Close enough that the arm MIGHT be on its hard stop -- but it might also be stalled a
            // few degrees short (a cup in the intake, six-bar/chain friction). Hand over to FINAL, which
            // keeps real downward authority and forces for a fixed window rather than trusting "looks
            // stopped" the instant it crosses this line (see "Seating on the hard stop" in config.hpp
            // for why the original single-step version could be fooled by that).
            enterFinal(now);
        } else if (stalled(now, position)) {  // stopped well above the bottom: caught on something
            blocked_ = true;
            enterHold(now, position);  // hold where it stopped, never push into the jam
        }
    } else if (phase_ == Phase::FINAL && now - phaseStart_ >= static_cast<std::uint32_t>(config::LIFT_FINAL_FORCE_MS)) {
        // TIME-BASED, ON PURPOSE: force at LIFT_FINAL_DEG_PER_S/LIFT_FINAL_MAX_MV for the whole window
        // regardless of how settled the arm looks partway through, then judge by POSITION alone, once,
        // right here -- within LIFT_SEAT_ZERO_DEG of 0 is a true SEAT, anything more is SEAT(short) (a
        // genuine jam, not just friction; LIFT_SEAT_MV still holds it down either way).
        enterSeat(now, position > config::LIFT_SEAT_ZERO_DEG);
    }

    // --- output ---
    if (phase_ == Phase::SEAT) {
        // Constant downward pressure, forever: the arm stays forced onto the stop and cannot bounce
        // back up. Temperature is the one thing that can stop it (checked every 50th tick, i.e. twice
        // a second); see the heat note on LIFT_SEAT_MV in config.hpp.
        if (++seatTempTick_ >= 50) {
            seatTempTick_ = 0;
            seatHot_ = getTemperature() >= config::LIFT_SEAT_TEMP_LIMIT_C;
        }
        if (seatHot_) {
            motors_.brake();
            lastOutput_ = 0.0;
        } else {
            motors_.move_voltage(static_cast<int>(config::LIFT_SEAT_MV));
            lastOutput_ = config::LIFT_SEAT_MV;
        }
        lastVRef_ = 0.0;
        trace(now, 'S', position, lastOutput_, false);

        // ...and then let go. Once the seated arm has been genuinely still for LIFT_REST_AFTER_MS,
        // REST coasts the motors so they draw nothing at all for the rest of the match (the driver's
        // request, 2026-10-03). LIFT_REST_AFTER_MS = 0 turns that off and keeps pressing forever.
        if (config::LIFT_REST_AFTER_MS > 0 && restWakes_ < config::LIFT_REST_MAX_WAKES) {
            if (std::fabs(velocity_) <= config::LIFT_REST_STILL_DEG_PER_S) {
                if (seatStillSince_ == 0) seatStillSince_ = now;
                if (now - seatStillSince_ >= static_cast<std::uint32_t>(config::LIFT_REST_AFTER_MS)) {
                    enterRest(now);
                }
            } else {
                seatStillSince_ = 0;
            }
        }
        return;
    }
    if (phase_ == Phase::REST) {
        // Nothing to do: the motors are coasting at 0 V and the frame is holding the arm. The wake
        // guard above is the only thing that can end it from in here; any command, the D-pad or
        // clamped-mode R1/R2 ends it from outside.
        lastOutput_ = 0.0;
        lastVRef_ = 0.0;
        trace(now, 'E', position, 0.0, false);
        return;
    }
    if (phase_ == Phase::HOLD) {  // the descent was BLOCKED above
        runHold(now, dt, position);
        return;
    }
    if (phase_ == Phase::PARK) {  // ...and that blocked hold has since settled and frozen
        if (updatePark(now, position)) return;
        runHold(now, dt, position);
        return;
    }
    if (phase_ == Phase::FINAL) {
        // Constant slow creep, closed loop on the MEASURED speed exactly like DESCEND, but with its
        // own cap: LIFT_FINAL_MAX_MV, stronger than SEAT's hold but well under DESCEND's full
        // LIFT_VEL_MAX_MV, so it can push through light resistance without slamming a real contact.
        const double maxStep = config::LIFT_DESCENT_RAMP_DEG_PER_S2 * dt;
        descentRef_ += std::clamp(config::LIFT_FINAL_DEG_PER_S - descentRef_, -maxStep, maxStep);
        lastVRef_ = -descentRef_;  // signed: down is negative
        const double mv = runVelocityLoop(lastVRef_, dt, config::LIFT_VEL_MAX_MV, config::LIFT_FINAL_MAX_MV);
        trace(now, 'Z', position, mv, true);
        return;
    }

    // DESCEND: ramp the banded reference and let the velocity loop track it. Closed loop on the
    // MEASURED arm speed, so gravity plus the pin's weight cannot make the descent run away.
    const double want = descentSpeed(position);
    const double maxStep = config::LIFT_DESCENT_RAMP_DEG_PER_S2 * dt;
    descentRef_ += std::clamp(want - descentRef_, -maxStep, maxStep);
    lastVRef_ = -descentRef_;  // signed: down is negative
    const double mv = runVelocityLoop(lastVRef_, dt, config::LIFT_VEL_MAX_MV, config::LIFT_VEL_MAX_MV);
    trace(now, returnFast_ ? 'R' : 'V', position, mv, true);
}

// ---------------------------------------------------------------------------
// The one control loop
// ---------------------------------------------------------------------------
void Lift::update() {
    const std::uint32_t now = pros::millis();
    double dt = lastUpdate_ == 0 ? 0.01 : (now - lastUpdate_) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastUpdate_ = now;
    const double position = getPosition();

    // Velocity, every tick in every mode, so it is already warm when a move starts: differenced
    // position, with LIFT_VEL_FILTER of each new sample blended in.
    if (!havePrevPosition_) {
        prevPosition_ = position;
        havePrevPosition_ = true;
    }
    velocity_ += config::LIFT_VEL_FILTER * ((position - prevPosition_) / dt - velocity_);
    prevPosition_ = position;

    // 1) Manual voltage (D-pad Up/Down at full send, or R1 at R_LIFT_UP_PCT while a pin is clamped).
    //    This is the ONLY thing allowed to preempt everything else, and it always wins: cancel
    //    whatever target was active so nothing "fights" the driver (including an R2 descent), cancel
    //    homing (and don't retry it this run), and make sure release doesn't jump back to an old
    //    target either (see class comment).
    if (std::fabs(manualPower_) > 0.02) {
        wakeMotors();  // the driver is taking over: plain voltage, and brake-hold when they let go
        if (active_) cancel();
        if (home_ == Home::RUNNING) {
            home_ = Home::CANCELLED;
            queuedTarget_ = false;
        }
        wasManual_ = true;
        const double mv = manualPower_ * config::LIFT_MAX_MV;
        motors_.move_voltage(static_cast<int>(mv));
        lastError_ = 0.0;
        lastOutput_ = mv;
        lastVRef_ = 0.0;
        trace(now, 'M', position, mv, true);
        return;
    }
    if (wasManual_) {
        wasManual_ = false;
        faulted_ = false;  // driver took manual control; give the next command a clean slate
    }

    // 2) R2 (clamped mode) let go. Part-way down: brake-hold right here, no old target resumes --
    //    the same rule as releasing the D-pad. Already SEATED: leave the DOWN target in SEAT so the
    //    arm stays pressed onto its stop, which is what makes "hold R2 to the bottom" end exactly
    //    like an L2 score.
    if (descendOwned_ && !descendHeld_) {
        descendOwned_ = false;
        if (phase_ != Phase::SEAT && phase_ != Phase::REST) cancel();
    }

    // 3) Homing: find the hard stop and zero there. Runs as a mode in this same task, so it is still
    //    this one function writing the motors, and nothing anywhere blocks on it. The stall and
    //    runaway guards are not reached while homing -- deliberately: homing's whole job is to drive
    //    the arm into a hard stop and notice that it stopped, which is exactly what those guards
    //    exist to prevent in normal operation. updateHoming() has its own timeout instead.
    if (home_ == Home::RUNNING) {
        wakeMotors();
        updateHoming(now);
        trace(now, 'h', position, lastOutput_, true);
        return;
    }

    // 4) No commanded height: brake and hold right here. This is the state at power-on (nothing
    //    moves until something explicitly calls setTarget()/moveTo()) and right after the driver
    //    lets go of the manual override (no old target resumes).
    if (!active_) {
        wakeMotors();  // the no-target state is a BRAKE-hold, never a coast: an arm that is up must
                       // not fall (CLAUDE.md safety rule 1). Only REST at the bottom ever coasts.
        motors_.brake();
        lastError_ = 0.0;
        lastOutput_ = 0.0;
        lastVRef_ = 0.0;
        trace(now, 'I', position, 0.0, false);
        return;
    }

    if (faulted_) {
        wakeMotors();
        motors_.brake();
        lastOutput_ = 0.0;
        trace(now, 'F', position, 0.0, false);
        return;
    }

    // 5) The grab push (L1 at the handoff, auton::grab()): a constant downward force while the jaws
    //    close. Before the score/descent branches; the manual override above still preempts it.
    if (grab_) {
        updateGrab(now, dt, position);
        return;
    }

    // 6) An L2 score: press the pin down, sense the landing, release, post-push, then rise and hold.
    if (score_) {
        updateScore(now, dt, position);
        return;
    }

    // 7) A trip to the bottom (L2 press 2's run home, B, auton DOWN, R2): banded descent + seating.
    if (descent_) {
        updateDescent(now, dt, position);
        return;
    }

    // 8) A position move -- including L2 press 1's closing rise. Everything below is a pure function of where the arm is, where the goal is
    //    and how fast the arm is going THIS TICK -- which is the whole trick behind a fluid retarget:
    //    moving the goal mid-move is enough on its own, nothing has to be stopped and restarted.
    const double gap = goal_ - position;
    const double dist = std::fabs(gap);
    const double dir = gap >= 0.0 ? 1.0 : -1.0;
    lastError_ = gap;
    const double vMag = speedProfile(dist, dir);
    lastVRef_ = dir * vMag;

    // PARKED: a frozen constant voltage and no feedback at all. The only way out is a real drift away
    // from where it parked (and not before LIFT_PARK_COOLDOWN_MS, so PARK and HOLD cannot ping-pong),
    // or any new command from outside. See "Settling instead of hunting" in config.hpp.
    if (phase_ == Phase::PARK && updatePark(now, position)) return;

    if (phase_ != Phase::HOLD) {
        // A move that is getting nowhere: stop chasing it and PARK where it is rather than grinding on
        // (or hunting) for the rest of the match. Shown as PARK(t/o).
        if (now - moveStart_ >= static_cast<std::uint32_t>(config::LIFT_MOVE_TIMEOUT_MS)) {
            enterHold(now, position);   // seeds gravityMv_ from the velocity loop, then freeze it
            enterPark(now, position, true);
            motors_.move_voltage(static_cast<int>(parkMv_));
            lastOutput_ = parkMv_;
            trace(now, 'K', position, parkMv_, true);
            return;
        }

        // Arrival LATCHES into HOLD -- that is the only part of a move that is a one-way door.
        const bool arrived = dist <= arrivalTolerance() && std::fabs(velocity_) <= config::LIFT_STOPPED_DEG_PER_S;
        // The crawl can't go on forever (arm fighting something, loop badly tuned): hand over to the
        // hold, whose position term then pulls in whatever is left, gently.
        const bool timedOut = phase_ == Phase::APPROACH &&
                              now - phaseStart_ >= static_cast<std::uint32_t>(config::LIFT_APPROACH_TIMEOUT_MS);
        if (arrived || timedOut) {
            enterHold(now, goal_);
        } else {
            // Runaway guard: we are pushing toward the goal, so the gap can only shrink. If it GROWS
            // by LIFT_FAULT_DEG the sensor or the motors are reversed.
            minDist_ = std::min(minDist_, dist);
            if (dist - minDist_ > config::LIFT_FAULT_DEG) {
                faulted_ = true;
                motors_.brake();
                lastOutput_ = 0.0;
                trace(now, 'F', position, 0.0, true);
                return;
            }

            // WHICH PHASE, decided fresh every tick from (distance, speed) rather than latched:
            //   APPROACH  inside the approach zone, whatever the arm is doing.
            //   DRIVE     the arm is slower than the profile allows, so full voltage is the answer.
            //   BRAKE     the arm is at (or above) the profile, so the velocity loop tracks it down.
            // The only state here is the previous phase, used purely for hysteresis: DRIVE ends the
            // moment the arm reaches the profile, and is only re-entered once it has fallen
            // LIFT_DRIVE_RESUME_DEG_PER_S below it, so the two can't flicker at the boundary.
            //
            // This is what replaced a latching DRIVE -> BRAKE -> APPROACH state machine, and it is why
            // rapid L1 taps now merge: when Scoring moves the goal up a level mid-rise, the next tick
            // simply sees a bigger distance, a profile that allows more speed, and therefore DRIVE
            // again -- with the velocity loop's integral, the stall window and the approach clock all
            // untouched. The old code had to call enterDrive() for a retarget, which wiped velI_ and
            // reset the clocks, and its APPROACH phase had no way back out at all (it was only ever
            // entered, never left), so a retarget from the crawl depended on that restart to escape.
            const Phase was = phase_;
            const double vAlong = velocity_ * dir;  // + = already heading toward the goal
            // An arm travelling AWAY from the goal faster than the hysteresis band is a real
            // reversal (a goal set below a rising arm). Let the velocity loop turn it around under
            // the braking profile rather than slamming full voltage against 150 deg/s of momentum.
            const bool reversing = vAlong < -config::LIFT_DRIVE_RESUME_DEG_PER_S;
            if (dist <= config::LIFT_APPROACH_ZONE_DEG || reversing) {
                phase_ = dist <= config::LIFT_APPROACH_ZONE_DEG ? Phase::APPROACH : Phase::BRAKE;
            } else if (was == Phase::DRIVE) {
                phase_ = vAlong >= vMag ? Phase::BRAKE : Phase::DRIVE;
            } else {
                phase_ = vAlong < vMag - config::LIFT_DRIVE_RESUME_DEG_PER_S ? Phase::DRIVE : Phase::BRAKE;
            }
            if (phase_ != was) {
                phaseStart_ = now;
                stallStart_ = 0;  // a phase change is progress; don't carry a stale stall window in
                // DRIVE -> BRAKE is the one handover that needs the velocity loop primed.
                if (phase_ == Phase::BRAKE && was == Phase::DRIVE) enterBrake(now);
            }

            // The stall guard only measures ticks that really want motion (see config.hpp). Below
            // 5 deg/s of reference, slow progress is the plan, not a jam.
            const bool watchStall = phase_ == Phase::DRIVE || vMag >= 5.0;
            if (!watchStall) {
                stallStart_ = 0;
            } else if (stalled(now, position)) {
                blocked_ = true;              // can't get there: hold where it stopped, never push into
                enterHold(now, position);     // whatever it is caught on (cleared by the next target)
            }

            if (phase_ == Phase::DRIVE && !blocked_) {
                const double mv = dir * config::LIFT_MAX_MV;
                motors_.move_voltage(static_cast<int>(mv));
                lastOutput_ = mv;
                trace(now, 'D', position, mv, true);
                return;
            }
            if (!blocked_) {
                const double mv = runVelocityLoop(lastVRef_, dt, config::LIFT_VEL_MAX_MV, config::LIFT_VEL_MAX_MV);
                trace(now, phase_ == Phase::APPROACH ? 'A' : 'B', position, mv, true);
                return;
            }
        }
    }

    // HOLD: the damped position hold. Arrival latches here -- nothing re-drives, because the hold's
    // own position term is the correction and it is gentle by construction.
    runHold(now, dt, position);
}

// ---------------------------------------------------------------------------
// Trace: lift task pushes, trace task prints. Single producer / single consumer ring.
// ---------------------------------------------------------------------------
void Lift::trace(std::uint32_t now, char phase, double position, double mv, bool moving, const TraceAux& aux) {
    if (!config::LIFT_TRACE) return;
    if (moving) traceUntil_ = now + static_cast<std::uint32_t>(config::LIFT_TRACE_TAIL_MS);
    else if (now > traceUntil_) return;  // quiet: nothing has moved for LIFT_TRACE_TAIL_MS
    if (++traceTick_ < config::LIFT_TRACE_EVERY_TICKS) return;
    traceTick_ = 0;
    const std::uint32_t head = traceHead_.load(std::memory_order_relaxed);
    const std::uint32_t tail = traceTail_.load(std::memory_order_acquire);
    if (head - tail >= kTraceSize) return;  // consumer fell behind: drop this sample, never block
    TraceSample& s = trace_[head % kTraceSize];
    s.ms = now;
    s.phase = phase;
    s.pos10 = toI16(position * 10.0);
    s.goal10 = active_ ? toI16(goal_ * 10.0) : static_cast<std::int16_t>(-1);
    s.vel = toI16(velocity_);
    s.mv = toI16(mv);
    s.ratio = toI16(aux.ratioPct);
    s.lag10 = toI16(aux.lag10);
    s.eff = toI16(aux.effortMv);
    s.trg = toI16(aux.trigger);
    traceHead_.store(head + 1, std::memory_order_release);
}

void Lift::traceDrain() {
    std::uint32_t tail = traceTail_.load(std::memory_order_relaxed);
    const std::uint32_t head = traceHead_.load(std::memory_order_acquire);
    if (tail == head) return;
    while (tail != head) {
        const TraceSample& s = trace_[tail % kTraceSize];
        std::printf("LIFT %u %c %d %d %d %d %d %d %d %d\n", static_cast<unsigned>(s.ms), s.phase, s.pos10,
                    s.goal10, s.vel, s.mv, s.ratio, s.lag10, s.eff, s.trg);
        ++tail;
    }
    traceTail_.store(tail, std::memory_order_release);
    std::fflush(stdout);
}

void Lift::startTask() {
    if (task_) return;
    task_ = std::make_unique<pros::Task>([this] {
        while (true) {
            update();
            pros::delay(10);
        }
    });
    if (config::LIFT_TRACE && !traceTask_) {
        traceTask_ = std::make_unique<pros::Task>(
            [this] {
                std::printf("LIFT trace: ms phase(D=drive B=brake A=approach H=hold K=park E=rest(coast) "
                            "V=descend R=return Z=final S=seat P=press r=release p=postpush U=unstick "
                            "G=grab M=manual h=homing F=fault I=idle) "
                            "pos*10 goal*10 vel(deg/s) out(mV) | L2 only: v/base%% lag*10 effort(mV) "
                            "trg(0=- 1=stall 2=lag 3=effort 4=speed 5=height)\n");
                while (true) {
                    traceDrain();
                    pros::delay(20);
                }
            },
            TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "lift-trace");
    }
}

}  // namespace subsystems
