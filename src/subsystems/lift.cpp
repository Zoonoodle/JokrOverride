#include "subsystems/lift.hpp"

#include "config.hpp"
#include "pros/error.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

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

Lift::Gains& Lift::gains() {
    // Seeded ONCE from config, then owned here. The loops read this every tick, so writing it (the
    // LIFT_PID_TUNE route does) changes the arm's behaviour immediately and without a rebuild.
    // Function-local static so there is no static-init-order question with config's constants.
    static Gains g{config::LIFT_HOLD_KP, config::LIFT_HOLD_KI, config::LIFT_HOLD_KD,
                   config::LIFT_VEL_KP, config::LIFT_VEL_KI};
    return g;
}

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

void Lift::initState() {
    // THIS FUNCTION TOUCHES NO SENSOR AND COMMANDS NO MOTION. It only puts this class's own control
    // state in a known place before startTask() begins ticking. It used to be called zero(), and it
    // used to do two things that are now gone (the driver, 2026-10-07: "the rotation sensor on the
    // lift is an absolute positioning sensor, so don't reset the position ANYWHERE in the code ... We
    // need to be reading the all-time position with no code resetting the sensor's position"):
    //
    //  1. motors_.tare_position_all() -- the motor encoders are now left alone. They are only a
    //     fallback position source and a brain diagnostic (see getMotorPosition()).
    //  2. rotation_->set_position(foldCentideg(get_angle())) -- the ±180-sensor-degree "seed". THIS
    //     WAS THE BUG THE DRIVER IS DESCRIBING. It overwrote the sensor's accumulated multi-turn
    //     count at every program start with the single-turn angle folded into (-180, 180] sensor
    //     degrees, i.e. (-45, 45] ARM degrees -- so an arm sitting anywhere outside that window read
    //     a whole sensor revolution (90 arm degrees) out, and a lift left up could never report its
    //     real height. Nothing seeds, folds or offsets the reading now: getPosition() is the sensor's
    //     own all-time count, straight through.
    //
    // set_reversed() is still applied once in the constructor, so every read is consistently signed.
    //
    // NO SOFTWARE OFFSET ANY MORE. The old startup unwrap guessed which 90-degree window of the
    // sensor's single turn the arm booted in; it could not work, because get_position() is a RETAINED
    // multi-turn counter that only re-folds on a real sensor power cycle, so cold and warm starts
    // disagreed. HOMING defines zero now (see updateHoming()), and lift_preset::DOWN is 0.0.
    // The sensor read is still CHECKED here, purely so a dead sensor is caught before the task starts.
    sensorBad_ = false;
    lastGoodPosition_ = 0.0;
    if (rotation_) {
        // Bounded retry: a sensor still waking up answers PROS_ERR for a few ms.
        const std::uint32_t deadline = pros::millis() + config::LIFT_SENSOR_START_RETRY_MS;
        std::int32_t raw = rotation_->get_position();
        while (raw == PROS_ERR && pros::millis() < deadline) {
            pros::delay(5);
            raw = rotation_->get_position();
        }
        if (raw == PROS_ERR) sensorBad_ = true;  // update() brakes and the brain shows SENSOR BAD
        else lastGoodPosition_ = raw / 100.0 / config::LIFT_ROTATION_RATIO;
    }
    active_ = false;
    goal_ = 0.0;
    precise_ = descent_ = false;
    phase_ = Phase::HOLD;
    holdGoal_ = 0.0;
    velI_ = 0.0;
    gravityMv_ = 0.0;
    descendHeld_ = descendOwned_ = descendRequest_ = false;
    fastRequest_ = descentFast_ = toggleWait_ = false;
    coarse_ = coarseRequest_ = false;
    clampRise_ = clampRiseRequest_ = false;
    seatHot_ = false;
    seatStillSince_ = 0;
    finalSteadySince_ = 0;
    restWakes_ = 0;
    parkMv_ = 0.0;
    parkTimeout_ = false;
    holdSettleSince_ = 0;
    idleStillSince_ = 0;
    score_ = returnFast_ = scoreStuck_ = releasedByTravel_ = releasedByFloor_ = false;
    scoredAbove_ = scoreCycle_ = false;
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
    // Motor encoders read motor degrees; divide by the gear ratio to get arm degrees.
    //
    // NOT ABSOLUTE, AND NO LONGER TARED. initState() used to tare these at program start, which at
    // least made them read 0 at boot; that tare is gone with every other position reset, so this is
    // now a bare accumulated count whose datum is wherever the encoders happened to be. It has NO
    // defined relationship to the arm's real height, so the presets are meaningless against it.
    //
    // It is only reached when NO rotation sensor is configured (config::LIFT_ROTATION_PORT == 0, a
    // build-time choice -- the robot has one on port 16), and it is otherwise read purely for the
    // brain's ROT/MOT cross-check. Kept as a last resort so a sensorless build still compiles and the
    // D-pad still works, but a sensorless build CANNOT run presets correctly and never could.
    return motors_.get_position() / config::LIFT_GEAR_RATIO;
}

double Lift::getSensorPosition() const {
    if (!rotation_) return 0.0;
    const std::int32_t raw = rotation_->get_position();
    // THE SENSOR STOPPED ANSWERING. PROS_ERR is INT32_MAX, so dividing it through would report about
    // 5.4 million arm degrees and every loop would chase it. Hold the last good reading instead and
    // latch sensorBad_; update() turns that into the lift's ordinary FAULT, which brakes and stops.
    if (raw == PROS_ERR) {
        sensorBad_ = true;
        return lastGoodPosition_;
    }
    // Arm degrees, straight from the counter. The frame comes from HOMING's zero at the hard stop,
    // not from any software offset here.
    lastGoodPosition_ = raw / 100.0 / config::LIFT_ROTATION_RATIO;
    return lastGoodPosition_;
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
    // Homing is switched OFF on this robot (config::LIFT_HOME_ON_ENABLE, the driver's decision
    // 2026-10-03: the arm is set at its hard stop every time, so zero()'s absolute-angle seed is
    // already right and a route should be able to start lifting immediately). Gated HERE, in the one
    // place every caller goes through, so autonomous() and opcontrol() are both covered and neither
    // can move the arm on enable. The brain says HOME SKIPPED.
    if (!config::LIFT_HOME_ON_ENABLE) {
        home_ = Home::SKIPPED;
        return;
    }
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

    // Timeout: never reached the stop. Stop pushing and report HOME FAIL -- NO zero is written, so the
    // lift stays in the raw sensor frame and every preset will land low. The brain's LIFT0 line says
    // "frame RAW" when this happens.
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

    // HOMING ZEROES THE SENSOR HERE, and this is the ONLY place in the program that writes it (the
    // driver, 2026-10-07: "go back to how we reset our position to 0 by moving lift down at the
    // start"). The arm has just stopped moving while being driven down, so it is on its hard stop:
    // that IS zero, and every lift_preset height is measured up from it.
    //
    // There is no absolute-angle plausibility gate any more. It used to refuse to zero unless the
    // sensor's own calibrated zero was nearby, but the sensor's zero no longer has any fixed
    // relationship to the stop -- that is exactly why the software-offset approach was abandoned -- so
    // the gate could only ever reject good homes. The mechanical stop is the ground truth now. The
    // stall detector above is what distinguishes "reached the stop" (DONE) from "never stopped"
    // (FAILED, on the timeout), and only DONE zeroes.
    rotation_->set_position(0);
    home_ = Home::DONE;
    havePrevPosition_ = false;  // the position just jumped to 0 by decree; not a velocity
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
    // ...and whether it is the FAST descent (the front toggle's way down).
    const bool fromFast = fastRequest_;
    fastRequest_ = false;
    // ...and whether it is a COARSE move (fast, overshoot allowed -- the front toggle rise).
    const bool fromCoarse = coarseRequest_;
    coarseRequest_ = false;
    // ...and whether it is the CLAMP_RISE_DEG hop (brain: CLAMP UP). Consumed either way, so an ignored
    // or queued request can never label a later, unrelated move.
    const bool fromClampRise = clampRiseRequest_;
    clampRiseRequest_ = false;
    // Any deliberate new command cancels a fast descent that was queued behind a toggle rise.
    toggleWait_ = false;
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
    // Any new height leaves the two-press score cycle, press 2's own run home included -- see
    // startScoreReturn(), which now ends the cycle itself so the autoclamp is live for the whole descent
    // rather than only once the arm has seated. (There used to be a returnRequest_ one-shot here whose
    // only job was to keep the flag alive through press 2; it is gone with that behaviour.)
    scoredAbove_ = false;
    scoreCycle_ = false;

    // "Flying" = really in the middle of a position move, so moving the goal alone is enough (this is
    // what merges rapid L1 taps). Settled states -- HOLD, PARK, REST, SEAT -- and every special mode
    // must start a fresh move instead, or a retarget from a parked arm would never leave PARK.
    const bool flying = active_ && !descent_ && !faulted_ && !blocked_ &&
                        (phase_ == Phase::DRIVE || phase_ == Phase::BRAKE || phase_ == Phase::APPROACH);
    const bool newIsDescent = !wantPrecise && clamped <= config::LIFT_BOTTOM_GOAL_DEG;
    if (flying && wantPrecise == precise_ && !newIsDescent) {
        clampRise_ = fromClampRise;
        goal_ = clamped;
        minDist_ = std::fabs(goal_ - getPosition());
        return;
    }

    speedCapPct_ = 0.0;  // an ordinary move is never speed-capped; setTargetSlow() re-applies it after
    active_ = true;
    descendOwned_ = fromR2;
    descentFast_ = fromFast;
    coarse_ = fromCoarse;
    clampRise_ = fromClampRise;
    // A deliberate new command clears a latched runaway fault or a stall: the guards' job is to stop
    // the motion that was happening, not to brick every button until someone wiggles the D-pad. A real
    // wiring fault or jam simply re-trips on the next DRIVE, so nothing is lost.
    faulted_ = false;
    blocked_ = false;
    goal_ = clamped;
    precise_ = wantPrecise;
    // A non-precise goal at the bottom is not a position move at all: it is the banded descent that
    // ends SEATED on the hard stop (L2, B, auton DOWN, R2). Since 2026-10-07 the bottom is -31.9 and
    // LIFT_BOTTOM_GOAL_DEG is -26.9, so INTAKE (-3.3) is comfortably ABOVE the threshold and is an
    // ordinary precise move -- it no longer relies on the precise_ flag to escape being a descent.
    descent_ = !precise_ && goal_ <= config::LIFT_BOTTOM_GOAL_DEG;
    const std::uint32_t now = pros::millis();
    const double position = getPosition();
    if (descent_) {
        // R2's descent has its own, gentler top band; L2/B/auton use the config bands.
        // R2's descent has its own gentler top band; the front toggle's has its own FASTER ones;
        // everything else uses the config bands.
        const double top = descendOwned_  ? config::R_LIFT_DOWN_PCT
                           : descentFast_ ? config::LIFT_FAST_DESCENT_TOP_PCT
                                          : config::LIFT_DESCENT_TOP_PCT;
        const double mid = descendOwned_  ? config::R_LIFT_DOWN_PCT
                           : descentFast_ ? config::LIFT_FAST_DESCENT_MID_PCT
                                          : config::LIFT_DESCENT_MID_PCT;
        enterDescend(now, position, top, mid);
        return;
    }
    // Already inside this goal's band: arrived, no motion at all (so a level re-tap never twitches).
    if (std::fabs(goal_ - position) <= arrivalTolerance()) enterHold(now, goal_);
    else enterDrive(now, position);
}

void Lift::setTargetFast(double degrees) {
    coarseRequest_ = true;
    setTarget(degrees);
}

void Lift::startClampRise(double degrees) {
    // PRECISE, and not negotiable: see the header and "THE CLAMP-RISE" in config.hpp -- a 5 deg goal is
    // inside LIFT_TOLERANCE_DEG, so a non-precise move would latch "arrived" without moving at all.
    clampRiseRequest_ = true;
    setTarget(degrees, true);
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
    wakeMotors();
    toggleWait_ = false;
    clampRise_ = clampRiseRequest_ = false;
    fastRequest_ = false;  // never leave the motors coasting once nothing is commanding the lift
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
}

const char* Lift::activePresetName() const {
    if (!active_) return "none";
    namespace preset = config::lift_preset;
    if (goal_ == preset::DOWN) return "DOWN";
    if (goal_ == preset::INTAKE) return "INTAKE";
    if (goal_ == preset::MATCHLOAD) return "MATCHLOAD";
    if (goal_ == preset::FRONT_TOGGLE) return "FRONT TOGGLE";
    if (goal_ == preset::FIRST_SCORE) return "FIRST SCORE";
    if (goal_ == preset::MAX) return "MAX";
    static const char* const kLevelNames[preset::STACK_LEVEL_COUNT] = {"STACK 1", "STACK 2", "STACK 3", "STACK 4",
                                                                        "STACK 5"};
    for (int i = 0; i < preset::STACK_LEVEL_COUNT; ++i)
        if (goal_ == preset::STACK_LEVELS[i]) return kLevelNames[i];
    return "custom";
}

const char* Lift::phaseName() const {
    if (!active_) return "";
    // A fast descent queued behind a toggle rise: the arm is still going UP, on its way to touching
    // the toggle height, and will turn around the moment it gets there.
    if (toggleWait_) return "TOGGLE ^";
    // The CLAMP_RISE_DEG hop: short enough that DRIVE/BRAKE/APPROACH/HOLD would all flick past before
    // they could be read, so say what the move IS for as long as it is running. PARK (and anything that
    // follows) reports normally -- by then the hop is over and the arm is just settled at a height.
    // The word carries no number on purpose: CLAMP_RISE_DEG is a TUNE constant (5 -> 8 already), and a
    // baked-in "+5" would quietly start lying. The goal is printed next to it on the same line anyway.
    if (clampRise_ && (phase_ == Phase::DRIVE || phase_ == Phase::BRAKE || phase_ == Phase::APPROACH ||
                       phase_ == Phase::HOLD)) {
        return "CLAMP UP";
    }
    switch (phase_) {
        case Phase::DRIVE: return "DRIVE";
        case Phase::BRAKE: return "BRAKE";
        case Phase::APPROACH: return "APPROACH";
        case Phase::HOLD:
            // (No longer reachable: the one-press score runs straight to the bottom. Kept for the
            // stack with the jaws open, waiting for a second L2 to run home and seat.
            return scoredAbove_ ? "SCORED UP" : "HOLD";
        case Phase::PARK:
            if (scoredAbove_) return "SCORED UP";
            return parkTimeout_ ? "PARK(t/o)" : "PARK";
        case Phase::REST: return "REST";
        case Phase::DESCEND:
            if (returnFast_) return "RETURN";
            return descentFast_ ? "DESC FAST" : "DESCEND";
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
    }
    return "";
}

void Lift::driverPower(double input) { manualPower_ = std::clamp(input, -1.0, 1.0); }

void Lift::descendFast() {
    // QUEUE BEHIND A TOGGLE RISE. The route asks for the way down barely 300 ms after the way up,
    // which is less than a 0 -> 30 deg rise takes, so obeying immediately would turn the arm around
    // short of the toggle height and the toggle would never flip. Wait for the arm to TOUCH the
    // height instead -- not to settle on it -- and then go straight down.
    const bool togglingUp = active_ && !descent_ && !score_ && !grab_ && !blocked_ && !faulted_ &&
                            goal_ == config::lift_preset::FRONT_TOGGLE &&
                            getPosition() < goal_ - config::LIFT_FRONT_TOGGLE_REACH_DEG;
    if (togglingUp) {
        toggleWait_ = true;
        toggleWaitStart_ = pros::millis();
        return;
    }
    fastRequest_ = true;
    setTarget(config::lift_preset::DOWN);
}

void Lift::setTargetSlow(double degrees, double maxPct) {
    // A PRECISE move with a hard speed ceiling. setTarget() clears speedCapPct_, so it is applied
    // after; if setTarget() decided this was the same move already running and returned early, the cap
    // is simply re-asserted, which is what a re-command of the same slow goal should do.
    setTarget(degrees, true);
    speedCapPct_ = maxPct;
}

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
    if (precise_) return config::LIFT_PRECISE_TOLERANCE_DEG;
    return coarse_ ? config::LIFT_COARSE_TOLERANCE_DEG : config::LIFT_TOLERANCE_DEG;
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
    // A COARSE move brakes later and keeps a quick tail instead of the crawl: it is allowed to arrive
    // still moving and sail past (see "A COARSE move" in config.hpp).
    const double decel = coarse_ ? config::LIFT_COARSE_DECEL_DEG_PER_S2 : config::LIFT_DECEL_DEG_PER_S2;
    const double tailMin =
        coarse_ ? config::LIFT_COARSE_MIN_DEG_PER_S : config::LIFT_APPROACH_MIN_DEG_PER_S;
    double cap = config::LIFT_FREE_SPEED_DEG_PER_S *
                 (dir >= 0.0 ? config::LIFT_UP_SPEED_PCT : config::LIFT_DOWN_SPEED_PCT) / 100.0;
    // A PER-MOVE CEILING, for the moves that have to be deliberately slow -- the intake descent to
    // 70.5 (see setTargetSlow() and INTAKE_DESCEND_SPEED_PCT). 0 means "no extra cap". It only ever
    // lowers the limit, so the braking curve and the final walk-in below are untouched.
    if (speedCapPct_ > 0.0) {
        cap = std::min(cap, config::LIFT_FREE_SPEED_DEG_PER_S * speedCapPct_ / 100.0);
    }
    const double curve =
        std::sqrt(2.0 * decel * std::max(dist - config::LIFT_APPROACH_ZONE_DEG, 0.0));
    const double close = std::clamp(config::LIFT_APPROACH_KP_PER_S * dist, tailMin,
                                    std::max(config::LIFT_APPROACH_DEG_PER_S, tailMin));
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
    // ...but ONLY if the velocity loop actually ran. A move that arrived straight out of DRIVE never
    // built an integral (enterDrive() zeroes it), so velI_ is 0 and this seed would be
    // -kv * arrival speed -- a big DOWNWARD voltage, which PARK would then freeze and drive the arm
    // into the floor with. That is reachable from any short or COARSE move. In that case keep whatever
    // the hold learned last time and let its integral correct from there.
    if (velI_ != 0.0) {
        const double kv = config::LIFT_MAX_MV / config::LIFT_FREE_SPEED_DEG_PER_S;  // mV per deg/s
        gravityMv_ = std::clamp(velI_ - kv * velocity_, -static_cast<double>(config::LIFT_HOLD_MAX_MV),
                                static_cast<double>(config::LIFT_HOLD_MAX_MV));
    }
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
    finalSteadySince_ = 0;  // "has it been at 0 and still?" starts fresh for this attempt
    phaseStart_ = now;  // FINAL's own clock -- LIFT_FINAL_FORCE_MS minimum, LIFT_FINAL_MAX_MS cap
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
    // FREEZE WHAT WAS ACTUALLY HOLDING THE ARM -- the hold loop's last output -- not just its
    // integral. By now the arm is inside the band and slow, so that output IS the steady voltage this
    // height costs; the integral alone can be stale (nothing learned yet) or carry an arrival
    // transient. Getting this wrong makes the arm sag and the hold re-engage, over and over.
    parkMv_ = std::clamp(lastOutput_, -static_cast<double>(config::LIFT_HOLD_MAX_MV),
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
    double out = gains().velKp * err + velI_;
    // The limits are per-call, and ASYMMETRIC, because L2's press needs a much tighter cap on how hard
    // it may push DOWN (L2_PRESS_MAX_MV, the crush limit -- the integral winds up against whatever is
    // resisting, so without it the press would keep raising the push until something moved, i.e.
    // flatten the stack) while still needing full UPWARD authority: holding a loaded six-bar back to
    // only 90 deg/s at a high angle can easily cost more than the press cap, and capping that side too
    // would let the arm outrun its own commanded descent and land hard.
    if (out > upMaxMv) out = upMaxMv;
    else if (out < -downMaxMv) out = -downMaxMv;
    else velI_ = std::clamp(velI_ + gains().velKi * err * dt, -downMaxMv, upMaxMv);  // unsaturated only
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
    if (err > band) p = gains().holdKp * (err - band);
    else if (err < -band) p = gains().holdKp * (err + band);
    double out = p - gains().holdKd * velocity_ + gravityMv_;
    const double sat = config::LIFT_HOLD_MAX_MV;
    if (out > sat) out = sat;
    else if (out < -sat) out = -sat;
    // LEARN GRAVITY, inside the band as well as outside it. Freezing the integral in the band (which
    // this used to do) looked like the cure for the limit cycle, but it was the cause of a worse bug:
    // a move that arrives already inside the band never learns anything, so PARK froze a voltage that
    // had never been measured -- often 0 after a REST, or a large NEGATIVE one after a fast arrival --
    // and the arm sagged, re-engaged at LIFT_REENGAGE_DEG, got pushed back, parked wrong again... which
    // is the "moving up and down randomly" the driver saw when driver control took over. The integral
    // is what makes parkMv_ right, and it cannot wind for long: PARK takes the loop away within
    // LIFT_SETTLE_TIMEOUT_MS. The P term staying silent in the band is what stops the hunting.
    else gravityMv_ = std::clamp(gravityMv_ + gains().holdKi * err * dt, -sat, sat);
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
// L2 scoring, ONE PRESS: PRESS -> RELEASE -> (UNSTICK) -> carry on down to DOWN and SEAT
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
void Lift::startGrab() { startPush(config::L1_GRAB_PUSH_MS, true); }

void Lift::startClampPush(int pushMs) { startPush(pushMs, false); }

void Lift::startPush(int pushMs, bool needsGrip) {
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
    grabNeedsGrip_ = needsGrip;  // false for the autoclamp's handoff push: the jaws are already shut
    grabPushMs_ = pushMs;
    gripWanted_ = false;
    gripAcked_ = false;
    grabStart_ = now;
    velI_ = 0.0;
    clampRise_ = false;  // a push is not a hop; don't leave CLAMP UP on the brain
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
    // pressed down onto it rather than above it. The autoclamp's handoff push skips this entirely
    // (grabNeedsGrip_ == false): the jaws were already shut by the autoclamp before the push started,
    // so there is nothing to request and nothing to wait on.
    if (grabNeedsGrip_ && !gripWanted_ && t >= static_cast<std::uint32_t>(config::L1_GRAB_CLOSE_AT_MS)) {
        gripWanted_ = true;
    }

    // Stop pushing only when the push has had its time AND the jaws have had theirs. The arm must
    // never pull up out of a claw that is still physically closing -- the same interlock CLAMPING
    // enforces. L1_GRAB_MAX_MS is the safety net for "nobody ever acknowledged" (no Scoring tick).
    const bool pushedLongEnough = t >= static_cast<std::uint32_t>(grabPushMs_);
    const bool jawsShut = !grabNeedsGrip_ ||
                          (gripAcked_ &&
                           now - gripAckedAt_ >= static_cast<std::uint32_t>(config::CLAW_CLOSE_TIME_MS));
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
    // ONE PRESS, NOT TWO (driver, 2026-10-07): "we no longer need to go up, our claw won't knock over
    // the tower ... Once the claw is released, keep the same speed down and have it go all the way down
    // to the bottom position." So the pin is off, and instead of rising L2_CLEAR_RISE_DEG and latching
    // SCORED to wait for a second L2, the arm simply KEEPS GOING DOWN -- at the same
    // L2_SCORE_SPEED_PCT the press was using -- all the way to DOWN, where FINAL and SEAT end it
    // exactly as any other trip to the bottom does. There is no clear rise and no post-release push.
    score_ = false;
    clawReleaseWanted_ = false;
    returnFast_ = false;
    scoredAbove_ = false;  // nothing is held above a stack any more
    descendRequest_ = false;
    blocked_ = false;
    setTarget(config::lift_preset::DOWN);
    // KEEP THE AUTOCLAMP INHIBITED ALL THE WAY DOWN. setTarget() clears scoreCycle_, which would hand
    // the autoclamp back the instant the descent starts -- with the jaws open, travelling down right
    // past the pin that was just placed, which it would then re-grab. Press 2 used to be the thing
    // that gave it back, and there is no press 2 any more, so the inhibit is re-asserted here and
    // enterSeat() ends it at the bottom.
    scoreCycle_ = true;
    // setTarget() picked the config descent bands; keep the SCORING speed for both fast bands instead,
    // so the run to the bottom is the continuation of the press and not a separate, faster descent.
    // The slow zone, FINAL and SEAT are untouched.
    if (descent_) enterDescend(now, position, config::L2_SCORE_SPEED_PCT, config::L2_SCORE_SPEED_PCT);
}

void Lift::startScoreReturn() {
    const std::uint32_t now = pros::millis();
    const double position = getPosition();
    // THE AUTOCLAMP COMES BACK ON THE INSTANT PRESS 2 STARTS -- it does NOT wait for the arm to seat.
    // The driver, 2026-10-03: "from the moment the second l2, (so when it starts to move down back) auto
    // clamp should be back on, should not wait for that at all, but the settle and movement still
    // happens." So scoreCycle_ (what periodic::tick() turns into the autoclamp inhibit) ends here rather
    // than in enterSeat(), and Claw::update()'s inhibit-falling-edge re-arm fires on this tick.
    //
    // WHAT THIS GIVES UP, deliberately. The cycle used to be held through press 2 so the pin just placed
    // could not be re-grabbed as the arm descended past it. That guard is now the driver's job, and the
    // two-press design is exactly what makes that reasonable: press 1 ends with the arm held clear ABOVE
    // the stack precisely so the robot can drive away, and press 2 is pressed after it has. If press 2 is
    // ever used while still standing over the stack, the pin can be re-grabbed on the way down.
    // The lift cannot be FOUGHT by that, which is the part that would actually have been dangerous: a
    // clamp during the descent raises the CLAMP_RISE_DEG hop request, and Scoring::handleClampEvent()
    // refuses it while a lift move is in flight (hasActiveTarget() && !isSettled()), which a descent is.
    scoreCycle_ = false;
    descendRequest_ = false;
    returnFast_ = true;     // brain/trace: RETURN
    // ...and ask for the FAST descent's RAMP. At L2_RETURN_SPEED_PCT = 110 (~165 deg/s) the normal 1200
    // deg/s^2 ramp cannot shed enough speed inside the 14 -> 3 deg approach zone -- it needs 10.5 deg of
    // the 11 available just to reach the 45 deg/s cruise, leaving nothing in which to slow to FINAL's
    // handover, so the arm would arrive at the floor still carrying real speed. The fast ramp (2000)
    // does it in 6.3 deg. The bands themselves are overridden below; this one-shot is only here for the
    // ramp, which enterDescend() does not take as an argument. See L2_RETURN_SPEED_PCT in config.hpp.
    fastRequest_ = true;
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
    } else {  // Phase::RELEASE
        // Carry the detector's last readings so the r lines show what fired and what it saw.
        aux.ratioPct = static_cast<int>(contact_.speedRatio() * 100.0);
        aux.lag10 = static_cast<int>(contact_.lagDeg() * 10.0);
        aux.effortMv = static_cast<int>(contact_.effortMv());
        aux.trigger = releasedByTravel_ ? 5 : (releasedByFloor_ ? 6 : ContactDetector::triggerCode(contactTrigger_));


        // Don't judge anything until the jaws have actually had time to open. If nobody ever
        // acknowledges the request (no Scoring::update() running, e.g. a bare auton), carry on after
        // L2_RELEASE_ACK_MS rather than pressing down forever.
        if (!clawReleased_ && now - phaseStart_ >= static_cast<std::uint32_t>(config::L2_RELEASE_ACK_MS)) {
            clawReleased_ = true;
            clawReleasedAt_ = now;
        }
        const bool jawsOpen = clawReleased_ &&
                              now - clawReleasedAt_ >= static_cast<std::uint32_t>(config::CLAW_OPEN_TIME_MS);
        // THE 200 ms PAUSE BEFORE ANY DESCENT (L2_RELEASE_SETTLE_MS). Measured from when the jaws were
        // told to open, so it overlaps CLAW_OPEN_TIME_MS instead of stacking on top of it. BOTH ways
        // out of this phase require it, so the run to the bottom can never start with the jaws still
        // closed on the pin.
        const bool releaseSettled =
            clawReleased_ &&
            now - clawReleasedAt_ >= static_cast<std::uint32_t>(config::L2_RELEASE_SETTLE_MS);
        // With the jaws open the arm either moves or it does not -- nothing subtle to detect, so this
        // is a plain speed test rather than the full detector.
        if (descendSpeed >= config::L2_CONTACT_STALL_DEG_PER_S) {
            if (freeSince_ == 0) freeSince_ = now;
            stuckSince_ = 0;
        } else {
            freeSince_ = 0;
            if (stuckSince_ == 0) stuckSince_ = now;
        }
        // ONE EXIT, AND IT ALWAYS GOES DOWN. The release has settled, so carry straight on to the
        // bottom at the scoring speed -- which is the whole of what a one-press score does.
        //
        // THIS REPLACES A FREED-vs-STUCK ARBITRATION THAT WAS THE BUG. That code asked "is the claw
        // hooked on the pin?" by watching whether the arm was still moving, and it only made sense
        // back when the next move was a RISE, where a hooked claw would have dragged the pin up with
        // it. But the arm has just pressed a pin DOWN onto a stack, so it is STOPPED at the moment of
        // release: freeSince_ stayed 0, stuckSince_ started at the first RELEASE tick, and at
        // L2_STUCK_MS (250 ms) the "stuck" branch won every time -- two L2_UNSTICK_UP_DEG hops upward
        // and then enterHold(), holding the arm at the release height and never descending. That is
        // exactly "after it releases cup/pin, it stays in that position", in driver and in auton,
        // because both run this same code.
        //
        // The question is moot now: the arm continues in the direction it was ALREADY travelling, so a
        // hooked claw cannot drag anything up, and if it is still touching the pin it slides off on the
        // way down -- which is what the old post-release push did anyway. Phase::UNSTICK and
        // scoreStuck_ are no longer reachable from here; they are left in place as inert safety net
        // rather than ripped out of the enum, phase names and trace in a bug fix.
        if (releaseSettled) {
            endScoreClear(now, position);
            return;
        }
    }

    // Pressing down, with the push capped so the loop cannot crush the stack while the arm slows.
    const double mv = runVelocityLoop(lastVRef_, dt, config::LIFT_VEL_MAX_MV, config::L2_PRESS_MAX_MV);
    trace(now, phase_ == Phase::PRESS ? 'P' : 'r', position, mv,
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
    } else if (phase_ == Phase::FINAL) {
        // FORCE DOWN UNTIL THE ARM IS REALLY AT 0 -- the driver's 2026-10-03 request for the auton reset:
        // "needs to constant apply down so its in that position until rotation reads steady around 0".
        //
        // LIFT_FINAL_FORCE_MS is a MINIMUM, not the window. It cannot seat before that has elapsed no
        // matter how settled the arm looks (a friction stall a few degrees short looks perfectly still,
        // which is the whole reason this phase stopped trusting stillness in the first place). After it,
        // the phase keeps forcing until the position really is within LIFT_SEAT_ZERO_DEG of 0 AND has
        // been steady there for LIFT_FINAL_STEADY_MS -- or until LIFT_FINAL_MAX_MS, the backstop, which
        // hands over as SEAT(short) so the driver can see it never got there.
        //
        // WHY THIS IS NOT THE DESIGN CLAUDE.md BANS. That one was CONDITIONAL IN THE OTHER DIRECTION: it
        // could exit EARLY the moment the arm looked "close and still", so a stall short of the stop was
        // declared SEAT and only the weak LIFT_SEAT_MV was ever applied again -- the arm parked above the
        // frame. This version can only ever exit LATE. A stall short of 0 no longer ends the phase at
        // all; it keeps getting LIFT_FINAL_MAX_MV of push until it either moves or the cap gives up on it.
        const bool forcedLongEnough =
            now - phaseStart_ >= static_cast<std::uint32_t>(config::LIFT_FINAL_FORCE_MS);
        // MEASURED FROM THE HARD STOP, not from a bare 0 (2026-10-07): the bottom is
        // lift_preset::DOWN (-31.9) and LIFT_SEAT_ZERO_DEG is the WINDOW around it. Comparing
        // fabs(position) against the window here is what made the half-finished version of this change
        // dangerous -- it declared SEAT at position 0, ~32 deg above the real stop.
        const bool atZero =
            std::fabs(position - config::lift_preset::DOWN) <= config::LIFT_SEAT_ZERO_DEG;
        if (atZero && std::fabs(velocity_) <= config::LIFT_FINAL_STEADY_DEG_PER_S) {
            if (finalSteadySince_ == 0) finalSteadySince_ = now;
        } else {
            finalSteadySince_ = 0;  // moved, or drifted back out of the band: the window restarts
        }
        const bool steadyAtZero =
            finalSteadySince_ != 0 &&
            now - finalSteadySince_ >= static_cast<std::uint32_t>(config::LIFT_FINAL_STEADY_MS);
        const bool capped = now - phaseStart_ >= static_cast<std::uint32_t>(config::LIFT_FINAL_MAX_MS);
        if (capped || (forcedLongEnough && steadyAtZero)) enterSeat(now, !atZero);
    }

    // --- output ---
    if (phase_ == Phase::SEAT) {
        // CONSTANT DOWNWARD PRESSURE, FOREVER, AND AT THE HOMING VOLTAGE. The arm is not holding a
        // position around 0 -- it is actively trying to go below it, which is what the driver asked
        // for at the bottom and at matchload (they are the same height). It never eases off: with
        // LIFT_REST_AFTER_MS at 0 this phase no longer hands over to REST's coast, so the push lasts
        // as long as the target stays DOWN. Any new target leaves SEAT and ends it on that tick.
        // Temperature is the one thing that can stop it (checked every 50th tick, i.e. twice a
        // second); see the heat note on LIFT_DOWN_PUSH_MV in config.hpp.
        if (++seatTempTick_ >= 50) {
            seatTempTick_ = 0;
            seatHot_ = getTemperature() >= config::LIFT_SEAT_TEMP_LIMIT_C;
        }
        if (seatHot_) {
            motors_.brake();
            lastOutput_ = 0.0;
        } else {
            motors_.move_voltage(static_cast<int>(config::LIFT_DOWN_PUSH_MV));
            lastOutput_ = config::LIFT_DOWN_PUSH_MV;
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
    // MEASURED arm speed, so gravity plus the pin's weight cannot make the descent run away. The fast
    // descent ramps harder (LIFT_FAST_DESCENT_RAMP_DEG_PER_S2) because it has much more speed to shed
    // before the approach zone -- see the sizing arithmetic in config.hpp. FINAL deliberately keeps
    // the normal ramp: it is unchanged in every mode.
    const double want = descentSpeed(position);
    const double maxStep = (descentFast_ ? config::LIFT_FAST_DESCENT_RAMP_DEG_PER_S2
                                         : config::LIFT_DESCENT_RAMP_DEG_PER_S2) * dt;
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

    // THE SENSOR IS NOT ANSWERING. getSensorPosition() has latched sensorBad_ and is holding the last
    // good reading, so `position` is stale, not garbage -- but a stale position must not drive a loop.
    // Turn it into the lift's ordinary FAULT: brake, drop the output, and stop. isFaulted() already
    // makes every macro let go and the brain already shows FAULT (plus SENSOR BAD, see screen.cpp).
    if (sensorBad_) {
        if (active_) cancel();
        faulted_ = true;
        motors_.brake();
        lastOutput_ = 0.0;
        lastVRef_ = 0.0;
        trace(now, 'F', position, 0.0, true);
        return;
    }

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
        // AT THE BOTTOM, WITH NOTHING COMMANDED: coast. The arm is resting on its hard stop, so the
        // motors have nothing to do and should not be asked to hold anything -- the driver's words,
        // "during down state like scored state, it needs to not use arm pid and let the motors just
        // coast and chill". This is the same REST the descent ends in, reached from the idle path.
        // ANYWHERE ELSE it is still a BRAKE-hold: an arm that is up must not fall when a target is
        // dropped (CLAUDE.md safety rule 1), and letting go of the manual override still brake-holds.
        if (position <= config::LIFT_BOTTOM_GOAL_DEG &&
            std::fabs(velocity_) <= config::LIFT_REST_STILL_DEG_PER_S && config::LIFT_REST_AFTER_MS > 0) {
            if (idleStillSince_ == 0) idleStillSince_ = now;
            if (now - idleStillSince_ >= static_cast<std::uint32_t>(config::LIFT_REST_AFTER_MS)) {
                if (!coasting_) {
                    motors_.set_brake_mode_all(pros::v5::MotorBrake::coast);
                    coasting_ = true;
                }
                motors_.move_voltage(0);
                lastError_ = 0.0;
                lastOutput_ = 0.0;
                lastVRef_ = 0.0;
                trace(now, 'E', position, 0.0, false);
                return;
            }
        } else {
            idleStillSince_ = 0;
        }
        wakeMotors();
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

    // 4b) A fast descent queued behind a FRONT_TOGGLE rise: start it the moment the arm TOUCHES the
    //     height (not when it settles there), or if it has run out of time / given up trying.
    if (toggleWait_) {
        const bool reached = position >= goal_ - config::LIFT_FRONT_TOGGLE_REACH_DEG;
        const bool timedOut =
            now - toggleWaitStart_ >= static_cast<std::uint32_t>(config::LIFT_FRONT_TOGGLE_MAX_WAIT_MS);
        const bool gaveUp = blocked_ || faulted_ || descent_ || score_ || grab_;
        if (reached || timedOut || gaveUp) {
            toggleWait_ = false;
            descendFast();  // togglingUp is false now, so this starts the descent for real
            return;         // the next tick runs it; one tick is nothing
        }
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
    // "No target" sentinel. It used to be -1, which meant goal 0 -- fine when 0 was the bottom and
    // nothing commanded a negative height. The bottom is now -31.9, so -1 is a REACHABLE goal (-0.1
    // deg) and the sentinel has to sit outside the whole range: INT16_MIN is -3276.8 deg.
    s.goal10 = active_ ? toI16(goal_ * 10.0) : std::numeric_limits<std::int16_t>::min();
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
