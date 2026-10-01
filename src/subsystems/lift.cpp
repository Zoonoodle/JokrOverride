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
        startupAngleDeg_ = angleCentideg / 100.0;  // kept for the brain screen only
        rotation_->set_position(angleCentideg);    // seed the counter -- NOT reset_position()
    }
    active_ = false;
    goal_ = 0.0;
    precise_ = creep_ = false;
    phase_ = Phase::HOLD;
    redrives_ = 0;
    velI_ = 0.0;
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
        // Hold at the bottom exactly like a normal arrival at DOWN: an active DOWN goal in HOLD, so
        // the loop has an owner and the motors are not left stalled against the frame.
        active_ = true;
        goal_ = config::lift_preset::DOWN;
        precise_ = false;
        creep_ = true;
        blocked_ = false;
        redrives_ = 0;
        enterHold(now);
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

    if (active_ && clamped == goal_ && wantPrecise == precise_ && !faulted_ && !blocked_) return;  // same move, already running
    active_ = true;
    // A deliberate new command clears a latched runaway fault or a stall: the guards' job is to stop
    // the motion that was happening, not to brick every button until someone wiggles the D-pad. A real
    // wiring fault or jam simply re-trips on the next DRIVE, so nothing is lost.
    faulted_ = false;
    blocked_ = false;
    goal_ = clamped;
    precise_ = wantPrecise;
    // INTAKE and DOWN (anything at or below LIFT_BOTTOM_GOAL_DEG) get the creep approach: INTAKE for
    // precision, DOWN so the arm can never reach the hard stop at speed.
    creep_ = precise_ || goal_ <= config::LIFT_BOTTOM_GOAL_DEG;
    redrives_ = 0;
    const std::uint32_t now = pros::millis();
    const double position = getPosition();
    // Already inside this goal's band: arrived, no motion at all (so a B press at the bottom, or a
    // level re-tap, never twitches the arm).
    if (std::fabs(goal_ - position) <= arrivalTolerance()) enterHold(now);
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
    active_ = false;
    blocked_ = false;
    stallStart_ = 0;
    velI_ = 0.0;
    phase_ = Phase::HOLD;
    queuedTarget_ = false;
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
        case Phase::BRAKE: return creep_ && lastVRef_ <= config::LIFT_CREEP_DEG_PER_S + 1e-9 ? "CREEP" : "BRAKE";
        case Phase::HOLD: return "HOLD";
    }
    return "";
}

void Lift::driverPower(double input) { manualPower_ = std::clamp(input, -1.0, 1.0); }

// ---------------------------------------------------------------------------
// Phase helpers
// ---------------------------------------------------------------------------
double Lift::arrivalTolerance() const {
    if (precise_) return config::LIFT_PRECISE_TOLERANCE_DEG;
    if (creep_) return config::LIFT_REST_BAND_DEG;  // a trip to the bottom: park this far above the stop
    return config::LIFT_TOLERANCE_DEG;
}

double Lift::brakeCurve(double dist) const {
    // v = sqrt(2 a d): the fastest the arm may go and still reach zero speed `lead` deg before the
    // goal. For a creep goal the curve is pushed out by the creep zone instead, so the arm is down to
    // the creep speed with that much left to go and the creep covers the rest.
    const double lead = creep_ ? config::LIFT_CREEP_ZONE_DEG : config::LIFT_BRAKE_MARGIN_DEG;
    const double d = std::max(dist - lead, 0.0);
    return std::sqrt(2.0 * config::LIFT_BRAKE_DECEL_DEG_PER_S2 * d);
}

void Lift::enterDrive(std::uint32_t now, double position) {
    phase_ = Phase::DRIVE;
    phaseStart_ = now;
    dir0_ = goal_ >= position ? 1.0 : -1.0;
    minDist_ = std::fabs(goal_ - position);
    stallStart_ = 0;
    velI_ = 0.0;
}

void Lift::enterBrake(std::uint32_t now, bool fromDrive) {
    phase_ = Phase::BRAKE;
    phaseStart_ = now;
    stallStart_ = 0;
    // Seed the velocity loop's integral with the torque voltage the motors were just producing: at full
    // DRIVE voltage the part not eaten by back-EMF (LIFT_MAX_MV / free speed per deg/s) is what gravity
    // and friction were costing at this angle. Zero at free speed, a few thousand mV when climbing
    // under load -- a far better first guess than 0, and the integral corrects it from there.
    if (fromDrive) {
        const double kv = config::LIFT_MAX_MV / config::LIFT_FREE_SPEED_DEG_PER_S;
        velI_ = std::clamp(dir0_ * config::LIFT_MAX_MV - kv * velocity_, -static_cast<double>(config::LIFT_BRAKE_MAX_MV),
                           static_cast<double>(config::LIFT_BRAKE_MAX_MV));
    } else {
        velI_ = 0.0;
    }
}

void Lift::enterHold(std::uint32_t now) {
    phase_ = Phase::HOLD;
    phaseStart_ = now;
    stallStart_ = 0;
    velI_ = 0.0;
}

bool Lift::stalled(std::uint32_t now, double position) {
    if (stallStart_ == 0 || std::fabs(position - stallStartPos_) >= config::LIFT_STALL_DEG) {
        stallStart_ = now;  // window (re)opens: either just started, or the arm did move
        stallStartPos_ = position;
        return false;
    }
    return now - stallStart_ >= static_cast<std::uint32_t>(config::LIFT_STALL_MS);
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

    // 1) Driver manual override (D-pad Up/Down): full send, plain motor power. This is the ONLY thing
    //    allowed to preempt everything else, and it always wins: cancel whatever target was active so
    //    nothing "fights" the driver, cancel homing (and don't retry it this run -- the arm is wherever
    //    the driver put it, and zero()'s seed stands), and make sure release doesn't jump back to an
    //    old target either (see class comment).
    if (std::fabs(manualPower_) > 0.02) {
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

    // 2) Homing: find the hard stop and zero there. Runs as a mode in this same task, so it is still
    //    this one function writing the motors, and nothing anywhere blocks on it. The stall and
    //    runaway guards are not reached while homing -- deliberately: homing's whole job is to drive
    //    the arm into a hard stop and notice that it stopped, which is exactly what those guards
    //    exist to prevent in normal operation. updateHoming() has its own timeout instead.
    if (home_ == Home::RUNNING) {
        updateHoming(now);
        trace(now, 'h', position, lastOutput_, true);
        return;
    }

    // 3) No commanded height: brake and hold right here. This is the state at power-on (nothing
    //    moves until something explicitly calls setTarget()/moveTo()) and right after the driver
    //    lets go of the manual override (no old target resumes).
    if (!active_) {
        motors_.brake();
        lastError_ = 0.0;
        lastOutput_ = 0.0;
        lastVRef_ = 0.0;
        trace(now, 'I', position, 0.0, false);
        return;
    }

    if (faulted_) {
        motors_.brake();
        lastOutput_ = 0.0;
        trace(now, 'F', position, 0.0, false);
        return;
    }

    // 4) The move. Everything below is a function of where the arm is and how fast it is going.
    const double gap = goal_ - position;
    const double dist = std::fabs(gap);
    const double dir = gap >= 0.0 ? 1.0 : -1.0;
    lastError_ = gap;
    const double vCurve = brakeCurve(dist);
    lastVRef_ = creep_ ? std::max(vCurve, config::LIFT_CREEP_DEG_PER_S) : vCurve;

    // DRIVE: full voltage toward the goal until the arm reaches the braking curve.
    if (phase_ == Phase::DRIVE) {
        // Runaway guard: full voltage toward the goal and the gap GREW. Sensor or motors reversed.
        minDist_ = std::min(minDist_, dist);
        if (dist - minDist_ > config::LIFT_FAULT_DEG) {
            faulted_ = true;
            motors_.brake();
            lastOutput_ = 0.0;
            trace(now, 'F', position, 0.0, true);
            return;
        }
        if (stalled(now, position)) {
            blocked_ = true;  // can't get there: hold here, no re-drives (cleared by the next new target)
            enterHold(now);
        } else if (velocity_ * dir >= vCurve) {
            enterBrake(now, true);
        } else {
            const double mv = dir * config::LIFT_MAX_MV;
            motors_.move_voltage(static_cast<int>(mv));
            lastOutput_ = mv;
            trace(now, 'D', position, mv, true);
            return;
        }
    }

    // BRAKE / CREEP: velocity loop onto the curve (or the creep floor), then hand over to HOLD.
    if (phase_ == Phase::BRAKE) {
        const bool passed = dir != dir0_;  // the arm is on the far side of the goal from where it set off
        bool arrived;
        double vRef;  // signed, + = up
        if (creep_) {
            // Creep toward the goal from either side until inside the band. The creep floor means
            // the arm reaches the band at a crawl, so the firmware hold grabs it with no spring-back.
            vRef = dir * std::max(vCurve, config::LIFT_CREEP_DEG_PER_S);
            arrived = dist <= arrivalTolerance();
        } else {
            // A normal goal is never chased back the way the arm came: past the goal, the reference is
            // simply "stop". Stopped inside the band (or stopped anywhere past the goal) = arrived; a
            // big miss is dealt with by the bounded re-drive from HOLD, not by reversing here.
            vRef = passed ? 0.0 : dir * vCurve;
            arrived = std::fabs(velocity_) < config::LIFT_STOPPED_DEG_PER_S &&
                      (passed || dist <= config::LIFT_TOLERANCE_DEG);
        }
        const bool timedOut = now - phaseStart_ >= static_cast<std::uint32_t>(config::LIFT_APPROACH_TIMEOUT_MS);
        // The creep wants motion, so a jam during it is a stall; plain braking is stopping on purpose.
        const bool creeping = creep_ && std::fabs(vRef) <= config::LIFT_CREEP_DEG_PER_S + 1e-9;
        if (creeping && stalled(now, position)) {
            blocked_ = true;
            enterHold(now);
        } else if (arrived || timedOut) {
            enterHold(now);
        } else {
            const double err = vRef - velocity_;
            double out = config::LIFT_VEL_KP * err + velI_;
            const double sat = config::LIFT_BRAKE_MAX_MV;
            if (out > sat) out = sat;
            else if (out < -sat) out = -sat;
            else velI_ = std::clamp(velI_ + config::LIFT_VEL_KI * err * dt, -sat, sat);  // integrate only unsaturated
            motors_.move_voltage(static_cast<int>(out));
            lastOutput_ = out;
            trace(now, creeping ? 'C' : 'B', position, out, true);
            return;
        }
    }

    // HOLD: the firmware's brake-hold, every tick. Arrival latches here. The only way out is a miss
    // bigger than LIFT_REENGAGE_DEG, at most LIFT_MAX_REDRIVES times -- and never after a stall.
    motors_.brake();
    lastOutput_ = 0.0;
    if (!blocked_ && dist > config::LIFT_REENGAGE_DEG && redrives_ < config::LIFT_MAX_REDRIVES) {
        ++redrives_;
        enterDrive(now, position);
        trace(now, 'H', position, 0.0, true);
        return;
    }
    trace(now, 'H', position, 0.0, false);
}

// ---------------------------------------------------------------------------
// Trace: lift task pushes, trace task prints. Single producer / single consumer ring.
// ---------------------------------------------------------------------------
void Lift::trace(std::uint32_t now, char phase, double position, double mv, bool moving) {
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
    traceHead_.store(head + 1, std::memory_order_release);
}

void Lift::traceDrain() {
    std::uint32_t tail = traceTail_.load(std::memory_order_relaxed);
    const std::uint32_t head = traceHead_.load(std::memory_order_acquire);
    if (tail == head) return;
    while (tail != head) {
        const TraceSample& s = trace_[tail % kTraceSize];
        std::printf("LIFT %u %c %d %d %d %d\n", static_cast<unsigned>(s.ms), s.phase, s.pos10, s.goal10, s.vel, s.mv);
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
                std::printf("LIFT trace: ms phase(D=drive B=brake C=creep H=hold M=manual h=homing F=fault I=idle) "
                            "pos*10 goal*10 vel(deg/s) out(mV)\n");
                while (true) {
                    traceDrain();
                    pros::delay(20);
                }
            },
            TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "lift-trace");
    }
}

}  // namespace subsystems
