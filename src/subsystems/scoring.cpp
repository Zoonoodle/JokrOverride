#include "subsystems/scoring.hpp"

#include "config.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace subsystems {

namespace preset = config::lift_preset;

Scoring::Scoring(Lift& lift, Claw& claw) : lift_(lift), claw_(claw), autoclamp_(config::CLAW_AUTOCLAMP_AT_START) {
    if (config::PIVOT_PISTON_PORT != ' ') {
        // The pivot starts logically OUT, and the pin is written to the level OUT means. With
        // PIVOT_OUT_IS_ADI_HIGH == false that is LOW -- the level the ADI already sits at unpowered,
        // which is where the pivot physically is at power-on -- so this writes what is already there
        // and nothing moves at startup (CLAUDE.md safety rule 1).
        pivot_ = std::make_unique<pros::adi::DigitalOut>(static_cast<std::uint8_t>(config::PIVOT_PISTON_PORT),
                                                         pivotLevelFor(true));
    }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
void Scoring::setPivot(bool out) {
    if (!pivot_ || pivotOut_ == out) return;
    pivot_->set_value(pivotLevelFor(out) ? HIGH : LOW);
    pivotOut_ = out;
    pivotMovedAt_ = pros::millis();
}

bool Scoring::pivotDone() const {
    return !pivot_ || pros::millis() - pivotMovedAt_ >= static_cast<std::uint32_t>(config::PIVOT_TIME_MS);
}

void Scoring::command(double degrees, bool precise) {
    // Only send a target when it changes, so the driver's Up/Down fine-tune isn't overwritten every loop.
    if (degrees == commanded_) return;
    commanded_ = degrees;
    lift_.setTarget(degrees, precise);
}

bool Scoring::liftArrived(double degrees) const {
    // isSettled() already means "in HOLD on this goal" (inside LIFT_PRECISE_TOLERANCE_DEG for INTAKE,
    // LIFT_TOLERANCE_DEG for everything else, LIFT_REST_BAND_DEG for DOWN), so the only other thing to
    // check is that the lift is settled on the height WE asked for and not an older one. Do NOT re-test
    // the position against a tolerance of this class's own invention here: that is what made R1 report
    // "arrived at INTAKE" while the arm sat at 0, because the old 2 * LIFT_SETTLE_DEG (3.0 deg) was
    // wider than the entire INTAKE height.
    return lift_.isSettled() && std::fabs(lift_.getTarget() - degrees) < 0.01;
}

double Scoring::levelHeight() const { return preset::STACK_LEVELS[static_cast<std::size_t>(level_ - 1)]; }

double Scoring::heightOf(State s) const {
    if (s == State::MATCHLOAD_POS) return preset::MATCHLOAD;
    if (s == State::IDLE) return preset::DOWN;  // MOVING's destination after an L2 score() -- "go home"
    return preset::INTAKE;
}

void Scoring::openClawForPickup() {
    claw_.disarm();
    if (autoclamp_) claw_.arm();
}

void Scoring::moveTo(State destination) {
    moveDest_ = destination;
    enter(State::MOVING);
}

void Scoring::enter(State next) {
    state_ = next;
    stateStart_ = pros::millis();
    commanded_ = -1.0;
    switch (next) {
        case State::IDLE: level_ = 1; break;  // arrived home after score() lowers to DOWN (reset() sets IDLE directly, without going through enter())
        case State::INTAKE_POS:
        case State::MATCHLOAD_POS:
            command(heightOf(next));
            openClawForPickup();
            // The intake handoff is the one place the pivot is BACK (the driver's definition). The
            // arm is already at the height when this runs (MOVING waits for arrival), so this is the
            // same swing-at-the-handoff the robot has always done at R1.
            setPivot(next == State::INTAKE_POS ? false : !config::PIVOT_BACK_AT_MATCHLOAD);
            break;
        case State::CLAMPING: claw_.clamp(); break;
        case State::STACKING: break;       // update() drives the lift + pivot
        case State::RELEASING:
            claw_.disarm();  // jaws open now, autoclamp off so it can't re-grab the stack
            if (scoreToHome_) {
                // L2 score: claw open and the lift's own descent to DOWN start in this same tick --
                // one continuous motion. The pivot is already OUT at every stack level and OUT is the
                // default down position, so normally nothing happens here; if the driver had toggled
                // it BACK with X, put it OUT now, without waiting for it -- there is no height at
                // which the pivot may not be OUT (see interlock 2 in scoring.hpp).
                setPivot(true);
                command(preset::DOWN);
            }
            break;
        case State::MOVING:
            claw_.disarm();
            setPivot(true);  // OUT is the travel position; update() gives it PIVOT_TIME_MS if it did move
            break;
    }
}

// ---------------------------------------------------------------------------
// driver actions
// ---------------------------------------------------------------------------
void Scoring::reset() {
    state_ = State::IDLE;
    commanded_ = -1.0;
    level_ = 1;
}

void Scoring::stow() {
    reset();
    setPivot(true);  // the default down position is pivot OUT
    lift_.setTarget(preset::DOWN);
}

void Scoring::goToIntake() {
    // Interlock 4: never drag a held pin back down.
    if (claw_.isClamped()) return;
    // Mid-sequence states that must not be interrupted. STACKING is deliberately NOT one of them any
    // more: interlock 4 above already covers the dangerous case (a pin in the jaws), so if the driver
    // has opened the claw up at a stack level, R1 should be able to send the arm back down to the
    // handoff instead of silently doing nothing until they remember to press B.
    if (state_ == State::CLAMPING || state_ == State::RELEASING) return;
    if (state_ == State::INTAKE_POS) {
        commanded_ = -1.0;  // re-tap snaps it back to INTAKE after a manual nudge
        command(preset::INTAKE);
        return;
    }
    if (state_ == State::MOVING && moveDest_ == State::INTAKE_POS) {
        commanded_ = -1.0;  // already heading there, just re-send it
        return;
    }
    moveTo(State::INTAKE_POS);
}

void Scoring::stepUp() {
    switch (state_) {
        case State::CLAMPING:
        case State::STACKING: level_ = std::min(level_ + 1, preset::STACK_LEVEL_COUNT); break;
        case State::RELEASING: break;  // mid-score, ignore
        default:
            level_ = 1;
            enter(State::CLAMPING);
            break;
    }
}

void Scoring::score() {
    // Already parked at the bottom: there is nothing to score and nowhere to descend to, so L2 is a
    // NO-OP -- it must not touch the claw.
    //
    // This is the bug behind "tapping L2 while all the way down activates the clamp and deactivates".
    // The claw can legitimately be left CLAMPED with the arm at 0, because the two things that bring
    // the arm home without scoring -- B (stow()) and the D-pad manual override, which calls reset() --
    // both deliberately leave the claw alone so they can't drop a pin. The macro forgets it was holding
    // anything (state_ goes back to IDLE) but the jaws are still shut. L2 then saw isClamped() == true,
    // entered RELEASING, fired the jaws open and ran a zero-length "descent" that arrived instantly:
    // one cylinder actuation, with the arm already down and nothing to score. Use A to open the jaws
    // down here -- that is the claw button.
    if (lift_.getPosition() <= config::LIFT_BOTTOM_GOAL_DEG) return;
    // Holding a pin (clamped, or mid-clamp/stacking): open the jaws and lower all the way to DOWN (via
    // RELEASING), resetting the level for next time. Nothing held: no matchload fallback here any more
    // (see scoreOrMatchload()) -- just do nothing.
    if (!(claw_.isClamped() || state_ == State::CLAMPING || state_ == State::STACKING)) return;
    scoreToHome_ = true;
    enter(State::RELEASING);
}

void Scoring::scoreOrMatchload() {
    if (claw_.isClamped() || state_ == State::CLAMPING || state_ == State::STACKING) {
        enter(State::RELEASING);
        return;
    }
    if (state_ == State::RELEASING) return;
    if (state_ == State::MATCHLOAD_POS) {
        commanded_ = -1.0;  // re-tap snaps it back to MATCHLOAD after a manual nudge
        command(preset::MATCHLOAD);
        return;
    }
    if (state_ == State::MOVING && moveDest_ == State::MATCHLOAD_POS) {
        commanded_ = -1.0;
        return;
    }
    moveTo(State::MATCHLOAD_POS);
}

void Scoring::resetLevel() {
    if (state_ == State::CLAMPING || state_ == State::STACKING) level_ = 1;
}

void Scoring::toggleClaw() {
    const bool pickup = state_ == State::INTAKE_POS || state_ == State::MATCHLOAD_POS;
    if (!claw_.isClamped()) claw_.clamp();
    else if (pickup && autoclamp_) claw_.release();  // lockout, then autoclamp watches again
    else claw_.disarm();
}

void Scoring::setAutoclamp(bool on) {
    autoclamp_ = on;
    const bool pickup = state_ == State::INTAKE_POS || state_ == State::MATCHLOAD_POS;
    if (pickup && !claw_.isClamped()) openClawForPickup();
}

// ---------------------------------------------------------------------------
// state machine
// ---------------------------------------------------------------------------
void Scoring::update() {
    // The lift is homing against its hard stop: it owns the motors, and nothing here may command a
    // height or swing the pivot over the top of it. Buttons pressed during homing still take effect --
    // enter() already ran for them -- they just wait here, and the state machine picks up the instant
    // homing ends, which also means their elapsed timers have already run down by then.
    if (lift_.isHoming()) return;

    const std::uint32_t elapsed = pros::millis() - stateStart_;
    switch (state_) {
        case State::IDLE:
        case State::INTAKE_POS:
        case State::MATCHLOAD_POS:
            break;  // waiting for the driver (autoclamp, if on, runs in claw.update())

        case State::CLAMPING:
            // Interlock 1: let the jaws shut before the lift pulls away.
            if (elapsed >= static_cast<std::uint32_t>(config::CLAW_CLOSE_TIME_MS)) enter(State::STACKING);
            break;

        case State::STACKING: {
            // Driver opened the claw by hand up here: treat it as a score.
            if (!claw_.isClamped()) {
                moveTo(State::INTAKE_POS);
                break;
            }
            const double h = levelHeight();
            command(h);  // the lift starts rising now, every time -- the pivot never gates it
            // Interlock 2: the pivot swings BACK -> OUT once the pin is clear of the handoff, or once
            // the arm has arrived at the level (so a level can never be left waiting for it). From
            // IDLE (pivot already OUT) this is a no-op.
            if (lift_.getPosition() >= preset::INTAKE + config::PIVOT_OUT_CLEARANCE_DEG || liftArrived(h)) setPivot(true);
            break;
        }

        case State::RELEASING:
            if (scoreToHome_) {
                // score(): the descent is already running (started in enter() above). Drop into IDLE
                // once the arm has actually arrived and is holding at the bottom.
                if (liftArrived(preset::DOWN)) {
                    scoreToHome_ = false;
                    enter(State::IDLE);
                }
            } else if (elapsed >= static_cast<std::uint32_t>(config::CLAW_OPEN_TIME_MS)) {
                // Interlock 3 (scoreOrMatchload() only -- score()/L2 above no longer waits on this).
                moveTo(State::INTAKE_POS);
            }
            break;

        case State::MOVING: {
            if (!pivotDone()) break;  // the pivot was just swung OUT for travel: give it PIVOT_TIME_MS
            const double h = heightOf(moveDest_);
            command(h);
            if (liftArrived(h)) enter(moveDest_);
            break;
        }
    }
}

const char* Scoring::stateName() const {
    switch (state_) {
        case State::IDLE: return "IDLE";
        case State::INTAKE_POS: return "INTAKE";
        case State::MATCHLOAD_POS: return "MATCHLOAD";
        case State::CLAMPING: return "CLAMPING";
        case State::STACKING: return "STACKING";
        case State::RELEASING: return "RELEASING";
        case State::MOVING: return "MOVING";
    }
    return "?";
}

}  // namespace subsystems
