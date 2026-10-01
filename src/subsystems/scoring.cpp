#include "subsystems/scoring.hpp"

#include "config.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace subsystems {

namespace preset = config::lift_preset;

Scoring::Scoring(Lift& lift, Claw& claw) : lift_(lift), claw_(claw), autoclamp_(config::CLAW_AUTOCLAMP_AT_START) {
    if (config::PIVOT_PISTON_PORT != ' ') {
        pivot_ = std::make_unique<pros::adi::Pneumatics>(config::PIVOT_PISTON_PORT, false);
    }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
void Scoring::setPivot(bool out) {
    if (!pivot_ || pivotOut_ == out) return;
    if (out) pivot_->extend();
    else pivot_->retract();
    pivotOut_ = out;
    pivotMovedAt_ = pros::millis();
}

bool Scoring::pivotDone() const {
    return !pivot_ || pros::millis() - pivotMovedAt_ >= static_cast<std::uint32_t>(config::PIVOT_TIME_MS);
}

void Scoring::command(double degrees, double speedDegPerSec) {
    // Only send a target when it changes, so the driver's Up/Down fine-tune isn't overwritten every loop.
    if (degrees == commanded_) return;
    commanded_ = degrees;
    lift_.setTarget(degrees, speedDegPerSec);
}

bool Scoring::liftArrived(double degrees) const {
    return lift_.isSettled() && std::fabs(lift_.getPosition() - degrees) <= 2 * config::LIFT_SETTLE_DEG;
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

void Scoring::moveTo(State destination, double speedDegPerSec) {
    moveDest_ = destination;
    moveSpeed_ = speedDegPerSec;
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
            setPivot(next == State::INTAKE_POS ? config::PIVOT_ON_AT_INTAKE : config::PIVOT_ON_AT_MATCHLOAD);
            break;
        case State::CLAMPING: claw_.clamp(); break;
        case State::STACKING: break;       // update() drives the lift + pivot
        case State::RELEASING: claw_.disarm(); break;  // jaws open, autoclamp off so it can't re-grab the stack
        case State::MOVING:
            claw_.disarm();
            setPivot(false);  // interlock 2: pivot in before the lift moves
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

void Scoring::goToIntake() {
    // Interlock 4: never drag a held pin back down.
    if (claw_.isClamped()) return;
    if (state_ == State::CLAMPING || state_ == State::STACKING || state_ == State::RELEASING) return;
    if (state_ == State::INTAKE_POS) {
        commanded_ = -1.0;  // re-tap snaps it back to INTAKE after a manual nudge
        command(preset::INTAKE);
        return;
    }
    if (state_ == State::MOVING && moveDest_ == State::INTAKE_POS) {
        commanded_ = -1.0;  // already heading there: keep the speed (slow after a score), just re-send it
        return;
    }
    moveTo(State::INTAKE_POS, 0.0);
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
    // Holding a pin (clamped, or mid-clamp/stacking): open the jaws, then (via the RELEASING case in
    // update()) pivot in and lower all the way to DOWN, resetting the level for next time. Nothing
    // held: no matchload fallback here any more (see scoreOrMatchload()) -- just do nothing.
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
    moveTo(State::MATCHLOAD_POS, 0.0);
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
    const std::uint32_t elapsed = pros::millis() - stateStart_;
    switch (state_) {
        case State::IDLE:
        case State::INTAKE_POS:
        case State::MATCHLOAD_POS:
            break;  // waiting for the driver (autoclamp, if on, runs in claw.update())

        case State::CLAMPING:
            // Let the jaws shut before the lift pulls away.
            if (elapsed >= static_cast<std::uint32_t>(config::CLAW_CLOSE_TIME_MS)) enter(State::STACKING);
            break;

        case State::STACKING: {
            // Driver opened the claw by hand up here: treat it as a score.
            if (!claw_.isClamped()) {
                moveTo(State::INTAKE_POS, config::LIFT_LOWER_SPEED_DEG_PER_S);
                break;
            }
            const double h = levelHeight();
            if (h >= preset::PIVOT_CLEAR) {
                command(h);
                // Interlock 1: flip out only once the lift clears the intake/drivetrain.
                if (lift_.getPosition() >= preset::PIVOT_CLEAR) setPivot(true);
            } else {
                // Level below the pivot clearance (e.g. resetLevel() back to level 1): pivot in first, then lower.
                setPivot(false);
                if (pivotDone()) command(h);
            }
            break;
        }

        case State::RELEASING:
            // Interlock 3: the arm doesn't move until the jaws are open.
            if (elapsed >= static_cast<std::uint32_t>(config::CLAW_OPEN_TIME_MS)) {
                if (scoreToHome_) {
                    scoreToHome_ = false;
                    moveTo(State::IDLE, config::LIFT_LOWER_SPEED_DEG_PER_S);  // score(): all the way home
                } else {
                    moveTo(State::INTAKE_POS, config::LIFT_LOWER_SPEED_DEG_PER_S);  // scoreOrMatchload()
                }
            }
            break;

        case State::MOVING: {
            if (!pivotDone()) break;  // interlock 2
            const double h = heightOf(moveDest_);
            command(h, moveSpeed_);
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
