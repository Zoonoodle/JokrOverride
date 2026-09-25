#include "subsystems/scoring.hpp"

#include "config.hpp"
#include "pros/rtos.hpp"

namespace subsystems {

namespace preset = config::lift_preset;

Scoring::Scoring(Lift& lift, Claw& claw, Intake& intake) : lift_(lift), claw_(claw), intake_(intake) {
    if (config::PIVOT_PISTON_PORT != ' ') {
        pivot_ = std::make_unique<pros::adi::Pneumatics>(config::PIVOT_PISTON_PORT, false);
    }
}

void Scoring::setPivot(bool scoring) {
    if (!pivot_ || pivotOut_ == scoring) return;
    if (scoring) pivot_->extend();
    else pivot_->retract();
    pivotOut_ = scoring;
    pivotMovedAt_ = pros::millis();
}

void Scoring::enter(State next) {
    state_ = next;
    stateStart_ = pros::millis();
    switch (next) {
        case State::INTAKING:
            setPivot(false);
            lift_.setTarget(preset::INTAKE);
            claw_.disarm();
            claw_.arm();                         // autoclamp watches for the next pin
            intake_.set(Intake::Mode::IN);
            break;
        case State::HOLDING:
            intake_.set(Intake::Mode::STOP);
            lift_.setTarget(preset::CARRY);      // lift the pin clear of the field
            break;
        case State::PIVOTING:
            lift_.setTarget(scoreTarget_);       // pivot flips in update() once the lift clears CARRY
            break;
        case State::SCORING:
            break;                               // wait for the driver
        case State::RETURNING:
            claw_.release();                     // drop the pin
            break;
    }
}

void Scoring::begin() { enter(State::INTAKING); }

void Scoring::requestScore(double liftDegrees) {
    if (state_ != State::HOLDING && state_ != State::SCORING) return;  // interlock 1: must hold a pin
    if (!claw_.isClamped()) return;
    scoreTarget_ = liftDegrees;
    enter(State::PIVOTING);                                             // also lets the driver switch goal height
}

void Scoring::release() {
    if (state_ == State::SCORING && lift_.isSettled()) enter(State::RETURNING);  // interlock 3
}

void Scoring::cancel() {
    if (claw_.isClamped()) claw_.release();
    enter(State::RETURNING);
}

void Scoring::update() {
    const std::uint32_t now = pros::millis();
    switch (state_) {
        case State::INTAKING:
            // The autoclamp closed on a pin: carry it.
            if (claw_.isClamped()) enter(State::HOLDING);
            break;

        case State::HOLDING:
            // If the driver manually opened the claw, go back to intaking.
            if (!claw_.isClamped()) enter(State::INTAKING);
            break;

        case State::PIVOTING: {
            // Interlock 2: flip only once the lift is above CARRY.
            if (lift_.getPosition() >= preset::CARRY) setPivot(true);
            const bool pivotDone =
                !pivot_ || (pivotOut_ && now - pivotMovedAt_ >= static_cast<std::uint32_t>(config::PIVOT_TIME_MS));
            if (lift_.isSettled() && pivotDone) enter(State::SCORING);
            break;
        }

        case State::SCORING:
            break;  // release() moves us on

        case State::RETURNING:
            // Give the jaws time to open before moving away from the goal.
            if (now - stateStart_ < static_cast<std::uint32_t>(config::CLAW_OPEN_TIME_MS)) break;
            // Interlock 4: pivot back before the lift drops below CARRY.
            setPivot(false);
            if (pivot_ && now - pivotMovedAt_ < static_cast<std::uint32_t>(config::PIVOT_TIME_MS)) break;
            lift_.setTarget(preset::INTAKE);
            if (lift_.isSettled()) enter(State::INTAKING);
            break;
    }
}

const char* Scoring::stateName() const {
    switch (state_) {
        case State::INTAKING: return "INTAKING";
        case State::HOLDING: return "HOLDING";
        case State::PIVOTING: return "PIVOTING";
        case State::SCORING: return "SCORING";
        case State::RETURNING: return "RETURNING";
    }
    return "?";
}

}  // namespace subsystems
