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
    // hasCommanded_ rather than a sentinel height: every double is now a reachable height.
    if (hasCommanded_ && degrees == commanded_) return;
    commanded_ = degrees;
    hasCommanded_ = true;
    lift_.setTarget(degrees, precise);
}

bool Scoring::liftArrived(double degrees) const {
    // isSettled() already means "arrived on this goal" -- in HOLD inside LIFT_PRECISE_TOLERANCE_DEG
    // (INTAKE) or LIFT_TOLERANCE_DEG (everything else), or SEATED on the hard stop for a trip to the
    // bottom (DOWN). So the only other thing to
    // check is that the lift is settled on the height WE asked for and not an older one. Do NOT re-test
    // the position against a tolerance of this class's own invention here: that is what made R1 report
    // "arrived at INTAKE" while the arm sat at 0, because the old 2 * LIFT_SETTLE_DEG (3.0 deg) was
    // wider than the entire INTAKE height.
    return lift_.isSettled() && std::fabs(lift_.getTarget() - degrees) < 0.01;
}

// L1's ladder, not a stack level: whatever the fixed-step ladder is currently asking for. See
// "L1's LADDER" in config.hpp and advanceStep() below.
double Scoring::levelHeight() const { return stepGoal_; }

void Scoring::resetStep() {
    stepGoal_ = preset::DOWN;
    level_ = 1;
}

void Scoring::advanceStep() {
    const double step = config::L1_STEP_DEG;
    const double ceiling = config::L1_STEP_CEILING_DEG;

    // UP ONLY (2026-10-07, the driver's call): "move it up 20 degrees each time". The ladder used to
    // reverse at the top and walk back down; it now STOPS at the ceiling, and further presses are a
    // no-op. Coming down is L2 / B / R2, never L1.
    //
    // THE STEP IS RELATIVE TO WHERE THE LIFT IS HEADING, not to a stored rung. Using the lift's own
    // active TARGET (not its position) is what makes rapid L1 taps stack correctly -- the arm is still
    // flying when the second tap lands, so its position lags, but its target is already the first
    // rung. With nothing commanded -- which is exactly the state the D-pad override leaves behind,
    // because manual control calls Lift::cancel() and that clears active_ -- fall back to the measured
    // position, so the next press steps 20 deg above wherever the driver left the arm. That makes the
    // manual handoff automatic and needs no extra bookkeeping in Lift.
    const double base = lift_.hasActiveTarget() ? lift_.getTarget() : lift_.getPosition();

    // The last step up lands ON the top measured level rather than at the nearest multiple of the
    // step, so the top of the ladder is a real scoring height. The low clamp means a step can never
    // command the arm below its own hard stop.
    //
    // ...AND NEVER BELOW WHERE THE ARM ALREADY IS. "UP ONLY" is the rule above, but the rung was
    // computed from the lift's TARGET, and the target is not always under the arm: a descent to DOWN
    // that gets BLOCKED on something, or PARKed by LIFT_MOVE_TIMEOUT_MS, leaves goal_ at the bottom
    // with the arm held well above it and hasActiveTarget() still true -- so base was 0, the rung came
    // out at L1_STEP_DEG, and an L1 press DROVE THE ARM DOWN to it at LIFT_DOWN_SPEED_PCT. That is a
    // real path to "I pressed L1 and it went down rapidly", and this one line closes it for every
    // stale base there could ever be: the worst an L1 can now do is ask for the height the arm is
    // already at. (The ladder keeps stepping from the TARGET when the target IS ahead of the arm --
    // that is what merges rapid taps into one continuous rise.)
    stepGoal_ = std::clamp(std::max(base + step, lift_.getPosition()), preset::DOWN, ceiling);

    // The rung number is for the brain only; it is derived from the height so the two cannot disagree,
    // and it is counted FROM THE BOTTOM (which is -31.9, not 0).
    level_ = std::max(1, static_cast<int>(std::lround((stepGoal_ - preset::DOWN) / step)));
}

double Scoring::heightOf(State s) const {
    if (s == State::MATCHLOAD_POS) return preset::MATCHLOAD;
    if (s == State::IDLE) return preset::DOWN;  // MOVING's destination after an L2 score() -- "go home"
    return preset::INTAKE;
}

void Scoring::openClawForPickup() {
    claw_.disarm();
    if (autoclamp_) claw_.arm();  // armed immediately: the jaws are empty and this IS the pickup spot
}

void Scoring::openClaw() {
    if (autoclamp_) claw_.release();  // keeps the autoclamp enabled; it re-arms once the sensor is clear
    else claw_.disarm();
}

void Scoring::moveTo(State destination) {
    moveDest_ = destination;
    enter(State::MOVING);
}

void Scoring::enterWhenSettled(State next) {
    // A lift move that is still flying must not be interrupted by a press or a push: Lift::startScore()
    // would capture scoreStartPos_ from a MOVING arm (so L2_AUTO_RELEASE_DEG's travel budget starts
    // from the wrong place), the contact detector would be fed the move's own deceleration, and an arm
    // still coasting upward can trip the press-phase runaway guard outright. So wait for it to arrive.
    // No target active at all = nothing to wait for.
    if (!lift_.hasActiveTarget() || lift_.isSettled() || lift_.isBlocked() || lift_.isFaulted()) {
        enter(next);
        return;
    }
    pendingAfterSettle_ = next;
    enter(State::LIFT_SETTLE);
}

// ---------------------------------------------------------------------------
// THE CLAMP REACTION: every clamp either hops the arm up a little, or (at the R1 handoff) forces it
// down onto the pin. See "THE CLAMP-RISE" and AUTOCLAMP_INTAKE_PUSH_MS in config.hpp.
// ---------------------------------------------------------------------------
void Scoring::handleClampEvent() {
    // ALWAYS consume, even when nothing is going to be done with it. A stored event is an event that
    // fires late -- and the one that matters is competition_initialize()'s pre-match grip, which happens
    // while the robot is DISABLED. periodic::reset() clears it at the start of each enabled mode too;
    // this is the second half of the same guarantee.
    const Claw::ClampSource source = claw_.takeClampEvent();
    if (source == Claw::ClampSource::NONE) return;

    // AT THE R1 HANDOFF the driver's rule is the opposite of everywhere else: do not lift clear, press
    // DOWN onto the pin the intake has just handed over, and stay there. Only for a clamp the autoclamp
    // made -- a manual A press at the handoff is left alone entirely (no hop, no push), because the
    // driver may well be gripping something by hand and L1 is the button that pushes.
    if (isAtIntakeHandoff()) {
        if (source != Claw::ClampSource::AUTOCLAMP) return;
        if (lift_.isGrabbing() || lift_.isScoring() || lift_.isScoreCycle()) return;  // already busy
        // THE INTAKE CAUGHT SOMETHING. Run the automatic grab (driver, 2026-10-07): settle, push down
        // into it, then rise to level 1 releasing the pivot on the way.
        //
        // No startClampPush() any more: the push down is now a real, measured move to
        // GRAB_PUSH_DEPTH_DEG in GRAB_PUSH, not a timed voltage shove, so the depth is what the
        // driver asked for ("move down lower to like -5 degrees") rather than however far 300 ms of
        // push happened to get.
        if (config::AUTOCLAMP_INTAKE_AUTO_STACK) enter(State::GRAB_SETTLE);
        return;
    }

    // ONLY A REAL AUTOCLAMP GRAB HOPS (driver, 2026-10-07). Claw::clamp() records MANUAL, so this one
    // test excludes the driver's A press AND every clamp the scoring macro makes for itself (CLAMPING,
    // the L1 GRAB) -- only the detector in Claw::update() ever produces AUTOCLAMP.
    if (source != Claw::ClampSource::AUTOCLAMP) return;

    // A COMMANDED MOVE ALWAYS WINS. The hop must never fight the L1 grab's own push, an L2 score cycle,
    // a lift move that is still in flight (an auton route's `clawGrip(); liftToFrontToggle();` is exactly
    // that), or a macro state that is about to command a height of its own.
    if (lift_.isGrabbing() || lift_.isScoring() || lift_.isScoreCycle() || lift_.isScoredAbove()) return;
    switch (state_) {
        case State::CLAMPING:     // about to rise to a stack level
        case State::STACKING:
        case State::LIFT_SETTLE:
        case State::GRABBING:
        case State::RELEASING:
        case State::SCORED:
        case State::RETURNING:
        case State::MOVING:
        // The INTAKE SEQUENCE'S OWN GRAB. These were missing from this list when the sequence was
        // added, which let a clamp during the automatic grab fall out of the switch and hop the arm
        // on top of it. They are excluded explicitly now: the intake has its own reaction entirely.
        case State::GRAB_SETTLE:
        case State::GRAB_PUSH:
        case State::GRAB_RISE:
        case State::INTAKE_APPROACH:  // unreachable: isAtIntakeHandoff() took these above
        case State::INTAKE_PIVOT_WAIT:
        case State::INTAKE_DESCEND:
        // L2's intake reset: the arm is deliberately on its way home past the very cup it was
        // taking. The autoclamp is inhibited for all of it anyway (periodic::tick()), so this is
        // belt-and-braces -- and it is what stops a clamp that fired on the tick before the inhibit
        // landing a hop on top of the reset.
        case State::INTAKE_RESET:
            return;
        case State::IDLE:
        case State::INTAKE_POS:      // unreachable: isAtIntakeHandoff() took it above
        case State::MATCHLOAD_POS:
            break;
    }

    // RECORD THE HOP, and the delay this mode wants. serviceClampHop(), called from update()
    // immediately after this, does the rise once the delay is up and re-checks the conditions then,
    // because the driver's 200 ms is long enough for them to change. In AUTON the delay is 0, so that
    // call fires the rise on THIS VERY TICK -- no wait at all, which is the point. Clearing this one
    // timestamp is all it takes to cancel, which is why enter() and reset() can do it for free.
    pendingHopAt_ = pros::millis();
    pendingHopDelayMs_ =
        autonMode_ ? config::CLAMP_RISE_DELAY_AUTON_MS : config::CLAMP_RISE_DELAY_DRIVER_MS;
    hopCmdSeq_ = lift_.commandSeq();  // ...and what the lift had been asked for as of now
}

void Scoring::serviceClampHop() {
    if (pendingHopAt_ == 0) return;
    if (pros::millis() - pendingHopAt_ < static_cast<std::uint32_t>(pendingHopDelayMs_)) return;
    pendingHopAt_ = 0;  // one-shot, whatever happens below

    // THE GRIP DID NOT HOLD. Knocked out, or shaken loose during the wait: stay down rather than lift
    // an empty claw and pretend we have something. KEPT UNCONDITIONALLY, including for auton's 0-delay
    // path, where it can only ever pass: Claw::clampFrom() sets state_ = CLAMPED synchronously before
    // it raises the event, so isClamped() is already true on the tick the event is consumed. It costs
    // one comparison and stays a real guard for the driver's 200 ms wait, which is worth more than a
    // branch that could only behave identically.
    if (!claw_.isClamped()) return;
    // Re-check what the delay may have changed. The STATE cannot have changed without enter() clearing
    // this, so only the lift's own business needs re-testing.
    if (lift_.isGrabbing() || lift_.isScoring() || lift_.isScoreCycle() || lift_.isScoredAbove()) return;
    // A COMMANDED MOVE STILL IN FLIGHT WINS -- with one deliberate exception: an auton descent that is
    // FORCING ITSELF DOWN at L2_RUN_DOWN_MV. That descent exists to put the claw on the floor with the
    // autoclamp armed, so a grab part way down is not an interruption, it is the point: the hop takes
    // over, the arm stops shoving down (setTarget() clears runDownHard_) and lifts what it caught to
    // CLAMP_GRAB_HEIGHT. The L2 score's run home is NOT this case -- it keeps its own refusal, because
    // there the jaws have just put a pin on a stack and the arm must finish going home.
    const bool interruptibleDescent = lift_.isForcedDown() && !lift_.isScoreReturn();
    if (!interruptibleDescent && lift_.hasActiveTarget() && !lift_.isSettled() && !lift_.isBlocked() &&
        !lift_.isFaulted()) {
        return;
    }
    // ANYTHING NEWER WINS. enter() and reset() cancel a pending hop, which covers every input that
    // goes through them -- but not all of them do: stepUp()'s CLAMPING/STACKING case only advances the
    // ladder, and auton can command the lift without touching this class at all. So the hop also
    // checks the lift's own command counter: if the lift has been asked for ANYTHING since the clamp
    // fired, this reaction is stale and is dropped rather than overriding it. CLAMP_GRAB_HEIGHT is an
    // ABSOLUTE height (f480e95), so a stale hop does not nudge the arm -- it yanks it to 37.5.
    if (lift_.commandSeq() != hopCmdSeq_) {
        lift_.note("HOP stale", lift_.getPosition());
        return;
    }
    // A HOP NEVER GOES DOWN. It is the reaction to a grab -- "lift what you just picked up clear" --
    // and CLAMP_GRAB_HEIGHT being absolute means that from anywhere above it the very same call is a
    // DESCENT instead. With the arm parked at the intake handoff this is not hypothetical: the
    // working-tree lift_preset::INTAKE is 42, i.e. ABOVE the 37.5 hop height, so the hop would command
    // the arm DOWN 4.5 deg -- and startClampRise() would even make it a PRECISE move, because the
    // distance is inside LIFT_TOLERANCE_DEG. At the old INTAKE of 22 the same call was a 15.5 deg rise.
    if (lift_.getPosition() >= config::CLAMP_GRAB_HEIGHT - config::LIFT_TOLERANCE_DEG) {
        lift_.note("HOP skip hi", lift_.getPosition());
        return;
    }

    // GO TO CLAMP_GRAB_HEIGHT -- an ABSOLUTE height, not a rise (driver, 2026-10-07: "make it 37.5
    // height, not a rise"). Grabs happen at the bottom, so this is normally a ~37.5 deg lift; a grab
    // with the arm already higher than that moves it DOWN to 37.5, which is what "go to 37.5" means.
    // There is no never-go-down guard any more for exactly that reason; MAX still caps it.
    const double goal = std::min(config::CLAMP_GRAB_HEIGHT, preset::MAX);
    hasCommanded_ = false;  // this class is not tracking the height any more; the hop is the lift's move
    lift_.startClampRise(goal);
}

void Scoring::startIntakeResetDescent() {
    resetDown_ = true;
    hasCommanded_ = false;  // this class stops tracking the height; the descent is the lift's move
    // THE SAME FORCED RUN-DOWN AN L2 SCORE USES (Lift::descendForced() -> L2_RUN_DOWN_MV, no slow
    // zone, no FINAL, the same three exits into SEAT's continuous push). It is an L2 action and it ends
    // on the hard stop, which is the whole point of "reset back to our bottom spot" -- and the arm is
    // coming down with the jaws EMPTY, so there is nothing to land gently on.
    lift_.descendForced();
    lift_.note("INTAKE RESET dn", lift_.getPosition());
}

void Scoring::enter(State next) {
    state_ = next;
    stateStart_ = pros::millis();
    hasCommanded_ = false;
    // Any state change cancels a pending clamp hop: if the driver pressed anything during the 400 ms
    // wait, their button wins. This is also what guarantees the 7.5 deg hop can never fire on top of
    // the intake sequence's own grab -- entering GRAB_SETTLE comes through here.
    pendingHopAt_ = 0;
    switch (next) {
        case State::IDLE: resetStep(); break;  // arrived home after score() lowers to DOWN (reset() sets IDLE directly, without going through enter())
        case State::INTAKE_POS:
            // STEP (d): just HOLD -2.2. The pivot is already ENGAGED (INTAKE_DESCEND did it at 13.5)
            // and the arm is already here; setPivot(false) is belt-and-braces for the MOVING path,
            // which can still land in this state.
            openClawForPickup();
            setPivot(false);
            command(preset::INTAKE, true);  // precise: this is the one height that must line up
            break;
        case State::MATCHLOAD_POS:
            command(heightOf(next));
            openClawForPickup();
            setPivot(!config::PIVOT_BACK_AT_MATCHLOAD);
            break;

        // --- THE INTAKE SEQUENCE ------------------------------------------------------------
        case State::INTAKE_APPROACH:
            // (a) Pivot DISENGAGED -- setPivot(TRUE) is "OUT", which is the driver's "not engaged"
            // (see the polarity note in config.hpp) -- and run to 13.5. The jaws open now so a cup
            // can be taken the moment the sequence reaches the bottom.
            openClawForPickup();
            setPivot(true);
            command(config::INTAKE_PIVOT_TRIGGER_DEG);
            break;
        case State::INTAKE_PIVOT_WAIT:
            // (b) ENGAGE THE PIVOT AND STAY PUT. The arm keeps actively holding 13.5 -- the height is
            // re-commanded here, not left to whatever INTAKE_APPROACH last sent -- while the cylinder
            // retracts fully in. update() starts the descent only after INTAKE_PIVOT_SETTLE_MS.
            setPivot(false);
            command(config::INTAKE_PIVOT_TRIGGER_DEG);
            break;
        case State::INTAKE_DESCEND:
            // (c) the pivot is already engaged and settled, so now walk down to the intake height --
            // VERY slowly ("it has to go down very slowly to this height and stay there"). setTargetSlow
            // caps the whole move at INTAKE_DESCEND_SPEED_PCT of free speed on top of the precise
            // approach, so it creeps down and still walks the last degrees in as tuned. Commanded
            // straight on the lift because this needs the capped entry point, not plain command().
            hasCommanded_ = false;
            lift_.setTargetSlow(preset::INTAKE, config::INTAKE_DESCEND_SPEED_PCT);
            break;

        // --- THE INTAKE AUTOCLAMP GRAB ------------------------------------------------------
        case State::GRAB_SETTLE:
            // Jaws just shut on a cup or pin. Command NOTHING: the arm holds exactly where the clamp
            // caught it for GRAB_SETTLE_MS so the grip settles before any motion.
            break;
        case State::GRAB_PUSH:
            // Down into what we just caught, still clamped. Precise, because -5 is a shallow,
            // deliberate seating depth and overshooting it presses the cup into the floor.
            command(config::GRAB_PUSH_DEPTH_DEG, true);
            break;
        case State::GRAB_RISE:
            // Up to level 1, still clamped. The pivot stays ENGAGED for now: update() disengages it
            // the moment the arm passes PIVOT_RELEASE_UP_DEG on the way up.
            command(preset::STACK_LEVELS[0]);
            break;
        case State::LIFT_SETTLE:
            break;  // update() watches the lift and starts pendingAfterSettle_ when it arrives
        case State::GRABBING:
            // THE GRAB. The lift forces the claw down onto the pin at L1_GRAB_PUSH_PCT and tells us
            // when to shut the jaws (Lift::needsClawGrip(), polled in update()); the intake is stopped
            // for the whole push by periodic::tick(). Same split as the L2 scorer: the lift owns the
            // motion, this class owns the cylinder.
            lift_.startGrab();
            break;
        case State::CLAMPING: claw_.clamp(); break;
        case State::STACKING: break;       // update() drives the lift + pivot
        case State::RELEASING:
            if (scoreToHome_) {
                // L2 SCORE. The jaws stay SHUT: the lift presses the pin down onto the stack and
                // tells us when to open them (Lift::needsClawRelease(), polled in update() below).
                // Do NOT disarm the claw here -- opening it now would drop the pin from stack height,
                // which is exactly what the old sequence did.
                // The pivot is already OUT at every stack level and OUT is the default down position,
                // so normally nothing happens here; if the driver had toggled it BACK with X, put it
                // OUT now, without waiting for it -- there is no height at which the pivot may not be
                // OUT (see interlock 2 in scoring.hpp).
                setPivot(true);
                lift_.startScore();
            } else {
                openClaw();  // scoreOrMatchload(): jaws open now, and the autoclamp setting is kept
            }
            break;
        case State::SCORED:
            resetStep();  // rung counter back to 1; the next L1 steps up from wherever the arm is
            break;       // the lift is holding above the stack; wait for the next button
        case State::RETURNING:
            // L2 press 2 (or B from SCORED): the fast run home that ends SEATED on the hard stop.
            // The pivot is OUT, the default down position.
            setPivot(true);
            lift_.startScoreReturn();
            break;
        case State::INTAKE_RESET:
            // L2 DURING THE INTAKE SEQUENCE: abandon it and go home (driver, 2026-10-09). Two halves,
            // and which one starts depends on where the pivot already is.
            //
            // PIVOT ALREADY OUT -- which is L2 during INTAKE_APPROACH, before the pivot has ever been
            // engaged -- means there is nothing to clear, so SKIP THE RISE and just go down. Anything
            // else has the pivot ENGAGED underneath the arm, so the arm must get above
            // PIVOT_RELEASE_UP_DEG before it may retract, exactly as the automatic grab's rise does.
            resetDown_ = pivotOut_;
            if (resetDown_) {
                setPivot(true);  // a no-op; says out loud what the descent is relying on
                startIntakeResetDescent();
            } else {
                command(config::INTAKE_RESET_CLEAR_DEG);
                lift_.note("INTAKE RESET", lift_.getPosition());
            }
            break;
        case State::MOVING:
            openClaw();  // jaws open for the pickup; update() places the pivot by height
            break;
    }
}

double Scoring::liftPositionForGate() const { return lift_.getPosition(); }

bool Scoring::inHandoffZone() const {
    return lift_.getPosition() <= preset::INTAKE + config::PIVOT_HANDOFF_ZONE_DEG;
}

// ---------------------------------------------------------------------------
// driver actions
// ---------------------------------------------------------------------------
void Scoring::reset() {
    state_ = State::IDLE;
    hasCommanded_ = false;
    // reset() sets state_ directly instead of going through enter(), so the pending hop has to be
    // dropped here too -- the D-pad override calls this every tick it is held.
    pendingHopAt_ = 0;
    resetStep();
}

void Scoring::stow() {
    // From the held-above position after an L2 score, B is the same thing as the second L2: go home
    // through RETURNING, which keeps the autoclamp inhibited all the way down so the pin that was
    // just released cannot be re-grabbed on the way past it.
    if (state_ == State::SCORED) {
        enter(State::RETURNING);
        return;
    }
    reset();
    setPivot(true);  // the default down position is pivot OUT
    lift_.setTarget(preset::DOWN);
}

void Scoring::grabHere() {
    if (state_ == State::GRABBING || state_ == State::RELEASING || state_ == State::LIFT_SETTLE) return;
    grabRise_ = false;  // stay where the push ends
    enterWhenSettled(State::GRABBING);
}

void Scoring::goToIntake() {
    // Interlock 4: never drag a held pin back down. (Unpowered is OPEN on this claw, so the power-on
    // state is not clamped and R1 works straight away without a special case.)
    if (claw_.isClamped()) return;
    // Mid-sequence states that must not be interrupted. STACKING is deliberately NOT one of them any
    // more: interlock 4 above already covers the dangerous case (a pin in the jaws), so if the driver
    // has opened the claw up at a stack level, R1 should be able to send the arm back down to the
    // handoff instead of silently doing nothing until they remember to press B.
    if (state_ == State::CLAMPING || state_ == State::RELEASING || state_ == State::GRABBING ||
        state_ == State::RETURNING || state_ == State::LIFT_SETTLE) {
        return;
    }
    // Already running the sequence: a re-tap is a no-op rather than a restart, so it cannot throw the
    // arm back up to 13.5 once it is already down at -2.2.
    if (state_ == State::INTAKE_POS || state_ == State::INTAKE_APPROACH ||
        state_ == State::INTAKE_PIVOT_WAIT || state_ == State::INTAKE_DESCEND) {
        return;
    }
    // R1 AND EVERY AUTON INTAKE CALL START HERE -- one shared path (driver, 2026-10-07). Step (a) of
    // "THE INTAKE SEQUENCE" in config.hpp: pivot disengaged, run to 13.5. It replaces the old
    // moveTo(INTAKE_POS), which went straight to a single handoff height and placed the pivot by
    // height on the way down.
    enter(State::INTAKE_APPROACH);
}

void Scoring::stepUp() {
    switch (state_) {
        case State::CLAMPING:
        case State::STACKING: advanceStep(); break;  // L1_STEP_DEG further up, stopping at the ceiling
        case State::RELEASING:
        case State::GRABBING:
        case State::LIFT_SETTLE:
        case State::RETURNING: break;  // mid-sequence, ignore
        case State::INTAKE_POS:
            // THE GRAB (2026-10-02, the driver's own design). L1 from the intake handoff no longer
            // just closes the jaws and rises: the arm first FORCES ITSELF DOWN onto the pin while the
            // jaws close around it, and only then rises to level 1. The intake stops for the whole
            // push. Everything about the push is config::L1_GRAB_*; see "THE GRAB" in lift.hpp.
            // advanceStep() is the first rung: L1_STEP_DEG above where the lift is heading, which here
            // is the handoff (23.0), so STACKING is asked for 43.0. It seeds itself from the lift, so
            // no resetStep() is needed in front of it any more -- that used to be required back when
            // the ladder counted from a stored 0.
            advanceStep();
            grabRise_ = true;
            enterWhenSettled(State::GRABBING);
            break;
        default:
            advanceStep();  // the first rung: L1_STEP_DEG above wherever the arm is heading right now
            // The claw can already be CLAMPED here -- L2's down-state clamp (score(), above) shuts it
            // with the arm still at the bottom, well before this tap lands. CLAMPING's
            // CLAW_CLOSE_TIME_MS wait exists so the lift doesn't pull out of jaws that are still
            // physically closing; a claw that was already shut has nothing left to wait for, so skip
            // straight to STACKING and let the arm rise this tick. Taps after this one still land in
            // the STACKING case above and stack normally.
            enter(claw_.isClamped() ? State::STACKING : State::CLAMPING);
            break;
    }
}

void Scoring::score() {
    // L2 DURING THE INTAKE SEQUENCE IS "GO HOME" (driver, 2026-10-09: "we need l2 to be a reset back to
    // our bottom spot with pivot ... when we hit l2 during intake position active"). It comes first,
    // before every other test, because the arm is empty and up at the handoff -- the old code reached
    // its "nothing held" return and did nothing at all. DRIVER ONLY: auton reaches this same function
    // through auton::score(), and a route asking to score mid-intake means the score, not a reset.
    if (!autonMode_ && isIntakeSequenceActive()) {
        enter(State::INTAKE_RESET);
        return;
    }
    // LEGACY TWO-PRESS PATH, unreachable since the one-press score: nothing enters SCORED any more.
    // the jaws open: this L2 is the run home that seats it (see "SCORING WITH L2" in lift.hpp and the
    // L2 note in config.hpp). Nothing else about press 1 is re-run.
    if (state_ == State::SCORED) {
        enter(State::RETURNING);
        return;
    }

    // Mid-move: a score is already running (RELEASING), the arm is forcing down onto a pin
    // (GRABBING), the jaws are still physically closing (CLAMPING), the arm is travelling to
    // INTAKE/MATCHLOAD (MOVING), or press 2's descent is in flight (RETURNING). Ignore L2 until
    // whichever of those finishes.
    if (state_ == State::RELEASING || state_ == State::CLAMPING || state_ == State::MOVING ||
        state_ == State::GRABBING || state_ == State::RETURNING || state_ == State::LIFT_SETTLE) {
        return;
    }

    // At the bottom (seated on the hard stop, or within LIFT_BOTTOM_GOAL_DEG of it) and not mid-move:
    // L2 is now the CLAMP TRIGGER, not a no-op. Toggle the claw through the exact same logical API A /
    // toggleClaw() uses -- not clamped -> clamp, clamped -> release -- and touch nothing else: the
    // lift stays put and the pivot stays OUT (toggleClaw() never reaches either).
    //
    // This replaces the old "L2 at the bottom does nothing" rule. That rule existed only because L2
    // used to fire the claw open and run a zero-length "descent" whenever the arm was already down --
    // the claw can legitimately be left CLAMPED with the arm at 0 (B's stow() and the D-pad manual
    // override both bring the arm home without touching the claw), and the old score() saw
    // isClamped() == true and ran its score sequence for nothing. Now that case is a deliberate clamp
    // toggle instead: a driver with a pin sitting at floor level, or wanting to let one go down here,
    // has a button for it without reaching for A.
    if (lift_.getPosition() <= config::LIFT_BOTTOM_GOAL_DEG || lift_.isSeated()) {
        toggleClaw();
        return;
    }

    // Holding a pin (clamped, or mid-stacking) above the bottom: open the jaws and lower all the way
    // to DOWN (via RELEASING), resetting the level for next time. Nothing held: no matchload fallback
    // here any more (see scoreOrMatchload()) -- just do nothing.
    if (!(claw_.isClamped() || state_ == State::STACKING)) return;
    scoreToHome_ = true;
    // If a lift move is still in flight -- an auton route that fired liftTo() off alongside a chassis
    // move, or a driver tapping L2 during a rise -- wait for the arm to get there before pressing.
    enterWhenSettled(State::RELEASING);
}

void Scoring::scoreOrMatchload() {
    if (claw_.isClamped() || state_ == State::CLAMPING || state_ == State::STACKING) {
        enter(State::RELEASING);
        return;
    }
    if (state_ == State::RELEASING) return;
    if (state_ == State::MATCHLOAD_POS) {
        hasCommanded_ = false;  // re-tap snaps it back to MATCHLOAD after a manual nudge
        command(preset::MATCHLOAD);
        return;
    }
    if (state_ == State::MOVING && moveDest_ == State::MATCHLOAD_POS) {
        hasCommanded_ = false;
        return;
    }
    moveTo(State::MATCHLOAD_POS);
}

void Scoring::resetLevel() {
    if (state_ == State::CLAMPING || state_ == State::STACKING) resetStep();
}

void Scoring::toggleClaw() {
    // Shut (CLAMPED, or CLOSED since power-on) -> open; open -> clamp.
    if (!claw_.isClosed()) claw_.clamp();
    else openClaw();  // opening by hand must not switch the autoclamp off -- see openClaw()
}

void Scoring::setAutoclamp(bool on) {
    autoclamp_ = on;
    // The autoclamp is no longer only for the INTAKE/MATCHLOAD wait: the driver's rule is "whenever
    // the claw is unclamped, something within the trigger range gets gripped", wherever the arm is.
    // So arm/disarm the claw itself here, in any state except CLAMPED (never open jaws that are
    // holding something). Claw::arm() handles the RELEASED case by leaving the re-arm to the clear
    // reading rule -- see autoclamp_detector.hpp.
    //
    // THE ENABLE ITSELF ALWAYS GETS THROUGH, CLAMPED OR NOT. This function used to return at the test
    // below before the Claw ever heard about it, and that was a real bug on the one path a match
    // actually takes: competition_initialize() grips the preloaded pin, so the claw is CLAMPED when
    // autonomous() switches the autoclamp on -- the flag was set on THIS class but never on the Claw,
    // whose own enabled_ stayed false. The first time the route opened the jaws, Claw::update()'s
    // RELEASED case read that false and went to OPEN (autoclamp off) instead of ARMED, so the autoclamp
    // could not fire for the whole rest of the match, with nothing on the brain to explain why.
    // setAutoclampEnabled() is the flag and nothing else -- arm() would be a no-op on a clamped claw and
    // disarm() would open it and drop the pin.
    claw_.setAutoclampEnabled(on);
    if (claw_.isClamped()) return;
    if (on) claw_.arm();
    else claw_.disarm();
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

    // EVERY CLAMP, from any source, reacted to in exactly one place -- before the state machine, so a
    // transition made this tick cannot be seen half-done. It always consumes the event (see
    // handleClampEvent()), so nothing queues.
    handleClampEvent();
    // ...and the delayed rise that a clamp outside the intake may have queued.
    serviceClampHop();

    const std::uint32_t elapsed = pros::millis() - stateStart_;
    switch (state_) {
        case State::INTAKE_POS:
            // STEP (d): HOLD -2.2 and wait (the oscillation was removed 2026-10-07 -- "just keep it at
            // -2.2"). command() only writes when the height actually changes, so this costs nothing per
            // tick once the target is in. The two ways out are a clamp (handleClampEvent() above enters
            // GRAB_SETTLE) and any driver button.
            command(preset::INTAKE, true);
            break;

        // --- THE INTAKE SEQUENCE ----------------------------------------------------------------
        case State::INTAKE_APPROACH: {
            // (a) running UP to INTAKE_PIVOT_TRIGGER_DEG with the pivot disengaged. (b) THE TRIGGER:
            // move on, and INTAKE_PIVOT_WAIT's entry engages the pivot immediately.
            //
            // THE ARM HAS TO REALLY BE UP THERE, not merely "arrived" (driver, 2026-10-09: "it needs
            // to go higher before pivoting and going back down"). liftArrived() is the lift's own
            // arrival latch, and for a non-precise move that latches anywhere inside
            // LIFT_TOLERANCE_DEG -- 5 deg -- so this used to let the pivot swing in with the claw up
            // to a whole tolerance band BELOW the trigger. However high the trigger was tuned, the
            // height the pivot actually engaged at was a 5 deg range under it. The position test is
            // the fix; the arrival latch is kept as well, so a move that settles high still counts.
            const bool highEnough =
                lift_.getPosition() >= config::INTAKE_PIVOT_TRIGGER_DEG - config::INTAKE_PIVOT_REACH_DEG;
            // The timeout is the backstop so a blocked or short-stopping arm cannot sit here forever;
            // carrying on is safe because the next step is also downward.
            if ((highEnough && liftArrived(config::INTAKE_PIVOT_TRIGGER_DEG)) || lift_.isBlocked() ||
                lift_.isFaulted() ||
                elapsed >= static_cast<std::uint32_t>(config::INTAKE_APPROACH_TIMEOUT_MS)) {
                enter(State::INTAKE_PIVOT_WAIT);
            }
            break;
        }

        case State::INTAKE_PIVOT_WAIT:
            // (b) holding 13.5 with the pivot engaged. The only exit is the timer: the pivot has no
            // sensor, so INTAKE_PIVOT_SETTLE_MS is how we know it is in.
            if (elapsed >= static_cast<std::uint32_t>(config::INTAKE_PIVOT_SETTLE_MS)) {
                enter(State::INTAKE_DESCEND);
            }
            break;

        case State::INTAKE_DESCEND:
            // (c) the slow, precise walk down to -2.2 with the pivot engaged. Then the bounce.
            if (liftArrived(preset::INTAKE) || lift_.isBlocked() || lift_.isFaulted() ||
                elapsed >= static_cast<std::uint32_t>(config::INTAKE_DESCEND_TIMEOUT_MS)) {
                enter(State::INTAKE_POS);
            }
            break;

        // --- THE INTAKE AUTOCLAMP GRAB ----------------------------------------------------------
        case State::GRAB_SETTLE:
            // Holding still on the fresh grip. If it turns out there is nothing in the jaws after all
            // go straight back to intaking rather than pushing and rising with an empty claw.
            if (!claw_.isClamped()) {
                enter(State::INTAKE_POS);
                break;
            }
            if (elapsed >= static_cast<std::uint32_t>(config::GRAB_SETTLE_MS)) enter(State::GRAB_PUSH);
            break;

        case State::GRAB_PUSH:
            // Seating down into the cup/pin at -5, still clamped. BLOCKED is the expected outcome as
            // often as arrival is -- the cup is on the floor -- so both move on.
            if (liftArrived(config::GRAB_PUSH_DEPTH_DEG) || lift_.isBlocked() || lift_.isFaulted() ||
                elapsed >= static_cast<std::uint32_t>(config::GRAB_PUSH_TIMEOUT_MS)) {
                enter(State::GRAB_RISE);
            }
            break;

        case State::GRAB_RISE: {
            // Rising to level 1 with the cup held. THE PIVOT DISENGAGES ON THE WAY UP, by HEIGHT:
            // engaged until the arm passes PIVOT_RELEASE_UP_DEG (3), then OUT. Checked every tick, so
            // it happens mid-rise exactly as asked and not on arrival.
            if (lift_.getPosition() > config::PIVOT_RELEASE_UP_DEG) setPivot(true);
            // Lost the cup on the way up: stop pretending to stack it.
            if (!claw_.isClamped()) {
                enter(State::IDLE);
                break;
            }
            // Arrived at level 1: hand over to STACKING so the ladder is seeded there and the next L1
            // steps up from level 1 rather than from the floor.
            if (liftArrived(preset::STACK_LEVELS[0]) || lift_.isBlocked() || lift_.isFaulted() ||
                elapsed >= static_cast<std::uint32_t>(config::GRAB_RISE_TIMEOUT_MS)) {
                setPivot(true);  // make sure, even if the rise never got past 3 deg
                stepGoal_ = preset::STACK_LEVELS[0];
                level_ = 1;
                enter(State::STACKING);
            }
            break;
        }

        case State::IDLE:
        case State::MATCHLOAD_POS:
            break;  // waiting for the driver (autoclamp, if on, runs in claw.update())

        case State::LIFT_SETTLE:
            // Waiting for an in-flight lift move to arrive before starting the action that was asked
            // for. Bounded: the lift's own LIFT_MOVE_TIMEOUT_MS parks a move that is getting nowhere
            // (and PARK counts as settled), so this timeout is only the backstop behind that. Going
            // ahead anyway on BLOCKED/FAULT matches what the lift itself does -- stop waiting, act on
            // where the arm actually is.
            if (lift_.isSettled() || lift_.isBlocked() || lift_.isFaulted() ||
                elapsed >= static_cast<std::uint32_t>(config::SCORING_SETTLE_WAIT_MS)) {
                enter(pendingAfterSettle_);
            }
            break;

        case State::GRABBING:
            // The lift is pressing the claw down onto the pin. Shut the jaws the moment it asks
            // (L1_GRAB_CLOSE_AT_MS into the push) -- unless the autoclamp already got there -- and
            // tell it so; it keeps pressing until the jaws have had CLAW_CLOSE_TIME_MS.
            if (lift_.needsClawGrip()) {
                if (!claw_.isClamped()) claw_.clamp();
                lift_.clawGripDone();
            }
            if (!lift_.isGrabbing()) {
                // Push over (or refused, which is what happens if homing was still running -- the
                // lift will not fight it). The arm is holding wherever it ended.
                if (!claw_.isClamped()) {
                    // Nothing was gripped: fall back to the ordinary close-and-wait interlock rather
                    // than rising with open jaws.
                    if (grabRise_) enter(State::CLAMPING);
                    else enter(State::IDLE);
                    break;
                }
                enter(grabRise_ ? State::STACKING : State::IDLE);
            }
            break;

        case State::SCORED:
            break;  // holding above the stack; the next L2 (or B) runs home, R1/L1 move on

        case State::RETURNING:
            // Press 2's descent. It ends seated on the hard stop; if the lift gives up instead
            // (BLOCKED / FAULT) let go of the macro so the other buttons work again.
            if (lift_.isBlocked() || lift_.isFaulted()) {
                enter(State::IDLE);
                break;
            }
            if (liftArrived(preset::DOWN)) enter(State::IDLE);
            break;

        case State::CLAMPING:
            // Interlock 1: let the jaws shut before the lift pulls away.
            if (elapsed >= static_cast<std::uint32_t>(config::CLAW_CLOSE_TIME_MS)) enter(State::STACKING);
            break;

        case State::STACKING: {
            // Driver opened the claw by hand up here: treat it as a score. Goes through the INTAKE
            // SEQUENCE (13.5 trigger -> pivot -> -2.2 -> bounce), not the old straight-to-the-handoff
            // MOVING path, so there is exactly ONE way the arm ever arrives at the intake.
            if (!claw_.isClamped()) {
                enter(State::INTAKE_APPROACH);
                break;
            }
            const double h = levelHeight();
            command(h);  // the lift starts rising now, every time -- the pivot never gates it
            // Interlock 2: THE PIVOT WAITS FOR THE ARM (driver, 2026-10-07). It used to swing OUT the
            // moment the pin rose out of the handoff zone -- part way up, with the arm still moving.
            // Now it is left exactly where it is until the lift has FINISHED getting into position,
            // and only then swings to the scoring position. config::PIVOT_RISE_WAIT_MS is the backstop
            // behind that: if arrival never comes (blocked, parked short, an unreachable target) the
            // pivot still swings, so a level can never be left with the pivot stuck BACK.
            // From IDLE (pivot already OUT) all of this is a no-op.
            if (liftArrived(h) || elapsed >= static_cast<std::uint32_t>(config::PIVOT_RISE_WAIT_MS)) {
                setPivot(true);
            }
            break;
        }

        case State::RELEASING:
            if (scoreToHome_) {
                // L2 SCORE. The lift owns the whole press / sense / release / run-home motion (see
                // "SCORING WITH L2" in lift.hpp). All this class does is own the claw: the instant the
                // lift says the pin has landed on the stack, open the jaws and tell it so. The split
                // is deliberate -- only the lift can see the measured-vs-commanded speed gap that
                // contact detection is made on, and only this class is allowed to touch a cylinder.
                if (lift_.needsClawRelease()) {
                    claw_.release();  // RELEASED, so the autoclamp can't re-grab the stack either
                    lift_.clawReleaseDone();
                }
                // The lift gave up (claw still caught after every unstick try): let go of the macro so
                // L1 / R1 / B work again. The lift holds where it is and keeps showing SCORE STUCK.
                if (lift_.isScoreStuck()) {
                    scoreToHome_ = false;
                    enter(State::IDLE);
                    break;
                }
                // ONE PRESS, 2026-10-07. The lift releases the pin and then carries straight on down
                // to DOWN at the same scoring speed (Lift::endScoreClear()), so this state now ends
                // when the arm is HOME, not when it is parked above the stack. No SCORED, no second
                // press. BLOCKED / FAULT also end it, so a lift that gives up on the way down still
                // hands the buttons back.
                if (lift_.isSeated() || liftArrived(preset::DOWN)) {
                    scoreToHome_ = false;
                    enter(State::IDLE);
                }
            } else if (elapsed >= static_cast<std::uint32_t>(config::CLAW_OPEN_TIME_MS)) {
                // Interlock 3 (scoreOrMatchload() only -- score()/L2 above no longer waits on this).
                // Same single intake path as R1 and the auton calls.
                enter(State::INTAKE_APPROACH);
            }
            break;

        case State::INTAKE_RESET:
            if (!resetDown_) {
                // THE RISE, with the pivot swung OUT only once the arm is properly clear of it --
                // INTAKE_RESET_PIVOT_UP_DEG, which is the height the pivot swings IN at less the same
                // reach margin step (a) uses. It used to borrow PIVOT_RELEASE_UP_DEG and retract with
                // the arm barely five degrees off the handoff, which is what caught (driver,
                // 2026-10-10). Checked every tick, so it happens mid-rise rather than on arrival. The
                // descent then waits for the cylinder's own PIVOT_TIME_MS as well (pivotDone()),
                // because coming back DOWN past an engaged pivot is what this manoeuvre exists to
                // avoid.
                if (lift_.getPosition() >= config::INTAKE_RESET_PIVOT_UP_DEG) setPivot(true);
                const bool clear = pivotOut_ && pivotDone() &&
                                   lift_.getPosition() >= config::INTAKE_RESET_PIVOT_UP_DEG;
                // A lift that gave up (caught on something, or faulted) still gets the pivot out and
                // still goes home: the timeout is the backstop behind both.
                if (clear || lift_.isBlocked() || lift_.isFaulted() ||
                    elapsed >= static_cast<std::uint32_t>(config::INTAKE_RESET_RISE_TIMEOUT_MS)) {
                    setPivot(true);
                    startIntakeResetDescent();
                }
                break;
            }
            // ...and the trip home. Seated is done; BLOCKED/FAULT hand the buttons back rather than
            // leaving the macro holding them.
            if (lift_.isSeated() || lift_.isBlocked() || lift_.isFaulted()) enter(State::IDLE);
            break;

        case State::MOVING: {
            // Pivot by height, every tick: heading to the intake it is BACK inside the handoff zone and
            // OUT above it, so on R1 it swings BACK as the arm comes down into the zone -- not only once
            // the lift reports "arrived", so a slightly short stop still gets it there. Heading
            // anywhere else it is OUT (the travel position).
            const bool wantBack = moveDest_ == State::INTAKE_POS ? inHandoffZone()
                                  : moveDest_ == State::MATCHLOAD_POS ? config::PIVOT_BACK_AT_MATCHLOAD && liftArrived(heightOf(moveDest_))
                                                                       : false;
            setPivot(!wantBack);
            // If that swing started from a stack level (pivot was BACK up there via X), let it finish
            // before the arm moves; a swing inside the handoff zone never holds the lift.
            if (!pivotDone() && !inHandoffZone()) break;
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
        case State::INTAKE_APPROACH: return "IN>13.5";
        case State::INTAKE_PIVOT_WAIT: return "IN PIV";
        case State::INTAKE_DESCEND: return "IN>-2.2";
        case State::GRAB_SETTLE: return "GRAB SET";
        case State::GRAB_PUSH: return "GRAB DN";
        case State::GRAB_RISE: return "GRAB UP";
        case State::CLAMPING: return "CLAMPING";
        case State::STACKING: return "STACKING";
        case State::LIFT_SETTLE: return "WAIT LIFT";
        case State::GRABBING: return "GRABBING";
        case State::RELEASING: return "RELEASING";
        case State::SCORED: return "SCORED UP";
        case State::RETURNING: return "RETURNING";
        case State::INTAKE_RESET: return resetDown_ ? "IN RESETv" : "IN RESET^";
        case State::MOVING: return "MOVING";
    }
    return "?";
}

}  // namespace subsystems
