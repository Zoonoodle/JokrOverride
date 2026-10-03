#pragma once
/**
 * Scoring — the "intake macro": owns the lift + claw + pivot sequence so every driver button is
 * one tap and the mechanisms always move in a safe order. Every lift move it makes goes through
 * Lift::setTarget() (the lift's own DRIVE -> BRAKE -> HOLD loop), never raw motor power -- see
 * include/subsystems/lift.hpp.
 *
 * Driver buttons (src/main.cpp; D-pad Up/Down are the lift's plain manual override and are NOT
 * part of this macro -- holding either cancels it, see main.cpp's opcontrol()):
 *
 *   IDLE ──R1──> INTAKE_POS ──L1──> CLAMPING ──(jaws shut)──> STACKING (level 1..5)
 *    ^                                                            │  L1 = level+1, stays at 5 (top)
 *    │                                                            L2
 *    │                                                            v
 *    └──────────────────────────────────────────── RELEASING (claw open + lift descent to DOWN,
 *                                                               both at once; pivot stays OUT)
 *
 *   L2 while the arm is already at the bottom (IDLE or INTAKE_POS, not mid-move) is a special case
 *   handled entirely inside score() without a state change: it toggles the claw in place -- see
 *   score() below and RELEASING's note. The diagram's L2 arrow is everywhere else, above the bottom.
 *
 *   IDLE          power-on state, and also where the manual override leaves it. Nothing moves
 *                 until the driver presses something (or the manual override, which bypasses
 *                 this macro entirely). Resets level back to 1.
 *   INTAKE_POS    lift at INTAKE, claw open (autoclamp armed if it's on), pivot BACK
 *   MATCHLOAD_POS lift at MATCHLOAD, claw open (autoclamp armed if it's on), pivot per
 *                 config::PIVOT_BACK_AT_MATCHLOAD -- not currently wired to a driver button;
 *                 scoreOrMatchload() still reaches it for auton or a future button.
 *   CLAMPING      jaws closing; waits CLAW_CLOSE_TIME_MS so the lift doesn't pull out of an open claw
 *   STACKING      holding a pin, lift at STACK_LEVELS[level-1], pivot OUT (it swings OUT once the arm
 *                 has risen out of the handoff zone, INTAKE + PIVOT_HANDOFF_ZONE_DEG, or has arrived)
 *   RELEASING     L2/score() with something held above the bottom -- THE SENSING SCORER. The jaws
 *                 stay SHUT and Lift::startScore() takes over the motion: it presses the pin down
 *                 onto the stack, watches the rotation sensor for the landing, and tells this class
 *                 when to open the claw (Lift::needsClawRelease(), polled every tick here). The split
 *                 is deliberate: only the lift can see the measured-vs-commanded speed gap contact
 *                 detection is made on, and only this class is allowed to touch a cylinder. Read
 *                 "SCORING WITH L2" in lift.hpp for the phases (PRESS / REL / UNSTK / RETURN). The
 *                 lift ALWAYS releases now -- a real detection, or Lift's own L2_AUTO_RELEASE_DEG
 *                 forcing it by height if nothing was ever detected -- so this ends in IDLE once the
 *                 arm is seated at the bottom with the pin off, or immediately if the lift reports
 *                 SCORE STUCK, so L1 / R1 / B work again while the lift holds where it stopped.
 *                 The pivot is already OUT at every stack level and OUT is the default down position,
 *                 so it does not move (if the driver had toggled it BACK up there, it is put back
 *                 OUT, without waiting). L2 at the bottom never enters this state -- see score().
 *                 scoreOrMatchload() (not on a button) still uses the older CLAW_OPEN_TIME_MS wait
 *                 then a move to INTAKE_POS, unchanged.
 *   MOVING        scoreOrMatchload()/goTo*() only: claw open, lift to the destination -- INTAKE or
 *                 MATCHLOAD, see heightOf() -- with the pivot placed by height every tick: heading to
 *                 the intake it is BACK inside the handoff zone and OUT above it, so it swings BACK as
 *                 the arm comes down into the zone; heading elsewhere it is OUT. A swing that starts
 *                 up at a level gets PIVOT_TIME_MS before the lift moves; a swing inside the zone
 *                 never holds the lift.
 *
 * THE PIVOT is on ADI config::PIVOT_PISTON_PORT, which is 'C'. Until 2026-10-01 the code had the claw
 * and the pivot on each other's ports, which is why several interlocks here were tuned against the
 * wrong cylinder; config.hpp carries the full re-derivation.
 * In the driver's words: OUT (code: true) = swung out, where it sits at power-on, at every stack level
 * while scoring, and in the default down position after L2 / B. BACK (code: false) = pulled in toward
 * the intake handoff, ONLY for taking a pin from the intake (R1 / INTAKE_POS). The ADI level that
 * means OUT is config::PIVOT_OUT_IS_ADI_HIGH (LOW is OUT, confirmed twice); the brain's CLAW line
 * shows the word, the port letter and the raw level so X can be used to confirm it. At power-on the
 * pivot is logically OUT and the ADI pin is written to the level it already sits at unpowered, so
 * nothing moves at startup.
 *
 * Interlocks:
 *   1. The arm never leaves the handoff until the jaws have had CLAW_CLOSE_TIME_MS to shut (CLAMPING).
 *   2. The handoff zone (lift_preset::INTAKE up to INTAKE + PIVOT_HANDOFF_ZONE_DEG) times the pivot:
 *      heading to or at the intake it is BACK inside the zone and OUT above it, so on R1 it swings
 *      BACK as the arm comes down into the zone and on L1 it swings OUT as the arm rises out of it (or
 *      when the arm arrives at the level, whichever is first -- it can never leave a level waiting).
 *      This only times the swing, it never gates the lift: the lift starts rising the same tick
 *      CLAMPING ends. It replaces the old PIVOT_CLEAR interlock and L2 descent floor, which were
 *      written for the opposite pivot meaning and would have forbidden the very positions the driver
 *      wants (pivot BACK near the bottom at INTAKE, OUT at the bottom after L2). There is no height at
 *      which the pivot is forbidden to be OUT or BACK.
 *   3. Nothing is ever dropped from height. scoreOrMatchload() waits CLAW_OPEN_TIME_MS before the arm
 *      moves away. L2's score() goes further: the jaws stay SHUT for the whole descent onto the stack
 *      and only open at the moment the lift decides the pin has landed -- a real detection, or (if
 *      nothing is ever detected) Lift's own guaranteed release at L2_AUTO_RELEASE_DEG, so the jaws
 *      always open somewhere above the floor, never at it.
 *   4. R1 never pulls a held pin back down to the intake: it is a no-op while the claw is CLAMPED
 *      (shut by L1, A, L2 at the bottom, or the autoclamp). This claw's unpowered position is OPEN,
 *      so power-on is never clamped and R1 works straight away. Starting a match with a pin in the
 *      claw: press A or L1 first, because the claw needs air to grip it at all.
 *   5. The manual override always wins: main.cpp calls reset() whenever D-pad Up/Down is held, so
 *      this macro can never re-command the lift out from under a manual nudge (it does not touch
 *      the claw or pivot, which stay physically where they were).
 *
 * If config::PIVOT_PISTON_PORT is ' ' the pivot steps are skipped.
 */

#include "pros/adi.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/lift.hpp"

#include <cstdint>
#include <memory>

namespace subsystems {

class Scoring {
  public:
    enum class State {
        IDLE, INTAKE_POS, MATCHLOAD_POS,
        GRABBING,   // L1 from the handoff / auton grab: the lift is forcing down, the jaws close
        CLAMPING, STACKING,
        RELEASING,  // L2 press 1: press / sense / release / post-push / rise
        SCORED,     // press 1 done: holding L2_CLEAR_RISE_DEG above the stack. The next L2 runs home
        RETURNING,  // L2 press 2 (or B from SCORED): the fast descent that seats at 0
        MOVING,
    };

    Scoring(Lift& lift, Claw& claw);

    /** Back to IDLE: nothing moves, lift holds where it is, level resets to 1. Also what main.cpp
     *  calls whenever the driver holds the manual override, so this macro can't fight it. Touches
     *  neither the claw nor the pivot. */
    void reset();

    /** B: stow -- reset() the macro, put the pivot OUT (the default down position) and send the lift
     *  straight down to DOWN. Leaves the claw alone so it can't drop a pin. */
    void stow();

    /** R1 (new-press): put the claw at the intake handoff (ignored while holding a pin or mid-score). */
    void goToIntake();
    /** Auton's "just grab": run the SAME force-down-and-close sequence L1 does at the handoff, but
     *  STAY where the push ended (the lift holds there) instead of rising to a level. The jaws are
     *  closed by this class, as always; the push is Lift::startGrab(). */
    void grabHere();
    /** L1: first tap clamps and lifts to level 1; every tap after goes up one level, staying at
     *  STACK_LEVEL_COUNT (the top) once reached. If the claw is already CLAMPED (L2 clamped it at the
     *  bottom, see score() below), the first tap skips CLAW_CLOSE_TIME_MS's wait and rises straight
     *  away, since the jaws have nothing left to close. */
    void stepUp();
    /** L2: at the bottom (seated, or within LIFT_BOTTOM_GOAL_DEG of it) and not mid-move -- the CLAMP
     *  TRIGGER: toggles the claw in place through the same logical API A / toggleClaw() uses, and
     *  touches neither the lift nor the pivot. Above the bottom, holding a pin -> THE SENSING SCORER
     *  (RELEASING + Lift::startScore(): press the pin onto the stack with the jaws shut, release at
     *  the detected landing, slide off, run home fast and seat) and reset the level count so the next
     *  L1 starts at level 1 again; not holding anything -> no matchload fallback on this button any
     *  more, see scoreOrMatchload(). L2 pressed again mid-sequence is ignored. */
    void score();
    /** Not currently wired to a driver button (previously Down/L2 when empty). Holding a pin ->
     *  same as score() but lowers to INTAKE instead of DOWN. Empty -> go to the matchload position.
     *  Left in place for auton or a future button. */
    void scoreOrMatchload();
    /** Not currently wired to a driver button (previously Right/R2). Holding a pin -> back to stack
     *  level 1. Left in place for auton or a future button -- L2's score() already resets the level
     *  after scoring, so this is only useful to back up mid-stack without scoring. */
    void resetLevel();
    /** X: flip the pivot OUT <-> BACK. */
    void togglePivot() { setPivot(!pivotOut_); }
    /** Put the pivot OUT (the power-on / default position). main.cpp calls this just before
     *  Lift::requestHome() so the arm always homes in its power-on configuration; it is a no-op when
     *  the pivot is already OUT, which at power-on it is. */
    void pivotOut() { setPivot(true); }
    /** Put the pivot BACK (the intake handoff position). */
    void pivotBack() { setPivot(false); }
    /** True when the pivot is OUT (see the class comment). */
    bool isPivotOut() const { return pivotOut_; }
    /** The raw level the pivot's ADI pin is currently driven to (for the brain's CLAW line). */
    bool pivotAdiHigh() const { return pivotLevelFor(pivotOut_); }
    /** Manual claw override (clamp / open). */
    void toggleClaw();

    /** Autoclamp on/off (only acts while the claw is waiting at INTAKE/MATCHLOAD). */
    void setAutoclamp(bool on);
    bool autoclampOn() const { return autoclamp_; }

    /** Run one step. Call every loop (10 ms). */
    void update();

    State getState() const { return state_; }
    /** Current stack level 1..STACK_LEVEL_COUNT, 0 when not stacking. */
    int getLevel() const { return state_ == State::STACKING || state_ == State::CLAMPING ? level_ : 0; }
    /** True for the whole L2 SCORE descent -- RELEASING with scoreToHome_ set -- from the first PRESS
     *  tick through REL/UNSTK/RETURN/SEAT, as distinct from RELEASING via scoreOrMatchload() (lowering
     *  to INTAKE instead) or L2's at-the-bottom clamp toggle, neither of which drop anything. main.cpp
     *  uses this to auto-outtake while the arm is near the bottom (see config::L2_OUTTAKE_BELOW_DEG). */
    bool isScoreDescent() const {
        return (state_ == State::RELEASING && scoreToHome_) || state_ == State::RETURNING;
    }
    /** True while the arm is parked above a finished score, waiting for the second L2. */
    bool isScoredUp() const { return state_ == State::SCORED; }
    /** True while the grab push is running (the intake must not spin -- see periodic::tick()). */
    bool isGrabbing() const { return state_ == State::GRABBING; }
    const char* stateName() const;

  private:
    void enter(State next);
    void setPivot(bool out);
    static bool pivotLevelFor(bool out) { return out == config::PIVOT_OUT_IS_ADI_HIGH; }
    void openClawForPickup();
    /** Open the jaws WITHOUT turning the autoclamp off. With the autoclamp enabled this is
     *  Claw::release() -- the jaws open and the autoclamp re-arms itself only once the sensor reads
     *  clear, so whatever was just let go of is not re-grabbed (see autoclamp_detector.hpp). With it
     *  disabled it is a plain Claw::disarm(). Used everywhere the macro opens the claw to put
     *  something down or to travel empty; the old code called disarm() in those places, which
     *  silently switched the autoclamp off for the rest of the match. */
    void openClaw();
    void moveTo(State destination);
    void command(double degrees, bool precise = false);
    /** True while the arm is inside the handoff zone, INTAKE .. INTAKE + PIVOT_HANDOFF_ZONE_DEG. */
    bool inHandoffZone() const;
    bool pivotDone() const;
    bool liftArrived(double degrees) const;
    double levelHeight() const;
    double heightOf(State s) const;

    Lift& lift_;
    Claw& claw_;
    std::unique_ptr<pros::adi::DigitalOut> pivot_;  // null when there is no pivot actuator
    bool pivotOut_ = true;                          // OUT at power-on, see the class comment
    bool autoclamp_ = false;

    State state_ = State::IDLE;
    State moveDest_ = State::INTAKE_POS;  // where MOVING ends up (IDLE means "go home to DOWN")
    double commanded_ = -1.0;             // last lift target this class sent (-1 = none yet)
    int level_ = 1;
    bool scoreToHome_ = false;            // true while RELEASING on its way to score() rather than
                                           // scoreOrMatchload()'s INTAKE_POS destination
    bool grabRise_ = false;               // GRABBING: rise to a stack level afterwards (L1), or stay
                                           // exactly where the push ended (auton::grab())
    std::uint32_t stateStart_ = 0;
    std::uint32_t pivotMovedAt_ = 0;
};

}  // namespace subsystems
