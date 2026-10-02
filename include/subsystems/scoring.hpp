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
 *   RELEASING     L2/score(): the jaws open the instant this state is entered, then the arm stands
 *                 completely still for config::L2_RELEASE_WAIT_MS (it is still holding its stack
 *                 level, so this costs nothing) so the pin is free and settled in the cup before the
 *                 end effector moves away from it. Only then is DOWN commanded -- which is the lift's
 *                 banded, seating descent (see lift.hpp), not a position move. The pivot is already
 *                 OUT at every stack level and OUT is the default down position, so it does not move
 *                 (if the driver had toggled it BACK up there, it is put back OUT, without waiting).
 *                 Settles into IDLE once the arm is actually SEATED on its hard stop.
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
 *   3. The arm never moves away from the goal until the jaws have had time to open: CLAW_OPEN_TIME_MS
 *      for scoreOrMatchload(), L2_RELEASE_WAIT_MS for L2's score().
 *   4. R1 never pulls a held pin back down to the intake: it is a no-op while the claw is CLAMPED
 *      (shut by L1, A or the autoclamp). This claw's unpowered position is OPEN, so power-on is never
 *      clamped and R1 works straight away. Starting a match with a pin in the claw: press A or L1
 *      first, because the claw needs air to grip it at all.
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
    enum class State { IDLE, INTAKE_POS, MATCHLOAD_POS, CLAMPING, STACKING, RELEASING, MOVING };

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
    /** L1: first tap clamps and lifts to level 1; every tap after goes up one level, staying at
     *  STACK_LEVEL_COUNT (the top) once reached. */
    void stepUp();
    /** L2: holding a pin -> score (open the jaws and lower all the way to DOWN in one motion, pivot
     *  OUT) and reset the level count so the next L1 starts at level 1 again. Not holding anything:
     *  no-op -- there is no matchload fallback on this button any more, see scoreOrMatchload(). */
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
    const char* stateName() const;

  private:
    void enter(State next);
    void setPivot(bool out);
    static bool pivotLevelFor(bool out) { return out == config::PIVOT_OUT_IS_ADI_HIGH; }
    void openClawForPickup();
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
    std::uint32_t stateStart_ = 0;
    std::uint32_t pivotMovedAt_ = 0;
};

}  // namespace subsystems
