#pragma once
/**
 * Scoring — the "intake macro": owns the lift + claw + pivot sequence so every driver button is
 * one tap and the mechanisms always move in a safe order. Every lift move it makes goes through
 * Lift::setTarget() (the PID), never raw motor power -- see include/subsystems/lift.hpp.
 *
 * Driver buttons (src/main.cpp; D-pad Up/Down are the lift's plain manual override and are NOT
 * part of this macro -- holding either cancels it, see main.cpp's opcontrol()):
 *
 *   IDLE ──R1──> INTAKE_POS ──L1──> CLAMPING ──(jaws shut)──> STACKING (level 1..5)
 *    ^                                                            │  L1 = level+1, stays at 5 (top)
 *    │                                                            L2
 *    │                                                            v
 *    └──────────────────────────────────────────── RELEASING (claw open + pivot retract
 *                                                               + lift descent to DOWN, all at once)
 *
 *   IDLE          power-on state, and also where the manual override leaves it. Nothing moves
 *                 until the driver presses something (or the manual override, which bypasses
 *                 this macro entirely). Resets level back to 1.
 *   INTAKE_POS    lift at INTAKE, claw open (autoclamp armed if it's on), pivot ON (PIVOT_ON_AT_INTAKE)
 *   MATCHLOAD_POS lift at MATCHLOAD, claw open (autoclamp armed if it's on), pivot per PIVOT_ON_AT_MATCHLOAD
 *                 -- not currently wired to a driver button; scoreOrMatchload() still reaches it
 *                 for auton or a future button.
 *   CLAMPING      jaws closing; waits CLAW_CLOSE_TIME_MS so the lift doesn't pull out of an open claw
 *   STACKING      holding a pin, lift at STACK_LEVELS[level-1]; pivot flips out once above PIVOT_CLEAR
 *   RELEASING     L2/score(): claw opens, pivot starts retracting, and the lift's own profiled
 *                 descent to DOWN all start the instant this state is entered -- one continuous
 *                 motion (see Lift's class comment: mid -> high -> slow -> brake), not a
 *                 wait-then-move sequence. Settles into IDLE once the arm actually arrives at 0.
 *                 scoreOrMatchload() (not on a button) still uses the older CLAW_OPEN_TIME_MS wait
 *                 then a move to INTAKE_POS, unchanged.
 *   MOVING        scoreOrMatchload()/goTo*() only now: pivot back in (if it was out), then lift to
 *                 the destination -- INTAKE or MATCHLOAD, see heightOf().
 *
 * Interlocks:
 *   1. The pivot only flips out once the lift is above PIVOT_CLEAR (clears the intake and drivetrain).
 *   2. On the way down the pivot retracts before the lift passes back below PIVOT_CLEAR. For L2's
 *      RELEASING this is enforced live, not with a wait: the pivot starts retracting the same instant
 *      the descent starts, and Lift::setDescentFloor() holds the walking setpoint at PIVOT_CLEAR (if
 *      it would otherwise get there first) until the pivot reports done -- see Lift's class comment.
 *      For MOVING (scoreOrMatchload()) the lift still simply waits for pivotDone() before it moves.
 *   3. The arm never moves away from the goal until the jaws have had time to open -- scoreOrMatchload()
 *      only; L2's score() deliberately starts the descent in the same tick the jaws open, per spec.
 *   4. R1 never pulls a held pin back down to the intake.
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
     *  calls whenever the driver holds the manual override, so this macro can't fight it. */
    void reset();

    /** R1 (new-press): put the claw at the intake handoff (ignored while holding a pin or mid-score). */
    void goToIntake();
    /** L1: first tap clamps and lifts to level 1; every tap after goes up one level, staying at
     *  STACK_LEVEL_COUNT (the top) once reached. */
    void stepUp();
    /** L2: holding a pin -> score (open the jaws, pivot in, then lower all the way to DOWN) and
     *  reset the level count so the next L1 starts at level 1 again. Not holding anything: no-op --
     *  there is no matchload fallback on this button any more, see scoreOrMatchload(). */
    void score();
    /** Not currently wired to a driver button (previously Down/L2 when empty). Holding a pin ->
     *  same as score() but lowers to INTAKE instead of DOWN. Empty -> go to the matchload position.
     *  Left in place for auton or a future button. */
    void scoreOrMatchload();
    /** Not currently wired to a driver button (previously Right/R2). Holding a pin -> back to stack
     *  level 1. Left in place for auton or a future button -- L2's score() already resets the level
     *  after scoring, so this is only useful to back up mid-stack without scoring. */
    void resetLevel();
    /** Driver: flip the pivot in/out. */
    void togglePivot() { setPivot(!pivotOut_); }
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
    void openClawForPickup();
    void moveTo(State destination, double speedDegPerSec);
    void command(double degrees, double speedDegPerSec = 0.0);
    bool pivotDone() const;
    bool liftArrived(double degrees) const;
    double levelHeight() const;
    double heightOf(State s) const;

    Lift& lift_;
    Claw& claw_;
    std::unique_ptr<pros::adi::Pneumatics> pivot_;  // null when there is no pivot actuator
    bool pivotOut_ = false;
    bool autoclamp_ = false;

    State state_ = State::IDLE;
    State moveDest_ = State::INTAKE_POS;  // where MOVING ends up (IDLE means "go home to DOWN")
    double moveSpeed_ = 0.0;
    double commanded_ = -1.0;             // last lift target this class sent (-1 = none yet)
    int level_ = 1;
    bool scoreToHome_ = false;            // true while RELEASING on its way to score() rather than
                                           // scoreOrMatchload()'s INTAKE_POS destination
    std::uint32_t stateStart_ = 0;
    std::uint32_t pivotMovedAt_ = 0;
};

}  // namespace subsystems
