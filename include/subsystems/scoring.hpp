#pragma once
/**
 * Scoring — the "intake macro": owns the lift + claw + pivot sequence so every driver button is
 * one tap and the mechanisms always move in a safe order. Every lift move it makes goes through
 * Lift::setTarget() (the PID), never raw motor power -- see include/subsystems/lift.hpp.
 *
 * Driver buttons (src/main.cpp; L1/L2 are the lift's plain manual override and are NOT part of
 * this macro -- pressing either cancels it, see main.cpp's opcontrol()):
 *
 *   IDLE ──Up───> INTAKE_POS ──Left──> CLAMPING ──(jaws shut)──> STACKING (level 1..7)
 *    │               ^                                             │  Left = level+1, Right = level 1
 *    └──Down─> MATCHLOAD_POS ──Left──> (same CLAMPING path)        │
 *                    ^                                             Down
 *                    │                                             v
 *                    └── Down (no pin) ── MOVING <── (jaws open) RELEASING
 *                                         │  pivot in first, then lift down
 *                                         └──> INTAKE_POS (after a score, lowered slowly)
 *
 *   IDLE          power-on state, and also where L1/L2 leave it. Nothing moves until the driver
 *                 presses something (or L1/L2, which bypass this macro entirely).
 *   INTAKE_POS    lift at INTAKE, claw open (autoclamp armed if it's on), pivot ON (PIVOT_ON_AT_INTAKE)
 *   MATCHLOAD_POS lift at MATCHLOAD, claw open (autoclamp armed if it's on), pivot per PIVOT_ON_AT_MATCHLOAD
 *   CLAMPING      jaws closing; waits CLAW_CLOSE_TIME_MS so the lift doesn't pull out of an open claw
 *   STACKING      holding a pin, lift at STACK_LEVELS[level-1]; pivot flips out once above PIVOT_CLEAR
 *   RELEASING     jaws opening; waits CLAW_OPEN_TIME_MS before the arm moves
 *   MOVING        pivot back in (if it was out), then lift to the destination (INTAKE or MATCHLOAD)
 *
 * Interlocks:
 *   1. The pivot only flips out once the lift is above PIVOT_CLEAR (clears the intake and drivetrain).
 *   2. On the way down the pivot flips back in BEFORE the lift moves.
 *   3. The arm never moves away from the goal until the jaws have had time to open.
 *   4. Up never pulls a held pin back down to the intake.
 *   5. L1/L2 always win: main.cpp calls reset() whenever either is held, so this macro can never
 *      re-command the lift out from under a manual nudge (it does not touch the claw or pivot,
 *      which stay physically where they were).
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

    /** Back to IDLE: nothing moves, lift holds where it is. Also what main.cpp calls whenever the
     *  driver holds L1/L2, so this macro can't fight the manual override. */
    void reset();

    /** Up: put the claw at the intake handoff (ignored while holding a pin or mid-score). */
    void goToIntake();
    /** Left: first tap clamps and lifts to level 1; every tap after goes up one level (max 7). */
    void stepUp();
    /** Down: holding a pin -> score (open, then lower slowly). Empty -> go to the matchload position. */
    void scoreOrMatchload();
    /** Right: holding a pin -> back to stack level 1. */
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
    /** Current stack level 1..7, 0 when not stacking. */
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
    State moveDest_ = State::INTAKE_POS;  // where MOVING ends up
    double moveSpeed_ = 0.0;
    double commanded_ = -1.0;             // last lift target this class sent (-1 = none yet)
    int level_ = 1;
    std::uint32_t stateStart_ = 0;
    std::uint32_t pivotMovedAt_ = 0;
};

}  // namespace subsystems
