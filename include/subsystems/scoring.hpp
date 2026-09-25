#pragma once
/**
 * Scoring — the "pivot" from the intake state to the scoring state.
 *
 * One button press should take the robot from "pin in the claw at the intake"
 * to "pin over the goal", with every mechanism moving in a safe order. This
 * state machine owns that order so the driver can't, for example, open the
 * claw while the lift is still swinging.
 *
 *   INTAKING ──(claw clamps)──> HOLDING ──(score pressed)──> PIVOTING ──(lift settled)──> SCORING
 *      ^                                                                                   │
 *      └───────────────(lift back at INTAKE)──── RETURNING <──(release done / cancel)──────┘
 *
 *   INTAKING   lift at INTAKE, claw ARMED (autoclamp on), intake rollers IN
 *   HOLDING    claw CLAMPED, lift raised to CARRY, rollers stopped
 *   PIVOTING   lift driving to the chosen score height; pivot flips once the lift clears CARRY
 *   SCORING    lift settled at the goal; waiting for the driver to drop the pin
 *   RETURNING  claw opening, pivot back, then lift down to INTAKE
 *
 * Interlocks (the reason this exists):
 *   1. No pivot unless the claw is CLAMPED  (don't swing an empty claw into the goal)
 *   2. The pivot only flips once the lift is above CARRY  (clears the intake and drivetrain)
 *   3. The claw only opens once the lift is SETTLED at the score height (the pin lands on the goal)
 *   4. On return the pivot flips back BEFORE the lift drops below CARRY
 *
 * ASSUMPTION (TODO(team): confirm): the notebook does not describe a separate
 * pivot/wrist yet. If config::PIVOT_PISTON_PORT is ' ', the "pivot" is the DR6-4B
 * itself moving from the intake handoff to the score height, and the pivot steps
 * are skipped. If the claw gets its own flip cylinder, set the port and the same
 * sequence drives it.
 */

#include "pros/adi.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/lift.hpp"

#include <cstdint>
#include <memory>

namespace subsystems {

class Scoring {
  public:
    enum class State { INTAKING, HOLDING, PIVOTING, SCORING, RETURNING };

    Scoring(Lift& lift, Claw& claw, Intake& intake);

    /** Start (or restart) the cycle at INTAKING. */
    void begin();
    /** Driver: go score at this height. Ignored unless HOLDING (interlock 1). */
    void requestScore(double liftDegrees);
    /** Driver: drop the pin. Ignored unless SCORING and settled (interlock 3). */
    void release();
    /** Driver: abort and go back to intaking from any state. */
    void cancel();

    /** Run one step. Call every loop (10 ms). */
    void update();

    State getState() const { return state_; }
    const char* stateName() const;

  private:
    void enter(State next);
    void setPivot(bool scoring);

    Lift& lift_;
    Claw& claw_;
    Intake& intake_;
    std::unique_ptr<pros::adi::Pneumatics> pivot_;  // null when there is no pivot actuator
    bool pivotOut_ = false;

    State state_ = State::INTAKING;
    double scoreTarget_ = 0.0;
    std::uint32_t stateStart_ = 0;
    std::uint32_t pivotMovedAt_ = 0;
};

}  // namespace subsystems
