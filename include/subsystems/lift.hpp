#pragma once
/**
 * Lift — position PID for the DR6-4B.
 *
 * Why PID: a DR6-4B carries more load going up than coming down, and the load
 * changes with what the claw is holding. Driving it with plain voltage makes it
 * overshoot or sag. A PID loop keeps pushing until the arm is at the exact
 * preset height, then holds it there.
 *
 *   output = kG (gravity feedforward)  +  kP*error  +  kI*sum(error)  +  kD*d(error)/dt
 *
 * We reuse lemlib::PID for the P, I and D terms (it already handles the
 * integral windup range and sign-flip reset). This class adds what a lift needs
 * on top: gravity feedforward, an output limit, a hard top stop, a "settled"
 * check, and a manual override for the driver.
 */

#include "lemlib/exitcondition.hpp"
#include "lemlib/pid.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include <memory>

namespace subsystems {

class Lift {
  public:
    Lift();

    /** Zero the lift at its current position. Call with the lift fully down. */
    void zero();

    /** Set a new target height in lift-arm degrees (clamped to 0..MAX). */
    void setTarget(double degrees);
    double getTarget() const { return target_; }

    /** Current lift-arm angle in degrees. */
    double getPosition() const;

    /** True once the lift has stayed within LIFT_SETTLE_DEG of the target for LIFT_SETTLE_MS. */
    bool isSettled() const { return settled_; }

    /**
     * Manual override: -1..1 from the driver. Non-zero input takes over from PID;
     * when the driver lets go, the current height becomes the new PID target so
     * the lift holds where it was left.
     */
    void manual(double input);

    /** Run one control step. Called every 10 ms by the lift task. */
    void update();

    /** Start the background task that calls update() every 10 ms. */
    void startTask();

  private:
    pros::MotorGroup motors_;
    std::unique_ptr<pros::Rotation> rotation_;  // null when no rotation sensor is fitted
    lemlib::PID pid_;
    lemlib::ExitCondition settle_;

    double target_ = 0.0;
    double manualInput_ = 0.0;
    bool wasManual_ = false;
    bool settled_ = false;
    std::unique_ptr<pros::Task> task_;
};

}  // namespace subsystems
