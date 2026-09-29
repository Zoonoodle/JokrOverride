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
 * check, a speed-limited move (for lowering slowly after a score), a manual
 * override for the driver, and a runaway guard.
 *
 * Runaway guard: if the sensor and the motors disagree on which way is "up"
 * (rotation sensor reversed, or LIFT_PORTS reversed), the PID pushes the wrong
 * way, the error grows, and it pushes harder -> the lift drives itself into the
 * bottom. The guard sees "pushing hard one way, moving the other way", cuts the
 * power, and shows LIFT FAULT on the brain. Manual Up/Down still works.
 */

#include "lemlib/exitcondition.hpp"
#include "lemlib/pid.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include <cstdint>
#include <memory>

namespace subsystems {

class Lift {
  public:
    Lift();

    /** Zero the lift at its current position. Call with the lift fully down. Clears a fault. */
    void zero();

    /**
     * Set a new target height in lift-arm degrees (clamped to 0..MAX).
     * speedDegPerSec > 0 moves the setpoint there at that speed (smooth, slow moves);
     * 0 jumps straight to it (fastest the PID allows).
     */
    void setTarget(double degrees, double speedDegPerSec = 0.0);
    /** Final target (where the lift is heading). */
    double getTarget() const { return goal_; }

    /** Current lift-arm angle in degrees (rotation sensor if fitted, else motor encoders). */
    double getPosition() const;
    /** Lift degrees from the motor encoders only. Used to check the rotation sensor's direction. */
    double getMotorPosition() const;
    /** Lift degrees from the rotation sensor only (0 if none fitted). */
    double getSensorPosition() const;

    /** True once the lift has reached its final target and stayed in band for LIFT_SETTLE_MS. */
    bool isSettled() const { return settled_; }
    /** Hottest lift motor, deg C. */
    double getTemperature() const;

    /** True if the runaway guard tripped. */
    bool isFaulted() const { return faulted_; }

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

    double goal_ = 0.0;     // final target
    double target_ = 0.0;   // current setpoint (walks toward goal_ when speed_ > 0)
    double speed_ = 0.0;    // deg/s, 0 = jump
    double manualInput_ = 0.0;
    bool wasManual_ = false;
    bool settled_ = false;
    std::uint32_t lastUpdate_ = 0;

    // runaway guard
    bool faulted_ = false;
    std::uint32_t pushStart_ = 0;  // when the output first went over LIFT_FAULT_MV (0 = not pushing)
    double pushStartPos_ = 0.0;
    int pushDir_ = 0;

    std::unique_ptr<pros::Task> task_;
};

}  // namespace subsystems
