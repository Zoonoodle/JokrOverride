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
 * ONE owner, always: update() is the only place that ever writes to the lift
 * motors, and it is only ever called by the 10 ms background task started in
 * initialize(). There is no second "direct/manual" code path that bypasses it
 * (that split used to exist -- a driver-only MotorGroup and a setDirect()
 * on/off switch -- and it was the actual bug: two things could end up owning
 * the motors at once, and the lift bounced at startup before anyone even
 * pressed a button). Instead, driverPower() just feeds a manual voltage INTO
 * this same update() loop, ahead of the PID, so it is always this one task
 * doing the physical motors_.move_voltage()/brake() calls.
 *
 *   - No target commanded yet (active_ == false, e.g. fresh boot): update()
 *     just brakes every tick. Nothing moves on its own.
 *   - Driver holding L1/L2 (driverPower() nonzero): full manual voltage, PID
 *     target cancelled immediately, no old preset resumes.
 *   - Driver just let go: back to the "no target" brake-hold above. It does
 *     NOT snap back to whatever preset was active before -- the driver has to
 *     ask for a height again (setTarget()/moveTo()) if they want the PID back.
 *   - A height was commanded (setTarget()/moveTo(), from driver preset
 *     buttons or auton): normal PID + gravity feedforward + guards.
 *
 * Runaway guard: if the sensor and the motors disagree on which way is "up"
 * (rotation sensor reversed, or LIFT_PORTS reversed), the PID pushes the wrong
 * way, the error grows, and it pushes harder -> the lift drives itself into the
 * bottom. The guard sees "pushing hard one way, moving the other way", cuts the
 * power, and shows LIFT FAULT on the brain. Manual L1/L2 still works and clears it.
 *
 * Stall guard: if the lift pushes hard but can't get there (preset above the
 * real top, or jammed), it gives up on that target, holds where it stopped,
 * and shows BLOCKED instead of grinding the motors.
 */

#include "config.hpp"
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

    /** Zero the lift at its current position. Call once, with the lift fully down, at power-on. */
    void zero();

    /** Start the background task that calls update() every 10 ms. Call once from initialize(). */
    void startTask();

    // -----------------------------------------------------------------------
    // Shared driver/auton height API. Named heights live in config::lift_preset
    // (include/config.hpp) and both the driver preset buttons (src/main.cpp) and
    // autonomous use the exact same constants.
    // -----------------------------------------------------------------------

    /**
     * Set a new target height in lift-arm degrees (clamped to 0..MAX). This is the low-level
     * primitive: it never blocks. speedDegPerSec > 0 moves the setpoint there at that speed
     * (smooth, slow moves, e.g. lowering after a score); 0 jumps straight to it (fastest the PID
     * allows). Commanding a target makes the PID active (see class comment).
     */
    void setTarget(double degrees, double speedDegPerSec = 0.0);

    /**
     * Auton-friendly wrapper: jump to a height, optionally blocking the CALLING task (not the lift
     * task, which keeps running regardless) until isSettled() or timeoutMs elapses.
     *   lift.moveTo(config::lift_preset::MATCHLOAD, 1500, true);
     * timeoutMs = 0 waits forever (only safe combined with waitUntilSettled == false, or if you
     * trust the stall guard to eventually set isBlocked() -- moveTo() also returns on that).
     */
    void moveTo(double degrees, int timeoutMs = config::LIFT_DEFAULT_TIMEOUT_MS, bool waitUntilSettled = false);

    /** Stop chasing whatever target is active and brake-hold right where the lift is. Does not
     *  resume any previous target later -- same "no old target" rule as releasing L1/L2. */
    void cancel();

    /** Final target (where the lift is heading). Only meaningful while hasActiveTarget() is true. */
    double getTarget() const { return goal_; }
    /** False when the PID has no commanded height (fresh boot, just cancelled, or driver manual). */
    bool hasActiveTarget() const { return active_; }
    /** Name of the config::lift_preset height the current target matches ("custom" if it doesn't
     *  match any of them, "none" if there is no active target). For the brain screen. */
    const char* activePresetName() const;

    /** Current lift-arm angle in degrees (rotation sensor if fitted, else motor encoders). */
    double getPosition() const;
    /** Lift degrees from the motor encoders only. Used to check the rotation sensor's direction. */
    double getMotorPosition() const;
    /** Lift degrees from the rotation sensor only (0 if none fitted). */
    double getSensorPosition() const;

    /** True once the lift has reached its final target and stayed in band for LIFT_SETTLE_MS. */
    bool isSettled() const { return settled_; }
    /** True if the last target couldn't be reached (stalled) and the lift is holding where it stopped. */
    bool isBlocked() const { return blocked_; }
    /** True if the runaway guard tripped. */
    bool isFaulted() const { return faulted_; }

    /** Hottest lift motor, deg C. */
    double getTemperature() const;
    /** Highest lift motor current draw, mA. */
    double getCurrentDraw() const;

    // -----------------------------------------------------------------------
    // Tuning telemetry (brain screen only, see src/screen.cpp). Cheap reads:
    // everything below just returns a value update() already computed this tick.
    // -----------------------------------------------------------------------
    double getError() const { return lastError_; }
    double getOutput() const { return lastOutput_; }
    double getPTerm() const { return lastP_; }
    double getITerm() const { return lastI_; }
    double getDTerm() const { return lastD_; }

    /**
     * Driver manual override for L1/L2: -1..1, full send both directions (same feel as plain
     * motor power -- no feedforward, no clamping beyond the input range). Non-zero input
     * immediately cancels any active PID target (see class comment). Call with 0 every loop the
     * driver isn't holding L1/L2.
     */
    void driverPower(double input);

    /** Run one control step. Called every 10 ms by the lift task -- see class comment: this is the
     *  ONLY function that is allowed to write to the lift motors. */
    void update();

  private:
    void resetPid();

    pros::MotorGroup motors_;
    std::unique_ptr<pros::Rotation> rotation_;  // null when no rotation sensor is fitted
    lemlib::PID pid_;
    lemlib::ExitCondition settle_;

    bool active_ = false;   // false = no commanded height; update() just brakes (see class comment)
    double goal_ = 0.0;     // final target
    double target_ = 0.0;   // current setpoint (walks toward goal_ when speed_ > 0)
    double speed_ = 0.0;    // deg/s, 0 = jump

    double manualPower_ = 0.0;  // -1..1 from driverPower(), 0 = driver not overriding
    bool wasManual_ = false;

    bool settled_ = false;
    std::uint32_t lastUpdate_ = 0;

    // tuning telemetry, set every tick by update()
    double lastError_ = 0.0, lastOutput_ = 0.0, lastP_ = 0.0, lastI_ = 0.0, lastD_ = 0.0;
    double shadowPrevError_ = 0.0;  // kept in lockstep with pid_'s own prevError, see update()

    // stall guard
    bool blocked_ = false;
    std::uint32_t stallStart_ = 0;
    double stallStartPos_ = 0.0;

    // runaway guard
    bool faulted_ = false;
    std::uint32_t pushStart_ = 0;  // when the output first went over LIFT_FAULT_MV (0 = not pushing)
    double pushStartPos_ = 0.0;
    int pushDir_ = 0;

    std::unique_ptr<pros::Task> task_;
};

}  // namespace subsystems
