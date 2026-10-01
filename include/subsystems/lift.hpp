#pragma once
/**
 * Lift — motion profile + position PID for the DR6-4B.
 *
 * ONE owner, always: update() is the only place that ever writes to the lift
 * motors, and it is only ever called by the 10 ms background task started in
 * initialize(). There is no second "direct/manual" code path that bypasses it
 * (that split used to exist -- a driver-only MotorGroup and a setDirect()
 * on/off switch -- and it was the actual bug: two things could end up owning
 * the motors at once, and the lift bounced at startup before anyone even
 * pressed a button). Instead, driverPower() just feeds a manual voltage INTO
 * this same update() loop, ahead of the PID, and homing is a MODE inside it, so
 * it is always this one task doing the physical move_voltage()/brake() calls.
 *
 * update() picks exactly one of these every tick, in this order:
 *
 *   1. Driver holding the manual override, D-pad Up/Down (driverPower() nonzero):
 *      full +-LIFT_MAX_MV, PID target cancelled immediately, homing cancelled
 *      immediately, no old preset resumes when they let go.
 *   2. Homing (see "Homing" below), if it has been requested and not finished.
 *   3. A commanded height (setTarget()/moveTo(), from the driver preset buttons,
 *      the intake macro or autonomous): profile + feedforward + PID + guards.
 *   4. Nothing commanded (active_ == false, e.g. fresh boot): brake every tick.
 *      Nothing moves on its own. Letting go of the manual override lands here --
 *      it does NOT snap back to whatever preset was active before.
 *
 * MOTION. Every move, up or down, is a TRAPEZOIDAL PROFILE on the setpoint:
 * accelerate at LIFT_ACCEL_DEG_PER_S2 (from LIFT_START_SPEED_DEG_PER_S, so the
 * very first tick already puts real voltage down and the button feels instant),
 * cruise at LIFT_MAX_SPEED_DEG_PER_S, and decelerate at LIFT_DECEL_DEG_PER_S2 so
 * the setpoint lands on the goal with zero speed. Three things make that both fast
 * and impossible to ring:
 *
 *   - VELOCITY FEEDFORWARD. LIFT_KV mV per arm deg/s of setpoint speed supplies
 *     the voltage the speed needs, so the PID is left correcting a small error
 *     instead of generating the whole drive out of a large one.
 *   - THE LEASH. The setpoint is never allowed more than LIFT_MAX_LEAD_DEG ahead
 *     of the arm. If the arm can't keep up, the setpoint waits. The tracking
 *     error is therefore bounded no matter how far away the goal is, so the PID
 *     can never wind up or store the error that an overshoot is made of.
 *   - LOW GAINS. kP only has to pull in that bounded error (see config.hpp).
 *
 * ARRIVAL. Once the arm is within the goal's tolerance (LIFT_TOLERANCE_DEG, or
 * LIFT_PRECISE_TOLERANCE_DEG for a precise goal) and has stayed there for
 * LIFT_HOLD_MS, the loop STOPS CHASING and holds: gravity feedforward plus a
 * low-gain, low-authority P term (LIFT_HOLD_KP / LIFT_HOLD_MAX_MV). It only goes
 * back to chasing if the arm drifts LIFT_HOLD_RELEASE_DEG beyond the tolerance,
 * so it can't flicker at the band edge. A deliberate trade: a few degrees of
 * final error at a stack level in exchange for a stop with no hunting. INTAKE is
 * the exception -- a precise goal crawls the last LIFT_PRECISE_ZONE_DEG at
 * LIFT_PRECISE_SPEED_DEG_PER_S and keeps the full loop inside a tight band.
 *
 * Going to DOWN, the last LIFT_REST_BAND_DEG is a plain brake-hold: that is the
 * one place the gravity feedforward must not be able to push UP, because an arm
 * sitting on the hard stop with kG under it is what made the lift bounce at
 * startup (17ab865). setDescentFloor() lets Scoring hold the walking setpoint at
 * PIVOT_CLEAR while its pivot retracts, without ever stopping the profile.
 *
 * HOMING. The arm often powers on sitting well above the bottom, and zero() can
 * only seed from the sensor's folded absolute angle, which is unambiguous within
 * +-45 arm deg of the bottom and wrong by a whole sensor revolution past that.
 * So requestHome() asks the lift to find its own hard stop: drive down at a steady
 * LIFT_HOME_MV until the raw sensor stops changing (less than LIFT_HOME_STILL_DEG
 * of SENSOR movement across LIFT_HOME_STILL_MS, checked only after
 * LIFT_HOME_MIN_DRIVE_MS of driving), then -- only if the sensor's absolute angle
 * is within LIFT_HOME_ZERO_WINDOW_DEG of its calibrated zero -- set_position(0)
 * and stop pushing. If it stops somewhere else, or times out, it stops pushing,
 * keeps zero()'s seed and reports HOME FAIL rather than zeroing to a wrong place.
 *
 * Homing is the ONE deliberate exception to "nothing moves at startup", and it is
 * kept narrow on purpose: it never runs from initialize(). main.cpp requests it at
 * the start of the first ENABLED mode (autonomous(), or opcontrol() if auton
 * already did it), so it is the field switch and not power-on that makes the arm
 * move; it runs as a mode inside this task so nothing blocks on it and the
 * drivetrain starts immediately; and the manual override cancels it instantly and
 * for the rest of the run. Targets requested while homing are QUEUED, not dropped,
 * and start the moment homing ends.
 *
 * Runaway guard: if the sensor and the motors disagree on which way is "up"
 * (rotation sensor reversed, or LIFT_PORTS reversed), the loop pushes the wrong
 * way, the gap to the goal grows, and it pushes harder -> the lift drives itself
 * into the bottom. The guard sees "pushing hard in the direction that should close
 * the gap to the goal, and the gap grew anyway", cuts power, and shows LIFT FAULT.
 * It watches the distance to the GOAL rather than the tracking error, because the
 * leash above deliberately keeps the tracking error small and a reversed sensor
 * would otherwise hide behind it. A new explicit target clears the fault (a real
 * wiring fault just re-faults), and so does the manual override.
 *
 * Stall guard: if the lift pushes hard but can't get there (preset above the real
 * top, or jammed), it gives up on that target, holds where it stopped, and shows
 * BLOCKED instead of grinding the motors.
 */

#include "config.hpp"
#include "lemlib/pid.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include <cstdint>
#include <memory>

namespace subsystems {

class Lift {
  public:
    /** What homing is doing. Shown on the brain (see homeStateName()). */
    enum class Home {
        IDLE,      // not requested yet this run
        RUNNING,   // driving down, looking for the hard stop
        DONE,      // found the stop, sensor zeroed there
        FAILED,    // stopped somewhere implausible, or timed out -- seed from zero() kept
        CANCELLED, // driver took manual control mid-home; not retried this run
        SKIPPED,   // no rotation sensor answering, so there is nothing to home against
    };

    Lift();

    /** Seed the position counter from the rotation sensor's own calibrated absolute zero (does NOT
     *  reset/re-zero the sensor itself -- see the long comment in lift.cpp). Call once, at power-on,
     *  from initialize(). It commands no motion. It is only correct within +-45 arm degrees of the
     *  true bottom, which is why requestHome() exists. */
    void zero();

    /** Start the background task that calls update() every 10 ms. Call once from initialize(). */
    void startTask();

    /**
     * Ask the lift to home against its hard stop (see "Homing" in the class comment). Call once at
     * the start of the first ENABLED mode -- autonomous(), and opcontrol() in case there was no
     * auton. Never from initialize(). It returns immediately; homing runs inside the lift task.
     * Calling it again after homing has finished, failed or been cancelled does nothing, so a
     * match's auton -> opcontrol only ever homes once.
     */
    void requestHome();
    /** True while homing is actually driving the arm. */
    bool isHoming() const { return home_ == Home::RUNNING; }
    /** True once homing has finished successfully this run. */
    bool isHomed() const { return home_ == Home::DONE; }
    Home getHomeState() const { return home_; }
    /** "HOMING" / "HOMED" / "HOME FAIL" / "HOME CANCELLED" / "HOME SKIPPED" / "" for the brain. */
    const char* homeStateName() const;
    /** The sensor's absolute angle (signed sensor degrees, folded to +-180) at the moment homing
     *  decided the arm had stopped -- what the plausibility window was checked against. Lets the
     *  user see on the brain why a home passed or failed. 0 until homing stops. */
    double getHomeAngle() const { return homeStopAngleDeg_; }

    // -----------------------------------------------------------------------
    // Shared driver/auton height API. Named heights live in config::lift_preset
    // (include/config.hpp) and both the driver preset buttons (src/main.cpp) and
    // autonomous use the exact same constants.
    // -----------------------------------------------------------------------

    /**
     * Set a new target height in lift-arm degrees (clamped to 0..MAX). Never blocks. Every move is
     * profiled (see the class comment) -- there is no "jump straight there" mode any more, because
     * that is what oscillated.
     *
     * precise = true keeps the full loop running inside LIFT_PRECISE_TOLERANCE_DEG of the goal and
     * crawls the last LIFT_PRECISE_ZONE_DEG, for a height that has to line up (INTAKE). Any goal
     * within half a degree of lift_preset::INTAKE is promoted to precise automatically, so a call
     * site cannot forget. Everything else gets LIFT_TOLERANCE_DEG and a clean stop.
     *
     * Also clears a latched LIFT FAULT: a deliberate new command is the retry (see the runaway guard
     * note in config.hpp). If homing is still running the target is QUEUED and starts when it ends.
     */
    void setTarget(double degrees, bool precise = false);

    /**
     * Auton-friendly wrapper: command a height, optionally blocking the CALLING task (not the lift
     * task, which keeps running regardless) until isSettled() or timeoutMs elapses.
     *   lift.moveTo(config::lift_preset::MATCHLOAD, 1500, true);
     * A blocking call made while homing waits for homing and then the move, so allow time for both.
     * timeoutMs = 0 waits forever (only safe combined with waitUntilSettled == false, or if you
     * trust the stall guard to eventually set isBlocked() -- moveTo() also returns on that).
     */
    void moveTo(double degrees, int timeoutMs = config::LIFT_DEFAULT_TIMEOUT_MS, bool waitUntilSettled = false);

    /** Stop chasing whatever target is active and brake-hold right where the lift is. Does not
     *  resume any previous target later -- same "no old target" rule as releasing the manual override. */
    void cancel();

    /**
     * While descending, don't let the profiled setpoint go below this many arm degrees this tick.
     * For Scoring's pivot interlock: the pivot (which Lift knows nothing about) starts retracting the
     * same instant a score's descent is commanded, not before it, so Scoring raises this to
     * config::lift_preset::PIVOT_CLEAR until its own pivotDone() is true, then drops it back to 0 --
     * a floor on the last bit of an already-moving profile, never a stop-then-resume. Call every tick
     * it should apply; setTarget() always resets it to 0 for a brand-new move, so a stale floor from
     * a past descent can never leak into the next one. 0 (the default) = no floor.
     */
    void setDescentFloor(double degrees) { descentFloor_ = degrees; }

    /** Final target (where the lift is heading). Only meaningful while hasActiveTarget() is true. */
    double getTarget() const { return goal_; }
    /** The setpoint the PID is actually chasing this tick -- the profile's current position, which
     *  walks from where the arm was toward getTarget(). The brain screen shows it as "(sp ...)" so
     *  the goal and the moving setpoint can't be confused for each other. */
    double getSetpoint() const { return target_; }
    /** True while the profile is still walking toward the goal. */
    bool isProfiling() const { return active_ && target_ != goal_; }
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
    /** Raw rotation sensor angle, in SENSOR degrees (not divided by LIFT_ROTATION_RATIO), 0..360,
     *  absolute -- this is Rotation::get_angle() read live, every call (0 if no sensor fitted). For
     *  the brain screen: lets you check the sensor's own calibrated absolute zero by eye. */
    double getRawAngle() const;
    /** The signed sensor-degree angle zero() seeded the position counter from at startup (see its
     *  comment) -- captured once, not live. 0 if zero() hasn't run yet or no sensor is fitted. */
    double getStartupAngle() const { return startupAngleDeg_; }

    /** True once the lift has arrived within the goal's tolerance and is holding there. */
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
     * Driver manual override, D-pad Up/Down: -1..1, full send both directions (same feel as plain
     * motor power -- no feedforward, no clamping beyond the input range). Non-zero input immediately
     * cancels any active PID target AND any homing in progress (see class comment). Call with 0 every
     * loop the driver isn't holding the override.
     */
    void driverPower(double input);

    /** Run one control step. Called every 10 ms by the lift task -- see class comment: this is the
     *  ONLY function that is allowed to write to the lift motors. */
    void update();

  private:
    void resetPid();
    /** One homing step. Called by update() while home_ == Home::RUNNING; owns the motors for that
     *  tick exactly like the other modes do. Sets home_ to DONE/FAILED when it finishes. */
    void updateHoming(std::uint32_t now);
    /** Apply a queued target, if homing was in the way when one was asked for. */
    void startQueuedTarget();
    /** This goal's arrival tolerance -- LIFT_PRECISE_TOLERANCE_DEG for a precise goal, else
     *  LIFT_TOLERANCE_DEG. See the class comment's ARRIVAL paragraph. */
    double tolerance() const {
        return precise_ ? config::LIFT_PRECISE_TOLERANCE_DEG : config::LIFT_TOLERANCE_DEG;
    }
    /** True when the active goal is low enough that Scoring's pivot-retract floor may hold the
     *  setpoint (at or below config::LIFT_BOTTOM_GOAL_DEG, i.e. DOWN or INTAKE). */
    bool headingToBottom() const { return goal_ <= config::LIFT_BOTTOM_GOAL_DEG; }

    pros::MotorGroup motors_;
    std::unique_ptr<pros::Rotation> rotation_;  // null when no rotation sensor is fitted
    lemlib::PID pid_;

    double startupAngleDeg_ = 0.0;  // signed sensor-degree angle zero() seeded position from, see zero()

    bool active_ = false;   // false = no commanded height; update() just brakes (see class comment)
    double goal_ = 0.0;     // final target
    double target_ = 0.0;   // profiled setpoint, walks toward goal_
    double profVel_ = 0.0;  // the profile's current speed, arm deg/s (always >= 0; direction is goal-target)
    bool precise_ = false;  // this goal gets the tight tolerance + slow final approach
    double descentFloor_ = 0.0;  // see setDescentFloor()

    double manualPower_ = 0.0;  // -1..1 from driverPower(), 0 = driver not overriding
    bool wasManual_ = false;

    // homing
    Home home_ = Home::IDLE;
    bool homeRequested_ = false;
    std::uint32_t homeStart_ = 0;       // when this homing attempt began driving
    std::uint32_t homeStillSince_ = 0;  // when the sensor was last seen to move (0 = window not open)
    double homeStillRef_ = 0.0;         // raw sensor degrees at the start of the current still window
    double homeStopAngleDeg_ = 0.0;     // absolute angle at the stop, see getHomeAngle()
    bool queuedTarget_ = false;         // a setTarget() arrived while homing
    double queuedGoal_ = 0.0;
    bool queuedPrecise_ = false;

    // arrival / hold
    bool settled_ = false;
    bool holding_ = false;          // inside tolerance: gentle hold instead of chasing
    std::uint32_t inBandSince_ = 0;  // when the arm first entered tolerance (0 = not in band)
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
    double pushStartGap_ = 0.0;    // |goal - position| when that push started -- faults if it GREW
    int pushDir_ = 0;

    std::unique_ptr<pros::Task> task_;
};

}  // namespace subsystems
