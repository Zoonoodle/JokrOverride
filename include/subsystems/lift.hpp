#pragma once
/**
 * Lift — DRIVE -> BRAKE -> HOLD position control for the DR6-4B.
 *
 * ONE owner, always: update() is the only place that ever writes to the lift
 * motors, and it is only ever called by the 10 ms background task started in
 * initialize(). There is no second "direct/manual" code path that bypasses it
 * (that split used to exist -- a driver-only MotorGroup and a setDirect()
 * on/off switch -- and it was the actual bug: two things could end up owning
 * the motors at once, and the lift bounced at startup before anyone even
 * pressed a button). Instead, driverPower() just feeds a manual voltage INTO
 * this same update() loop, ahead of everything else, and homing is a MODE
 * inside it, so it is always this one task doing the physical
 * move_voltage()/brake() calls.
 *
 * update() picks exactly one of these every tick, in this order:
 *
 *   1. Driver holding the manual override, D-pad Up/Down (driverPower() nonzero):
 *      full +-LIFT_MAX_MV, any target cancelled immediately, homing cancelled
 *      immediately, no old preset resumes when they let go.
 *   2. Homing (see "Homing" below), if it has been requested and not finished.
 *   3. A commanded height (setTarget()/moveTo(), from the driver preset buttons,
 *      the intake macro or autonomous): the DRIVE -> BRAKE -> HOLD machine below.
 *   4. Nothing commanded (active_ == false, e.g. fresh boot): brake every tick.
 *      Nothing moves on its own. Letting go of the manual override lands here --
 *      it does NOT snap back to whatever preset was active before.
 *
 * MOTION. A preset move is three phases, decided every tick from the arm's
 * measured position and velocity (position differenced each tick, lightly
 * filtered). There is no position PID, no setpoint profile and no gravity
 * feedforward:
 *
 *   DRIVE  full +-LIFT_MAX_MV toward the goal -- the exact thing the D-pad does,
 *          from the very first tick -- until the arm is moving as fast as the
 *          braking curve allows for the distance that is left:
 *          v_curve = sqrt(2 * LIFT_BRAKE_DECEL_DEG_PER_S2 * (dist - LIFT_BRAKE_MARGIN_DEG)).
 *   BRAKE  a velocity loop (LIFT_VEL_KP / LIFT_VEL_KI, clamped to LIFT_BRAKE_MAX_MV)
 *          that follows that curve down. Its integral learns whatever voltage
 *          gravity needs at this angle, so nothing has to know the six-bar's
 *          torque curve in advance. For a normal goal the reference never points
 *          back the way the arm came: a small overshoot is simply stopped, never
 *          chased. Once the arm is slower than LIFT_STOPPED_DEG_PER_S and inside
 *          LIFT_TOLERANCE_DEG (or has passed the goal), -> HOLD.
 *          INTAKE (precise) and DOWN use the same loop with a floor speed: the
 *          curve is offset by LIFT_CREEP_ZONE_DEG so the arm is down to
 *          LIFT_CREEP_DEG_PER_S that far out, then CREEPs until it is inside
 *          LIFT_PRECISE_TOLERANCE_DEG (INTAKE) or LIFT_REST_BAND_DEG above the
 *          hard stop (DOWN) -> HOLD. The creep may creep back if it overshoots.
 *   HOLD   motors_.brake() every tick with MotorBrake::hold -- the firmware's own
 *          position hold, the same one that holds the arm when the D-pad is let
 *          go. Nothing in this phase pushes toward the goal, so nothing can hunt.
 *          Arrival latches: HOLD only goes back to DRIVE if the arm is more than
 *          LIFT_REENGAGE_DEG from the goal, and at most LIFT_MAX_REDRIVES times
 *          per command. The number of pushes per button press is finite.
 *
 * The one number that decides where the arm stops is LIFT_BRAKE_DECEL_DEG_PER_S2
 * (under the real capability = stops short, over = overshoots). Read it off the
 * trace (LIFT_TRACE) rather than guessing.
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
 * GUARDS. Runaway: during DRIVE the distance to the goal can only shrink; if it
 * grows by LIFT_FAULT_DEG the sensor or the motors are reversed -> cut power,
 * LIFT FAULT. Stall: a phase that wants motion (DRIVE, CREEP) but the arm moved
 * less than LIFT_STALL_DEG in LIFT_STALL_MS -> give up on that goal, HOLD where
 * it is, BLOCKED. Neither is checked in BRAKE or HOLD, which are stopping on
 * purpose, and neither can brick the buttons: a new setTarget() clears both.
 *
 * TRACE. With config::LIFT_TRACE on, one compact integer line per 20 ms goes to
 * the PROS terminal while the arm moves (and for LIFT_TRACE_TAIL_MS after), from
 * a separate low-priority task that drains a ring buffer -- this control loop
 * never calls printf.
 */

#include "config.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#include <cstddef>
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

    /** Where a commanded move is (see the class comment). CREEP is BRAKE with the floor speed
     *  active; phaseName() tells them apart for the brain and the trace. */
    enum class Phase { DRIVE, BRAKE, HOLD };

    Lift();

    /** Seed the position counter from the rotation sensor's own calibrated absolute zero (does NOT
     *  reset/re-zero the sensor itself -- see the long comment in lift.cpp). Call once, at power-on,
     *  from initialize(). It commands no motion. It is only correct within +-45 arm degrees of the
     *  true bottom, which is why requestHome() exists. */
    void zero();

    /** Start the background task that calls update() every 10 ms (and, if config::LIFT_TRACE, the
     *  trace task that prints the ring buffer). Call once from initialize(). */
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
     * Set a new target height in lift-arm degrees (clamped to 0..MAX). Never blocks. The move runs
     * DRIVE -> BRAKE -> HOLD as described in the class comment, at full voltage from the first tick.
     *
     * precise = true gives the goal the tight LIFT_PRECISE_TOLERANCE_DEG band and the slow CREEP
     * final approach, for a height that has to line up (INTAKE). Any goal within half a degree of
     * lift_preset::INTAKE is promoted to precise automatically, so a call site cannot forget.
     * Everything else gets LIFT_TOLERANCE_DEG and a plain stop. A goal already inside its band is
     * arrived at once, without moving.
     *
     * Also clears a latched LIFT FAULT or BLOCKED: a deliberate new command is the retry (see the
     * guard notes in config.hpp). If homing is still running the target is QUEUED and starts when it
     * ends.
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

    /** Final target (where the lift is heading). Only meaningful while hasActiveTarget() is true. */
    double getTarget() const { return goal_; }
    /** False when there is no commanded height (fresh boot, just cancelled, or driver manual). */
    bool hasActiveTarget() const { return active_; }
    /** Name of the config::lift_preset height the current target matches ("custom" if it doesn't
     *  match any of them, "none" if there is no active target). For the brain screen. */
    const char* activePresetName() const;
    /** Which phase the active move is in. Only meaningful while hasActiveTarget() is true. */
    Phase getPhase() const { return phase_; }
    /** "DRIVE" / "BRAKE" / "CREEP" / "HOLD", or "" with no active target. */
    const char* phaseName() const;

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
    /** Arm velocity, deg/s (+ = up), as update() measured it this tick. Runs in every mode. */
    double getVelocity() const { return velocity_; }

    /** True once the lift has arrived (HOLD) on the active goal. */
    bool isSettled() const { return active_ && phase_ == Phase::HOLD && !blocked_; }
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
    /** goal - position, lift degrees (0 with no active target). */
    double getError() const { return lastError_; }
    /** What update() last sent to the motors, mV (0 when it braked). */
    double getOutput() const { return lastOutput_; }
    /** The speed the braking curve allows at the current distance, deg/s (what BRAKE is tracking). */
    double getBrakeCurve() const { return lastVRef_; }
    /** How many times this command has re-driven from HOLD (see LIFT_REENGAGE_DEG). */
    int getRedrives() const { return redrives_; }

    /**
     * Driver manual override, D-pad Up/Down: -1..1, full send both directions (same feel as plain
     * motor power -- no feedforward, no clamping beyond the input range). Non-zero input immediately
     * cancels any active target AND any homing in progress (see class comment). Call with 0 every
     * loop the driver isn't holding the override.
     */
    void driverPower(double input);

    /** Run one control step. Called every 10 ms by the lift task -- see class comment: this is the
     *  ONLY function that is allowed to write to the lift motors. */
    void update();

  private:
    /** One homing step. Called by update() while home_ == Home::RUNNING; owns the motors for that
     *  tick exactly like the other modes do. Sets home_ to DONE/FAILED when it finishes. */
    void updateHoming(std::uint32_t now);
    /** Apply a queued target, if homing was in the way when one was asked for. */
    void startQueuedTarget();

    void enterDrive(std::uint32_t now, double position);
    void enterBrake(std::uint32_t now, bool fromDrive);
    void enterHold(std::uint32_t now);
    /** Stall guard window: true once the arm has made less than LIFT_STALL_DEG of progress for
     *  LIFT_STALL_MS. Resets itself whenever the arm does move. */
    bool stalled(std::uint32_t now, double position);
    /** The braking curve: fastest the arm may be going at `dist` deg from the goal and still stop in
     *  time (zero speed at LIFT_BRAKE_MARGIN_DEG before it, or at LIFT_CREEP_ZONE_DEG for a creep goal). */
    double brakeCurve(double dist) const;
    /** This goal's arrival band: LIFT_PRECISE_TOLERANCE_DEG (precise), LIFT_REST_BAND_DEG (a trip to
     *  the bottom), else LIFT_TOLERANCE_DEG. */
    double arrivalTolerance() const;

    // trace ring (single producer: the lift task; single consumer: the trace task)
    struct TraceSample {
        std::uint32_t ms;
        char phase;
        std::int16_t pos10, goal10, vel, mv;
    };
    static constexpr std::size_t kTraceSize = 128;  // power of two
    void trace(std::uint32_t now, char phase, double position, double mv, bool moving);
    void traceDrain();

    pros::MotorGroup motors_;
    std::unique_ptr<pros::Rotation> rotation_;  // null when no rotation sensor is fitted

    double startupAngleDeg_ = 0.0;  // signed sensor-degree angle zero() seeded position from, see zero()

    // velocity estimate, every tick, every mode
    double velocity_ = 0.0;      // arm deg/s, + = up
    double prevPosition_ = 0.0;
    bool havePrevPosition_ = false;
    std::uint32_t lastUpdate_ = 0;

    // the active command
    bool active_ = false;   // false = no commanded height; update() just brakes (see class comment)
    double goal_ = 0.0;     // final target, arm deg
    bool precise_ = false;  // tight tolerance + creep (INTAKE)
    bool creep_ = false;    // precise, or a trip to the bottom: floor the braking curve at the creep speed
    Phase phase_ = Phase::HOLD;
    std::uint32_t phaseStart_ = 0;
    double dir0_ = 1.0;     // which way DRIVE set off (+1 up / -1 down); a normal goal never reverses it
    double minDist_ = 0.0;  // smallest distance-to-goal seen during this DRIVE (runaway guard)
    double velI_ = 0.0;     // velocity loop integral, mV
    int redrives_ = 0;

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

    // tuning telemetry, set every tick by update()
    double lastError_ = 0.0, lastOutput_ = 0.0, lastVRef_ = 0.0;

    // stall guard
    bool blocked_ = false;
    std::uint32_t stallStart_ = 0;
    double stallStartPos_ = 0.0;

    // runaway guard
    bool faulted_ = false;

    // trace
    TraceSample trace_[kTraceSize] = {};
    std::atomic<std::uint32_t> traceHead_{0}, traceTail_{0};
    std::uint32_t traceUntil_ = 0;
    int traceTick_ = 0;

    std::unique_ptr<pros::Task> task_;
    std::unique_ptr<pros::Task> traceTask_;
};

}  // namespace subsystems
