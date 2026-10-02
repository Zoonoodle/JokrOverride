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
 * MOTION. There are exactly two kinds of commanded move plus one manual
 * input, all decided every tick from the arm's measured position and velocity.
 * Every number they use is in the "LIFT TUNING -- start here" block at the top
 * of config.hpp; README "Tuning the lift" is the order to turn them in.
 *
 * A POSITION MOVE -- every goal except a trip to the bottom -- is
 * DRIVE -> BRAKE -> APPROACH -> HOLD:
 *
 *   DRIVE     full +-LIFT_MAX_MV toward the goal, the exact thing the D-pad
 *             does, from the very first tick, until the arm is up to the speed
 *             the profile allows. A move is still instant off the line.
 *   BRAKE     the velocity loop (LIFT_VEL_KP / LIFT_VEL_KI, clamped to
 *             LIFT_VEL_MAX_MV) tracks
 *               min(cruise cap, sqrt(2 * LIFT_DECEL_DEG_PER_S2 * (dist - zone)))
 *             where the cruise cap is LIFT_UP_SPEED_PCT / LIFT_DOWN_SPEED_PCT of
 *             the free speed. Computed from DISTANCE, not time, so an arm ahead
 *             of or behind the profile is corrected automatically.
 *   APPROACH  inside LIFT_APPROACH_ZONE_DEG the same loop tracks
 *             LIFT_APPROACH_KP_PER_S * (degrees left), capped at
 *             LIFT_APPROACH_DEG_PER_S -- a crawl that tapers to nothing at the
 *             goal, so the arm is practically stopped before the hold takes it.
 *             This is what stops the six-bar springing on arrival.
 *   HOLD      a DAMPED position hold, not the firmware's brake-hold:
 *               LIFT_HOLD_KP * (error outside the deadband)
 *               + integral(LIFT_HOLD_KI * error)      <- learns gravity
 *               - LIFT_HOLD_KD * velocity             <- absorbs the arrival
 *             clamped to LIFT_HOLD_MAX_MV. The integral is kept between moves,
 *             so the next hold starts already knowing roughly what this arm
 *             costs to hold up, and enterBrake() seeds the velocity loop from
 *             it too. Nothing re-drives and nothing re-chases: the hold IS the
 *             correction, and it is gentle.
 *
 * A DESCENT -- a non-precise goal at or below LIFT_BOTTOM_GOAL_DEG, i.e. L2, B,
 * auton DOWN and R2 in clamped mode -- is DESCEND -> SEAT:
 *
 *   DESCEND   the velocity loop tracks a banded speed reference read off the
 *             arm's position: LIFT_DESCENT_TOP_PCT of free speed from the start
 *             height down to the mid boundary, LIFT_DESCENT_MID_PCT through the
 *             middle, then a constant LIFT_DESCENT_SLOW_PCT crawl from
 *             LIFT_DESCENT_SLOW_ZONE_DEG to the stop. The mid boundary is a
 *             FRACTION (LIFT_DESCENT_MID_FRACTION) of the travel between the
 *             start height and the slow zone, so "the middle" is the middle of
 *             this descent whatever height it began at. Band changes are ramps
 *             at LIFT_DESCENT_RAMP_DEG_PER_S2, never stops.
 *   SEAT      the arm has gone still (under LIFT_SEAT_STILL_DEG_PER_S for
 *             LIFT_SEAT_STILL_MS) below LIFT_SEAT_DETECT_DEG, i.e. it is
 *             physically on its hard stop: hold a constant LIFT_SEAT_MV pressed
 *             into it so it cannot bounce back up. It does NOT park a few
 *             degrees above the frame any more. If a lift motor ever reaches
 *             LIFT_SEAT_TEMP_LIMIT_C the seat drops to a plain brake-hold.
 *
 * SCORING WITH L2 is a third kind of move, started by startScore() and run as
 * phases PRESS -> RELEASE -> (UNSTICK) -> the DESCENT above. It is the only
 * part of the lift that needs anything from another subsystem, and the split is:
 * the lift decides WHEN the claw must open (it is the only thing that can -- it
 * owns the velocity estimate and the commanded speed), and Scoring, which owns
 * the claw, actually opens it. The lift never touches a cylinder.
 *
 *   PRESS     descend at L2_SCORE_SPEED_PCT of free speed with the claw still
 *             SHUT, pushing the pin down onto the stack, with the downward push
 *             capped at L2_PRESS_MAX_MV so the loop cannot crush the tower while
 *             the arm slows. Watch the rotation sensor for CONTACT: the measured
 *             speed falling below L2_CONTACT_SPEED_FRAC of the commanded speed
 *             (only once the arm has actually reached L2_CONTACT_ARM_FRAC of it,
 *             so a spin-up can never count), or an absolute near-stop, held for
 *             L2_CONTACT_MS. Reaching L2_NO_CONTACT_DEG with no contact means
 *             there was no stack under the arm: nothing is released, and it slows
 *             into the bottom and SEATs with the pin still held (NO CONTACT).
 *   RELEASE   contact found: needsClawRelease() goes true for Scoring to act on,
 *             and the arm KEEPS pressing down at the same speed so the jaws slide
 *             off the pin. Once the jaws have had CLAW_OPEN_TIME_MS to open, the
 *             same detector decides: speed recovered for L2_FREE_MS -> free, go
 *             home; still stalled -> the claw is hooked on the pin -> UNSTICK.
 *   UNSTICK   lift L2_UNSTICK_UP_DEG with the claw open, then press again. At
 *             most L2_MAX_UNSTICK_TRIES times; after that it stops, holds where
 *             it is and reports SCORE STUCK rather than grinding.
 *   then      the ordinary DESCENT, with L2_RETURN_SPEED_PCT as both of its fast
 *             bands, so the run home is quick but still slows and seats.
 *
 * THE MANUAL INPUT (driverPower(): D-pad Up/Down at full send, and R1 at
 * R_LIFT_UP_PCT while a pin is clamped) is plain voltage fed into this same
 * loop ahead of everything else, and it cancels a score sequence at any stage.
 * R2 in clamped mode is not manual at all -- descendManual() runs the DESCENT
 * above with R_LIFT_DOWN_PCT as its top band, so holding R2 to the bottom seats
 * the arm exactly like L2 does.
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
 * GUARDS. Runaway: during DRIVE the distance to the goal can only shrink, and
 * during DESCEND the arm can only go down; if either reverses by LIFT_FAULT_DEG
 * the sensor or the motors are reversed -> cut power, LIFT FAULT. Stall: a tick
 * whose speed reference is a real one (at least 5 deg/s) but the arm moved less
 * than LIFT_STALL_DEG in LIFT_STALL_MS -> give up on that goal, HOLD where it
 * is, BLOCKED. The stall guard is deliberately off once APPROACH has tapered
 * the reference below that, in HOLD, and below LIFT_SEAT_DETECT_DEG during a
 * descent -- an arm that has stopped down there is seated, which is the point.
 * Neither guard can brick the buttons: a new setTarget() clears both.
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

    /** Where a commanded move is (see the class comment). DRIVE/BRAKE/APPROACH/HOLD are a position
     *  move; DESCEND/SEAT are a trip to the bottom. For a position move the first three are chosen
     *  fresh from (distance, speed) every tick -- only arrival into HOLD latches. */
    enum class Phase { DRIVE, BRAKE, APPROACH, HOLD, DESCEND, SEAT, PRESS, RELEASE, UNSTICK };

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
     * Set a new target height in lift-arm degrees (clamped to 0..MAX). Never blocks. The move runs as
     * described in the class comment, at full voltage from the first tick.
     *
     * precise = true gives the goal the tight LIFT_PRECISE_TOLERANCE_DEG band, for a height that has
     * to line up (INTAKE). Any goal within half a degree of lift_preset::INTAKE is promoted to precise
     * automatically, so a call site cannot forget. Everything else gets LIFT_TOLERANCE_DEG. A goal
     * already inside its band is arrived at once, without moving.
     *
     * A NON-PRECISE goal at or below config::LIFT_BOTTOM_GOAL_DEG is not a position move at all: it
     * runs the banded DESCEND -> SEAT trip to the bottom (see the class comment), which ends pressed
     * onto the hard stop rather than parked above it.
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
    /** "DRIVE" / "BRAKE" / "APPROACH" / "HOLD" / "DESCEND" / "SEAT", or "" with no active target. */
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

    /** True once the lift has arrived on the active goal -- holding a height (HOLD), or seated on the
     *  hard stop at the bottom (SEAT), which is what arrival at DOWN means now. */
    bool isSettled() const { return active_ && (phase_ == Phase::HOLD || phase_ == Phase::SEAT) && !blocked_; }
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
    /** The signed speed reference the velocity loop is tracking this tick, deg/s (+ = up). 0 in HOLD
     *  and SEAT, which are not speed-controlled. */
    double getSpeedRef() const { return lastVRef_; }
    /** The hold loop's learned gravity voltage, mV -- what it currently reckons this arm costs to hold
     *  where it is. Carried between moves. */
    double getGravityMv() const { return gravityMv_; }
    /** True while the arm is pressed down onto its hard stop (phase SEAT). */
    bool isSeated() const { return phase_ == Phase::SEAT; }

    // -----------------------------------------------------------------------
    // L2 scoring (see "SCORING WITH L2" in the class comment). Scoring owns the
    // claw; the lift owns the motion and decides when the claw has to open.
    // -----------------------------------------------------------------------
    /** L2 with a pin held above the bottom: start the press / sense / release / go-home sequence.
     *  Never blocks. Cancelled instantly by driverPower(), cancel(), or any new setTarget(). */
    void startScore();
    /** True while a score sequence is running (PRESS / RELEASE / UNSTICK). */
    bool isScoring() const { return score_; }
    /** TRUE WHEN THE LIFT NEEDS THE JAWS OPENED, right now: the pin has landed on the stack. The
     *  owner of the claw (Scoring) polls this every tick, opens the claw, and calls
     *  clawReleaseDone(). The lift keeps pushing down meanwhile. */
    bool needsClawRelease() const { return clawReleaseWanted_ && !clawReleased_; }
    /** Acknowledge needsClawRelease(): the jaws have been commanded open as of now. The lift waits
     *  config::CLAW_OPEN_TIME_MS from this moment before judging whether the claw came free. */
    void clawReleaseDone();
    /** The score sequence gave up: the claw is still caught after L2_MAX_UNSTICK_TRIES. The lift is
     *  holding where it stopped and will not push again. Cleared by the next command. */
    bool isScoreStuck() const { return scoreStuck_; }
    /** The press reached the bottom without ever finding a stack. Nothing was released -- the pin is
     *  still clamped and the arm is seated. Cleared by the next command. */
    bool hadNoContact() const { return noContact_; }
    /** measured / commanded arm speed during PRESS and RELEASE, 0..1+ -- the number contact detection
     *  is actually made on. 0 outside those phases. Also the trace's `aux` column, as a percentage. */
    double getSpeedRatio() const { return lastRatio_; }

    /**
     * Driver manual override: -1..1 of LIFT_MAX_MV, plain motor power, no feedforward. The D-pad uses
     * +-1.0 (full send); R1 in clamped mode uses +R_LIFT_UP_PCT / 100. Non-zero input immediately
     * cancels any active target AND any homing in progress (see class comment), and letting go
     * brake-holds where it is -- no old target resumes. Call with 0 every loop nothing is holding it.
     */
    void driverPower(double input);

    /**
     * R2 in clamped mode: a controlled trip to the bottom that ends SEATED on the hard stop. Pass
     * true every loop the button is held and false the moment it is let go. It is the same banded
     * DESCEND -> SEAT as L2/B (see the class comment) with R_LIFT_DOWN_PCT as its top band, so it is
     * velocity-controlled on the measured arm speed and gravity cannot run it away.
     *
     * Letting go part-way brake-holds where it is. Letting go once the arm is SEATED leaves it seated
     * (the DOWN target stays active in SEAT), so holding R2 all the way down ends exactly like L2.
     * driverPower() -- the D-pad -- still preempts it, as does any new setTarget().
     */
    void descendManual(bool held);

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
    void enterBrake(std::uint32_t now);
    void enterHold(std::uint32_t now, double holdAt);
    void enterDescend(std::uint32_t now, double position, double topPct, double midPct);
    void enterSeat(std::uint32_t now);
    /** Hand a running score sequence over to the trip home (freed, or nothing was ever there). */
    void endScore(std::uint32_t now, double position, double topPct, double midPct);
    /** One tick of PRESS / RELEASE / UNSTICK. Writes the motors and traces. */
    void updateScore(std::uint32_t now, double dt, double position);
    /** Stall guard window: true once the arm has made less than LIFT_STALL_DEG of progress for
     *  LIFT_STALL_MS. Resets itself whenever the arm does move. Only ever called with a speed
     *  reference that really wants motion -- see the guard notes in config.hpp. */
    bool stalled(std::uint32_t now, double position);
    /** The whole speed profile of a position move, as an UNSIGNED deg/s: the cruise cap, the braking
     *  curve sqrt(2 a d), and the final-approach taper, whichever is smallest at this distance. */
    double speedProfile(double dist, double dir) const;
    /** One tick of the velocity loop: pushes the motors toward `vRef` (signed deg/s) and returns the
     *  mV it sent. The only place LIFT_VEL_KP/KI are used. */
    /** Limits are asymmetric because L2's press needs a tight DOWNWARD cap (the crush limit) while
     *  still being allowed full upward authority to hold the arm back against gravity. */
    double runVelocityLoop(double vRef, double dt, double upMaxMv, double downMaxMv);
    /** One tick of the damped position hold (LIFT_HOLD_KP/KI/KD). Writes the motors and traces. */
    void runHold(std::uint32_t now, double dt, double position);
    /** One tick of a trip to the bottom: DESCEND's bands and ramp, the seat detector, and SEAT's
     *  constant pressure. Writes the motors and traces. */
    void updateDescent(std::uint32_t now, double dt, double position);
    /** The banded descent's speed reference for this position, as an UNSIGNED deg/s. */
    double descentSpeed(double position) const;
    /** This goal's arrival band: LIFT_PRECISE_TOLERANCE_DEG (precise) else LIFT_TOLERANCE_DEG. */
    double arrivalTolerance() const;

    // trace ring (single producer: the lift task; single consumer: the trace task)
    struct TraceSample {
        std::uint32_t ms;
        char phase;
        std::int16_t pos10, goal10, vel, mv, aux;
    };
    static constexpr std::size_t kTraceSize = 128;  // power of two
    /** aux is a per-phase extra: during PRESS/RELEASE/UNSTICK it is the measured/commanded speed
     *  ratio in percent (the number contact detection is made on), 0 everywhere else. */
    void trace(std::uint32_t now, char phase, double position, double mv, bool moving, int aux = 0);
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
    bool precise_ = false;  // tight tolerance (INTAKE)
    bool descent_ = false;  // a trip to the bottom: the banded DESCEND -> SEAT, not a position move
    Phase phase_ = Phase::HOLD;
    std::uint32_t phaseStart_ = 0;
    double minDist_ = 0.0;  // smallest distance-to-goal seen during this DRIVE (runaway guard)
    double velI_ = 0.0;     // velocity loop integral, mV
    double holdGoal_ = 0.0; // the height HOLD is holding: the goal on a normal arrival, or wherever the
                            // arm stopped if the move was BLOCKED (so a jammed arm is never pushed into
                            // whatever it is caught on).
    double gravityMv_ = 0.0;  // the hold loop's integral: the learned voltage this arm costs to hold.
                              // Kept between moves on purpose, and used to seed velI_ in enterBrake().

    // the banded descent (DESCEND / SEAT)
    double descentTopPct_ = 0.0;    // top-band speed, % of free speed (L2/B vs R2 differ)
    double descentMidPct_ = 0.0;    // middle-band speed, % of free speed
    double descentMidDeg_ = 0.0;    // position at which the top band becomes the middle band
    double descentStart_ = 0.0;     // height the descent began at (runaway guard + the mid boundary)
    double descentRef_ = 0.0;       // ramped speed reference, unsigned deg/s
    std::uint32_t seatStillSince_ = 0;  // when the arm was last seen moving near the bottom
    bool seatHot_ = false;          // a lift motor passed LIFT_SEAT_TEMP_LIMIT_C: brake-hold instead of pressing
    int seatTempTick_ = 0;          // temperature is only read every 50th SEAT tick

    // L2 scoring
    bool score_ = false;                 // a press/sense/release sequence is running
    bool returnFast_ = false;            // this DESCEND is a post-score run home (for the phase name)
    bool scoreStuck_ = false;            // gave up: claw still caught after every unstick try
    bool noContact_ = false;             // pressed to the bottom and never found a stack
    bool clawReleaseWanted_ = false;     // the lift is asking Scoring to open the jaws
    bool clawReleased_ = false;          // Scoring acknowledged (clawReleaseDone())
    std::uint32_t clawReleasedAt_ = 0;
    bool contactArmed_ = false;          // the arm reached L2_CONTACT_ARM_FRAC of the press speed once
    std::uint32_t contactSince_ = 0;     // when the speed first dropped below threshold (0 = above it)
    std::uint32_t freeSince_ = 0;        // when the speed first recovered after the release
    double pressRef_ = 0.0;              // ramped press/unstick speed reference, unsigned deg/s
    double unstickGoal_ = 0.0;           // height the unstick hop is climbing to
    int unstickTries_ = 0;
    double lastRatio_ = 0.0;             // measured / commanded, see getSpeedRatio()

    double manualPower_ = 0.0;  // -1..1 from driverPower(), 0 = driver not overriding
    bool wasManual_ = false;
    bool descendHeld_ = false;  // R2 held, from descendManual()
    bool descendOwned_ = false; // this descent was started by descendManual(), so releasing R2 ends it
    bool descendRequest_ = false;  // one-shot: the next setTarget() came from descendManual()

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
