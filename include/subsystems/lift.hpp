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
 *   HOLD      a DAMPED position hold, not the firmware's brake-hold -- and only
 *             for as long as it takes to settle (see PARK below):
 *               LIFT_HOLD_KP * (error outside the deadband)
 *               + integral(LIFT_HOLD_KI * error)      <- learns gravity
 *               - LIFT_HOLD_KD * velocity             <- absorbs the arrival
 *             clamped to LIFT_HOLD_MAX_MV. The integral is kept between moves,
 *             so the next hold starts already knowing roughly what this arm
 *             costs to hold up, and enterBrake() seeds the velocity loop from
 *             it too. Nothing re-drives and nothing re-chases: the hold IS the
 *             correction, and it is gentle. Inside holdDeadband() the P term is
 *             silent AND the integral is FROZEN -- an integral that keeps
 *             winding against stiction while P says nothing is a limit cycle,
 *             which is what "still oscillating a good amount" was.
 *   PARK      then it stops correcting altogether. Once the arm has been inside
 *             the settle band and slow for LIFT_PARK_CONFIRM_MS, or the hold has
 *             simply run LIFT_SETTLE_TIMEOUT_MS, PARK freezes the learned
 *             gravity voltage and pushes exactly that -- no P, no D, no integral
 *             update, no feedback of any kind, so it physically cannot hunt. The
 *             hold only wakes again if the arm drifts LIFT_REENGAGE_DEG away
 *             (and not within LIFT_PARK_COOLDOWN_MS, so the two cannot
 *             ping-pong). A position move that has not arrived within
 *             LIFT_MOVE_TIMEOUT_MS also parks, where it is, as PARK(t/o): it
 *             stops chasing something it cannot reach. (Driver's request,
 *             2026-10-03: "id rather it just settle instead of keeping on trying
 *             to do it. neds a timeout.")
 *
 * A DESCENT -- a non-precise goal at or below LIFT_BOTTOM_GOAL_DEG, i.e. L2, B,
 * auton DOWN and R2 in clamped mode -- is DESCEND -> FINAL -> SEAT:
 *
 *   DESCEND   the velocity loop tracks a banded speed reference read off the
 *             arm's position: LIFT_DESCENT_TOP_PCT of free speed from the start
 *             height down to the mid boundary, LIFT_DESCENT_MID_PCT through the
 *             middle, then a constant LIFT_DESCENT_SLOW_PCT "approach" cruise
 *             from LIFT_DESCENT_SLOW_ZONE_DEG down to LIFT_FINAL_START_DEG. The
 *             mid boundary is a FRACTION (LIFT_DESCENT_MID_FRACTION) of the
 *             travel between the start height and the slow zone, so "the
 *             middle" is the middle of this descent whatever height it began
 *             at. Band changes are ramps at LIFT_DESCENT_RAMP_DEG_PER_S2, never
 *             stops -- sized (see config.hpp) so the approach band is already
 *             down near its own target speed well before FINAL takes over, not
 *             still braking hard right at the floor.
 *   FINAL     below LIFT_FINAL_START_DEG: a dedicated closed-loop push at a
 *             constant LIFT_FINAL_DEG_PER_S, with up to LIFT_FINAL_MAX_MV of
 *             downward authority -- more force than SEAT's hold (which only has
 *             to fight bounce, not resistance) but capped well under DESCEND's
 *             full 12000 mV so a real hard-stop contact doesn't slam. This is
 *             the fix for an arm that stalls a few degrees above 0 on a cup
 *             caught in the intake or plain six-bar/chain friction: the
 *             original single-step version read that stall as "seated" and
 *             left only the weak SEAT voltage to finish the job, which it often
 *             couldn't. FINAL is TIME-BASED: it forces for a fixed
 *             LIFT_FINAL_FORCE_MS regardless of how settled it looks partway
 *             through (a stall can look momentarily still without being truly
 *             seated), then hands over to SEAT -- plain SEAT if it is within
 *             LIFT_SEAT_ZERO_DEG of 0 at that moment, SEAT(short) otherwise (a
 *             genuine jam, not just friction) so the driver can tell it did not
 *             actually reach 0.
 *   SEAT      a constant LIFT_SEAT_MV pressed into the stop so the arm cannot
 *             bounce back up. It does NOT park a few degrees above the frame
 *             (that was FINAL's whole point). If a lift motor ever reaches
 *             LIFT_SEAT_TEMP_LIMIT_C the seat drops to a plain brake-hold.
 *   REST      and then the motors are let go entirely. Once the seated arm has
 *             been still for LIFT_REST_AFTER_MS, REST sets the brake mode to
 *             COAST and commands 0 V: no PID, no seat push, not even the
 *             firmware hold (which is a position loop and does draw current).
 *             The frame holds the arm; the motors draw nothing for the rest of
 *             the match. (Driver's request, 2026-10-03: "keep motors loose so
 *             they arent being usefd and burn out.") REST counts as seated and
 *             settled everywhere. COASTING HAPPENS ONLY HERE: the no-target
 *             state and every manual release are still brake-holds, so an arm
 *             that is up cannot fall. Because this lift's bands pull UP hardest
 *             at the folded position, a resting arm that creeps past
 *             LIFT_REST_WAKE_DEG is re-seated (FINAL, SEAT, REST again), at most
 *             LIFT_REST_MAX_WAKES times before it just stays SEATed.
 *             LIFT_REST_AFTER_MS = 0 disables REST completely.
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
 *             the arm slows. CONTACT is decided by subsystems::ContactDetector
 *             (its own header, pure and host-tested): a leaky integral of the
 *             POSITION SHORTFALL against a learned baseline, the loop's EFFORT
 *             against a learned baseline, and a speed-ratio / near-stop backstop
 *             -- all measured against what the arm was actually doing a moment
 *             ago, never against the nominal command, and (2026-10-02) at least
 *             two of those three signals must agree on the same tick before any
 *             of them (other than the absolute near-stop) counts at all -- see
 *             AGREEMENT in contact_detector.hpp, the fix for this six-bar's
 *             gravity torque swinging the loop's speed/effort fast enough, as
 *             the arm passes vertical, to false-fire a single signal in plain
 *             free air. GUARANTEED RELEASE: if the detector has not fired once the
 *             arm has travelled L2_AUTO_RELEASE_DEG DOWN FROM scoreStartPos_ (the
 *             position when startScore() ran, fixed for the whole sequence -- a
 *             RELATIVE travel budget, not an absolute reading, so every score gets
 *             the same chance to be detected whatever height it starts from), it
 *             releases anyway, right there, as REL(32) -- there is no longer a
 *             "press all the way down with nothing released" outcome. A start too
 *             low for that much travel to ever happen before the floor (level 1,
 *             31.0) instead falls back to releasing at L2_RELEASE_FLOOR_DEG, as
 *             REL(floor) -- see that constant in config.hpp for why it is shared
 *             with RELEASE's own safety net below.
 *   RELEASE   contact found (by the detector, or forced by the travel/floor rules
 *             above): needsClawRelease() goes true for Scoring to act on IN THE
 *             SAME TICK the trigger fires, and the arm KEEPS pressing down at the
 *             same speed so the jaws slide off the pin. Once the jaws have had
 *             CLAW_OPEN_TIME_MS to open, a plain speed test decides (with the
 *             jaws open the arm either moves or it does not): clearly moving for
 *             L2_FREE_MS -> free, go home; still stopped for L2_STUCK_MS -> the
 *             claw is hooked on the pin -> UNSTICK. If the arm reaches
 *             L2_RELEASE_FLOOR_DEG before either resolves (and the claw-open
 *             request has had a chance to land, see clawReleased_), it hands over
 *             to the fast run-home descent right there rather than keep pressing.
 *   UNSTICK   lift L2_UNSTICK_UP_DEG with the claw open, then press again. At
 *             most L2_MAX_UNSTICK_TRIES times; after that it stops, holds where
 *             it is and reports SCORE STUCK rather than grinding.
 *   POSTPUSH  the jaws are open and the arm is free: keep pressing down for
 *             L2_POST_RELEASE_PUSH_MS so they slide clear of the pin.
 *   then      RISE L2_CLEAR_RISE_DEG above wherever it stopped and HOLD there,
 *             latched, indefinitely -- brain: SCORED UP. PRESS 1 ENDS HERE.
 *             There is no automatic descent to the bottom any more: the driver
 *             can drive away with the claw above the pins.
 *   PRESS 2   startScoreReturn(): the ordinary DESCENT with L2_RETURN_SPEED_PCT
 *             as both fast bands, then FINAL and SEAT at 0 -- quick, but it
 *             still slows and seats. The held-above position is a deliberate,
 *             driver-requested stop that this press resolves; it is NOT the
 *             banned "parks above the bottom instead of seating" bug, which was
 *             a trip to the bottom that never seated and had no second step.
 *
 * THE GRAB (startGrab(), L1 at the intake handoff and auton::grab()) is a
 * fourth kind of move: a constant open-loop downward push at L1_GRAB_PUSH_PCT
 * that forces the claw onto the pin, asks Scoring to shut the jaws
 * L1_GRAB_CLOSE_AT_MS in (needsClawGrip()), keeps pressing until both
 * L1_GRAB_PUSH_MS and the jaws' own CLAW_CLOSE_TIME_MS are satisfied, and then
 * HOLDS wherever it ended. The rise afterwards is an ordinary position move
 * commanded by Scoring, so auton::grab() can simply not command one and stay.
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
 * the reference below that, in HOLD, and below LIFT_FINAL_START_DEG during a
 * descent -- an arm that has stopped down there is seated, which is the point.
 * Neither guard can brick the buttons: a new setTarget() clears both.
 *
 * TRACE. With config::LIFT_TRACE on, one compact integer line per 20 ms goes to
 * the PROS terminal while the arm moves (and for LIFT_TRACE_TAIL_MS after), from
 * a separate low-priority task that drains a ring buffer -- this control loop
 * never calls printf.
 */

#include "config.hpp"
#include "subsystems/contact_detector.hpp"
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
     *  move; DESCEND/FINAL/SEAT are a trip to the bottom (FINAL is the dedicated, time-boxed
     *  push-to-zero step below LIFT_FINAL_START_DEG, see "Seating on the hard stop" in config.hpp). For
     *  a position move the first three are chosen fresh from (distance, speed) every tick -- only
     *  arrival into HOLD latches. */
    enum class Phase {
        DRIVE, BRAKE, APPROACH, HOLD, DESCEND, FINAL, SEAT, PRESS, RELEASE, UNSTICK,
        GRAB,      // the L1/auton grab: a constant downward push while the jaws close (see THE GRAB)
        POSTPUSH,  // L2 press 1: still pressing down after the jaws were told to open
        PARK,      // settled: a CONSTANT voltage (the learned gravity term, frozen). No loop at all,
                   // so it cannot hunt. See "Settling instead of hunting" in config.hpp.
        REST,      // settled AT THE BOTTOM: brake mode COAST and 0 V -- the motors are loose.
    };

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
     * A COARSE move to a height: speed over precision, overshoot allowed (see "A COARSE move" in
     * config.hpp). Brakes much later (LIFT_COARSE_DECEL_DEG_PER_S2), keeps a quick tail instead of the
     * final crawl, and treats LIFT_COARSE_TOLERANCE_DEG as arrived -- so it gets there noticeably
     * faster than an ordinary rise and may sail a few degrees past. For a height that only has to be
     * TOUCHED, which today means the front toggle. Never blocks; identical to setTarget() in every
     * other way, including the guards, the hold and PARK.
     */
    void setTargetFast(double degrees);

    /**
     * Auton-friendly wrapper: command a height, optionally blocking the CALLING task (not the lift
     * task, which keeps running regardless) until isSettled() or timeoutMs elapses.
     *   lift.moveTo(config::lift_preset::MATCHLOAD, 1500, true);
     * A blocking call made while homing waits for homing and then the move, so allow time for both.
     * timeoutMs = 0 waits forever (only safe combined with waitUntilSettled == false, or if you
     * trust the stall guard to eventually set isBlocked() -- moveTo() also returns on that).
     */
    void moveTo(double degrees, int timeoutMs = config::LIFT_DEFAULT_TIMEOUT_MS, bool waitUntilSettled = false);

    /**
     * THE FAST TRIP TO THE BOTTOM (the front toggle's way back down; auton::liftDownFast()). Exactly
     * the banded DESCEND -> FINAL -> SEAT -> REST of setTarget(DOWN), but with LIFT_FAST_DESCENT_*
     * for the two FAST bands only -- the approach cruise, FINAL and the seat are unchanged, so it
     * still slows near 0 and settles the same way. Never blocks.
     *
     * QUEUED BEHIND A TOGGLE RISE: if a move to lift_preset::FRONT_TOGGLE is still climbing when this
     * is called, the descent WAITS until the arm is within LIFT_FRONT_TOGGLE_REACH_DEG of that height
     * (or LIFT_FRONT_TOGGLE_MAX_WAIT_MS runs out) and then starts immediately -- the arm touches the
     * toggle height and turns straight around rather than settling on it. The route is never blocked;
     * the waiting happens inside this task. Brain: `TOGGLE ^`.
     */
    void descendFast();

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
    /** "DRIVE" / "BRAKE" / "APPROACH" / "HOLD" / "DESCEND" / "FINAL" / "SEAT" (or "SEAT(short)" if
     *  FINAL's forcing window, LIFT_FINAL_FORCE_MS, ran out before reaching true 0), or "" with no
     *  active target. */
    const char* phaseName() const;

    /** Current lift-arm angle in degrees (rotation sensor if fitted, else motor encoders). */
    double getPosition() const;
    /** Lift degrees from the motor encoders only. Used to check the rotation sensor's direction. */
    double getMotorPosition() const;
    /** Lift degrees from the rotation sensor only (0 if none fitted). */
    double getSensorPosition() const;
    /** Arm velocity, deg/s (+ = up), as update() measured it this tick. Runs in every mode. */
    double getVelocity() const { return velocity_; }

    /** True once the lift has arrived on the active goal -- holding a height (HOLD), or seated on the
     *  hard stop at the bottom (SEAT), which is what arrival at DOWN means now. */
    bool isSettled() const {
        return active_ && !blocked_ &&
               (phase_ == Phase::HOLD || phase_ == Phase::SEAT || phase_ == Phase::PARK ||
                phase_ == Phase::REST);
    }
    /** True if the last target couldn't be reached (stalled) and the lift is holding where it stopped. */
    bool isBlocked() const { return blocked_; }
    /** True if the runaway guard tripped. */
    bool isFaulted() const { return faulted_; }

    /** Hottest lift motor, deg C. The seat's own temperature guard uses this; it is no longer on the
     *  brain screen (every live number moved to the LIFT_TRACE terminal output -- see screen.hpp). */
    double getTemperature() const;

    /** True while the arm is pressed down onto its hard stop (phase SEAT) -- including a SEAT(short)
     *  that ended FINAL's forcing window above true 0 (see isSeatShort()). It is still "done trying"
     *  either way, which is what callers that just want "stop waiting" (e.g. main.cpp's L2 auto-
     *  outtake) should react to. */
    bool isSeated() const { return phase_ == Phase::SEAT || phase_ == Phase::REST; }
    /** True while the lift is PARKED: holding a height on a frozen constant voltage with no feedback
     *  (see "Settling instead of hunting" in config.hpp). It is settled; it is just not correcting. */
    bool isParked() const { return phase_ == Phase::PARK; }
    /** True while the motors are RESTING: coasting at the bottom, drawing nothing. */
    bool isResting() const { return phase_ == Phase::REST; }
    /** True if the current/last SEAT ended FINAL's forcing window (LIFT_FINAL_FORCE_MS) above
     *  LIFT_SEAT_ZERO_DEG of 0 -- shown as SEAT(short) on the brain. Meaningless unless isSeated() is
     *  also true. */
    bool isSeatShort() const { return seatShort_; }

    // -----------------------------------------------------------------------
    // L2 scoring (see "SCORING WITH L2" in the class comment). Scoring owns the
    // claw; the lift owns the motion and decides when the claw has to open.
    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------
    // THE GRAB (L1 at the intake handoff, and auton::grab()). A constant
    // downward push at L1_GRAB_PUSH_PCT while the jaws close on the pin, so the
    // claw forces its way down onto it instead of closing on air. Timing and
    // the force are config::L1_GRAB_* (see config.hpp). It is a mode inside
    // update(), so this one task still owns the motors; D-pad manual cancels it
    // instantly, and it is refused while homing (the caller falls back to the
    // ordinary close-then-rise).
    // -----------------------------------------------------------------------
    /** Start the push. Never blocks. Ends by HOLDING wherever the push finished -- whoever wants the
     *  arm to rise afterwards (Scoring's L1 path) commands a height then; auton::grab() just stays. */
    void startGrab();
    /** True while the push is running. */
    bool isGrabbing() const { return grab_; }
    /** TRUE WHEN THE JAWS MUST CLOSE, right now (L1_GRAB_CLOSE_AT_MS into the push). The owner of the
     *  claw (Scoring) polls this, clamps, and calls clawGripDone(). The arm keeps pressing down
     *  meanwhile, and will not stop pressing until the jaws have had CLAW_CLOSE_TIME_MS. */
    bool needsClawGrip() const { return grab_ && gripWanted_ && !gripAcked_; }
    /** Acknowledge needsClawGrip(): the jaws have been commanded shut as of now. */
    void clawGripDone();

    /** L2 PRESS 1, with a pin held above the bottom: start the press / sense / release / post-push /
     *  rise-and-hold sequence (see "SCORING WITH L2"). Never blocks. Cancelled instantly by
     *  driverPower(), cancel(), or any new setTarget(). */
    void startScore();
    /** L2 PRESS 2: the fast run home from the held-above position -- the ordinary banded descent at
     *  L2_RETURN_SPEED_PCT that ends SEATED on the hard stop. Never blocks. */
    void startScoreReturn();
    /** True while the arm is holding ABOVE a finished score (press 1 done, press 2 not pressed yet) --
     *  the state the brain shows as SCORED UP. Cleared by any new command, the manual override, or
     *  press 2. */
    bool isScoredAbove() const { return scoredAbove_; }
    /** True for the WHOLE two-press score cycle: from press 1 until press 2 has seated the arm (or
     *  anything else takes the lift over). The claw's autoclamp is inhibited while this is true, so
     *  the pin that was just released cannot be re-grabbed on the way down. */
    bool isScoreCycle() const { return scoreCycle_; }
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
    // What the contact detector sees tick by tick -- the speed ratio, the accumulated shortfall and the
    // extra downward push -- is in the LIFT_TRACE terminal output's last four columns (see the trace
    // legend in startTask()), which is where a 20 ms signal belongs. It is no longer on the screen.

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
    /** Hand DESCEND over to FINAL (see the class comment): the dedicated, time-boxed closed-loop push
     *  through the last stretch to true 0, entered once the arm crosses LIFT_FINAL_START_DEG. */
    void enterFinal(std::uint32_t now);
    /** shortSeat = true marks a SEAT where FINAL's forcing window (LIFT_FINAL_FORCE_MS) ended with the
     *  arm still above LIFT_SEAT_ZERO_DEG (SEAT(short) on the brain); false (the default) is every
     *  other caller -- homing finding the real hard stop, and FINAL ending within LIFT_SEAT_ZERO_DEG. */
    void enterSeat(std::uint32_t now, bool shortSeat = false);
    /** Settled: freeze the hold's learned gravity voltage and push exactly that, forever, with no
     *  feedback of any kind. timedOut = true marks a move that was abandoned rather than arrived at
     *  (PARK(t/o) on the brain). */
    void enterPark(std::uint32_t now, double parkAt, bool timedOut);
    /** Settled at the bottom: brake mode COAST, 0 V, motors loose. */
    void enterRest(std::uint32_t now);
    /** One tick of PARK. Returns true if it is still parked (and has written the motors); false if the
     *  arm had drifted past LIFT_REENGAGE_DEG, in which case it has re-entered HOLD and the caller
     *  must run that instead. */
    bool updatePark(std::uint32_t now, double position);
    /** Put the brake mode back to hold if REST had set it to coast. Called before anything else writes
     *  the motors, so coasting can never leak out of REST. */
    void wakeMotors();
    /** One tick of PRESS / RELEASE / POSTPUSH / UNSTICK. Writes the motors and traces. */
    void updateScore(std::uint32_t now, double dt, double position);
    /** One tick of the grab push. Writes the motors and traces. */
    void updateGrab(std::uint32_t now, double dt, double position);
    /** End of L2 press 1: rise L2_CLEAR_RISE_DEG from here and latch a HOLD there. */
    void endScoreClear(std::uint32_t now, double position);
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
    /** This goal's settle band: LIFT_HOLD_PRECISE_DEADBAND_DEG (precise) else LIFT_HOLD_DEADBAND_DEG. */
    double holdDeadband() const;

    // trace ring (single producer: the lift task; single consumer: the trace task)
    struct TraceSample {
        std::uint32_t ms;
        char phase;
        std::int16_t pos10, goal10, vel, mv, ratio, lag10, eff, trg;
    };
    static constexpr std::size_t kTraceSize = 128;  // power of two
    /** The four score-only columns: what the contact detector saw this tick. All 0 outside PRESS /
     *  RELEASE / UNSTICK, so a normal move's trace lines end in four zeros. Deliberately no default
     *  member initializers: a nested class's NSDMIs cannot be used in a default argument inside the
     *  enclosing class, and `TraceAux{}` zero-initializes every member anyway. */
    struct TraceAux {
        int ratioPct;   // measured / baseline speed, percent
        int lag10;      // accumulated position shortfall, arm deg x10
        int effortMv;   // extra downward push over the free-descent baseline
        int trigger;    // 0 none, 1 stall, 2 lag, 3 effort, 4 speed, 5 travel(32), 6 floor (latched once fired)
    };
    void trace(std::uint32_t now, char phase, double position, double mv, bool moving, const TraceAux& aux = TraceAux{});
    void traceDrain();

    pros::MotorGroup motors_;
    std::unique_ptr<pros::Rotation> rotation_;  // null when no rotation sensor is fitted

    // velocity estimate, every tick, every mode
    double velocity_ = 0.0;      // arm deg/s, + = up
    double prevPosition_ = 0.0;
    bool havePrevPosition_ = false;
    std::uint32_t lastUpdate_ = 0;

    // the active command
    bool active_ = false;   // false = no commanded height; update() just brakes (see class comment)
    double goal_ = 0.0;     // final target, arm deg
    bool precise_ = false;  // tight tolerance (INTAKE)
    bool coarse_ = false;   // speed over precision, overshoot allowed (the front toggle rise)
    bool coarseRequest_ = false;  // one-shot: the next setTarget() is a coarse move
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
    bool seatHot_ = false;          // a lift motor passed LIFT_SEAT_TEMP_LIMIT_C: brake-hold instead of pressing
    int seatTempTick_ = 0;          // temperature is only read every 50th SEAT tick
    bool seatShort_ = false;        // this SEAT ended FINAL's forcing window still above LIFT_SEAT_ZERO_DEG
    std::uint32_t seatStillSince_ = 0;  // when the seated arm was first seen still (0 = moving)
    int restWakes_ = 0;             // how many times REST has had to re-seat a creeping arm this command

    // settling (PARK / REST)
    bool coasting_ = false;         // the brake mode is currently COAST (REST only)
    double parkMv_ = 0.0;           // the frozen constant voltage PARK pushes
    double parkGoal_ = 0.0;         // the height PARK believes it is at (re-engage is measured from it)
    bool parkTimeout_ = false;      // this PARK is an abandoned move, not an arrival
    std::uint32_t holdSettleSince_ = 0;  // when the hold was first seen inside the band and slow
    std::uint32_t moveStart_ = 0;   // when the current position move was commanded (LIFT_MOVE_TIMEOUT_MS)

    // L2 scoring
    bool score_ = false;                 // a press/sense/release sequence is running
    bool returnFast_ = false;            // this DESCEND is a post-score run home (for the phase name)
    bool scoreStuck_ = false;            // gave up: claw still caught after every unstick try
    double scoreStartPos_ = 0.0;         // position when startScore() ran -- fixed for the whole score;
                                          // the origin for L2_AUTO_RELEASE_DEG's RELATIVE travel rule.
                                          // NOT the same as descentStart_, which UNSTICK re-baselines.
    bool releasedByTravel_ = false;      // this RELEASE was forced by L2_AUTO_RELEASE_DEG's travel rule
    bool releasedByFloor_ = false;       // this RELEASE was forced by L2_RELEASE_FLOOR_DEG (low start)
    bool scoredAbove_ = false;           // press 1 finished: holding L2_CLEAR_RISE_DEG above the stack
    bool scoreCycle_ = false;            // anywhere inside the two-press score (autoclamp inhibit)
    bool returnRequest_ = false;         // one-shot: this setTarget() is press 2's run home
    bool clawReleaseWanted_ = false;     // the lift is asking Scoring to open the jaws
    bool clawReleased_ = false;          // Scoring acknowledged (clawReleaseDone())
    std::uint32_t clawReleasedAt_ = 0;
    ContactDetector contact_;            // the landing detector, PRESS only (pure, see its header)
    ContactDetector::Trigger contactTrigger_ = ContactDetector::Trigger::NONE;  // which signal fired (NONE if
                                                                                 // releasedByTravel_/releasedByFloor_)
    std::uint32_t freeSince_ = 0;        // when the arm was first seen clearly moving after the release
    std::uint32_t stuckSince_ = 0;       // when it was first seen stopped after the release
    double pressRef_ = 0.0;              // ramped press/unstick speed reference, unsigned deg/s
    double unstickGoal_ = 0.0;           // height the unstick hop is climbing to
    int unstickTries_ = 0;

    double manualPower_ = 0.0;  // -1..1 from driverPower(), 0 = driver not overriding
    bool wasManual_ = false;
    bool descendHeld_ = false;  // R2 held, from descendManual()
    bool descendOwned_ = false; // this descent was started by descendManual(), so releasing R2 ends it
    bool descendRequest_ = false;  // one-shot: the next setTarget() came from descendManual()
    bool fastRequest_ = false;     // one-shot: the next setTarget() is the FAST descent
    bool descentFast_ = false;     // this descent is using the LIFT_FAST_DESCENT_* bands and ramp
    bool toggleWait_ = false;      // a fast descent is queued behind a FRONT_TOGGLE rise
    std::uint32_t toggleWaitStart_ = 0;

    // the grab push
    bool grab_ = false;
    std::uint32_t grabStart_ = 0;
    bool gripWanted_ = false;   // the lift is asking Scoring to shut the jaws
    bool gripAcked_ = false;    // Scoring acknowledged (clawGripDone())
    std::uint32_t gripAckedAt_ = 0;

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
