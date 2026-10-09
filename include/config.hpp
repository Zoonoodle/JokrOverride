#pragma once
/**
 * config.hpp — every port, ratio and gain for the JOKR (57249C) Override robot.
 *
 * Rule: nothing robot-specific is typed anywhere else. If a value is not
 * confirmed from the robot or the notebook it is marked TODO(team): confirm.
 * PID gains are STARTING values only and are marked TUNE: they have not been
 * tuned on the robot.
 *
 * Sources (engineering notebook, 2026-27 Override):
 *   - Drivetrain: 55W, 6 wheels, 2.75" omnis, 450 RPM (Best Solution | Drivetrain RPM & Wheel Size)
 *   - Lift: DR6-4B, 2 motors on a high-strength axle, 12T -> 48T (confirmed on the robot, corrects
 *     an earlier 12T -> 36T note from Build | Lift Drive, 9/10/26)
 *   - Claw: one pneumatic cylinder (Build | Pneumatic Claw, Improve | Claw 9/16/26)
 *   - Intake: three-stage roller intake + bottom counter roller (Season Summary)
 *   - Odometry: LemLib, ONE odometry pod (Best Solution | Odometry, 9/18/26)
 */

#include <cstdint>
#include <vector>

namespace config {

// ---------------------------------------------------------------------------
// Drivetrain (LemLib)
// ---------------------------------------------------------------------------
// Negative port = reversed motor.
// Ports CONFIRMED on the robot 2026-10-02. TODO(team): confirm whether the 55W is 5x 11W or
// 4x 11W + 2x 5.5W (cartridge colour only, not the ports).
inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS = {-8, -2, -1};   // confirmed 2026-10-02
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = {3, 9, 10};     // confirmed 2026-10-02

inline constexpr float DRIVE_WHEEL_DIAMETER_IN = 2.75f;      // notebook: 2.75" omni wheels
inline constexpr float DRIVE_RPM = 450.0f;                   // notebook: 450 RPM
inline constexpr float TRACK_WIDTH_IN = 11.5f;               // TODO(team): measure wheel-center to wheel-center
inline constexpr float HORIZONTAL_DRIFT = 2.0f;              // LemLib: 2 for a drive with traction wheels

// Odometry: one vertical tracking wheel pod. No inertial sensor is on the robot yet.
inline constexpr int8_t ODOM_VERTICAL_ROTATION_PORT = 17;     // confirmed 2026-10-02 (negative to reverse)
inline constexpr float ODOM_WHEEL_DIAMETER_IN = 2.0f;        // TODO(team): confirm pod wheel (2" / 2.75")
inline constexpr float ODOM_VERTICAL_OFFSET_IN = 0.0f;       // TODO(team): measure left/right offset from center
// Inertial sensor: AN IMU IS NOW FITTED, on port 18 (confirmed 2026-10-02 -- this used to read 0,
// "none fitted", and before that 8, which is a right-drive motor). robot.cpp builds a pros::Imu on it
// and hands it to LemLib alongside the vertical tracking wheel, so heading comes from the IMU and
// distance from the pod. It is calibrated in a BACKGROUND task from initialize() and opcontrol()
// NEVER waits on it (CLAUDE.md safety rule 4: nothing may delay driver control, not even an unplugged
// or still-calibrating IMU). autonomous() does wait, because a drive move needs a heading.
// Set this back to 0 to run on the tracking wheel alone: robot.cpp then builds no pros::Imu at all,
// LemLib gets a null IMU pointer (its own documented way to do that), nothing calibrates it and the
// brain shows "IMU  none" instead of junk. Nothing else changes either way.
inline constexpr int8_t IMU_PORT = 18;                        // confirmed 2026-10-02: IMU fitted

// ---- Chassis PID: NOT HERE, ON PURPOSE ----
// The lateral/angular lemlib::ControllerSettings -- and the ExpoDriveCurve driver curves -- are typed
// as literal numbers directly in src/robot.cpp, where the driver tunes them. That is the user's own
// decision (2026-10-02): "remove these drive pid values from config, i want to be able to tune and
// enter them in here". Do NOT move them back into this file. Everything else about the robot still
// belongs here; the chassis PID and the drive curves are the one documented exception.

// ---------------------------------------------------------------------------
// Lift (DR6-4B)
// ---------------------------------------------------------------------------
// Two motors on the 12T motor axle, geared into the 48T lift-arm axle: 12T -> 48T = 4:1 torque.
// Everywhere else in the code (presets, PID target/error, lift.getPosition(), the brain screen)
// works in LIFT-ARM DEGREES -- this ratio is what converts the motor/sensor side of the 4:1 down
// to that one shared unit. See Lift::getMotorPosition()/getSensorPosition() in subsystems/lift.cpp.
inline const std::vector<std::int8_t> LIFT_PORTS = {20, -13};             // confirmed 2026-10-02
inline constexpr double LIFT_GEAR_RATIO = 48.0 / 12.0;       // confirmed: 12T -> 48T = 4:1 torque
// Rotation sensor on the lift, mounted on the 12T MOTOR axle (confirmed) -- not the 48T arm axle.
// 0 = none fitted, use the motor encoders instead. When a port is set, the lift PID runs off THIS
// sensor, not the motor encoders.
inline constexpr int8_t LIFT_ROTATION_PORT = 16;
// DIRECTION CHECK (do this before anything else, see README "Lift direction check"):
// raise the lift by hand with the robot disabled and watch "ROT" on the brain screen.
// It must go UP (positive). If it goes negative, flip this to true.
inline constexpr bool LIFT_ROTATION_REVERSED = false;        // TODO(team): check on the robot
// Sensor degrees per lift-arm degree = the 4:1 above (sensor is on the fast/motor side, so it spins
// 4 turns for every 1 turn of the arm). At the full ~105 deg of arm travel the sensor itself turns
// ~420 deg -- more than one full revolution -- so the code reads it with Rotation::get_position()
// (multi-turn centidegrees, keeps counting past 360) and NEVER get_angle() (wraps at 360 and would
// make the lift think it jumped positions partway up). The brain's ROT line shows the raw sensor
// degrees in brackets next to the converted arm degrees so this can be checked by hand.
inline constexpr double LIFT_ROTATION_RATIO = 4.0;           // confirmed: sensor is on the 12T motor axle

// ---------------------------------------------------------------------------
// THE STARTUP UNWRAP IS GONE (2026-10-07). HOMING DEFINES ZERO INSTEAD.
// ---------------------------------------------------------------------------
// There used to be a software offset here that tried to guess which 90-degree window of the sensor's
// single turn the arm powered on in. It could not work: get_position() is a RETAINED multi-turn
// counter that only re-folds to the single-turn angle on a real sensor power cycle, so a cold boot and
// a warm program restart hand back different values for the same physical arm and no fixed window is
// right for both. That is what made the arm read 79 one run and 120-130 the next.
//
// So the frame is established MECHANICALLY again, the way it was originally: homing drives the arm down
// onto its hard stop and zeroes the sensor THERE (LIFT_HOME_ON_ENABLE is back on). Every height in
// lift_preset is measured UP from that stop, so lift_preset::DOWN is 0.0 and everything is positive --
// no window, no guessing, and the same numbers on every boot regardless of how the brain was started.
// The one cost is the second or so the arm spends finding the stop, and that is non-blocking: it runs
// inside the lift's own task, so a route's drive moves carry on while it happens.
inline constexpr int LIFT_SENSOR_START_RETRY_MS = 250;       // TUNE

// Preset heights, in DEGREES OF THE LIFT ARM (what lift.getPosition() returns -- rotation sensor
// centidegrees already divided by LIFT_ROTATION_RATIO, see the lift gearing note above). 0 = fully
// down (where the robot is turned on). Measured straight off the brain's LIFT line in 7 reference
// photos (docs/robot/photos/levels/, 2026-09-30) with the robot powered on and the arm physically
// moved to each position by hand; see docs/robot/ROBOT.md "Lift levels" for the photo-by-photo readout.
//
// CAVEAT: those photos long predate the current frame, which is defined by HOMING -- the arm drives
// onto its hard stop and the sensor is zeroed there, so lift_preset::DOWN is 0.0 and every height is
// measured up from the stop. Anything derived from those photos carries whatever offset the arm
// happened to be at that session. Re-measure against the brain's LIFT line, with LIFT0 showing
// "frame OK", if anything here looks off on the real robot.
namespace lift_preset {
// THE BOTTOM IS 0, BECAUSE HOMING PUTS IT THERE (the driver, 2026-10-07: "go back to how we reset our
// position to 0 by moving lift down at the start ... so everything will be positive up"). The arm
// drives onto its hard stop on the first enabled mode and the sensor is zeroed at that point, so DOWN
// is 0.0 and every other height below is a positive number measured up from the stop. Earlier frames
// in this file's history had the bottom at 0, then -31.9, then -39, then 48.5; all of those were
// attempts to name whatever the un-homed sensor happened to read, and homing removes the question.
//
// EVERY "this many degrees above the bottom" THRESHOLD IS NOW WRITTEN AS `DOWN + <offset>` rather than
// as a bare height, so there is exactly one place the bottom is stated. The ones that moved with it:
// LIFT_BOTTOM_GOAL_DEG, LIFT_DESCENT_SLOW_ZONE_DEG, LIFT_FINAL_START_DEG, LIFT_REST_WAKE_DEG and
// L2_RELEASE_FLOOR_DEG. LIFT_SEAT_ZERO_DEG is a WINDOW and stays a plain width; the code now measures
// it from DOWN instead of from 0 (lift.cpp, FINAL's atZero test).
// If the hard stop is ever re-measured, change THIS line and nothing else.
//
// WHY THIS IS SAFE TO SET NEGATIVE NOW (and was not, half-done, earlier today): every threshold that
// means "this far above the bottom" is written below as `lift_preset::DOWN + <offset>`, and the two
// places in the code that compared against a bare 0 were changed to measure from DOWN --
// FINAL's `atZero` test and homing's re-zero (both in lift.cpp). Setting this negative while those
// still read 0 makes the lift declare SEAT about 32 deg ABOVE the stop and start FINAL's forcing push
// from there, so the two go together or not at all.
inline constexpr double DOWN = 0.0;                          // THE HARD STOP, and homing zeroes the sensor here
// THE INTAKE RESTING HEIGHT -- the bottom of the new intake sequence (driver, 2026-10-07). History:
// 1.1 -> 6.1 -> -3.3 -> 23.0 -> -2.2. It is no longer a single "go here and park" handoff: R1 and every
// auton intake call now run a FOUR-STEP sequence, and this constant is only the height step (c) ends
// at and step (d) bounces from. The whole sequence lives in "THE INTAKE SEQUENCE" below.
// Everything that keys off the handoff still keys off THIS number: the pivot handoff zone is
// INTAKE .. INTAKE + PIVOT_HANDOFF_ZONE_DEG = -2.2 .. 7.8, Scoring::heightOf(INTAKE_POS) is this, and
// setTarget() still auto-promotes any goal within half a degree of it to PRECISE.
inline constexpr double INTAKE = 22.0;                       // TUNE: 70.5 in the old frame
inline constexpr double MATCHLOAD = DOWN;                    // the matchload height IS the bottom (2026-10-07)
// FRONT TOGGLE: the height the arm goes to at the start of the match route to flip the front toggle,
// then comes straight back down from (fast -- see auton::liftDownFast()). Deliberately its OWN
// constant rather than STACK_LEVELS[0] (31.0), which it is near: the two are tuned for different jobs
// and must be able to move apart.
inline constexpr double FRONT_TOGGLE = 84.0;                 // PROVISIONAL -- RE-MEASURE (132.5 - 48.5)
// FIRST SCORE: a low "just off the floor" height for putting the FIRST pin down, where there is no stack
// under it yet. The driver, 2026-10-03: "we also need a slightly lift up state for our first pin score,
// we are doing level 1 rn but should be like 9 deg up" -- STACK_LEVELS[0] (31.0) is a whole stack's worth
// too high for a pin that is going onto bare floor. (Level 1 is 35.5 as of 2026-10-07.)
// ITS OWN CONSTANT, not a STACK_LEVELS entry, because it is not a stack level: nothing stacks on top of
// it and it is tuned for clearing the floor, not for landing on a tower.
// SCORING FROM IT WITH L2 (changed 2026-10-07). This used to say the L2 scorer could not sense
// anything from 9 deg, because the press started BELOW L2_RELEASE_FLOOR_DEG when that floor was a bare
// 10. The floor is now DOWN + 10 = -21.9, so 9.0 is ~31 deg ABOVE it and there is a real travel budget
// to detect a landing in: L2 from here now presses and senses like any other height, and `REL(floor)`
// is no longer the guaranteed outcome. The explicit sequence below still works and is still the
// predictable choice for putting a pin on bare floor, where there is nothing to sense a landing on:
//     auton::liftToFirstScoreAndWait();
//     auton::clawOpen();
//     auton::liftPushFor(300);   // press down while the jaws clear the pin
// It is deliberately kept clear of lift_preset::INTAKE (-2.2) by more than the half-degree window
// setTarget() uses to auto-promote a goal to PRECISE, so this stays an ordinary, fast position move.
inline constexpr double FIRST_SCORE = 48.0;                  // PROVISIONAL -- RE-MEASURE (96.5 - 48.5)
// L1 taps: tap 1 -> level 1, tap 2 -> level 2 ... tap 5 -> level 5 (stays at 5, the top). One entry
// per pin-stack height.
//
// MEASURED ON THE ROBOT, 2026-10-01, and these replace the photo-derived numbers below. The driver
// drove the arm to each scoring height with the D-pad and read the brain's LIFT / ROT line, so they
// are ARM DEGREES -- the UNBRACKETED number on both of those lines. (The bracketed figure on the ROT
// line is raw SENSOR degrees, 4x bigger and printed with no decimal: level 5 used to read
// "ROT 128.3 [513]". The values here have one decimal and are in the 31..107 range, so they are the
// arm-degree column; read as sensor degrees they would divide down to 8..27 arm deg, which is less
// than one old stack level of total travel.)
//
// The old photo-measured set was 26.6 / 54.4 / 72.8 / 116.3 / 128.3, taken before the frame was
// defined by homing, so they carried whatever offset the arm was sitting at that session. The values
// above are driver-measured in the current homed frame, where the hard stop is 0.
inline constexpr int STACK_LEVEL_COUNT = 5;
inline constexpr double STACK_LEVELS[STACK_LEVEL_COUNT] = {
     67.7,  // TUNE: level 1 (116.2 in the old frame)
     95.5,  // TUNE: level 2 (144.0)
    111.5,  // TUNE: level 3 (160.0)
    133.0,  // TUNE: level 4 (181.5)
    174.5,  // TUNE: level 5, the top (223.0) -- THERE IS NO LEVEL 6
};
// Software hard stop: top level (132.7) + 5 deg of margin -- raised with the levels, 2026-10-07
// (111.9 -> 137.7). It is the ONLY ceiling that needed changing: L1_STEP_CEILING_DEG is derived from
// STACK_LEVELS' last entry, and the runaway guard (LIFT_FAULT_DEG) is relative to a move's start. setTarget()/moveTo() clamp to this, so it
// also doubles as "never command past here" -- raise it only once a higher level is actually measured
// and added above, never just to let a bad preset through. (The D-pad is raw voltage and is NOT
// clamped by this; it is only a limit on commanded heights.)
inline constexpr double MAX = 179.5;
}  // namespace lift_preset

// ===========================================================================
// THE INTAKE SEQUENCE — R1, and every auton intake call, share this one path
// ===========================================================================
// The driver, 2026-10-07: "for intake in auto or in driver R1, lift must go to 13.5 height first with
// pivot not engaged yet (false), but once it hits that height (whether going up to it or down to it)
// then it can activate pivot, and pivot is fast so no need to delay, and then it goes down to -2.2
// lift height slow and steadier, and as it's at that height, it should automatically bounce up and
// down from -2.2 to -0.5 just slightly and steadily, this is so we can intake cup and pin (they are
// different sizes) so it'll fall into the range of one."
//
// THE BOUNCE WAS REMOVED 2026-10-07 at the driver's request -- "remove the oscillation thing on the
// intake, just keep it at -2.2". Step (d) now simply HOLDS -2.2. The quote above is kept because it is
// why the sequence exists at all; only the oscillation went.
//
// THREE STEPS, one per Scoring state (see scoring.hpp):
//   (a) INTAKE_APPROACH  pivot DISENGAGED, lift to INTAKE_PIVOT_TRIGGER_DEG (13.5)
//   (b) INTAKE_PIVOT_WAIT  the tick the lift ARRIVES at 13.5 -- from above OR below -- engage the
//                        pivot, and then HOLD 13.5 for INTAKE_PIVOT_SETTLE_MS while it retracts fully
//                        in. The lift is actively holding that height for the whole wait; nothing
//                        descends until the pivot is really home (driver, 2026-10-07: "keep it there,
//                        let pivot retract in, wait for like 500ms, THEN move slowly down to -2.2").
//   (c) INTAKE_DESCEND   lift to lift_preset::INTAKE (-2.2), as a PRECISE move so it walks in slowly
//                        and steadily rather than running at it (see LIFT_PRECISE_TOLERANCE_DEG).
//   (d) INTAKE_POS       HOLD at -2.2, steadily, until something clamps or the driver cancels.
//
// "PIVOT ENGAGED" IS Scoring::setPivot(FALSE). Careful: the code's bool means "is the pivot OUT (the
// travel / scoring position)", which is the OPPOSITE of the driver's "engaged". Engaged = pulled IN
// toward the intake to receive a cup = pivot BACK = setPivot(false) = pivotOut_ false, and with
// PIVOT_OUT_IS_ADI_HIGH false that is the ADI pin HIGH. Disengaged = setPivot(true) = OUT = pin LOW.
inline constexpr double INTAKE_PIVOT_TRIGGER_DEG = 51.5;     // TUNE: 100.0 in the old frame
// How long step (b) holds 13.5 after engaging the pivot, so the cylinder is fully retracted in
// before the descent starts. Raise it if the claw is still swinging in as the arm starts down.
inline constexpr int INTAKE_PIVOT_SETTLE_MS = 500;           // TUNE: driver 2026-10-07
// STEP (c)'s SPEED. "it has to go down very slowly to this height and stay there" (driver,
// 2026-10-07). This caps the descent to 70.5 at a fraction of LIFT_FREE_SPEED_DEG_PER_S, on top of
// the ordinary precise approach -- Lift::setTargetSlow() applies it as a ceiling in speedProfile(),
// so the braking curve and the final walk-in still work exactly as tuned, just never faster than this.
//   It still comes down too fast to catch a cup: lower it.
//   It takes too long to get there: raise it (the precise approach then governs the last few degrees).
inline constexpr double INTAKE_DESCEND_SPEED_PCT = 18.0;     // TUNE: % of free speed, ~27 deg/s
// R1'S ROLLERS ARE GATED DURING THE SEQUENCE. "until intake is around 70-80 deg height, intake should
// NOT spin when holding R1, so while the pivot is trying to activate at 100 and all that, intake does
// NOT spin." So while the intake macro is running, R1 only reaches the rollers once the arm is at or
// below this height -- nothing spins on the way to 100, through the pivot wait, or at the top of the
// descent. It gates ONLY the driver's R1 while the macro owns the arm; an auton route's explicit
// auton::intakeFor()/intakeIn() is untouched, and so is R1 at any other time.
inline constexpr double INTAKE_ROLLERS_ENABLE_BELOW_DEG = 31.5;
// Backstops, so a jammed arm can never leave the sequence stuck in one step forever. On a timeout the
// step gives up and moves to the next one, which is always safe here (both are downward).
inline constexpr int INTAKE_APPROACH_TIMEOUT_MS = 3000;      // TUNE: step (a) waiting for 13.5
inline constexpr int INTAKE_DESCEND_TIMEOUT_MS = 3000;       // TUNE: step (c) waiting for -2.2

// ===========================================================================
// THE INTAKE AUTOCLAMP GRAB — what happens the moment the bounce catches something
// ===========================================================================
// The driver, 2026-10-07: "when intake autoclamp engages, it needs to clamp, give it a sec to settle,
// then actually move down lower to like -5 degrees into the cup/pin it just detected, and while still
// staying clamped, move back up, and as it moves up, the pivot disengages, so pivot is engaged until,
// while it goes back up, it passes 3 degrees. It should do this automatically and go to level 1."
//
// THREE STEPS, each a Scoring state, entered by the clamp event itself (handleClampEvent()):
//   GRAB_SETTLE  the jaws just shut. Hold still for GRAB_SETTLE_MS so the grip settles before any
//               motion -- the bounce STOPS the instant the clamp fires, which is what this replaces.
//   GRAB_PUSH   drive DOWN to GRAB_PUSH_DEPTH_DEG (-5.0), still clamped, seating the claw into the
//               cup or pin it just caught.
//   GRAB_RISE   rise to stack level 1, still clamped. The pivot stays ENGAGED until the arm passes
//               PIVOT_RELEASE_UP_DEG (3.0) on the way up and then DISENGAGES (setPivot(true)).
//               Hands over to STACKING once it arrives, so the next L1 steps up from level 1.
//
// ALL OF IT IS AUTOMATIC, and ANY driver input cancels it: every one of these is an ordinary Scoring
// state, so stepUp()/score()/stow()/goToIntake() and the D-pad's reset() all leave it the moment they
// are pressed, exactly as they leave any other state.
inline constexpr int GRAB_SETTLE_MS = 1000;                  // TUNE: "give it a sec to settle"
inline constexpr double GRAB_PUSH_DEPTH_DEG = lift_preset::INTAKE - 2.8;
inline constexpr int GRAB_PUSH_TIMEOUT_MS = 1200;            // TUNE: backstop for the push down
inline constexpr double PIVOT_RELEASE_UP_DEG = lift_preset::INTAKE + 5.2;
inline constexpr int GRAB_RISE_TIMEOUT_MS = 4000;            // TUNE: backstop for the rise to level 1

// ---------------------------------------------------------------------------
// L1's LADDER: a fixed 20 deg step, not the absolute stack levels
// ---------------------------------------------------------------------------
// THE DRIVER'S REQUEST, 2026-10-03: "have our l1 actauly just go up by 20 each time", and 2026-10-07:
// "Still have L1 smooth and fast, move it up 20 degrees each time". So L1 is not "jump to
// STACK_LEVELS[n]": each press moves the arm a fixed L1_STEP_DEG FURTHER UP.
//
// UP ONLY (changed 2026-10-07, the driver's call). The ladder used to REVERSE at the top and walk back
// down 20 at a time. It no longer does: at the ceiling it simply STOPS there, and extra presses are a
// no-op rather than a descent. Coming back down is L2 (score), B (stow) or R2 -- never L1. That removed
// Scoring::stepDown_ and isSteppingDown() entirely.
//
// EACH PRESS IS RELATIVE TO WHERE THE ARM IS HEADING, not to a press count: advanceStep() steps from
// the lift's own active target (or, with nothing commanded -- e.g. just after a D-pad nudge, which
// cancels the target -- from the measured position). So a manual nudge is absorbed automatically and
// the next L1 goes 20 deg above wherever the driver left the arm. Rapid taps still stack correctly,
// because the base is the TARGET, not the lagging position.
//
//   from the handoff (23.0):  43.0  63.0  83.0 103.0 106.9  106.9 ...
//   from the hard stop:      -11.9   8.1  28.1  48.1  68.1   88.1  106.9 ...
// WHAT THIS DOES NOT CHANGE: lift_preset::STACK_LEVELS is untouched and still means the measured scoring
// heights -- auton::liftTo(Level::THREE), the brain's preset names and everything else still use them.
// Only L1's own ladder is relative now.
// THE CEILING is the top measured stack level, derived rather than typed so it cannot drift away from it.
// The last step up is CLAMPED to it, so the top of the ladder is a real scoring height rather than
// wherever a multiple of 20 happens to land (100 would be 7 deg short of it).
//   The steps are too coarse / too fine: change L1_STEP_DEG. It needs no other number changed -- the
//     ladder is generated from it.
//   It should bounce back down at the top again: that is a code change in Scoring::advanceStep(),
//     not a constant.
// THE FLOOR is lift_preset::DOWN, so a step can never command the arm below its own hard stop.
inline constexpr double L1_STEP_DEG = 20.0;                  // TUNE: arm degrees per L1 press
inline constexpr double L1_STEP_CEILING_DEG =
    lift_preset::STACK_LEVELS[lift_preset::STACK_LEVEL_COUNT - 1];  // the top measured level (106.9)

// ===========================================================================
// TUNING THE LIFT PID — the five gains, and how to turn them
// ===========================================================================
// THESE FIVE ARE THE ONLY GAINS THAT MATTER. They live further down in this file, each with its own
// symptom notes, but this is the index and the order to use:
//
//   LIFT_VEL_KP   mV per (deg/s) of speed error   -- the inner speed loop every MOVE runs through
//   LIFT_VEL_KI   mV per (deg/s) per second       -- kills the steady speed sag
//   LIFT_HOLD_KP  mV per deg of sag               -- how hard a PARKED arm fights being pushed down
//   LIFT_HOLD_KI  mV per deg per second           -- learns gravity so it stops settling low
//   LIFT_HOLD_KD  mV per deg/s                    -- THE ANTI-BOUNCE TERM. Raise this first for
//                                                    overshoot or oscillation on arrival.
//
// YOU DO NOT NEED TO REBUILD TO TRY A VALUE. They are mirrored into Lift::gains() at startup, and that
// struct -- not these constants -- is what the loops read every tick. The LIFT_PID_TUNE auton route
// (autons.cpp) cycles the arm through the real heights and lets the controller nudge all five live,
// printing them and each move's OVERSHOOT / SETTLE TIME / STEADY ERROR on the brain. Tune there, note
// the numbers, then type the ones you liked in here so they survive a power cycle.
//
// THE ORDER (do not skip ahead -- each step assumes the one before it is right):
//   1. SPEED FIRST, with the arm swinging between two far-apart heights (the route's DOWN <-> level 5).
//      Raise LIFT_VEL_KP until the arm reaches its commanded speed promptly; back off the moment the
//      trace shows `vel` hunting above and below the reference. Then raise LIFT_VEL_KI until the speed
//      stops sagging under load.
//   2. ARRIVAL NEXT. Watch OVERSHOOT on the brain. Raise LIFT_HOLD_KD until the arm stops bouncing
//      past the goal and settles in one motion. This is the knob for "it oscillates", and it is almost
//      always the right answer before touching anything else.
//   3. HOLDING LAST. Watch STEADY ERROR with the arm parked high. Raise LIFT_HOLD_KI until it stops
//      drifting low over a few seconds. Only then raise LIFT_HOLD_KP, and only if it still sags
//      immediately when loaded -- too much KP buzzes.
//   4. CHECK THE PRECISE SPOTS, which have tighter tolerances than the levels and are where this all
//      has to be right: the 13.5 pivot trigger, the -2.2/-0.5 bounce, the -5 grab and the 3 deg pivot
//      release. SETTLE TIME matters more than OVERSHOOT at the levels (5-10 deg of slop is fine
//      there); at these four, overshoot is what to watch.
//
// IF YOU ONLY CHANGE ONE THING: LIFT_HOLD_KD for oscillation, LIFT_VEL_KI for sluggishness.

// ===========================================================================
// LIFT TUNING — start here
// ===========================================================================
// Everything that decides how the lift feels is in this block. Each line says what it does and which
// way to move it for which symptom. Nothing else needs touching to tune the lift. The structure these
// numbers feed is described under "Lift motion" below, and the step-by-step order to tune them in is
// in README "Tuning the lift". Watch a move with the Brain Terminal trace open (LIFT_TRACE).
//
// HARDWARE FACTS (not knobs -- these are what the robot is):
// Both lift motors are RED (100 RPM) cartridges (MotorGears::red in Lift::Lift()). Through the
// 12T -> 48T (4:1) reduction the ARM's free speed is 100 / 4 = 25 arm RPM = 150 arm deg/s.
inline constexpr double LIFT_FREE_SPEED_DEG_PER_S = 150.0;   // red 100 RPM / 4:1 = 25 arm RPM = 150 deg/s
inline constexpr float LIFT_MAX_MV = 12000.0f;               // full battery volts; the D-pad's full-send power

// --- How fast a preset move cruises, as a % of the free speed above ---------
//   Feels sluggish on a long rise: raise LIFT_UP_SPEED_PCT (100 is the hardware limit).
//   Runs away / slams going down to a LOWER level: lower LIFT_DOWN_SPEED_PCT.
// RAISED DOWN 60 -> 85 (2026-10-03: "each of our arm movements are just moving too slowly"). 60% capped
// a downward POSITION move at 90 deg/s, which is most of why coming down the L1 ladder felt sluggish.
// This is only the cap for a position move BETWEEN heights -- a trip to the bottom has its own
// LIFT_DESCENT_*_PCT bands and is unaffected.
inline constexpr double LIFT_UP_SPEED_PCT = 100.0;           // TUNE: cruise speed going up
inline constexpr double LIFT_DOWN_SPEED_PCT = 85.0;          // TUNE: cruise speed going down (was 60)

// --- When it starts slowing down (the braking curve) ------------------------
// The arm is never allowed to go faster than sqrt(2 * LIFT_DECEL_DEG_PER_S2 * d) with d = the degrees
// left minus LIFT_APPROACH_ZONE_DEG. This is the single number that decides whether it overshoots --
// and, just as much, how FAST a move feels, because it sets how much of the move is spent braking.
// Read it off the trace: in the B lines `vel` should fall by about a * 0.02 per 20 ms line.
//   Overshoots the level / arrives still moving: LOWER it (braking starts earlier): 1000, 850.
//   Feels slow / spends most of the move braking: RAISE it (braking starts later): 1400.
// SIZING IT. Braking distance from the 150 deg/s free speed is 150^2 / (2a), and braking begins that
// far out PLUS LIFT_APPROACH_ZONE_DEG. The stack levels are 12-44 deg apart, so for a move to have any
// full-power section at all, (150^2 / 2a) + zone has to be well under ~25 deg:
//   a =  700 -> 16.1 + 8 = 24.1 deg of braking. A 26.6 deg move is then ~2 deg of power and 24 of
//                braking -- that is exactly why the start and middle "felt slower".
//   a = 1200 -> 9.4 + 5 = 14.4 deg. Half the move is full power. This is the value here.
//   a = 2000 -> 5.6 + 5 = 10.6 deg, but 2000 deg/s^2 is more than the arm can actually do -- that was
//                the original overshoot (3723ef2). 1200 is 40% less demanding than that.
// RAISED 1200 -> 1800 (2026-10-03). THIS IS THE MAIN "it moves too slowly" KNOB, because it decides how
// much of a move is spent braking rather than at full voltage: braking starts 150^2/(2a) + zone degrees
// out, which was 9.4 + 5 = 14.4 deg of every move and is now 6.25 + 4 = 10.25. On a 20 deg L1 ladder step
// that is the difference between braking for most of the move and braking for half of it.
// WHY THIS IS SAFE TO RAISE NOW, given config history says 2000 "is more than the arm can actually do":
// that was 3723ef2, and it overshot for three reasons that have all since been fixed -- the velocity
// estimate lagged a whole tick, enterBrake() seeded the loop with an UPWARD kick, and arrival handed a
// still-moving arm to the firmware's stiff brake-hold. It is also corroborated from the other side: the
// COARSE move already brakes at 2600 (LIFT_COARSE_DECEL_DEG_PER_S2) and the front toggle works, so the
// arm tolerates late braking when a little overshoot is acceptable. 1800 sits between the two. And the
// reported symptom was UNDERshoot, i.e. there was headroom here, not a deficit.
//   It now OVERSHOOTS or rings on arrival: lower this first (1800 -> 1500 -> 1200), before touching
//     anything else in this block. That is the one symptom this change can cause.
inline constexpr double LIFT_DECEL_DEG_PER_S2 = 1800.0;      // TUNE: how hard it brakes, arm deg/s^2 (was 1200)

// --- A COARSE move: speed over precision, overshoot allowed ------------------
// For a height where only "get there quickly" matters and landing a few degrees past it costs
// nothing -- today that is the FRONT TOGGLE rise, which exists only to touch a height and come
// straight back down. The driver: "lift to front toggle quite fast too, like a little faster than how
// we rise to level 1, this movement doesnt have to be precise and requires speed over precision, can
// over shoot."
// A normal rise is already at full voltage off the line; what makes it feel measured is the BRAKING,
// which starts 150^2/(2a) + zone degrees out -- 14.4 deg of a 30 deg move at a = 1200, i.e. half the
// move. A coarse move brakes LATER, keeps a quick tail instead of the crawl, and counts a much wider
// band as arrived:
//   DECEL  2600 asks for more deceleration than this arm can actually deliver. On an ordinary move
//          that would be a bug (it is why 3723ef2's 2000 overshot and rang); here it is the POINT --
//          the arm arrives still moving and sails a little past, which is explicitly fine.
//   MIN    the approach tail does not taper to a crawl, it stays at this speed until the band.
//   TOL    a goal this wide counts as arrived, so it latches into the hold immediately instead of
//          walking the last few degrees in.
//   Overshoots too far / bangs into something above the goal: lower LIFT_COARSE_DECEL_DEG_PER_S2
//     (2600 -> 1800) first, then LIFT_COARSE_MIN_DEG_PER_S.
//   Still feels no faster than a normal rise: raise LIFT_COARSE_DECEL_DEG_PER_S2.
inline constexpr double LIFT_COARSE_DECEL_DEG_PER_S2 = 2600.0;  // TUNE: brake this late (normal 1200)
inline constexpr double LIFT_COARSE_MIN_DEG_PER_S = 45.0;       // TUNE: no crawl (normal 6)
inline constexpr double LIFT_COARSE_TOLERANCE_DEG = 10.0;       // TUNE: arrived band (normal 5)

// --- The last few degrees (the final approach) ------------------------------
// Inside LIFT_APPROACH_ZONE_DEG the speed reference becomes LIFT_APPROACH_KP_PER_S * (degrees left),
// clamped between LIFT_APPROACH_MIN_DEG_PER_S and LIFT_APPROACH_DEG_PER_S. So the arm walks the last
// few degrees in at a crawl and hands over to the hold already nearly stopped -- this is what stops
// the six-bar springing.
// THE MINIMUM MATTERS. kP * (degrees left) on its own decays exponentially, so the arm creeps toward
// the goal more and more slowly and never really gets there: with kP = 3 the last 6 deg took 0.8 s,
// which is most of why a short move felt slow. The floor turns the tail into a constant walk-in
// instead, and LIFT_STOPPED_DEG_PER_S is set above it so arrival still latches.
//   Still visibly moving when it grabs the hold: lower LIFT_APPROACH_DEG_PER_S, or
//     LIFT_APPROACH_MIN_DEG_PER_S, or raise LIFT_HOLD_KD (the anti-bounce term) first.
//   The last few degrees take too long: raise LIFT_APPROACH_KP_PER_S, then
//     LIFT_APPROACH_MIN_DEG_PER_S; shrink LIFT_APPROACH_ZONE_DEG to start the crawl later.
// At a = 1200 the braking curve is already down to 28 deg/s a third of a degree outside a 5 deg zone,
// so the handover from braking into the crawl is seamless rather than a step.
// ALL FOUR RETUNED FOR SPEED (2026-10-03: "moving too slowly rn and seem to undershoot"). The old
// numbers walked the last 5 deg in at 28 deg/s tapering to 6, which is ~0.3 s of crawl on EVERY move and
// is most of what "too slow" actually was. The floor going 6 -> 12 also attacks the UNDERSHOOT directly:
// at 6 deg/s the arm was barely moving when arrival latched, so wherever it latched is where it stayed;
// at 12 it still carries into the goal and the hold's damping absorbs the rest.
//   Still too slow in the last few degrees: raise LIFT_APPROACH_KP_PER_S, then _MIN_DEG_PER_S.
//   Now overshoots the goal: lower LIFT_APPROACH_DEG_PER_S, or raise LIFT_HOLD_KD (the anti-bounce
//     term) -- and see the note on LIFT_DECEL_DEG_PER_S2 first, which is the bigger lever.
inline constexpr double LIFT_APPROACH_ZONE_DEG = 4.0;        // TUNE: where the crawl starts (was 5)
inline constexpr double LIFT_APPROACH_DEG_PER_S = 50.0;      // TUNE: fastest the crawl may be (was 28)
inline constexpr double LIFT_APPROACH_MIN_DEG_PER_S = 8.0;   // TUNE: slowest -- no exponential tail; red retune 2026-10-07 (was 12)
inline constexpr double LIFT_APPROACH_KP_PER_S = 10.0;       // TUNE: crawl speed per degree left (1/s) (was 6)

// --- The velocity loop (phases BRAKE, APPROACH and DESCEND) -----------------
// Everything that is "go at this speed" runs through this one PI loop on the MEASURED arm speed:
//   mV = LIFT_VEL_KP * (reference - measured) + integral of LIFT_VEL_KI * (reference - measured)
// The integral is what learns gravity -- whatever voltage the arm needs at this angle to hold a speed,
// up or down -- so no gravity constant has to be right in advance.
//   `vel` swings above and below the reference in the trace (hunts): LOWER LIFT_VEL_KP.
//   Sags / is slow to reach the reference speed: RAISE LIFT_VEL_KI.
//   The braking feels violent: LOWER LIFT_VEL_MAX_MV (and lower LIFT_DECEL_DEG_PER_S2 to match).
// LIFT_VEL_MAX_MV IS 12000 ON PURPOSE. It was 9000, and that was a second reason the middle of a long
// move felt slow: holding the arm at its 150 deg/s free speed needs essentially the full 12000 mV
// (back-EMF alone is 12000/150 = 80 mV per deg/s), so a 9000 mV clamp physically could not sustain the
// cruise -- the arm sagged to whatever 9000 mV holds, around 110 deg/s, for the whole middle of the
// rise. The clamp was never what limits the braking either; LIFT_DECEL_DEG_PER_S2 is.
inline constexpr float LIFT_VEL_KP = 18.0f;                  // TUNE: mV per (deg/s) of speed error, red retune 2026-10-07 (was 30)
inline constexpr float LIFT_VEL_KI = 240.0f;                 // TUNE: mV per (deg/s) per second, red retune 2026-10-07 (was 400)
inline constexpr float LIFT_VEL_MAX_MV = 12000.0f;           // TUNE: most the velocity loop may push, either way
// Hysteresis between DRIVE (full voltage) and BRAKE (the velocity loop). DRIVE ends the moment the arm
// reaches the profile speed, and is only re-entered once the arm is this far BELOW it -- which is what
// stops the two flickering tick-to-tick around the boundary, and what lets a mid-move retarget drop
// straight back into full power.
//   The trace shows D and B alternating every line or two: RAISE it.
//   It takes too long to go back to full power after an L1 retarget: LOWER it.
inline constexpr double LIFT_DRIVE_RESUME_DEG_PER_S = 20.0;  // TUNE

// --- Holding position (phase HOLD) -----------------------------------------
// A damped position hold, NOT the firmware's brake-hold. The firmware hold is a stiff position loop
// with gains we don't control: handing a still-moving, springy six-bar (chain stretch, 4:1 backlash,
// rubber bands) to it is what made the arm ring up and down after L1. This one has an explicit damping
// term, so the arriving arm's energy is absorbed instead of stored:
//   mV = LIFT_HOLD_KP * (error outside the deadband) + integral(LIFT_HOLD_KI * error) - LIFT_HOLD_KD * speed
//   Sags below the level over a few seconds: raise LIFT_HOLD_KI first, then LIFT_HOLD_KP.
//   Buzzes / twitches while holding: raise LIFT_HOLD_DEADBAND_DEG, then lower LIFT_HOLD_KP.
//   Still bounces once or twice on arrival: RAISE LIFT_HOLD_KD. This is the anti-bounce knob.
//   Audibly straining while parked at a high level: lower LIFT_HOLD_MAX_MV (expect some sag).
// RAISED 180 -> 200 (2026-10-03, the driver: "turn up lift P gain a little just slightly"). This is THE
// lift's position P -- the hold's, not the velocity loop's LIFT_VEL_KP, which is an inner speed loop.
// WHERE IT ACTUALLY BITES, so the next person is not surprised by how little changes: P is SILENT inside
// holdDeadband() (4 deg at a level, 0.5 at INTAKE) and PARK takes the whole loop away after
// LIFT_PARK_CONFIRM_MS / LIFT_SETTLE_TIMEOUT_MS. So this only does work (a) over the brief active hold
// between arrival and parking, and (b) when a parked arm has drifted LIFT_REENGAGE_DEG and the hold wakes
// up to pull it back. If the symptom is "it settles a bit low and stays there", LIFT_HOLD_KI or
// LIFT_HOLD_DEADBAND_DEG is the knob, not this one.
inline constexpr float LIFT_HOLD_KP = 100.0f;                // TUNE: mV per deg of sag, red retune 2026-10-07 (was 200)
inline constexpr float LIFT_HOLD_KI = 200.0f;                // TUNE: mV per deg per second, red retune 2026-10-07 (was 400)
inline constexpr float LIFT_HOLD_KD = 140.0f;                // TUNE: mV per deg/s -- the anti-bounce term, RAISED 2026-10-07 (was 90)
inline constexpr float LIFT_HOLD_MAX_MV = 5000.0f;           // TUNE: most the hold may push, red retune 2026-10-07 (was 7000)
// THE SETTLE BAND (was LIFT_HOLD_DEADBAND_DEG = 0.8, which is most of why the arm hunted at a level --
// see "Settling instead of hunting" below). Inside it the hold applies NO position correction and
// FREEZES its integral: it just keeps pushing the gravity voltage it has already learned. 4 deg is
// well inside the 5 deg arrival tolerance the driver is happy with at a stack level ("5-10 deg is
// fine"), and the arm is simply left where it stopped instead of being argued with.
//   Still creeps down out of the band over a few seconds: raise LIFT_HOLD_KI a little, NOT this.
//   Visibly off the level: lower this (and LIFT_TOLERANCE_DEG with it).
// NARROWED 4 -> 2.5 (2026-10-03), to attack the UNDERSHOOT. Inside this band the hold applies no position
// correction at all, and PARK then freezes whatever voltage is holding it -- so the band is a hard floor
// on how far off the arm can settle and stay. 4 deg of that was most of the visible "it stops short".
//   It buzzes or twitches while holding: raise this back toward 4 (that is what it was for), or lower
//     LIFT_HOLD_KP.
inline constexpr double LIFT_HOLD_DEADBAND_DEG = 2.5;        // TUNE: stack levels, MATCHLOAD, any custom height (was 4)
// INTAKE must still line up, so a PRECISE goal gets a tight band and keeps correcting right in.
inline constexpr double LIFT_HOLD_PRECISE_DEADBAND_DEG = 0.5;  // TUNE: INTAKE only

// --- Settling instead of hunting: the PARK phase ------------------------------
// WHY. A damped position hold on a springy six-bar with 4:1 backlash cannot sit perfectly still: the
// integral winds up against stiction until the arm breaks free, overshoots the 0.8 deg deadband, and
// the integral then has to unwind -- a slow limit cycle, which is the "still oscillating a good
// amount" the driver sees. Arrival also latches with up to LIFT_TOLERANCE_DEG of error and up to
// LIFT_STOPPED_DEG_PER_S of speed, so the hold starts with something real to chase.
// THE FIX, in the driver's own words ("id rather it just settle instead of keeping on trying to do
// it"): stop using feedback at all once the arm is near enough. PARK is a CONSTANT voltage -- the
// gravity feedforward the hold had learned, frozen at that instant -- with no P, no D and no integral
// update. A constant voltage has no loop, so it physically cannot oscillate. Entered from HOLD either
// as soon as the arm has been inside the settle band and slow for LIFT_PARK_CONFIRM_MS, or
// unconditionally after LIFT_SETTLE_TIMEOUT_MS of holding, whichever comes first.
//   It parks visibly off the level: lower LIFT_HOLD_DEADBAND_DEG, or raise LIFT_SETTLE_TIMEOUT_MS so
//     the active hold gets longer to pull in first.
//   It sags away after parking: raise LIFT_HOLD_KI (so the learned voltage it freezes is bigger), or
//     lower LIFT_REENGAGE_DEG so the hold wakes sooner.
//   It still oscillates: LOWER LIFT_SETTLE_TIMEOUT_MS and LIFT_PARK_CONFIRM_MS -- that parks sooner,
//     i.e. less feedback for less time. This is the knob for "it still hunts".
// BOTH SHORTENED (2026-10-03: "we need quick timeout for position, so juist go there fast, and hold, even
// if its a bit off"). This is that request, precisely: PARK is the "hold, even if it's a bit off" state --
// a frozen constant voltage with no feedback -- so reaching it sooner is reaching "done" sooner.
//   It parks visibly off the level: raise LIFT_SETTLE_TIMEOUT_MS so the active hold gets longer to pull
//     in first (that is the trade being made here), or narrow LIFT_HOLD_DEADBAND_DEG further.
inline constexpr int LIFT_PARK_CONFIRM_MS = 100;             // TUNE: inside the band and slow for this long (was 150)
inline constexpr int LIFT_SETTLE_TIMEOUT_MS = 350;           // TUNE: ...or just hold this long, then park anyway (was 700)
// Once parked, the hold only wakes up again if the arm has really moved away from where it parked (a
// knock, or a slow sag the frozen voltage could not carry). Deliberately large: waking for anything
// smaller is exactly the hunting this exists to stop.
inline constexpr double LIFT_REENGAGE_DEG = 8.0;             // TUNE
// ...and never sooner than this after parking, so PARK and HOLD cannot ping-pong.
inline constexpr int LIFT_PARK_COOLDOWN_MS = 300;            // TUNE
// A position move that has not arrived by now has met something it cannot get past (a preset above the
// real top, a jam the stall guard did not catch, a badly tuned loop): stop chasing it and PARK where
// it is, shown as PARK(t/o). Sized from the longest legitimate move -- DOWN to level 5 is 107 deg,
// which at the braked profile is well under a second -- plus a wide margin, and it has to cover a
// CLAMPING/pivot wait in front of the move too.
//   A real long move gets cut short: raise it.
// SHORTENED 2500 -> 1500 (2026-10-03, the "quick timeout for position"). Still comfortably longer than
// any real move: the longest legitimate one is 0 -> the top level, 107 deg, which at the retuned profile
// above is well under a second, and moveStart_ is set when the height is actually COMMANDED (Scoring's
// CLAMPING wait happens before that, so this does not have to cover it).
//   A real long move gets cut short and shows PARK(t/o): raise it.
inline constexpr int LIFT_MOVE_TIMEOUT_MS = 3000;            // TUNE: half speed + ~32 deg more travel, 2026-10-07 (was 1500)

// --- The descent to the bottom (L2, B, auton DOWN, and R2 in clamped mode) --
// Three speed bands on the way down, as % of the free speed, so the arm walks itself onto its hard
// stop instead of falling onto it. Band boundaries are read off the arm's POSITION (degrees above 0):
//   top      : from wherever the descent started, down to the mid boundary -> LIFT_DESCENT_TOP_PCT
//   middle   : from the mid boundary down to LIFT_DESCENT_SLOW_ZONE_DEG    -> LIFT_DESCENT_MID_PCT
//   approach : LIFT_DESCENT_SLOW_ZONE_DEG down to LIFT_FINAL_START_DEG     -> LIFT_DESCENT_SLOW_PCT
// then FINAL (see "Seating on the hard stop" below) takes over at LIFT_FINAL_START_DEG. The mid
// boundary is a FRACTION of the travel between the start height and the slow zone, not a fixed height,
// so "the middle" really is the middle of this particular descent whether it starts at level 1 or
// level 5: mid boundary = SLOW_ZONE + LIFT_DESCENT_MID_FRACTION * (start height - SLOW_ZONE). Band
// changes are short ramps at LIFT_DESCENT_RAMP_DEG_PER_S2, never stops.
//
// RETUNED 2026-10-02: the arm was crawling the whole bottom 0-20 deg (SLOW_PCT was 12%, ~18 deg/s, from
// 20 deg all the way to the old 6 deg FINAL handover) and the user asked for it to "stay fast much
// lower". The fix is to shrink the approach zone AND raise its speed, moving the real slowdown to the
// last ~14 deg instead of the last ~20, with FINAL now starting at 3 deg (LIFT_FINAL_START_DEG) instead
// of 6:
//   LIFT_DESCENT_SLOW_ZONE_DEG   20 -> 14   (approach band starts later -- fast bands run longer)
//   LIFT_DESCENT_SLOW_PCT        12 -> 30   (~45 deg/s instead of ~18 -- a real cruise, not a crawl)
//   LIFT_DESCENT_RAMP_DEG_PER_S2 800 -> 1200 (see why below)
// WHY THE RAMP HAD TO GET FIRMER. Distance to decelerate from v1 to v2 at rate a is (v1^2-v2^2)/(2a).
// Worst case entering the approach zone is the MID band's ~142 deg/s (95%). At the OLD ramp (800),
// decelerating 142 -> 45 takes (142^2-45^2)/1600 = 11.3 deg -- almost the entire new 14-to-3 deg zone
// (11 deg) on its own, leaving no room to then decelerate 45 -> FINAL's 12 deg/s before hitting
// LIFT_FINAL_START_DEG, i.e. the arm would still be doing ~70+ deg/s AT 3 deg and FINAL would have to
// brake it hard right at the floor. At the NEW ramp (1200): 142 -> 45 takes (20164-2025)/2400 = 7.56
// deg, landing around position 14-7.56 = 6.4 deg, which leaves a real ~3.4 deg CRUISE at 45 deg/s down
// to the 3 deg mark; then 45 -> 12 (FINAL's target) takes only (2025-144)/2400 = 0.78 deg, settling by
// position ~2.2 deg with ~2.2 deg left to cruise into the stop at a steady 12 deg/s -- comfortable
// margin, no slam. 1200 deg/s^2 is not an arbitrary new number either: it is the same braking rate
// already proven safe for position moves (LIFT_DECEL_DEG_PER_S2, used since the original lift tuning).
//   Falls too fast off a high level: lower LIFT_DESCENT_TOP_PCT / LIFT_DESCENT_MID_PCT.
//   Still arrives at the approach band going faster than the approach speed (overshoots into FINAL
//     hot): raise LIFT_DESCENT_RAMP_DEG_PER_S2 further, or raise LIFT_DESCENT_SLOW_ZONE_DEG so there is
//     more room to decelerate.
//   The whole descent takes too long: raise LIFT_DESCENT_MID_PCT, lower LIFT_DESCENT_SLOW_ZONE_DEG, or
//     raise LIFT_DESCENT_SLOW_PCT (the approach cruise itself).
// RAISED 80 -> 95 (2026-10-03, the driver: the auton resets "need more speed/power to go down"). The
// top band is the one that runs while the arm is high and gravity is helping least, so it is where the
// time actually goes. MID was already 95, so the two fast bands are now the same speed and the descent
// is effectively one fast run down to LIFT_DESCENT_SLOW_ZONE_DEG -- the ramp sizing below was already
// computed for entering the slow zone at MID's ~142 deg/s, so nothing downstream had to change.
inline constexpr double LIFT_DESCENT_TOP_PCT = 95.0;         // TUNE: ~142 deg/s (was 80)
inline constexpr double LIFT_DESCENT_MID_PCT = 95.0;         // TUNE: ~142 deg/s
inline constexpr double LIFT_DESCENT_SLOW_PCT = 30.0;        // TUNE: ~45 deg/s, the approach cruise (was 12% / 18 deg/s)
// DOWN-relative (2026-10-07): 14 deg ABOVE THE HARD STOP, wherever that is, = -17.9 now.
inline constexpr double LIFT_DESCENT_SLOW_ZONE_DEG = lift_preset::DOWN + 14.0;  // TUNE: offset above the bottom
inline constexpr double LIFT_DESCENT_MID_FRACTION = 0.5;     // TUNE: 0.5 = the mid band is the lower half of the fast travel
inline constexpr double LIFT_DESCENT_RAMP_DEG_PER_S2 = 1200.0;// TUNE: how quickly the reference may change between bands (was 800)

// --- The FAST descent (auton::liftDownFast(), the front toggle's trip back down) ---------------
// "needs to go acautally fast and forcefully down to, but still slow when it approvhes our 0 and
// settle, just a bit faster down than normal for toggle." Same banded DESCEND -> FINAL -> SEAT ->
// REST, with ONLY the two fast bands raised; the approach cruise below LIFT_DESCENT_SLOW_ZONE_DEG,
// FINAL's forced push and SEAT/REST are byte-for-byte the normal ones, so it still slows near 0 and
// settles exactly the same way.
// From the toggle height (45 deg; it went 30 -> 38 -> 45 on 2026-10-03) BOTH fast bands still run: the
// mid boundary is SLOW_ZONE + 0.5 * (45 - SLOW_ZONE) = 29.5, so TOP covers 45 -> 29.5 and MID covers
// 29.5 -> 14. The boundary is a FRACTION of the travel, which is why raising the toggle height twice has
// needed no change here either time -- and the taller the toggle gets, the MORE room both fast bands
// have, so this only ever gets safer in that direction.
// 120% of free speed is not a speed the arm can reach on its own -- it is how you say "do not hold
// back": the velocity loop saturates at its full 12000 mV of downward authority, which is the
// "forcefully" the driver asked for.
inline constexpr double LIFT_FAST_DESCENT_TOP_PCT = 95.0;    // TUNE: ~143 deg/s, red retune 2026-10-07 (was 105)
inline constexpr double LIFT_FAST_DESCENT_MID_PCT = 100.0;   // TUNE: ~150 deg/s = free speed, red retune 2026-10-07 (was 120)
// AND A FIRMER RAMP, which is not optional at those speeds. Decelerating v1 -> v2 at rate a covers
// (v1^2 - v2^2) / 2a degrees. Entering the approach zone at MID's ~180 deg/s and needing the 45 deg/s
// cruise: at the NORMAL 1200 that takes (32400 - 2025) / 2400 = 12.7 deg -- more than the whole
// 14 -> 3 deg zone, so the arm would still be doing 70+ deg/s when FINAL took over, right at the
// floor. At 2000 it takes 7.6 deg, landing at about position 6.4 with ~3.4 deg of real cruise at
// 45 deg/s before FINAL -- the same comfortable margin the normal descent was sized for.
//   It thumps / arrives at the slow zone hot: LOWER LIFT_FAST_DESCENT_MID_PCT first (120 -> 110),
//     then raise this ramp.
//   It is not noticeably faster than a normal descent: raise LIFT_FAST_DESCENT_MID_PCT (and the ramp
//     with it, by the sum above).
inline constexpr double LIFT_FAST_DESCENT_RAMP_DEG_PER_S2 = 2000.0;  // TUNE (normal: 1200)
// THE QUEUED TOGGLE. The route asks for the way down only 300 ms after the way up, which is less than
// a 0 -> 30 deg rise takes, so a fast descent requested while a FRONT_TOGGLE rise is still climbing is
// QUEUED rather than obeyed: the arm keeps rising until it is within LIFT_FRONT_TOGGLE_REACH_DEG of
// the toggle height and then turns straight around -- it touches the height, it does not settle on it.
// Shown as `TOGGLE ^` on the brain's LIFT line. If the rise never gets there (jammed, BLOCKED), it
// descends anyway after LIFT_FRONT_TOGGLE_MAX_WAIT_MS rather than hanging on the way up.
inline constexpr double LIFT_FRONT_TOGGLE_REACH_DEG = 2.0;   // TUNE: close enough to call it touched
inline constexpr int LIFT_FRONT_TOGGLE_MAX_WAIT_MS = 1500;   // TUNE: then go down regardless

// --- Seating on the hard stop (phases FINAL, then SEAT) ---------------------
// The point of the final descent is that the arm does not STOP above the frame -- it keeps walking
// down until it physically cannot, and then stays pressed there so it cannot bounce back up.
//
// TWO STEPS, not one -- the original version used a single check ("stopped under 6 deg" == "seated")
// and that was a bug: a cup caught in the intake, or plain six-bar/chain friction, can stall the arm a
// few degrees above the true hard stop, read as "stopped", and get declared SEATed right there -- after
// which only LIFT_SEAT_MV (a HOLD voltage, meant to fight bounce, not resistance) was ever applied
// again, so the arm simply parked short of 0.
// Below LIFT_FINAL_START_DEG the arm hands over from the ordinary DESCEND bands into FINAL: a
// dedicated closed-loop push at a constant LIFT_FINAL_DEG_PER_S, with up to LIFT_FINAL_MAX_MV of
// downward authority -- more force than the SEAT hold, less than DESCEND's full 12000 mV so a real
// hard-stop contact doesn't slam -- specifically to force through that last bit of resistance.
// TIME-BASED, ON PURPOSE (2026-10-02 retune): FINAL now always runs for a FIXED LIFT_FINAL_FORCE_MS
// window -- it does not early-exit the moment it looks settled, it just forces at
// LIFT_FINAL_DEG_PER_S/LIFT_FINAL_MAX_MV for that whole window ("force its way down for a second or
// two," the user's words), because a cup/friction stall can look momentarily still without actually
// being seated. Once the window ends, it hands over to SEAT: plain `SEAT` if the arm is within
// LIFT_SEAT_ZERO_DEG of 0 at that moment, `SEAT(short)` otherwise (a genuine jam, not just friction --
// LIFT_SEAT_MV still holds it down either way so it cannot bounce). This replaced an earlier
// conditional version (seat early once "both close and still", else give up after LIFT_SEAT_MAX_MS) --
// LIFT_SEAT_MAX_MS, and the still-detection constants LIFT_SEAT_STILL_DEG_PER_S/LIFT_SEAT_STILL_MS it
// needed, are gone with it; a straight timer needs neither.
// After a true SEAT the lift holds LIFT_SEAT_MV forever (or until something physically disturbs it,
// see LIFT_SEAT_UNSEAT_DEG below).
// HEAT: -1500 mV is 12.5% of full volts -- a stalled V5 motor there still draws well under an amp (the
// winding is about 1.4 ohm), a bit over a watt per motor, and the arm is resting on the frame so the
// seat voltage is only fighting bounce, not holding the arm up. Safe for a whole match -- but watch the
// brain's TEMP line the first time, and LIFT_SEAT_TEMP_LIMIT_C drops the seat to a plain brake-hold if
// a lift motor ever does get that hot.
//   Still stops short of 0 / shows SEAT(short) every time: raise LIFT_FINAL_MAX_MV first (5000 -> 6500),
//     then LIFT_FINAL_FORCE_MS. Check the brain's ANGLE line reads HOMED first -- "0" is only true
//     against a homed zero; an un-homed seed can read short even when the arm is physically all the way
//     down.
//   Thumps into the frame during FINAL: lower LIFT_FINAL_DEG_PER_S.
//   Bounces back up off the stop once truly seated: raise the magnitude of LIFT_SEAT_MV further (-1800).
//   Motors warm up while parked at the bottom: lower LIFT_SEAT_MV's magnitude (e.g. -1000), or
//     LIFT_FINAL_FORCE_MS if the motors are warming during FINAL itself (unlikely at 1.5 s).
//   Declares true SEAT while still visibly short: lower LIFT_SEAT_ZERO_DEG.
inline constexpr float LIFT_SEAT_MV = -800.0f;               // LEGACY: superseded by LIFT_DOWN_PUSH_MV below
// ===========================================================================
// FORCING DOWN AT THE BOTTOM — the arm always tries to go below 0
// ===========================================================================
// The driver, 2026-10-07: "for matchlaod height, and bottom height when we are just down in code, it
// needs to be forcing itself down, below 0 even trying to, and maintaining that, the same speed it
// does when it resets lift position at the first."
//
// So whenever the commanded target is lift_preset::DOWN (and MATCHLOAD is DOWN, so matchload is the
// same thing), the arm does NOT settle into a position hold around 0 and does NOT ease off once it
// gets there: Phase::SEAT drives a constant LIFT_DOWN_PUSH_MV into the hard stop for as long as the
// target stays DOWN. "The same speed it resets at" is literal -- this defaults to LIFT_HOME_MV, the
// voltage homing drives down at, so the two stay equal unless someone deliberately splits them.
//
// WHAT LEAVES IT: any new target at all. A preset, the D-pad, R1's trip to the intake, the 7.5 deg
// clamp hop, the start of the intake sequence -- every one of those is a setTarget(), and SEAT is not
// one of the "flying" phases setTarget() merges into, so they all start a fresh move and the push
// stops on that tick. Nothing special was needed to make that work.
//
// WHAT STILL WORKS, because this stays inside the existing SEAT phase rather than being a new one:
// isSeated() and isSettled() are both true in SEAT, so auton::waitLiftSeated() still returns, the
// autoclamp still comes back on at the seat, the matchload grab still fires and the clamp hop can
// still lift off the bottom. Homing is unaffected: it only writes the zero on the tick it stops, and
// after HOME DONE it never re-zeroes, so a continuous push afterwards cannot move the frame.
// INDEPENDENT OF LIFT_HOME_MV, on purpose. It STARTS equal to it (-3500), which is "the same speed it
// does when it resets lift position at the first", but they are two separate knobs: lower THIS one on
// its own if SEAT(hot) shows up or the motors run warm, without slowing homing back down.
inline constexpr float LIFT_DOWN_PUSH_MV = -3500.0f;         // TUNE: hold-down drive at the bottom
// HEAT. This is a genuine stall, so the motors dissipate V^2/R with no back-EMF: at 3.5 V into a V5
// motor's ~4.8 ohm that is ~0.73 A and ~2.6 W each, ~5 W across the pair, versus ~0.14 W at the old
// -800 mV. Low in absolute terms but ~19x more, and it now runs for the whole time the arm is parked
// down rather than stopping after LIFT_REST_AFTER_MS. The EXISTING seat guard still covers it: SEAT
// checks the motor temperature twice a second and brakes above LIFT_SEAT_TEMP_LIMIT_C, so this cannot
// cook unattended. If the motors still get warm over a match, lower THIS constant -- holding against
// a hard stop very likely does not need the full homing voltage.
// Where DESCEND hands over to FINAL. Also doubles as the "close enough to re-check" threshold for
// un-seating (see LIFT_SEAT_UNSEAT_DEG below) -- was a separate constant (LIFT_SEAT_DETECT_DEG, 6 deg)
// before the 2026-10-02 retune moved the whole handover down to 3 so the fast bands run longer.
// DOWN-relative (2026-10-07): 3 deg above the hard stop = -28.9 now. L2_OUTTAKE_STOP_DEG is defined as
// this constant, so it follows automatically and needs no separate edit.
inline constexpr double LIFT_FINAL_START_DEG = lift_preset::DOWN + 3.0;  // TUNE: hand DESCEND over to FINAL this close to the bottom
inline constexpr double LIFT_FINAL_DEG_PER_S = 20.0;         // TUNE: constant creep speed for the last stretch (was 12)
// RAISED 5000 -> 7000 (2026-10-03). The driver's reset complaint was "more speed/power to go down to
// reset", and this is the "power" half: FINAL is the only phase with real authority below
// LIFT_FINAL_START_DEG, and 5000 mV was not always enough to push the last couple of degrees through a
// cup caught in the intake or plain six-bar/chain friction -- which is exactly what SEAT(short) means.
// 7000 is the same cap L2_PRESS_MAX_MV uses to press a pin onto a stack without crumpling it, so it is a
// force this mechanism is already known to survive; it is still well under DESCEND's full 12000.
inline constexpr float LIFT_FINAL_MAX_MV = 3500.0f;          // TUNE: magnitude, downward authority during FINAL, red retune 2026-10-07 (was 7000)
// FINAL forces at the above speed/authority for exactly this long, then hands to SEAT regardless of
// where it ended up (see LIFT_SEAT_ZERO_DEG for how that is judged). ~1.2 deg of this 1.5 s is normally
// spent just closing the remaining 3 deg at LIFT_FINAL_DEG_PER_S; the rest is pure forcing time against
// resistance.
inline constexpr int LIFT_FINAL_FORCE_MS = 1500;             // TUNE: "a second or two" of forcing, per the user
// ...AND THEN IT KEEPS GOING UNTIL THE ARM IS REALLY AT 0 (2026-10-03). The driver: the reset "needs to
// constant apply down so its in that position until rotation reads steady around 0". So
// LIFT_FINAL_FORCE_MS is now a MINIMUM, not the whole window: once it has elapsed, FINAL carries on
// forcing until the position is within LIFT_SEAT_ZERO_DEG of 0 AND the arm has been still for
// LIFT_FINAL_STEADY_MS -- or until LIFT_FINAL_MAX_MS, the backstop, which hands over as SEAT(short).
//
// THIS IS STRICTLY MORE FORCING THAN BEFORE, AND THAT IS THE POINT -- read this before "simplifying" it.
// CLAUDE.md bans an earlier CONDITIONAL version of this handover, and it is worth being precise about
// why this is not that: the banned one could exit EARLY, the moment the arm looked "close and still",
// and a friction stall three degrees short looks exactly like that -- so it declared SEAT above the
// frame. This one can only ever exit LATE. It cannot seat before LIFT_FINAL_FORCE_MS no matter how
// settled the arm looks, and after that the only thing that ends it early is being genuinely AT zero.
// A stall short of 0 no longer ends the phase at all; it just keeps getting pushed until the cap.
//   Still shows SEAT(short) with the arm visibly short: raise LIFT_FINAL_MAX_MV first, then
//     LIFT_FINAL_MAX_MS. Check the ROT line really does read ~0 at the hard stop.
//   The reset takes too long before the route moves on: lower LIFT_FINAL_MAX_MS (it is the only thing
//     that can make this phase long), or raise LIFT_FINAL_STEADY_DEG_PER_S so "still" is easier to meet.
inline constexpr double LIFT_FINAL_STEADY_DEG_PER_S = 3.0;   // TUNE: what counts as "steady" (mirrors REST's)
inline constexpr int LIFT_FINAL_STEADY_MS = 100;             // TUNE: ...held this long, within SEAT_ZERO of 0
inline constexpr int LIFT_FINAL_MAX_MS = 3000;               // TUNE: hard cap on the whole forcing phase
inline constexpr double LIFT_SEAT_ZERO_DEG = 1.0;            // TUNE: within this of 0 when FINAL ends = true SEAT
// If something knocks a seated arm this far back up above LIFT_FINAL_START_DEG, the descent restarts
// (DESCEND, then FINAL again) and walks it back down rather than leaving it hanging.
inline constexpr double LIFT_SEAT_UNSEAT_DEG = 4.0;          // TUNE
inline constexpr double LIFT_SEAT_TEMP_LIMIT_C = 50.0;       // safety net, not a tuning knob

// --- REST: motors loose once the arm is settled at the bottom ---------------
// THE DRIVER'S REQUEST, 2026-10-03: "turn off lift pid on default state once it settles and is in that
// down state, keep motors loose so they arent being usefd and burn out." So SEAT is no longer the
// final state at the bottom. Once the arm has been SEATed and genuinely still for LIFT_REST_AFTER_MS,
// the lift enters REST: brake mode COAST and 0 V. No PID, no LIFT_SEAT_MV push, and not the firmware
// brake-hold either (which is a position loop and does draw current) -- the motors are simply free.
// The frame is holding the arm, not the motors. Brain: `REST`.
// REST still counts as seated/settled everywhere (isSeated(), isSettled(), waitLiftSeated(), Scoring's
// at-the-bottom tests), and ANY new command, the D-pad, or clamped-mode R1/R2 leaves it instantly and
// puts the brake mode back.
// COASTING IS ONLY EVER DONE AT THE BOTTOM. The power-on / no-target state is still a BRAKE-hold, so
// an arm that powers on part-way up cannot fall (CLAUDE.md safety rule 1 is unchanged), and letting go
// of the manual override still brake-holds where it is.
// THE ONE RISK, and the guard. SEAT exists partly because the arm "does not park above the frame and
// cannot bounce": the lift's rubber bands pull UP, hardest at the folded position, so with the motors
// loose the arm may creep off its stop. If the arm rises past LIFT_REST_WAKE_DEG while resting, the
// lift re-seats it (FINAL's forced push, then SEAT) and rests again -- at most LIFT_REST_MAX_WAKES
// times, after which it stays in SEAT and keeps the gentle -1500 mV rather than pulsing forever.
//   The arm creeps up off the stop, or the brain cycles REST -> FINAL -> SEAT -> REST: set
//     LIFT_REST_AFTER_MS to 0, which TURNS REST OFF ENTIRELY and keeps the old always-seated
//     behaviour. (Then watch the TEMP line instead: -1500 mV is about a watt per motor.)
//   The motors still feel stiff at the bottom: check the brain says REST, not SEAT -- SEAT means the
//     stillness test never passed, so raise LIFT_REST_STILL_DEG_PER_S.
// 0 = NEVER COAST AT THE BOTTOM (driver, 2026-10-07). REST was the opposite of what is wanted now:
// it let go of a seated arm and coasted the motors, where the arm must instead keep forcing itself
// down. Setting this to 0 is the switch the phase was already written around -- SEAT simply never
// hands over, and keeps pushing LIFT_DOWN_PUSH_MV for as long as the target stays DOWN.
// (It also stops the IDLE-at-the-bottom branch coasting when NOTHING is commanded; that case
// brake-holds instead, which is correct -- with no target there is nothing asking for a push.)
inline constexpr int LIFT_REST_AFTER_MS = 0;                 // TUNE: 0 = never coast (was 300)
inline constexpr double LIFT_REST_STILL_DEG_PER_S = 3.0;     // TUNE: what counts as "not moving" for the above
// DOWN-relative (2026-10-07): 4 deg above the hard stop = -27.9 now.
inline constexpr double LIFT_REST_WAKE_DEG = lift_preset::DOWN + 4.0;  // TUNE: rising past this while resting = re-seat it
inline constexpr int LIFT_REST_MAX_WAKES = 3;                // TUNE: then stay in SEAT instead of pulsing

// --- The L1 GRAB at the intake handoff (see "SCORING WITH L2"'s sibling, "THE GRAB", in lift.hpp) --
// L1 pressed while the macro is parked at the INTAKE handoff no longer just closes the jaws and
// rises: the arm first FORCES ITSELF DOWN onto the pin for a moment, closes the jaws while it is
// pressed down there, and only then rises. The driver's words: "I want the claw to force its way
// down, stop spinning the intake when its doing that and grabbing (and force it down, like -40 speed
// or so) while grabbing, should be like a 300ms movement going down for 200ms then grab, then go up."
// The intake STOPS for the whole push (one decision per tick, in periodic::tick() -- never a second
// writer), so the rollers cannot drag the pin out from under the jaws while they close.
// SPEED. Open-loop, a plain percentage of LIFT_MAX_MV like every other *_PCT here. 40% is 4800 mV.
// (The driver said "-40 speed", which in their old library meant Motor::move(-40) on the -127..127
// scale, i.e. about 31%. 40% of volts is the same number in this codebase's units and a little
// firmer; drop it to 31 to match the old library exactly.)
//   The pin gets pushed out of the jaws / the intake is lifted off the floor: LOWER it.
//   The jaws close above the pin instead of around it: RAISE it, or raise L1_GRAB_CLOSE_AT_MS.
inline constexpr double L1_GRAB_PUSH_PCT = 40.0;             // TUNE: % of LIFT_MAX_MV, downward
// When the jaws close, measured from the start of the push.
inline constexpr int L1_GRAB_CLOSE_AT_MS = 200;              // TUNE
// How long the push lasts. The arm keeps pressing down until BOTH this has elapsed AND the jaws have
// had CLAW_CLOSE_TIME_MS to shut from the moment they were commanded -- it must never pull up out of
// a claw that is still closing (the same interlock CLAMPING enforces). With the numbers here that is
// max(300, 200 + 150) = 350 ms of push.
inline constexpr int L1_GRAB_PUSH_MS = 300;                  // TUNE
// Safety net: if nobody ever acknowledges the close request (no Scoring::update() running at all),
// stop pushing after this long anyway rather than leaning on the pin for the rest of the match.
inline constexpr int L1_GRAB_MAX_MS = 1000;                  // TUNE

// --- THE CLAMP-RISE: every clamp lifts the arm a little, at once ------------
// THE DRIVER'S REQUEST, 2026-10-03: "when we clamp whether manual in driver or auto clamp in auto, we
// need the lift to automatically always move up just very slightly, move up to 5 degrees quick and
// nice", then measured on the robot the same day: "after claw grabs in that down state, we need to
// raise to 8 deg not 5". So the instant the jaws shut -- A, the autoclamp, L2's at-the-bottom clamp
// toggle, auton::clawGrip() -- Scoring sends the arm UP by this much.
//
// RELATIVE, NEVER ABSOLUTE, and never downward: the goal is (wherever the arm is now) + this, clamped
// to lift_preset::MAX. Clamps normally happen with the arm at the bottom -- "that down state" -- which
// is where "raise TO 8 deg" and "raise BY 8 deg" are the same thing; starting from higher up it is still
// an 8 deg hop, and from a height where +8 would exceed MAX it simply does not move. There is no path
// through this that can command a LOWER height than the arm is already at.
//
// IT IS A PRECISE MOVE, and that is not a style choice: setTargetFast()'s LIFT_COARSE_TOLERANCE_DEG (10)
// is WIDER than this whole hop, so a coarse move would be "already arrived" on the first tick and the
// arm would never move at all. The ordinary LIFT_TOLERANCE_DEG (5) is narrower than 8 so it would at
// least move, but it would call anywhere from 3 to 13 deg "arrived" -- which is most of the hop's worth
// of slop. So Lift::startClampRise() asks for the tight LIFT_PRECISE_TOLERANCE_DEG (1) band.
// (At the original 5 deg this was not a judgement call at all: 5 is INSIDE LIFT_TOLERANCE_DEG, so even
// the ordinary band would have latched "arrived" without moving. Keep it precise if this is tuned back
// down.)
// WHAT THE MOVE LOOKS LIKE at 8 deg: 3 deg of braking curve (sqrt(2 * LIFT_DECEL_DEG_PER_S2 * 3) caps it
// around 85 deg/s) and then the LIFT_APPROACH_ZONE_DEG walk-in -- roughly 0.15 s, no full-voltage
// section, and the hold's LIFT_HOLD_KD absorbs the arrival. At 5 deg it was pure approach zone and
// slightly gentler still; both are the "quick and nice" the driver asked for.
// Brain's LIFT line: `CLAMP UP` -- deliberately NOT the number, which would be a lie the moment this
// constant is tuned (it just was, 5 -> 8). The height is on the same line anyway, as `-> 8.0`.
//
// IT WAITS FIRST (driver, 2026-10-07): "whenever auto clamp actually grabs ANYWHERE besides inside the
// intake (that's special), we need lift to move up 7.5 degrees from where it is automatically, but
// delay it so it clamps for 400ms before doing that movement." So the arm stays exactly where the
// clamp caught it for the mode's delay below, letting the jaws take a real grip on the cup or pin, and
// only then goes to CLAMP_GRAB_HEIGHT -- an absolute height, not a rise. Both numbers are below.
//
// WHAT TRIGGERS IT: a clamp the AUTOCLAMP DETECTOR made, outside the intake sequence -- picking up
// from matchload, from the bottom (-39), or off the field. Nothing else:
//   - a MANUAL clamp never hops. Claw::clamp() records ClampSource::MANUAL, so the driver's A press
//     and every clamp the scoring macro makes itself (CLAMPING, the L1 GRAB) are excluded by source
//     alone. Only the detector in Claw::update() produces ClampSource::AUTOCLAMP.
//   - THE INTAKE SEQUENCE IS EXCLUDED and has its own, different reaction: settle, push down to
//     GRAB_PUSH_DEPTH_DEG, rise to level 1 releasing the pivot. See "THE INTAKE AUTOCLAMP GRAB".
//     Entering those states clears any pending hop, so the two can never both run.
//   - Scoring states that are driving the arm themselves (STACKING, RELEASING, SCORED, RETURNING,
//     CLAMPING, GRABBING, LIFT_SETTLE, MOVING, and the GRAB_* states) are excluded: a commanded move
//     always wins. A re-clamp during an L2 cycle is excluded twice over -- the autoclamp is inhibited
//     for the whole cycle, and Lift::isScoreCycle() is checked again when the delay expires.
//   - a clamp that happened while the robot was DISABLED: periodic::reset() drops the event at the
//     start of each enabled mode, so a pre-match grip can never queue a rise that fires in auton.
//   - an EMPTY CLAW at the end of the delay (the pin was knocked out while it waited): no rise.
// ANY DRIVER INPUT CANCELS IT, during the delay or the rise, because the pending hop is cleared by
// Scoring::enter() and Scoring::reset() -- which every button goes through.
//
// THE DELAY IS PER MODE (driver, 2026-10-07): "when auto clamp grabs in auton, it needs to lift up
// immediately. When autoclamp grabs in driver ... it lifts up after 200ms." Auton has no time to
// spare, so 0 means the rise starts on the SAME TICK as the clamp -- handleClampEvent() records the
// hop and serviceClampHop() runs immediately after it in Scoring::update(), so a 0 delay never waits
// for the next tick. Which value is used is decided WHEN THE CLAMP FIRES and then held, from
// Scoring::setAutonMode(), which periodic::tick() feeds from Inputs::auton -- NOT from
// pros::competition::is_autonomous(), because an auton route run from the brain menu has no field
// switch and would otherwise be treated as driver control.
//   It does not clear the thing it just gripped: raise CLAMP_GRAB_HEIGHT.
//   It lifts far enough to matter / fights the next move: lower it (1-2 deg still moves, see above).
//   It rises before the grip is solid and drops the cup: raise CLAMP_RISE_DELAY_DRIVER_MS.
// AN ABSOLUTE HEIGHT, NOT A RISE (the driver, 2026-10-07: "make it 37.5 height, not a rise"). After a
// non-intake autoclamp grab the arm goes TO 37.5 in the zero-at-the-bottom frame, wherever it grabbed
// from. Grabs happen at the bottom (0), so in practice this is a ~37.5 deg lift.
// FOLLOW THE CONSEQUENCE: a grab with the arm ALREADY ABOVE 37.5 moves it DOWN to 37.5. That is what
// "go to 37.5" means and it is deliberate; the old relative version could only ever go up.
inline constexpr double CLAMP_GRAB_HEIGHT = 37.5;            // TUNE: arm degrees, ABSOLUTE
inline constexpr int CLAMP_RISE_DELAY_AUTON_MS = 0;          // TUNE: 0 = same tick as the clamp
inline constexpr int CLAMP_RISE_DELAY_DRIVER_MS = 200;       // TUNE: driver 2026-10-07 (was 400 both)

// --- The D-pad manual override's speeds -------------------------------------
// D-pad Up/Down are plain open-loop voltage straight into Lift::update(), ahead of everything else
// (CLAUDE.md safety rule 3). These are the only two numbers that decide how that feels.
// DOWN IS HALF SPEED, the driver's call 2026-10-03 ("dpad down needs to move lift at 50% speed"):
// gravity is already helping on the way down, so full voltage down is much faster than full voltage up
// and it ends against the hard stop -- 50% makes the two directions feel alike and makes nudging the arm
// down onto a pin controllable instead of a drop. UP stays at 100: nothing assists it and there is
// nothing at the top to run into but the software limit.
// Note these ONLY affect the D-pad. Clamped-mode R1/R2 have their own R_LIFT_UP_PCT / R_LIFT_DOWN_PCT,
// and a commanded move's full-voltage DRIVE phase is LIFT_MAX_MV regardless -- lowering these does not
// slow any preset, macro or auton move down.
//   Manual down still feels like a drop: lower LIFT_MANUAL_DOWN_PCT further (40, 35).
//   Manual down is too slow to be useful: raise it (65, 80).
inline constexpr double LIFT_MANUAL_UP_PCT = 100.0;          // TUNE: D-pad Up, % of LIFT_MAX_MV
inline constexpr double LIFT_MANUAL_DOWN_PCT = 50.0;         // TUNE: D-pad Down (the driver's 50%)

// --- THE AUTON LIFT PUSH (auton::liftPush() / liftPushStop()) ---------------
// THE DRIVER'S REQUEST, 2026-10-03: "we need a lift movement when scoring pin at very bottom that moves
// lift at constant -40% speed down until we stop it. just easy way to toggle in auto." So this is not a
// move to a height and not a profiled descent -- it is a plain, constant, open-loop DOWNWARD voltage
// that a route switches on and off, for pressing a pin down at the very bottom where there is no height
// left to command.
// It goes through Lift::driverPower(), the SAME manual path the D-pad uses, which is why it needs no new
// mode in the lift and still obeys every rule: one owner of the motors, it cancels any active target the
// instant it starts, and liftPushStop() brake-holds exactly where the arm is rather than resuming an old
// target. It is LATCHED (like holding a button), so a route must stop it -- and auton::end() stops it
// too, so a route that forgets, or is cut off by the field switch, cannot leave the arm leaning on the
// floor.
// SETTLED AT 45 (2026-10-03), after trying 20 on the robot and reverting: the driver asked for "half the
// speed", then "nvm, turn speed back to 45 actually" -- i.e. 20% was too weak and 45 is slightly FIRMER
// than the 40 it started at. That is the floor below being real: this is open-loop, and this lift's
// rubber bands pull UP hardest at exactly the folded position where this push is used, so under some
// percentage the bands win and the arm does not descend at all -- it just sits drawing current. 20 was
// evidently at or under that; the usable range is apparently nearer 40-45 than 20-30. Do not "restore"
// a slower value without re-checking it on the robot.
// This is the DOWN push only. AUTON_LIFT_PUSH_UP_PCT is separate (40) because they are different jobs:
// this one mostly has to HOLD the arm down on a pin while the jaws let go, working WITH gravity; the up
// nudge works against it. The 20 experiment did not touch the up nudge, which is why they are separate.
//   It does not press hard enough to seat the pin, or does not move at all: raise it.
//   It lifts the back of the robot / slams the pin down / stalls the motors warm: LOWER it -- but see
//     the band floor above before going far below 40.
inline constexpr double AUTON_LIFT_PUSH_PCT = 45.0;          // TUNE: % of LIFT_MAX_MV, downward (40 -> 20 -> 45)
// ...and the same thing UPWARD (auton::liftPushUp() / liftPushUpFor()), for the driver's first-pin nudge.
// THIS IS THE TRANSLATION OF THEIR OLD-LIBRARY LINES, which were:
//     liftMotors.move(50);  pros::delay(300);  liftMotors.move(0);
// move() took -127..127, so 50 there is 50/127 = 39.4% of full volts -- hence 40 here, and
// `auton::liftPushUpFor(300)` is those three lines exactly, in this codebase's units and through the one
// code path that owns the lift motors.
// WORTH KNOWING BEFORE CHOOSING IT: a timed open-loop push is not repeatable. How far the arm actually
// travels in 300 ms depends on battery voltage, whether a pin is held, and where in the six-bar's arc it
// starts -- a fresh battery and an empty claw go visibly further than a flat one with a pin. If what is
// wanted is "end up 9 degrees up", lift_preset::FIRST_SCORE + auton::liftToFirstScore() lands there
// repeatably instead. This is here because it is what the driver already knows works.
inline constexpr double AUTON_LIFT_PUSH_UP_PCT = 40.0;       // TUNE: % of LIFT_MAX_MV, upward (old move(50))

// --- Clamped-mode R1 / R2 lift (see src/main.cpp and README "Driver guide") --
// While the claw is CLAMPED on a pin, R1/R2 stop being intake/extake and become lift up/down.
// Up is plain voltage; down goes through the banded descent above (so it seats at the bottom exactly
// like L2 does) with this as its top-band speed.
inline constexpr double R_LIFT_UP_PCT = 90.0;                // TUNE: R1 up, % of full voltage
inline constexpr double R_LIFT_DOWN_PCT = 65.0;              // TUNE: R2 down, % of free speed (~98 deg/s)

// --- Arrival bands ---------------------------------------------------------
// How close counts as arrived for the stack levels and MATCHLOAD. 5 deg of final error at a stack
// level is fine -- the pin only has to land in the cup. A goal already inside this band is "arrived"
// without moving at all. With the final approach above, moves normally land inside ~2.5 deg anyway.
inline constexpr double LIFT_TOLERANCE_DEG = 5.0;            // TUNE: stack levels, MATCHLOAD
// INTAKE is the exception: the claw has to line up with the handoff, so it is a PRECISE move and gets
// this tight band. Lift::setTarget(h, true) marks a move precise, and any goal within half a degree of
// lift_preset::INTAKE is promoted automatically so no call site can forget.
inline constexpr double LIFT_PRECISE_TOLERANCE_DEG = 1.0;    // TUNE: INTAKE only
// Any NON-PRECISE goal at or below this is "a trip to the bottom": it uses the banded descent + seating
// above instead of a position move, and Scoring's score() treats an arm already here as "nothing to
// score" (L2 is then a no-op). INTAKE (1.1) is inside it numerically but is always precise, so R1 keeps
// its own exact approach and never seats.
// DOWN-relative (2026-10-07): 5 deg above the hard stop = -26.9 now. lift_preset::INTAKE (23.0) is
// comfortably ABOVE this, so R1's handoff is an ordinary precise position move and never a descent.
inline constexpr double LIFT_BOTTOM_GOAL_DEG = lift_preset::DOWN + 5.0;  // TUNE
// Hands over to HOLD once the arm is inside its band AND slower than this. It has to sit ABOVE
// LIFT_APPROACH_MIN_DEG_PER_S or the crawl's floor speed would mean arrival never latches and every
// move ended on the LIFT_APPROACH_TIMEOUT_MS instead. With kP = 6 /s, the reference is down to 12
// deg/s about 2 deg out, so that is where a move actually latches -- comfortably inside the 5 deg
// tolerance, and the hold's LIFT_HOLD_KD (90) then absorbs 12 deg/s as ~1080 mV of opposing voltage.
//   Any bounce as it grabs the hold: raise LIFT_HOLD_KD first; lowering this to 8 is the second lever
//   (gentler handover, slightly slower finish).
// RAISED 12 -> 25 (2026-10-03), and this one is NOT optional: it must stay ABOVE
// LIFT_APPROACH_MIN_DEG_PER_S (now 12) or arrival can never latch -- the crawl's own floor speed would
// keep failing the test and every move would end on LIFT_APPROACH_TIMEOUT_MS instead. 25 also means a
// move latches into the hold sooner and while still moving a little, which is exactly the "go there fast
// and hold, even if its a bit off" that was asked for; LIFT_HOLD_KD (90) absorbs 25 deg/s as ~2250 mV of
// opposing voltage, so the arrival is damped rather than bounced.
//   Any bounce as it grabs the hold: raise LIFT_HOLD_KD first, then lower this (but never to or below
//   LIFT_APPROACH_MIN_DEG_PER_S).
inline constexpr double LIFT_STOPPED_DEG_PER_S = 15.0;       // TUNE: must stay > LIFT_APPROACH_MIN_DEG_PER_S; red retune 2026-10-07 (was 25)
// The final approach can't take longer than this (arm fighting something, loop badly tuned): the lift
// hands over to HOLD, whose position term then pulls in whatever is left, gently.
// SHORTENED 1500 -> 400 (2026-10-03). This is the backstop that stops the final crawl dragging on: it
// hands over to the hold, whose position term pulls in whatever is left. With the retuned approach above
// the crawl covers 4 deg in well under 200 ms, so 400 is a real backstop and not the normal path.
inline constexpr int LIFT_APPROACH_TIMEOUT_MS = 700;         // TUNE: slower crawl at 8 deg/s floor, 2026-10-07 (was 400)
// Velocity estimate: position differenced every tick, then this much of each new sample is blended in
// (1.0 = raw, no filtering). The rotation sensor's quantisation is 0.0025 arm deg, i.e. 0.25 deg/s per
// count, so there is nothing to filter out except task jitter -- and filtering costs LAG, which makes
// the brake react late and overshoot. 0.8 is a quarter of a tick of lag; 0.5 (the old value) was a
// whole tick, which at 150 deg/s is 1.5 deg of stale position.
// (Checked while chasing the "slow start/middle": at 0.8 the EMA's effective lag is (1-a)/a * 10 ms =
// 2.5 ms, i.e. 0.4 deg at full speed. It is NOT what delays DRIVE ending or BRAKE responding -- that
// was the decel value and the 9000 mV velocity clamp above. Leave it at 0.8.)
inline constexpr double LIFT_VEL_FILTER = 0.8;

// ---------------------------------------------------------------------------
// Lift motion — how those knobs fit together. Read lift.hpp's class comment for the full picture.
// ---------------------------------------------------------------------------
// There are exactly two kinds of commanded move, and one manual input, all inside the single 10 ms
// Lift::update() that owns the motors:
//
// A POSITION MOVE (every goal except a trip to the bottom) is DRIVE -> BRAKE -> APPROACH -> HOLD:
//   DRIVE     full +-LIFT_MAX_MV, the exact thing the D-pad does, until the arm is up to the speed
//             the braking curve allows -- so a move is still off the line instantly.
//   BRAKE     the velocity loop tracks min(cruise cap, braking curve) all the way in.
//   APPROACH  the same loop, now tracking LIFT_APPROACH_KP_PER_S * (degrees left) capped at
//             LIFT_APPROACH_DEG_PER_S, so the arm is down to a crawl before it stops.
//   HOLD      the damped position hold (KP/KI/KD above), with no P correction and a FROZEN integral
//             inside the settle band. Its integral carries the learned gravity voltage over to the
//             next move, so no move ever starts by guessing it.
//   PARK      settled: that learned voltage, frozen, pushed as a constant with no feedback at all, so
//             it cannot hunt. Entered after LIFT_PARK_CONFIRM_MS inside the band or
//             LIFT_SETTLE_TIMEOUT_MS of holding; a move that never arrives parks as PARK(t/o) after
//             LIFT_MOVE_TIMEOUT_MS. Left only on a LIFT_REENGAGE_DEG drift or a new command.
//
// A DESCENT (a non-precise goal at or below LIFT_BOTTOM_GOAL_DEG: L2, B, auton DOWN, R2 in clamped
// mode) is DESCEND -> FINAL -> SEAT: the three speed bands above, then (below LIFT_FINAL_START_DEG) a
// dedicated closed-loop push at LIFT_FINAL_DEG_PER_S with up to LIFT_FINAL_MAX_MV of authority -- more
// force than the seat hold, enough to push through a cup or friction a few degrees short of 0 -- for a
// fixed LIFT_FINAL_FORCE_MS, and only then a constant LIFT_SEAT_MV pressed into the hard stop. It does
// not stop above the frame and it does not bounce; see "Seating on the hard stop" below for why the
// original single-step version could park short. Finally, once that seated arm has been still for
// LIFT_REST_AFTER_MS, REST coasts the motors at 0 V so they draw nothing (see "REST" below) -- the
// only place in the program that ever coasts.
//
// THE MANUAL INPUT (D-pad Up/Down, and R1 in clamped mode) is plain voltage straight into the same
// loop, ahead of everything else; it cancels any target and any homing instantly, and letting go
// brake-holds where it is.
//
// Why this and not the designs before it: a jumped setpoint with kP = 600 oscillated (d2978c7); a
// trapezoidal profile + leash + feedforward + PID + gravity term + arrive/hold hysteresis (6b8165c)
// was slow off the line and still oscillated because its hold used a FIXED gravity feedforward that is
// wrong at most angles of a six-bar; and DRIVE -> BRAKE -> firmware-hold (3723ef2) overshot and rang,
// because LIFT_BRAKE_DECEL_DEG_PER_S2 = 2000 asked for more braking than the arm can do, the velocity
// estimate lagged a whole tick, enterBrake() seeded the loop's integral with an upward kick, and the
// handover at up to 20 deg/s dropped a still-moving springy arm into the firmware's stiff hold, which
// stored the energy and gave it back. All four of those are fixed above: realistic decel, less filter
// lag, the integral seeded from the learned gravity voltage, a crawl before the stop, and a hold with
// an explicit damping term. Nothing re-drives or re-chases any more -- the hold IS the correction, and
// it is gentle, so LIFT_REENGAGE_DEG / LIFT_MAX_REDRIVES are gone.

// ---------------------------------------------------------------------------
// Lift guards
// ---------------------------------------------------------------------------
// Stall guard: a phase that WANTS real motion (its speed reference is at least 5 deg/s) but the arm
// moved less than STALL_DEG in STALL_MS = it can't get there (preset above the real top, or jammed).
// The lift gives up on that target, holds where it is and the brain shows BLOCKED. It is NOT checked
// once the final approach has tapered the reference below 5 deg/s (where slow progress is the plan,
// not a jam), nor in HOLD, nor below LIFT_FINAL_START_DEG during a descent -- an arm that has stopped
// down there is seated, which is the goal, not a fault. It cannot false-trip: 5 deg/s covers 0.5 deg
// in 100 ms, a fifth of the window.
//   Shows BLOCKED on a normal move: first check for a real jam or a preset above the top. Then raise
//   LIFT_STALL_MS.
inline constexpr double LIFT_STALL_DEG = 0.5;                // TUNE: arm deg of progress that counts as moving
inline constexpr int LIFT_STALL_MS = 500;                    // TUNE
// Runaway guard: during DRIVE the lift pushes full voltage toward the goal, so the distance to the goal
// can only shrink. If it instead GROWS by this much from the smallest it has been during this DRIVE,
// the sensor or the motors are reversed (rotation sensor direction, or a LIFT_PORTS sign): cut power,
// "LIFT FAULT" on the brain. A reversed drive runs away at 150 deg/s and trips this within ~60 ms. A
// B press while the arm is still rising fast coasts up by a degree or two before it reverses -- that is
// far below this threshold, and it is why the threshold is not smaller.
// A FAULT is cleared by the manual override OR by a new explicit target (see Lift::setTarget()): the
// guard's job is to stop the motion happening now, not to brick every button until someone remembers
// to wiggle the D-pad. A real wiring fault simply re-faults on the next DRIVE.
//   LIFT FAULT on a normal move: run the direction check in the README before touching this number.
inline constexpr double LIFT_FAULT_DEG = 8.0;                // TUNE

// ---------------------------------------------------------------------------
// Lift trace — one line per 20 ms to the PROS terminal while the arm moves
// ---------------------------------------------------------------------------
// VS Code -> PROS -> "Brain Terminal" (or `pros terminal`), press a lift button, paste the lines back.
// Columns: `LIFT <ms> <phase> <pos*10> <goal*10> <vel deg/s> <out mV>`; phase is D=DRIVE, B=BRAKE,
// A=APPROACH (the final crawl), H=HOLD, V=DESCEND (a banded trip to the bottom), R=RETURN (L2's fast
// run home, same bands as V), Z=FINAL (the forced push-to-0 step below LIFT_FINAL_START_DEG, always
// LIFT_FINAL_FORCE_MS long), S=SEAT (pressed onto the hard stop, or holding short -- see
// LIFT_SEAT_ZERO_DEG), M=manual (D-pad, or R1 in clamped
// mode), h=homing, F=fault, I=idle. Integers only (no float formatting on the
// control task), sampled every LIFT_TRACE_EVERY_TICKS, written from a separate low-priority task that
// drains a ring buffer -- the 10 ms control loop never calls printf. Keeps going LIFT_TRACE_TAIL_MS
// after the arm stops so the hold is visible too. Set false to silence it entirely (no task started).
inline constexpr bool LIFT_TRACE = true;
inline constexpr int LIFT_TRACE_EVERY_TICKS = 2;             // 2 x 10 ms = one line per 20 ms
inline constexpr int LIFT_TRACE_TAIL_MS = 500;

// ---------------------------------------------------------------------------
// Lift homing — find the hard stop and zero there, once per program run
// ---------------------------------------------------------------------------
// Why: the sensor's own counter cannot tell you where the arm is at startup. get_position() is a
// retained multi-turn value that only re-folds to its single-turn angle on a real power cycle, and one
// turn is just 90 arm degrees at LIFT_ROTATION_RATIO 4 against 179.5 degrees of travel -- so no
// software guess works for every boot. Homing removes the question mechanically: drive the arm down at
// a steady voltage until the sensor stops changing, then set_position(0) there. There is no
// absolute-angle plausibility gate; the hard stop is the ground truth, and the stall detector is what
// separates "reached it" (HOME DONE, zeroes) from "never stopped" (HOME FAIL, zeroes nothing).
//
// This is the ONE deliberate exception to "nothing moves at startup" (CLAUDE.md safety rule 1), and it
// is kept narrow: homing never runs in initialize(). It runs on the first ENABLED mode of a program
// run -- autonomous(), or opcontrol() if auton did not already do it -- so the field/competition
// switch, not power-on, is what makes the arm move. It runs inside Lift::update() as a mode, so the
// 10 ms task is still the only thing that writes the lift motors, nothing blocks on it, and holding
// D-pad Up/Down cancels it instantly (safety rule 3 is untouched). Lift targets asked for during
// homing are queued and run the moment it finishes.
// IS HOMING RUN AT ALL? YES, since 2026-10-07 -- the driver: "go back to how we reset our position to
// 0 by moving lift down at the start." It is how the frame is defined: updateHoming() drives the arm
// down onto its hard stop and set_position(0)s the sensor THERE, which is the one and only sensor
// write in the program. lift_preset::DOWN is 0.0 and every other height is a positive number measured
// up from that stop, so the numbers mean the same thing on every boot however the brain was started.
//
// It replaced a software startup offset that could not work: get_position() is a RETAINED multi-turn
// counter that only re-folds to the single-turn angle on a real sensor power cycle, so cold and warm
// starts disagreed and no fixed window was right for both. See "THE STARTUP UNWRAP IS GONE" above.
//
// IT RUNS ONCE PER PROGRAM RUN, non-blocking, on the first ENABLED mode: autonomous() asks for it, and
// opcontrol() asks again for the driver-only case. requestHome() only acts from Home::IDLE, so a real
// match homes in auton and the opcontrol call is a no-op. Homing runs as a mode inside Lift::update(),
// so the 10 ms task is still the only thing writing the lift motors, nothing blocks on it, drive moves
// carry on underneath it, and holding D-pad Up/Down cancels it. Lift targets asked for during homing
// are queued and start the moment it ends.
//
// IF IT FAILS, THE FRAME IS WRONG AND NOTHING ELSE CATCHES IT. A home that never reaches the stop
// times out, writes no zero, and leaves the lift in the raw sensor frame where every preset lands low.
// The brain's LIFT0 line is the check: "frame OK" means the zero was written, "frame RAW" means it was
// not. See LIFT_HOME_TIMEOUT_MS below -- that is the constant that decides whether a home from high up
// finishes at all.
inline constexpr bool LIFT_HOME_ON_ENABLE = true;            // BACK ON, the driver's call 2026-10-07
// 1.75x FASTER (the driver, 2026-10-07: "during that recalibration, lift needs to move at 1.75x the
// speed down it currently is"): -2000 * 1.75 = -3500. Capped by LIFT_MAX_MV (12000), which this is
// comfortably inside. It also shortens the trip, which matters because LIFT_HOME_TIMEOUT_MS below is
// what decides whether a home from high up finishes at all -- a timed-out home writes NO zero and
// leaves every height wrong, so speed here buys reliability, not just time.
inline constexpr float LIFT_HOME_MV = -3500.0f;              // TUNE: downward drive while homing (was -2000)

// Don't start looking for "stopped" until the arm has been driving this long, so a sticky start isn't
// mistaken for the bottom.
inline constexpr int LIFT_HOME_MIN_DRIVE_MS = 250;           // TUNE
// "Stopped" = the raw sensor moved less than LIFT_HOME_STILL_DEG (SENSOR degrees, not arm degrees)
// across a LIFT_HOME_STILL_MS window. Small wobble at the stop is expected and allowed.
inline constexpr double LIFT_HOME_STILL_DEG = 2.5;           // TUNE: sensor degrees
// REBOUND MARGIN. Hitting the stop 1.75x harder makes the arm bounce more, and this window is what
// keeps the bounce from being read as "stopped": any movement over LIFT_HOME_STILL_DEG re-opens it, so
// the zero can only be written once the arm has genuinely been still for this long. Raised with the
// speed (200 -> 300) so a bigger bounce still has to settle out before anything is written.
inline constexpr int LIFT_HOME_STILL_MS = 300;               // TUNE (was 200)
// Plausibility window: at the stop, the sensor's own absolute angle (folded to +-180 sensor deg) must
// be within this much of its calibrated zero before we trust it and set_position(0). The user's own
// observation is that it settles around 358-359-0-1, i.e. within a degree or two. If the arm stopped
// somewhere else it is jammed on something, NOT at the bottom: homing then stops pushing, keeps the
// absolute-angle seed from zero(), and shows HOME FAIL instead of zeroing to a wrong place.
// Hard stop on the whole attempt. Stops pushing and shows HOME FAIL.
// A HOME THAT TIMES OUT WRITES NO ZERO, and the lift then runs in the raw sensor frame where every
// preset lands low by however far the bottom happens to read -- the exact symptom of "all the heights
// are going way lower than they should be". So this has to be long enough to cover the WORST start,
// which is the arm up at level 5 (lift_preset::STACK_LEVELS[4] above the stop). Raised 3000 -> 4000
// alongside the faster drive; the brain's LIFT0 line says HOME FAIL / frame RAW if it still runs out.
inline constexpr int LIFT_HOME_TIMEOUT_MS = 4000;            // TUNE (was 3000)

// Default timeout for lift.moveTo() when the caller doesn't give one (autonomous safety net so a
// stuck lift can't hang a routine forever). Pass a different value per call, or 0 to wait forever.
// A blocking moveTo() called while homing waits for homing AND the move, so give it room for both.
inline constexpr int LIFT_DEFAULT_TIMEOUT_MS = 4000;         // TUNE: half speed + ~32 deg more travel, 2026-10-07 (was 2500)

// ---------------------------------------------------------------------------
// Claw (pneumatic) + autoclamp sensor
// ---------------------------------------------------------------------------
// THE TWO CYLINDERS WERE CROSSED IN CODE UNTIL 2026-10-01. The port the code called the claw ('C')
// is physically the PIVOT cylinder, and the port it called the pivot ('D') is physically the CLAW.
// The user found this on the robot; the ports below are the corrected, physical assignment, and every
// polarity note here was re-derived from scratch afterwards (see the derivation under each constant).
inline constexpr char CLAW_PISTON_PORT = 'C';                // physically the claw (was 'C', which is the pivot)
// Which ADI level CLOSES the jaws.
//
// DERIVATION. Every build this robot has ever run wrote BOTH ADI pins LOW at construction (the claw
// object: Pneumatics(port, start_extended=false), then DigitalOut(port, LOW); the pivot object the
// same), so at power-on C and D are both LOW, which is also their unpowered level. On the 3723ef2
// build, R1/INTAKE wrote the "claw" object's open level to C (LOW) and the "pivot" object's BACK level
// to D (HIGH). The driver's report of that run was "clamp and pivot reversed at the intake": the claw
// was gripping when the code wanted it open. The pin that was HIGH at that moment was D -- the real
// claw. So HIGH closes the jaws, and the unpowered LOW level is OPEN.
// That also makes mechanical sense: a single-acting VEX cylinder needs air to grip, and this is the
// arrangement where losing air opens the jaws rather than locking them shut.
// CONSEQUENCE FOR STARTUP: the jaws are physically OPEN at power-on, so Claw starts in State::OPEN and
// writes the pin the level it already sits at -- nothing moves at startup (CLAUDE.md rule 1). It also
// means the claw cannot hold a preloaded pin with the program off; press A or L1 to grip one.
// This is a one-observation deduction, so confirm it in 10 seconds: at power-on the brain's CLAW line
// reads `CLAW OPEN D[0]`; press A and the jaws must CLOSE while the line reads `CLAMPED D[1]`. If A
// opens them instead, flip this one constant and nothing else.
inline constexpr bool CLAW_CLOSED_IS_ADI_HIGH = true;        // TODO(team): confirm with A on the brain's CLAW line
// ===========================================================================
// AUTOCLAMP — the distance sensor in the jaws, and when it is allowed to clamp
// ===========================================================================
// The sensor looks out between the jaws and sees both pins and cups. The rule (the driver's own
// words): while the claw is open and the autoclamp is enabled, anything at or inside
// AUTOCLAMP_RANGE_MM for AUTOCLAMP_CONFIRM_MS straight closes the jaws on it. The debounce is the
// whole point -- "if there's a random spike and then not something 50/100 ms later and it spikes back
// down, it's not real" -- and it is implemented by subsystems::AutoclampDetector, which is PURE (no
// sensor, no clock, no cylinder: see its header for the full rule and for why fresh samples are
// counted separately from elapsed time). Every threshold it uses is here.
inline constexpr int8_t CLAW_DISTANCE_PORT = 21;             // confirmed 2026-10-03 by the user (was 11)
// ON BY DEFAULT IN DRIVER (opcontrol() enables it every time it starts) *AND IN AUTON* (autonomous()
// enables it from AUTOCLAMP_ON_IN_AUTON, 2026-10-03 -- it used to be hard-off there and every route had
// to remember auton::autoclamp(true)), and OFF at power-on -- CLAW_AUTOCLAMP_AT_START stays false so a
// preloaded pin cannot fire a cylinder in initialize() (CLAUDE.md safety rule 1). Y still toggles it in
// a match, and auton::autoclamp(false) still turns it off per route.
inline constexpr bool AUTOCLAMP_ON_IN_DRIVER = true;
inline constexpr bool AUTOCLAMP_ON_IN_AUTON = true;          // the driver's call, 2026-10-03 ("should work in auto")
inline constexpr bool CLAW_AUTOCLAMP_AT_START = false;       // power-on / initialize(): never armed
// THE TRIGGER, AND IT IS DIFFERENT IN EACH MODE. The driver's measured numbers (2026-10-03): 55 mm in
// driver control, 65 mm in autonomous, 75 mm at the R1 intake handoff in either mode. They replace the
// original 15 mm, which was a starting point from the first conversation and was never checked against
// the real mounting. All three are comfortably ABOVE the V5 Distance Sensor's ~20 mm specified minimum,
// so the old "15 sits below what this sensor is specified to read" caveat no longer applies at all.
//
// WHY AUTON GETS MORE REACH THAN DRIVER. In driver control a human is aiming the claw, so a shorter
// range is the safer default -- it is the driver who decides what to grab, and 55 mm is about the jaws.
// In autonomous nothing is aiming: the route drives to a surveyed spot and the pin is wherever placement
// and odometry drift left it, so the claw has to commit from further out or it drives past. The handoff
// (75) is further still, because there the intake pushes the pin ACROSS the jaws rather than the claw
// being brought to it.
//
// Which one is in force is NOT state this class keeps: Claw::setAutonMode() and setIntakeMode() are
// passed every tick by periodic::tick(), the same way setInhibit() is, so it can never be left latched
// in the wrong mode after a field switch. See Claw::activeRangeMm().
//
// Watch the brain's DIST line's CLAW value with a pin held in the open jaws:
//   it never reaches the range with a pin there -> raise the range for that mode to just above what you
//                                               see.
//   it reads 0 with the pin in the jaws      -> the pin is INSIDE the sensor's floor: raise the range
//                                               and/or move the sensor back a few mm. (A 0 is treated
//                                               as "cannot measure", never as "very close" -- see
//                                               AUTOCLAMP_MIN_VALID_MM.)
//   it clamps on a cup/pin going past        -> raise AUTOCLAMP_CONFIRM_MS (50 -> 80) first.
//   it clamps on the field/a wall/the intake -> LOWER the range for that mode. 55-75 mm is 5-7 cm of
//                                               reach, so this is the first number to suspect if the
//                                               jaws shut on nothing in particular; the sensor sees
//                                               whatever is in front of it, not only pins.
// THE CLEAR THRESHOLD FOLLOWS THE RANGE AUTOMATICALLY -- see AUTOCLAMP_REARM_GAP_MM, which exists
// precisely so that changing a range here cannot break the ordering the autoclamp depends on.
// SENSITIVITY IS THE RANGE, AND ESSENTIALLY ONLY THE RANGE -- worth knowing before reaching for the
// confirm window instead. AUTOCLAMP_MIN_SAMPLES (2) is the BINDING constraint on how fast this can
// possibly fire: the V5 Distance Sensor refreshes about every 30 ms, so two FRESH in-range readings
// cannot happen in less than ~30 ms however small AUTOCLAMP_CONFIRM_MS (50) is. Dropping the confirm
// window therefore buys almost nothing on its own -- it just moves the limit onto the sample count --
// and dropping AUTOCLAMP_MIN_SAMPLES to 1 removes spike rejection entirely, which is only an acceptable
// trade at the handoff (see AUTOCLAMP_INTAKE_*), where a false clamp costs one button press. In auton a
// false clamp is worse than a late one: the route carries on having gripped a wall. So "more sensitive"
// is tuned here, in millimetres.
inline constexpr int AUTOCLAMP_RANGE_DRIVER_MM = 65;         // TUNE: driver control (was 60, +5 driver 2026-10-09; originally 55)
inline constexpr int AUTOCLAMP_RANGE_AUTON_MM = 75;          // TUNE: autonomous -- "a bit more sensitive" (was 65)
inline constexpr int AUTOCLAMP_CONFIRM_MS = 50;              // TUNE: ...continuously for this long
// ...over at least this many FRESH readings. The sensor refreshes on the order of 30 ms, i.e. slower
// than the 10 ms tick, so the same measurement comes back two or three times in a row; without this,
// ONE lucky in-range measurement repeated for 50 ms would satisfy the time window on its own, which is
// exactly the spike this is meant to reject. 2 fresh samples at ~30 ms apart is ~60 ms of real
// evidence.
//   Clamps feel laggy: lower AUTOCLAMP_CONFIRM_MS first, this second (never below 2).
inline constexpr int AUTOCLAMP_MIN_SAMPLES = 2;              // TUNE
// How long an UNCHANGED reading has to persist before it is credited as another sample. About two
// sensor refreshes: long enough that the sensor must really have re-measured, short enough that a pin
// sitting dead still at exactly one millimetre value still gets its second sample and gets gripped.
//   Clamps on a steady reading feel a touch slow: lower it (but not below ~35 ms, one refresh).
inline constexpr int AUTOCLAMP_STALE_GRACE_MS = 60;          // TUNE
// RE-ARM. The instant the jaws open, the pin they just let go of is still millimetres away, so a
// timer-based re-arm would re-grab it every single time and no score could ever finish. After ANY
// open, the autoclamp waits until the sensor reads CLEAR -- further than (the range in force + this
// gap), or nothing in range at all -- for AUTOCLAMP_REARM_CLEAR_MS.
//
// A GAP, NOT AN ABSOLUTE DISTANCE, AND THAT IS THE WHOLE POINT. There are three ranges now (55 driver /
// 65 auton / 75 handoff) and each needs its own clear threshold, because a clear threshold at or below
// the range in force is a guaranteed re-grab loop: anything sitting between them reads as CLEAR (so the
// autoclamp arms) AND as IN RANGE (so it immediately clamps), so the claw grabs back what it just put
// down, every time, and no score can finish. Expressing the hysteresis as "range + 30 mm" makes that
// ordering true by construction, so changing any range above cannot reintroduce the bug -- which the
// earlier absolute AUTOCLAMP_REARM_MM / AUTOCLAMP_INTAKE_REARM_MM pair very nearly did when the range
// went 15 -> 50.
//   It re-grabs what it just put down: raise this, or AUTOCLAMP_REARM_CLEAR_MS.
//   The AC: field sits on CLEAR? forever and never re-arms: there is something permanently within
//     (range + this) of the sensor -- the intake, a wall, the frame. LOWER it; it may not go to or below
//     0, and the smaller it is the twitchier the arm/clamp boundary gets.
inline constexpr int AUTOCLAMP_REARM_GAP_MM = 30;            // TUNE: clear = (range in force) + this
inline constexpr int AUTOCLAMP_REARM_CLEAR_MS = 150;         // TUNE
// THE ESCAPE HATCH, and it is not optional -- without it this whole rule is an UNBOUNDED wait.
// WHAT WENT WRONG (2026-10-03): because the clear threshold is derived as range + gap, raising the
// trigger range also raises how far away the world has to be before the autoclamp will arm again. At the
// original 15 mm range that was 30 mm and essentially always satisfiable. At 60-75 mm it is 90-105 mm --
// and anything permanently inside 10 cm of the sensor (the claw's own structure, a cup, a wall, or the
// pin it just put down and has not driven away from) pins the detector in WAIT_CLEAR for the rest of the
// match. The jaws then never clamp again in EITHER mode, and the only clue is `AC:CLEAR?` on the brain.
// That is exactly what happened, and it is why a threshold rule that can wait forever needs a bound.
// So: after this long waiting with no clear reading, arm anyway. It is seconds, not milliseconds, so it
// is never the normal path -- a genuinely clear reading still arms in AUTOCLAMP_REARM_CLEAR_MS (150 ms).
// The risk it reintroduces is bounded and NOT the bug the clear rule exists to prevent: that one
// re-grabbed the released pin within ~150 ms every single time, so no score could finish. Re-grabbing
// something still in the jaws two seconds later, after the robot has had every chance to drive away,
// costs one press of A.
//   It re-grabs what it just put down, but only after a pause: raise this.
//   The autoclamp still goes dead after a release (brain sticks on `AC:CLEAR?` for seconds): LOWER it --
//     and check the DIST line's CLAW value, because something is permanently inside (range + gap).
//   0 disables the hatch entirely and restores the old wait-forever behaviour. Don't.
inline constexpr int AUTOCLAMP_REARM_MAX_MS = 2000;          // TUNE: give up waiting for clear after this
// VALIDITY of a raw reading. At or above MAX is the sensor's own "nothing in range" (9999) and counts
// as CLEAR. Below MIN (a 0: the sensor cannot measure, e.g. an object inside its floor) and any
// negative value (PROS_ERR / unplugged) count as NOT DETECTED and are not clear either -- so a dead or
// unplugged sensor makes the autoclamp do nothing at all, which is the correct failure.
inline constexpr int AUTOCLAMP_MAX_VALID_MM = 9999;
inline constexpr int AUTOCLAMP_MIN_VALID_MM = 1;
// The sensor's own 0..63 confidence. 0 = ignore it (the default: confidence drops at close range on
// this sensor, and requiring it there is a good way to never clamp). Raise it only if the DIST line
// shows plausible mm values that turn out to be junk.
inline constexpr int AUTOCLAMP_MIN_CONFIDENCE = 0;           // TUNE
// Minimum time after a release before the autoclamp may arm again, on top of the clear-reading rule
// above (belt and braces: it also covers the jaws physically swinging open past the sensor).
inline constexpr int CLAW_RELEASE_LOCKOUT_MS = 750;          // TUNE

// --- AT THE R1 INTAKE HANDOFF: fire instantly, then force the claw DOWN ------
// THE DRIVER'S REQUEST, 2026-10-03: "auto clamp should work in intake r1 tho too, and should move down
// a bit, forcing way down when it sees the object and trigger claw at the instant it sees the object."
// So while the macro is at (or coming down into) the INTAKE handoff -- Scoring::isAtIntakeHandoff(),
// which periodic::tick() feeds to Claw::setIntakeMode() every tick -- the autoclamp swaps the ordinary
// 50 ms / 2-sample confirm for these, and the clamp is followed by a forced push DOWN instead of the
// CLAMP_GRAB_HEIGHT hop (there is nothing to lift clear of at the handoff; the pin has to be pressed into
// the jaws, exactly as L1's GRAB does it).
//
// INSTANT, AND WHAT THAT COSTS. 0 ms + 1 fresh sample means the FIRST valid in-range reading fires the
// cylinder -- no debounce at all. That is the point (a pin arriving on the intake is there for a few
// tens of milliseconds, and a missed grab is a lost cycle), but it does give up exactly the spike
// rejection the rest of the AUTOCLAMP block exists for: one junk reading at <= AUTOCLAMP_RANGE_MM
// closes the jaws on nothing. It is only acceptable HERE because the arm is parked at the handoff with
// the jaws empty and a false clamp costs one R1/A press to undo, not a dropped pin. The pure detector is
// unchanged: these are just different Config numbers handed to the same code (see
// autoclamp_detector.hpp), so a bad clamp at the handoff is tuned here and nowhere else.
//   It clamps on nothing at the handoff: raise AUTOCLAMP_INTAKE_CONFIRM_MS (0 -> 20) first, then
//     AUTOCLAMP_INTAKE_MIN_SAMPLES (1 -> 2). At 50/2 it is identical to the ordinary rule again.
//   It misses pins the intake hands over: it cannot be made faster than this -- check the DIST line's
//     CLAW value actually falls to <= AUTOCLAMP_RANGE_MM as a pin arrives, and raise the range if not.
inline constexpr int AUTOCLAMP_INTAKE_CONFIRM_MS = 0;        // TUNE: 0 = the first in-range reading fires
inline constexpr int AUTOCLAMP_INTAKE_MIN_SAMPLES = 1;       // TUNE: ...over one fresh sample, i.e. no debounce
// AND A LONGER REACH. 75 mm at the handoff against 50 mm everywhere else, the driver's own numbers
// (2026-10-03: "clamp distance has to hit at 50 mm actually, and for when in the intake, 75"). It makes
// sense mechanically: everywhere else the pin is brought TO a waiting claw, so 50 mm is about the jaws;
// at the handoff the intake pushes the pin ACROSS the jaws, so the claw has to commit while it is still
// arriving or the rollers carry it straight past.
//   The jaws shut on the intake itself, or on a cup riding through: LOWER this toward 50.
//   Pins still get carried past before the jaws move: raise it -- but then also raise
//   AUTOCLAMP_INTAKE_REARM_MM below, which must stay above it.
// It is the same 75 in BOTH modes: unlike the driver/auton split above, what makes the handoff want more
// reach is the mechanism (the pin crossing the jaws on the rollers), not who is aiming. Its clear
// threshold comes from AUTOCLAMP_REARM_GAP_MM like every other range's.
// NOTE (2026-10-03): the auton range was raised to 75 too, so this no longer adds any REACH in auton --
// what still makes the handoff special there is the instant-fire confirm above (0 ms / 1 sample against
// 50 ms / 2 samples). It does still add reach in driver control (75 against 60). Keep it as its own
// constant: the two are tuned for different things and must be able to move apart again.
inline constexpr int AUTOCLAMP_INTAKE_RANGE_MM = 70;         // TUNE: driver 2026-10-07, the whole intake sequence (was 75)
// THE PUSH. The same open-loop downward force L1's GRAB uses (L1_GRAB_PUSH_PCT, 40% of volts, see "The
// L1 GRAB" in the lift block) for this long, and then the arm HOLDS wherever it ended -- the driver's
// "move down a bit". The jaws are already shut by the time it starts, so unlike L1's grab there is no
// grip request and no CLAW_CLOSE_TIME_MS wait inside it. The intake rollers are STOPPED for the whole
// push, exactly as they are for L1's grab (Lift::isGrabbing() is what periodic::tick() reads, and the
// push is a GRAB -- brain's LIFT line: `GRAB`), so they cannot drag the pin back out of the jaws.
// A following L1 from the handoff still works normally: it sees the claw already clamped, skips the
// close, pushes again, and rises to level 1.
//   The pin gets pushed out / the intake is levered off the floor: LOWER it (or L1_GRAB_PUSH_PCT).
//   The pin isn't seated in the jaws by the time the push ends: raise it.
inline constexpr int AUTOCLAMP_INTAKE_PUSH_MS = 300;         // TUNE: ms of forced push down after the clamp
// AFTER THE PUSH, GO STRAIGHT TO LEVEL 1 (driver, 2026-10-07). The autoclamp taking a pin at the
// handoff used to end with the arm holding where the push stopped, waiting for an L1 press to do
// anything else. With this on, the instant the push finishes the macro enters STACKING on the FIRST
// rung of L1's ladder (L1_STEP_DEG) by itself -- the same state, height and pivot behaviour an L1
// press from the handoff produces, so the pivot swings OUT to the scoring position once the arm has
// arrived (see PIVOT_RISE_WAIT_MS). An L1 press after it still steps on up the ladder as normal.
// Anything the driver presses in the meantime cancels it, because every state change clears the flag.
//   You want the arm to stay down at the handoff after an autoclamp: set this false.
inline constexpr bool AUTOCLAMP_INTAKE_AUTO_STACK = true;

// ---------------------------------------------------------------------------
// Field-facing distance sensors (brain screen, and moveB's feedback)
// ---------------------------------------------------------------------------
// Three of them, all confirmed on the robot 2026-10-03: one each side and one looking backward. The
// REAR one is the entire feedback loop for auton::moveB() (its own constants are in the moveB block
// further down). The left and right ones are read by the brain's DIST line only -- nothing in the
// control code uses them yet; they are there to be aimed at a wall or a stack while tuning a route.
// The fourth distance sensor on this robot is the one inside the claw (CLAW_DISTANCE_PORT, 11) and it
// belongs to the autoclamp, not here.
inline constexpr int8_t LEFT_DISTANCE_PORT = 15;             // confirmed 2026-10-03
inline constexpr int8_t RIGHT_DISTANCE_PORT = 19;            // confirmed 2026-10-03

// --- START-OF-AUTON ALIGNMENT (auton::startPose(), see README "Start alignment") ---
// The robot is set against the front wall every time, so the distance from the front and the heading
// are repeatable -- but how far along that wall it ends up is not. The LEFT sensor measures that:
// reading FURTHER than expected means the robot is shifted +x (toward the robot's right), CLOSER
// means -x, and because the heading is fixed the difference in the reading IS the lateral shift --
// no sensor-offset geometry is involved. Each route carries its own expected reading, next to the
// route in src/autons.cpp (it is a field position, not a robot constant). These are the shared rules.
//
// VALIDITY, mirroring the team's old accumulateReading(): a sample counts only if it is under
// START_ALIGN_MAX_MM and at least START_ALIGN_MIN_CONFIDENCE. 0, 9999 and PROS_ERR are rejected by
// the same test. If NOTHING valid comes back, the correction is skipped entirely.
inline constexpr int START_ALIGN_MAX_MM = 2000;              // TUNE: further than this is not our wall
inline constexpr int START_ALIGN_MIN_CONFIDENCE = 38;        // TUNE: 60% of the sensor's 0..63 scale
// SANITY GUARD. A shift bigger than this is not a hand-placement error, it is the sensor seeing a
// goal, a robot, or the wrong wall -- so the correction is dropped and the nominal pose is used. It
// must never be able to teleport the pose.
//   Legitimate setups keep getting skipped: raise it, but only after checking the DIST line.
inline constexpr double START_ALIGN_MAX_IN = 4.0;            // TUNE: inches
// IN-AUTON sampling window, used only when there is no usable disabled-mode history (e.g. autonomous
// launched straight from the brain menu). This time is spent at the start of the route.
inline constexpr int START_ALIGN_SAMPLE_MS = 300;            // TUNE
// DISABLED-MODE history. While the robot sits disabled before the match, the left sensor is sampled
// continuously and the last START_ALIGN_HISTORY_MS are kept. startPose() uses that average -- zero
// auton time spent, and a far longer average than a route could afford -- but only if the window is
// SETTLED: max minus min under START_ALIGN_HISTORY_SPREAD_MM, so a hand still placing the robot (or a
// person walking past the beam) does not count as a measurement.
inline constexpr int START_ALIGN_HISTORY_MS = 500;           // TUNE
inline constexpr int START_ALIGN_HISTORY_SPREAD_MM = 40;     // TUNE: a settled window is this tight
inline constexpr int START_ALIGN_HISTORY_STALE_MS = 2000;    // older than this = not this match's setup
// ...plus a short fresh sample at the start of auton to confirm the robot has not been moved since.
// If the confirm disagrees with the history by more than START_ALIGN_CONFIRM_TOL_MM, the CONFIRM wins
// (it is the newer truth) and that is what gets used.
inline constexpr int START_ALIGN_CONFIRM_MS = 80;            // TUNE
inline constexpr int START_ALIGN_CONFIRM_TOL_MM = 150;       // TUNE
// chassis.calibrate() RESETS THE POSE, so a calibration that finishes after startPose() would wipe
// the correction. startPose() waits this long for calibration before applying anything, and if it is
// still not done it applies the pose and re-applies it once calibration really finishes.
inline constexpr int START_ALIGN_CALIBRATE_WAIT_MS = 2500;   // TUNE
inline constexpr int START_ALIGN_REAPPLY_WINDOW_MS = 4000;   // how long that one-shot re-apply waits

// ---------------------------------------------------------------------------
// BACK-SENSOR Y CALIBRATION — the other axis, for MATCHLOAD_FINISH
// ---------------------------------------------------------------------------
// The driver, 2026-10-07: "when we start our matchload finish auton, backdistance expects to see
// 1520. If the value is less than 1250 or greater than 1700, don't use the value and skip using
// backdistance for pose calibration. This value should dictate the y value only: if it's greater than
// 1520, y start is increased; if it's less, the starting y offset is decreased below 0 (negative)."
//
// THIS IS THE Y AXIS ONLY, and it is a SEPARATE mechanism from the left-sensor X alignment above --
// a route may use either, both or neither. The back sensor (config::BACK_DISTANCE_PORT, 14, the same
// one auton::moveB drives on) measures how far the robot is from whatever is behind it, so with the
// heading fixed at 0 the difference between the reading and the expected reading IS the Y offset. No
// sensor-offset geometry and no heading rotation are involved.
//
//   y_offset_inches = (reading_mm - MATCHLOAD_BACK_EXPECTED_MM) / 25.4
//
// so reading > expected -> +y, reading < expected -> -y, reading == expected -> 0, exactly as asked.
//
// THE WINDOW IS A HARD GATE, not a clamp: a reading outside MIN..MAX is not trusted at all and the
// route starts at its nominal Y (offset 0) rather than a clamped guess. A reading the sensor itself
// rejects (0, 9999, PROS_ERR, under START_ALIGN_MIN_CONFIDENCE, or past START_ALIGN_MAX_MM) lands in
// the same place. The brain's START line says which happened, so it can be checked before a match.
//   Legitimate setups keep getting ignored: widen MIN/MAX, but check the DIST line's BACK figure
//     first -- if it disagrees with a tape measure, the problem is the sensor or what it is aimed at.
inline constexpr double MATCHLOAD_BACK_EXPECTED_MM = 1520.0;  // TUNE: driver-measured 2026-10-07
inline constexpr double MATCHLOAD_BACK_MIN_MM = 1250.0;       // TUNE: below this, ignore the reading
inline constexpr double MATCHLOAD_BACK_MAX_MM = 1700.0;       // TUNE: above this, ignore the reading
// The back sensor has no disabled-mode rolling history (that machinery is the left sensor's), so this
// is always a fresh in-auton average and it always costs this much of the autonomous period.
inline constexpr int MATCHLOAD_BACK_SAMPLE_MS = 300;          // TUNE

// ---------------------------------------------------------------------------
// Intake (ONE motor)
// ---------------------------------------------------------------------------
// Do NOT try to fix the roller direction by flipping this sign -- it was tried and the rollers
// kept spinning the same way. The direction lives in Intake::set() (src/subsystems/intake.cpp),
// where IN and OUT are swapped on purpose. Flip them there.
inline constexpr int8_t INTAKE_PORT = 12;
inline constexpr float INTAKE_MV = 12000.0f;

// AUTO-INTAKE ON L1. An L1 press that starts a rise FROM THE FLOOR -- the arm at or below
// LIFT_BOTTOM_GOAL_DEG, seated, or sitting at the INTAKE handoff -- also runs the rollers IN, from
// the press itself (so they are already going during the CLAW_CLOSE_TIME_MS clamp wait) until the arm
// climbs past this height. The point is to sweep up whatever is at floor level while the arm is on
// its way up anyway, without the driver having to hold R1 at the same time as tapping L1.
//   Cups get left behind / it stops sweeping too early: raise it.
//   The intake is still running when the arm is clear of the floor: lower it.
//
// HANDOFF-RELATIVE SINCE 2026-10-07, and it HAD to become so when the handoff moved to 23. As a bare
// 20 it was BELOW the handoff, so an L1 from the handoff began with the arm already past the
// threshold: periodic.cpp's `done = liftPos > L1_AUTO_INTAKE_UNTIL_DEG` was true on the sweep's very
// first tick and the rollers never turned at all on the most common path. Written as INTAKE + 20 it is
// 43.0, i.e. the sweep covers the whole first L1 step up from the handoff (23 -> 43).
// NOT YET DRIVEN: 20 deg of rise was the old intent, carried over. Tune it on the robot.
// From a SEATED arm (-31.9) the rise to the first rung (-11.9) never reaches 43, so that sweep ends on
// L1_AUTO_INTAKE_MAX_MS below instead -- which is exactly what that safety net is for.
inline constexpr double L1_AUTO_INTAKE_UNTIL_DEG = lift_preset::INTAKE + 20.0;  // TUNE: 43.0
// Safety net: the rollers stop after this long no matter what, so a rise that never gets to
// L1_AUTO_INTAKE_UNTIL_DEG (jam, BLOCKED lift, a preset below it) can't leave them spinning.
inline constexpr int L1_AUTO_INTAKE_MAX_MS = 1500;           // TUNE

// AUTO-OUTTAKE ON L2 SCORE. While an L2 score descent is running (Scoring::isScoreDescent(): RELEASING
// with scoreToHome_ set -- the sensing scorer, not L2's at-the-bottom clamp toggle) and the arm is
// below this height, run the rollers OUT at L2_OUTTAKE_PCT so anything still in the intake ejects
// instead of riding the lift down. Covers the whole trip through PRESS / REL / UNSTK / RETURN,
// whichever phase the arm happens to be under this height in.
//   NOTE: stack level 1 (31.0 deg) is already below this, so scoring at level 1 outtakes from the
//   start of the press, not partway down. That is this constant as specified, not a bug.
//   Pin/cup gets knocked loose too early: lower it. Rollers start too late to clear the handoff
//   before the arm seats: raise it.
inline constexpr double L2_OUTTAKE_BELOW_DEG = lift_preset::DOWN + 40.0;
// % of full intake power (config::INTAKE_MV) the rollers run OUT at during the above. Lower than
// R2's full-power outtake on purpose -- this runs unattended during a score, not under driver thumb.
inline constexpr double L2_OUTTAKE_PCT = 60.0;               // TUNE: % of INTAKE_MV
// Stop condition (2026-10-02 retune): as soon as the lift reaches LIFT_FINAL_START_DEG -- tied
// directly to it, not a separate guess, because that is also where the lift's own FINAL phase takes
// over forcing the arm the rest of the way down at real authority (see "Seating on the hard stop" in
// the LIFT TUNING block above). The roller does not need to keep running while FINAL does that forcing;
// if a cup really was the obstruction it has already been cleared by then. main.cpp uses this constant
// directly (see L2_OUTTAKE_STOP_DEG below) rather than position-matching LIFT_FINAL_START_DEG by eye.
inline constexpr double L2_OUTTAKE_STOP_DEG = LIFT_FINAL_START_DEG;
// Safety net: the rollers stop after this long no matter what, so a score that stalls above
// L2_OUTTAKE_STOP_DEG (BLOCKED, FAULT, stuck claw) can't leave them spinning for the rest of the match.
inline constexpr int L2_OUTTAKE_MAX_MS = 3000;               // TUNE

// ---------------------------------------------------------------------------
// Pivot + claw timing. See include/subsystems/scoring.hpp.
// ---------------------------------------------------------------------------
// If the claw/wrist flips with its own cylinder, set the port. ' ' = no pivot actuator.
inline constexpr char PIVOT_PISTON_PORT = 'B';               // physically the pivot (was 'D', which is the claw)
// THE PIVOT HAS TWO POSITIONS, named for where the end effector physically is (the driver's words):
//   OUT  = swung out. Where it sits at power-on, at every stack level while scoring, and in the
//          default down position after L2 / B. In the code, pivot OUT == true.
//   BACK = pulled in toward the intake handoff, for taking a pin from the intake (R1 / INTAKE).
//          In the code, pivot BACK == false.
// DERIVATION of which ADI level is OUT -- now from the correct pin, and confirmed twice:
//  1. At power-on every ADI output is LOW, the code writes LOW anyway, and the driver's own
//     observation is that the physical pivot sits OUT. C is the pivot. So C LOW = OUT.
//  2. Independently: on the 6b8165c build, Claw::clamp() at every stack level wrote HIGH to C (that
//     build's "claw" object lived on C), and the driver reported the pivot sat PULLED IN at every
//     stack level. So C HIGH = BACK. Same answer from a different run.
// So the unpowered/LOW level is OUT and HIGH is BACK -- which is also the only arrangement where
// Scoring can start with the pivot logically OUT without writing anything the ADI isn't already at, so
// nothing moves at startup (CLAUDE.md rule 1). Confirm with X: the line reads `PIV OUT C[0]` at
// power-on and must read `BACK C[1]` with the pivot pulled in after one press.
inline constexpr bool PIVOT_OUT_IS_ADI_HIGH = false;         // confirmed twice (see above); re-check with X
// The matchload spot (scoreOrMatchload(), not on a driver button): take the pin with the pivot BACK
// like at the intake, or OUT like everywhere else?
inline constexpr bool PIVOT_BACK_AT_MATCHLOAD = false;       // TODO(team): confirm
// The handoff zone: from lift_preset::INTAKE up to this many degrees above it. While the intake macro
// is heading to or sitting at the intake, the pivot is BACK inside this zone and OUT above it, so on
// R1 it swings BACK as the arm comes down into the zone (not only once the lift reports "arrived" --
// a slightly short stop still gets the pivot BACK), and on L1 it swings OUT as the arm rises out of
// the zone (or when it arrives at the level, whichever is first, so a level can never be left waiting
// for it). It only ever times the swing; it never gates the lift. This is the ONLY height the pivot's
// motion is tied to -- the old PIVOT_CLEAR interlock is gone (see scoring.hpp).
//   The pin swings into the intake as it leaves the handoff on L1: raise it.
//   The pivot is still swinging when the arm reaches level 1, or still OUT when the arm reaches the
//   handoff on R1: lower it.
inline constexpr double PIVOT_HANDOFF_ZONE_DEG = 10.0;       // TUNE
// How long the pivot takes to finish swinging. NOTE: until the port swap this timer was watching the
// CLAW cylinder, so it has never actually been measured against the pivot. 250 ms is a safe guess.
inline constexpr int PIVOT_TIME_MS = 250;                    // TUNE: never measured on the real pivot
// THE PIVOT WAITS FOR THE ARM TO GET THERE (driver, 2026-10-07). On the way UP to a level the pivot
// used to swing OUT as soon as the arm rose out of the handoff zone, i.e. part way through the rise,
// while the arm was still moving. The driver's rule is the opposite: leave the pivot exactly where it
// is until the LIFT HAS FINISHED getting into position, then swing it. So STACKING now waits for
// liftArrived() and this is only the BACKSTOP behind it -- if the arm never reports arrival (blocked,
// parked short, a target it cannot reach) the pivot swings anyway after this long, so a level can
// never be left with the pivot stuck BACK. The driver's own estimate was "shouldn't take more than
// 1 second"; the lift's own LIFT_MOVE_TIMEOUT_MS normally resolves a stuck move well before this.
//   The pivot swings before the arm has stopped: this is not the cause -- raise LIFT_TOLERANCE_DEG's
//   strictness or check liftArrived(); this timer only fires when arrival never happens.
//   A blocked arm sits too long with the pivot BACK: lower it.
inline constexpr int PIVOT_RISE_WAIT_MS = 1000;              // TUNE: ms before the pivot gives up waiting
// THIS IS THE L1 DELAY. The first L1 tap closes the jaws and waits here before the lift is given any
// target at all (Scoring::update(), case CLAMPING) -- interlock: the arm must not pull out of an open
// claw. So the gap between pressing L1 and the arm moving is at least this long, by design.
// NOTE: this was trimmed to 100 ms while the ports were crossed, i.e. it was timing the PIVOT
// cylinder, not the claw. Put back to 150 ms, which is a sane starting point for a pneumatic claw that
// has to actually grip a pin before the arm pulls away.
//   Pin slips / the arm starts up before the jaws are shut: raise it (175, 200).
//   The gap between the first L1 tap and the arm moving annoys you: lower it, carefully.
// (Every L1 tap AFTER the first one commands the next level immediately -- no wait.)
inline constexpr int CLAW_CLOSE_TIME_MS = 150;               // TUNE: never measured on the real claw
// How long the jaws take to actually open. Two users: scoreOrMatchload() waits it out before the arm
// moves away, and L2's scorer starts its "did the claw come free" check this long after it asked for
// the release (see L2_FREE_MS). Too short and L2 judges freedom while the jaws are still closing.
inline constexpr int CLAW_OPEN_TIME_MS = 200;                // TUNE: never measured on the real claw
// ---------------------------------------------------------------------------
// L2 — the sensing scorer. See "Scoring with L2" in lift.hpp and README.
// ---------------------------------------------------------------------------
// L2 above the bottom with a pin held no longer opens the jaws and lowers. It presses the pin DOWN
// onto the stack with the claw still SHUT, watches the rotation sensor for the moment the pin lands
// (the arm stops descending as fast as it is being told to), releases the claw at that instant, keeps
// pushing down to slide the jaws off the pin, checks the arm really is free, and only then runs home
// fast. Every phase of that lives in Lift::update() (phases PRESS / REL / UNSTK, then the ordinary
// DESCEND -> SEAT for the run home); Scoring still owns the claw and opens it when the lift asks.
//
// PRESS-DOWN SPEED, % of the free speed. The user's starting point. It is closed-loop on the measured
// arm speed, which is what makes contact detectable at all: an open-loop descent would just slow down
// against the stack and nothing would know.
//   Slams the pin into the stack / knocks the tower: LOWER it (45, 35).
//   Scoring feels slow: raise it, but contact detection gets less sensitive as the gap between
//   "moving" and "stopped" shrinks relative to the noise.
// 90 -> 100, which is the CEILING, not a choice: 100% of LIFT_FREE_SPEED_DEG_PER_S is 150 deg/s and
// the arm cannot be driven faster than its own free speed -- a reference above 100 is unreachable and
// only winds up the velocity integral (the same reason LIFT_FAST_DESCENT_* were pulled back to
// 95/100). So the asked-for 1.5x is not available here: 135 -> 150 deg/s is 1.11x.
// THIS CONSTANT ONLY GOVERNS THE RUN TO THE BOTTOM AFTER THE RELEASE, which uses full authority. The
// press-down BEFORE the release is limited by L2_PRESS_MAX_MV instead, and that is where the 1.5x
// actually comes from -- see the note there.
inline constexpr double L2_SCORE_SPEED_PCT = 100.0;          // TUNE: 150 deg/s, the free-speed ceiling (was 90)
// Most the loop may push DOWNWARD (magnitude, mV) while pressing. This is the crush limit: the
// velocity loop has 12000 mV of authority and an integral that winds up, so without a cap it would
// keep increasing the push until it moved -- i.e. flatten the stack. 7000 mV through the 4:1 is about
// 56% of stall torque: firm enough to seat a pin, not enough to crumple a tower.
// RAISED 5000 -> 7000 (2026-10-02): holding a pin+cup, this six-bar can need real authority just to
// keep descending at the commanded speed over the part of the arc where gravity resists most -- the
// extra weight adds directly to that resistance. At 5000 mV a loaded arm could saturate the velocity
// loop and settle into a steady creep under STALL's absolute floor (config::L2_CONTACT_STALL_DEG_PER_S)
// with NOTHING under the arm, which is most of why a loaded claw auto-released almost immediately every
// time (see "ARM ON MOTION, AND SPLIT STALL" in contact_detector.hpp for the other half of that fix).
// 7000 is still well short of the full 12000 the loop could use outside a score, so a real hard-stack
// contact still can't get the crush authority of an ordinary move.
//   The stack gets pushed over / the cup crumples: LOWER it (5500).
//   The pin does not seat fully before the release: raise it (8000).
//   A loaded arm still creeps under L2_CONTACT_STALL_DEG_PER_S with nothing under it: raise this
//   further, or raise L2_CONTACT_NOSTART_MS so it has longer to prove it is still accelerating.
// THE PRESS-DOWN'S REAL SPEED LIMIT, and the knob the driver's "1.5x faster descent" actually needs.
// The press phase runs runVelocityLoop(..., downMaxMv = THIS), so it is AUTHORITY-limited, not
// reference-limited: at 3500 mV the motor's back-EMF balances at 3500/12000 * 150 = ~44 deg/s, and
// above that the motors brake against gravity. L2_SCORE_SPEED_PCT's 135 deg/s was never reachable
// during the press -- it was always this cap doing the limiting.
// 3500 -> 5250 is a true 1.5x of the available downward authority, and still well under the 7000 this
// was before the red retune, so it is inside territory the arm has already run at.
// THE TRADE: this is also the crush cap -- how hard the arm may press a pin into a stack while the
// contact detector decides. 1.5x the authority is 1.5x the press force. Watch the first few scores.
inline constexpr float L2_PRESS_MAX_MV = 5250.0f;            // TUNE: magnitude, downward (was 3500)
// ...AND THE WAY ROUND THE TRADE (2026-10-09, the driver: "it needs to go down fast"). The cap above
// has to do two different jobs with one number: it limits how fast the arm can be DRIVEN DOWN through
// free air (where a bigger number is simply better) and how hard it may CRUSH once something is
// resisting (where it is the only thing protecting the stack). Those two never happen at the same
// time, so they no longer share a number: while the arm is measurably MOVING DOWN it gets
// L2_PRESS_FREE_MAX_MV, and the instant it slows -- which is exactly what a landing looks like -- the
// cap drops back to L2_PRESS_MAX_MV, within one 10 ms tick, before the loop can lean on anything.
// Hysteretic, on the MEASURED arm speed, so it cannot flicker:
//   below ON  -> the crush cap (this is also the state a press starts in, and the state a press that
//               begins with the pin already resting on the stack never leaves)
//   above ON  -> the free cap, until the speed falls back under OFF.
// 9000 mV balances back-EMF at 9000/12000 * 150 = ~112 deg/s, so the press runs at about 1.7x the
// 5250 cap's ~66 deg/s. It is NOT 12000: the detector still needs ~30-40 ms to confirm a landing, and
// the arm travels that window at whatever speed it is doing, so this is deliberately short of
// free-fall into a stack of cups.
//   The stack gets knocked over by the press itself: LOWER this (7000), not L2_PRESS_MAX_MV.
//   Still too slow in free air: raise it toward LIFT_VEL_MAX_MV (12000), a step at a time.
//   The press feels like it stalls and then surges: the ON/OFF pair are too close -- widen them.
inline constexpr float L2_PRESS_FREE_MAX_MV = 9000.0f;       // TUNE: magnitude, downward, free travel only
inline constexpr double L2_PRESS_FAST_ON_DEG_PER_S = 40.0;   // TUNE: measured descent speed that earns the free cap
inline constexpr double L2_PRESS_FAST_OFF_DEG_PER_S = 25.0;  // TUNE: ...and the speed that takes it away again
// CONTACT DETECTION. Implemented by subsystems::ContactDetector (include/subsystems/contact_detector.hpp),
// which is a PURE class with host-side tests in test/ -- run test/run.sh on the Mac after changing any
// of these, it proves the thresholds against synthetic landings before anything is uploaded.
//
// WHY THE FIRST VERSION NEVER FIRED, and what changed. It compared the measured speed against the
// NOMINAL commanded speed (~90 deg/s): a relative trigger armed only once the arm reached 85% of it,
// and an absolute trigger at 8 deg/s. Both are dead if the arm simply cannot make 90 deg/s -- and on
// this six-bar it often cannot, because the arm swings up and BACK, so over much of the arc gravity
// RESISTS the descent and the loop can only push down with L2_PRESS_MAX_MV of authority. A free
// descent settling at 50 deg/s never armed the relative trigger (50 < 76.5) and never tripped the
// absolute one (50 > 8); the landing then slowed it to 15, still over 8. Nothing could ever fire.
// Every signal below is now measured against WHAT THE ARM WAS ACTUALLY DOING a moment ago -- a learned
// baseline -- not against what it was told to do. Three signals:
//   LAG     a leaky one-sided integral of the speed shortfall against the baseline, in arm degrees:
//           "how far behind where it would have been the arm now is". The POSITION-vs-EXPECTED signal,
//           and the sensitive one -- it catches a landing that only slows the arm by a quarter.
//   EFFORT  the loop pushing DOWN harder than the free descent needed. The MOTOR-WORK signal: it
//           catches a pin being forced into the tower even while it still creeps.
//   SPEED   measured below a fraction of the baseline. The absolute near-stop (STALL) is a separate,
//           unconditional backstop that needs no baseline at all, so it still catches a pin already
//           resting on the stack by itself.
// AGREEMENT, added 2026-10-02: a lone LAG/EFFORT/SPEED signal is no longer enough -- at least TWO of
// the three must be over threshold on the SAME tick (STALL is exempt, see contact_detector.hpp for
// why). This is the fix for the robot false-firing in plain free descent: this six-bar's gravity torque
// swings up and back past vertical, which can move a single signal fast enough to look like a landing
// with nothing under the arm at all. On top of AGREEMENT, PRESS also has a GUARANTEED release height
// (L2_AUTO_RELEASE_DEG, below) -- so a real but gentle landing the detector is slow to confirm is no
// longer a lost score, just a later one.
// Which one fired is on the brain (REL(lag) / REL(eff) / REL(spd) / REL(stall), or REL(32) if nothing
// was ever detected and L2_AUTO_RELEASE_DEG forced it) and in the trace's `trg` column, so the next
// trace says exactly which signal moved -- or that none did.
//
//   Releases early, with the pin still in the air: raise L2_CONTACT_MS (30 -> 50), lower
//     L2_CONTACT_SPEED_FRAC (0.55 -> 0.45), raise L2_CONTACT_LAG_DEG (2.2 -> 3.0). Read the trace's
//     `trg` column first -- fix the signal that actually fired, not all three. Remember AGREEMENT (see
//     contact_detector.hpp) means this is now rare: a lone signal can no longer fire by itself.
//   Presses on after the pin has landed / never releases until L2_AUTO_RELEASE_DEG forces it anyway:
//     lower L2_CONTACT_LAG_DEG and L2_CONTACT_EFFORT_MV together (raising just one won't help once
//     agreement needs two), and lower L2_CONTACT_MS. Check the trace: if only ONE of `lag`/`eff`/`v-b`
//     ever crosses its threshold, that is AGREEMENT doing its job on a real landing too, which means
//     the thresholds are too tight for this stack's actual landing signature -- loosen the one that
//     stays furthest from firing, not all three.
//   False-fires in plain free descent with nothing under the arm: this is the failure mode AGREEMENT
//     and the 2026-10-02 retune exist to fix (six-bar gravity torque swings up and back past vertical,
//     moving speed/effort faster than the 200 ms baselines adapt). If it still false-fires, raise
//     L2_CONTACT_MS further, or lower L2_CONTACT_FREEZE_FRAC (0.82 -> 0.75) so the baseline keeps
//     tracking a genuine swing longer before it locks and starts measuring a shortfall against it.
//     AGREEMENT itself (contact_detector.hpp) is doing most of the real work here, not these three
//     thresholds alone -- they were only moved moderately (not all the way to the first-draft numbers
//     this comment used to suggest) because the host test's own creep-landing case stopped firing
//     altogether past a certain point: AGREEMENT plus thresholds that are TOO strict together can
//     reject a genuinely gentle landing as readily as a false one. L2_AUTO_RELEASE_DEG is the backstop
//     for whatever this still misses.
// SCALED WITH THE SPEED (2.2 -> 3.3, 1.5x). This is accumulated shortfall in DEGREES, so a faster
// descent racks degrees up quicker for the same disturbance -- leaving it at 2.2 would make it easier
// to false-fire in free descent, which has bitten this arm before. Scaling it up keeps the margin.
inline constexpr double L2_CONTACT_LAG_DEG = 3.3;            // TUNE: arm deg of accumulated shortfall (was 2.2)
inline constexpr double L2_CONTACT_LAG_LEAK_MS = 200.0;      // TUNE: makes LAG measure a change, not a drift
// SCALED WITH THE AUTHORITY (1800 -> 2700, 1.5x), because this is an absolute mV figure and
// L2_PRESS_MAX_MV just went up 1.5x: the loop now has half again as much headroom to push into a
// stack, so the same number would fire on less real resistance.
inline constexpr double L2_CONTACT_EFFORT_MV = 2700.0;       // TUNE: extra downward push over baseline (was 1800)
inline constexpr double L2_CONTACT_SPEED_FRAC = 0.55;        // TUNE: measured / baseline that counts as contact (was 0.70)
inline constexpr double L2_CONTACT_STALL_DEG_PER_S = 6.0;    // TUNE: absolute near-stop, needs no baseline, no AGREEMENT
inline constexpr int L2_CONTACT_MS = 30;                     // TUNE: confirm window (was 20 -- see AGREEMENT below)
// ARM ON MOTION, added 2026-10-02 (see contact_detector.hpp): arming now also needs this much
// cumulative travel since the press started, not just a lucky instantaneous sample over
// L2_CONTACT_STALL_DEG_PER_S once L2_CONTACT_SETTLE_MS has elapsed. This is what stops a loaded arm's
// slow, still-accelerating start from snapshotting a baseline too early.
//   Baseline looks wrong / fires early on a loaded press: raise this (3.5 -> 5.0).
//   Takes too long to arm on a normal unloaded press: lower it, but not below a degree or two -- it
//   needs to be more than sensor noise could ever integrate into over L2_CONTACT_SETTLE_MS.
// SCALED WITH THE SPEED (3.5 -> 5.25, 1.5x). The detector refuses to arm until the arm has travelled
// this far, so the start-of-move transient cannot fire it. A faster press covers 3.5 deg sooner, i.e.
// earlier in that transient, so the distance has to grow with the speed to keep the same margin.
inline constexpr double L2_CONTACT_ARM_TRAVEL_DEG = 5.25;    // TUNE: arm deg since PRESS started (was 3.5)
// How long the press gets to reach a steady descent speed before the baseline is snapshotted and the
// baseline-relative signals come alive. The absolute near-stop is live before this (after the dead
// time below), so a pin that is already resting is still caught immediately.
inline constexpr int L2_CONTACT_SETTLE_MS = 200;             // TUNE
// SPLIT STALL's "never really started" window, added 2026-10-02: a pin+cup's weight can keep a
// perfectly free arm under L2_CONTACT_STALL_DEG_PER_S for longer than the ordinary ignoreMs+confirmMs
// (L2_CONTACT_IGNORE_MS + L2_CONTACT_MS) while it fights gravity on the way to actually arming -- that
// was most of why a loaded claw auto-released almost immediately. Once the arm HAS armed_ at any point
// this press, a later stop is unambiguous and still confirms at the fast ignoreMs+confirmMs; only a
// STALL that has never seen the arm arm_ waits this long. See contact_detector.hpp's "ARM ON MOTION,
// AND SPLIT STALL".
//   A genuinely empty floor still auto-releases almost instantly: that is now L2_CONTACT_MS/IGNORE_MS
//     (fast path) if it never looked like it might be loaded, which is correct -- this window only
//     matters once the arm starts out moving slowly.
//   A loaded claw over bare floor still reads as resting before L2_AUTO_RELEASE_DEG: raise this
//     further (350 -> 500), and/or raise L2_PRESS_MAX_MV so the loaded arm can actually clear
//     L2_CONTACT_STALL_DEG_PER_S sooner.
//   A real resting pin now takes noticeably longer to confirm: that trade is intentional -- see the
//     class comment in contact_detector.hpp. L2_AUTO_RELEASE_DEG's travel rule is the backstop either way.
inline constexpr int L2_CONTACT_NOSTART_MS = 350;            // TUNE: ms, STALL's window while never armed_
// Baseline EMA time constants. The baselines must track the slow, genuine sag in a free descent's
// speed and cost as the gravity torque changes with angle -- otherwise LAG drifts into a false
// trigger -- but they FREEZE as soon as the arm is below L2_CONTACT_FREEZE_FRAC of the baseline, so
// they can never chase a developing landing and mask it. That two-level gate is the whole trick; the
// host test has a scenario for each half of it.
inline constexpr double L2_CONTACT_BASELINE_TAU_MS = 200.0;  // TUNE
inline constexpr double L2_CONTACT_OUTPUT_TAU_MS = 50.0;     // TUNE: low-pass on the mV, for EFFORT
// Lowered 0.90 -> 0.82 (2026-10-02): freeze LESS aggressively, so the baseline keeps tracking a real
// but transient gravity-torque swing a little longer before locking onto a stale value and reading the
// swing's continuation as an ever-growing shortfall. See AGREEMENT in contact_detector.hpp -- this and
// the threshold changes above are parts of the same false-fire fix, and AGREEMENT is the one doing
// most of the work; this just gives the baseline a little more rope first.
inline constexpr double L2_CONTACT_FREEZE_FRAC = 0.82;       // TUNE: loose gate -- stops LEARNING, never fires (was 0.90)
// Dead time at the very start of the press before any trigger may fire. Longer than before (60 -> 80)
// now that AGREEMENT does most of the false-fire rejection -- this just gives the baseline's own
// settle time (above) a little more cushion against a rough handover from HOLD.
inline constexpr int L2_CONTACT_IGNORE_MS = 80;              // TUNE (was 60)
// After the release, the arm has to be clearly moving again (faster than L2_CONTACT_STALL_DEG_PER_S)
// for this long to count as "the claw came off the pin". Then it goes home. The RELEASE phase uses
// this plain speed test rather than the full detector: with the jaws open the arm either moves or it
// does not, and there is nothing subtle to catch.
inline constexpr int L2_FREE_MS = 100;                       // TUNE
// THE PAUSE AFTER THE RELEASE, before the arm starts down (the driver, 2026-10-07: "after it releases
// pin, it should move 200ms later"). Measured from the moment the jaws were told to open
// (clawReleasedAt_), so it covers CLAW_OPEN_TIME_MS (200) rather than adding to it -- the cylinder is
// opening for the whole window. Both ways out of Phase::RELEASE wait for it, so the arm cannot start
// descending with the jaws still on the pin.
//   It drags the pin down with it: raise this.
//   The pause is dead time you want back: lower it (the jaws still need CLAW_OPEN_TIME_MS to open).
inline constexpr int L2_RELEASE_SETTLE_MS = 200;             // TUNE: driver 2026-10-07
// ...and conversely, this long below that speed with the jaws open means the claw is hooked on the pin.
inline constexpr int L2_STUCK_MS = 250;                      // TUNE
// If the release is never acknowledged (Scoring not running, e.g. an auton that never calls
// Scoring::update()), carry on anyway after this long rather than pressing down forever. Worst case
// the arm goes home with the pin still clamped either way.
inline constexpr int L2_RELEASE_ACK_MS = 400;
// STUCK RECOVERY. Still stalled after the jaws opened = the claw is hooked on the pin. Lift UP this
// far (claw open) and try pressing down again, up to L2_MAX_UNSTICK_TRIES times. After that it stops,
// holds where it is and shows SCORE STUCK -- it never keeps grinding.
inline constexpr double L2_UNSTICK_UP_DEG = 20.0;            // TUNE: arm degrees
inline constexpr double L2_UNSTICK_SPEED_PCT = 40.0;         // TUNE: ~60 deg/s, it is only a short hop
inline constexpr int L2_UNSTICK_TIMEOUT_MS = 1200;           // safety net: don't grind upward either
inline constexpr int L2_MAX_UNSTICK_TRIES = 2;               // TUNE
// GUARANTEED RELEASE, BY RELATIVE TRAVEL (2026-10-02, replaces L2_NO_CONTACT_DEG and the old
// "NO CONTACT: keep the pin, seat with it still clamped" behaviour; corrected 2026-10-02 from an
// earlier, WRONG absolute-height version -- the user's own words: "i dnt mean the actual 32 degree
// reading, but 32 degrees relative down from the starting position it was at when we first clicked
// L2"). The detector is a real sensor reading a real stack, but it is not the only thing allowed to
// decide the pin has to come off any more: once PRESS has travelled this many degrees DOWN FROM
// Lift::scoreStartPos_ -- the position captured the instant startScore() ran, fixed for the whole
// sequence -- without a detection, it releases anyway, exactly as if the detector had fired (claw
// opens, keep pushing so the jaws slide off, same freed/UNSTK handling, same fast run home). Shown as
// REL(32) on the brain/trace. If the detector fires first, that still wins and releases immediately --
// this is strictly a floor under it, never a delay.
// RELATIVE, NOT ABSOLUTE, because the whole point is "give the detector this much of THIS press to
// prove itself", the same travel budget whether scoring starts at level 1 (31.0) or level 5 (106.9).
// An absolute reading has no such thing to offer a press that starts below 32 in the first place --
// see L2_RELEASE_FLOOR_DEG below for that case.
//   Pin dropped from a bit too high / too early on a real stack: raise it only a little -- raising it a
//     lot just turns more of the press into a guaranteed-release, no-sensing region.
//   Height rule fires before a real landing that the detector should have caught: that is a tuning
//     problem in the CONTACT_* block above, not this constant -- the detector is supposed to win first.
inline constexpr double L2_AUTO_RELEASE_DEG = 48.0;          // TUNE: arm deg, RELATIVE; 1.5x, driver 2026-10-07 (was 32)
// TWO JOBS, both "we're close enough to the bottom, stop being delicate about it", at the same height
// on purpose:
//   1. PRESS fallback for a start too low for L2_AUTO_RELEASE_DEG's travel to ever complete before the
//      floor (level 1, 31.0, can only travel ~21-31 deg before 0 depending on exactly where it seats --
//      never the full 32). If nothing is detected and the travel rule can't apply, release here instead,
//      shown as REL(floor) -- so level 1 still gets roughly 21 deg of real sensing before it gives up,
//      rather than the old (wrong) behaviour of releasing on the very first PRESS tick.
//   2. RELEASE-phase safety net (the original role): if the jaws still haven't resolved free-or-stuck
//      by the time the arm has pressed down to this height -- a slow claw-open, a borderline UNSTICK
//      retry -- stop treating this as a press and hand over to the fast run-home descent right there,
//      instead of grinding further down with L2_PRESS_MAX_MV's authority. This check is gated on
//      clawReleased_ (the claw-open request has been acknowledged, or L2_RELEASE_ACK_MS has elapsed) so
//      a PRESS-fallback release landing exactly ON this floor can never race Scoring out of ever seeing
//      needsClawRelease() -- see Lift::updateScore().
// Below any plausible landing height and below L2_AUTO_RELEASE_DEG either way, so it can only ever
// matter once a release (by any path) has already happened or the travel rule structurally cannot.
// DOWN-relative (2026-10-07): 10 deg above the hard stop = -21.9 now.
inline constexpr double L2_RELEASE_FLOOR_DEG = lift_preset::DOWN + 10.0;  // TUNE: offset above the bottom
// TWO L2 PRESSES PER SCORE (2026-10-02, the driver's own design):
//   PRESS 1  press the pin down, detect the landing, open the jaws, keep pushing for
//            then carry straight on DOWN to the bottom at the same speed and SEAT (one press),
//            indefinitely. "this is so i am able to drive away with claw up above the pins without
//            any chance of knocking over pins."
//   PRESS 2  the ordinary fast run home: descend at L2_RETURN_SPEED_PCT, FINAL, SEAT at 0.
// The held position after press 1 is a DELIBERATE, driver-requested hold that press 2 resolves into a
// proper seat. It is NOT the old "parks a few degrees above the bottom instead of seating" bug that
// CLAUDE.md bans -- that one was a trip to the BOTTOM that failed to seat, with no second step.
// How long the arm keeps pressing DOWN after the jaws have been told to open, so they slide off the
// pin instead of lifting it back out of the cup.
//   The pin comes back up with the claw: raise it (100 -> 200).
//   The jaws grind on the pin / the stack gets pushed: lower it.
// How far the arm rises after that, relative to wherever it actually stopped, before it holds. It has
// to clear the top of the stack by enough that driving away cannot knock a pin over.
//   Still catches a pin on the way out: raise it.
//   The arm is in the way of the next pickup: lower it.
// A score or a grab asked for while a lift MOVE IS STILL IN FLIGHT waits for the arm to arrive first,
// then starts (Scoring::State::LIFT_SETTLE). That is what lets an auton route fire the lift and the
// drivetrain off together -- `auton::liftTo(level); ...drive...; auton::scoreAndWait();` -- without the
// press starting from a moving arm, and it fixes the same hazard for a driver who taps L2 mid-rise.
// Bounded, so a lift that never arrives cannot hang the macro: the lift's own LIFT_MOVE_TIMEOUT_MS
// parks it (which counts as settled) well before this, so this is only the backstop behind that.
inline constexpr int SCORING_SETTLE_WAIT_MS = 3000;          // TUNE
// The run home on PRESS 2, once the claw is free: % of free speed for the top and middle bands of the
// ordinary descent (the slow final zone and the SEAT are unchanged, so it still cannot slam or bounce).
//   "Don't hesitate to be going down faster than we currently are going when we are done scoring."
// RAISED 95 -> 110 (2026-10-03, same driver request as LIFT_DESCENT_TOP_PCT: the resets need more speed).
// Over 100% is not a speed the arm can reach -- it is how you say "do not hold back", i.e. let the
// velocity loop saturate at its full downward authority. It also now runs on the FAST descent's ramp
// (LIFT_FAST_DESCENT_RAMP_DEG_PER_S2, 2000) rather than the normal 1200, which is NOT optional at this
// speed: decelerating from ~165 deg/s to the 45 deg/s approach cruise takes (165^2 - 45^2) / (2 * 1200) =
// 10.5 deg at the old ramp -- nearly the whole 14 -> 3 deg approach zone, leaving nothing in which to
// slow to FINAL's handover -- but only 6.3 deg at 2000, which leaves a real cruise before the floor.
// See startScoreReturn() in lift.cpp, and the same arithmetic spelled out on LIFT_FAST_DESCENT_*.
inline constexpr double L2_RETURN_SPEED_PCT = 100.0;         // TUNE: red retune 2026-10-07 (was 110 -- unreachable)

// ---------------------------------------------------------------------------
// Autonomous — the auton API and its tick. See include/auton.hpp, include/periodic.hpp.
// ---------------------------------------------------------------------------
// WHICH ROUTE autonomous() RUNS IS NOT HERE. It is a strategy choice, not a robot constant, so it
// lives at the top of src/autons.cpp (`constexpr Route SELECTED = ...`) with the routes themselves --
// one line, in the one file the driver works in. See README "Writing an auton route".
// The auton tick's period. The same 10 ms the driver loop runs at, because it runs THE SAME per-loop
// logic (periodic::tick()) -- Scoring's state machine, the claw autoclamp, and the one intake
// decision. The lift PID is NOT in here: it has its own 10 ms task from initialize() and is the only
// thing that ever writes the lift motors (CLAUDE.md safety rule 2).
inline constexpr int AUTON_TICK_MS = 10;
// Default timeouts for the blocking waits in auton.hpp. EVERY wait takes one, so a jammed mechanism
// can slow a routine down but can never hang it; each wait also returns whether it succeeded.
// Homing has to cover LIFT_HOME_TIMEOUT_MS plus a little slack for the task to notice.
inline constexpr int AUTON_HOME_TIMEOUT_MS = LIFT_HOME_TIMEOUT_MS + 500;
// A lift move. RAISED 2026-10-07 with the red-cartridge retune: the longest travel is now DOWN (-31.9)
// -> level 5 (106.9), i.e. ~139 deg rather than 107, and the arm does it at half the old free speed.
// This still leaves room for a macro's CLAMPING / pivot wait in front of it.
inline constexpr int AUTON_LIFT_TIMEOUT_MS = 4000;           // was 2500
// A whole L2 score: PRESS (up to L2_AUTO_RELEASE_DEG of travel) + the release checks + up to
// L2_MAX_UNSTICK_TRIES hops + the fast run home + FINAL's LIFT_FINAL_FORCE_MS.
inline constexpr int AUTON_SCORE_TIMEOUT_MS = 6000;
// A cylinder: the jaws' own CLAW_CLOSE_TIME_MS / CLAW_OPEN_TIME_MS, plus slack for waitClamped() to
// sit on the autoclamp for a moment.
inline constexpr int AUTON_CLAW_TIMEOUT_MS = 500;
// auton::waitUntilClamped(): long enough to drive into a pin and have the autoclamp confirm it.
inline constexpr int AUTON_CLAMP_TIMEOUT_MS = 2000;
// auton::grabAndWait(): the push (L1_GRAB_PUSH_MS, plus the jaws' own close time) and a little slack.
inline constexpr int AUTON_GRAB_TIMEOUT_MS = 1500;
// How long autonomous() will wait for odometry/IMU calibration before starting the route anyway. It
// is a BOUND, not a requirement: an unplugged or non-answering IMU must not be able to eat the whole
// autonomous period, and a route that never drives (the lift test) does not need odometry at all.
inline constexpr int AUTON_CALIBRATE_TIMEOUT_MS = 3000;

// ---------------------------------------------------------------------------
// moveB — the rear-distance-sensor drive, ported 1:1 from the team's old library
// ---------------------------------------------------------------------------
// NOT an odometry move: a hand-rolled PD loop on the REAR distance sensor that drives both sides at
// equal power until that sensor reads a target distance from whatever is behind the robot, then
// settles. See auton::moveB() for the full description. Every number the old library had baked in is
// here, and EVERY ONE NEEDS RETUNING: the old robot had 3.25" traction wheels, this one has 2.75"
// omnis, so the same power produces a different speed and the same error produces a different
// overshoot.
inline constexpr int8_t BACK_DISTANCE_PORT = 14;             // confirmed 2026-10-03
// ===========================================================================
// TUNING moveB — THESE FOUR KNOBS, AND HOW TO TURN THEM
// ===========================================================================
// HOW TO TUNE IT: point the robot's BACK at a wall and run one move, e.g.
//     auton::moveB(1050, true, false, 90, 20, 1500);
// When it finishes, brain line 9 shows what happened:
//     MOVEB tgt 1050 got 1043 err  -7mm 820ms SETTLED
// "err" is the signed miss in mm: NEGATIVE = it stopped SHORT of the target, POSITIVE = it went PAST.
// Run it again and change ONE knob at a time. The same line prints to the terminal too.
//
//   SYMPTOM                             WHAT TO CHANGE
//   stops short every time, err small   lower MOVEB_TOLERANCE_MM (it is being allowed to stop early)
//   stops short, err large, SETTLED     raise MOVEB_KP
//   crawls the last bit / TIMED OUT     raise MOVEB_KP, or raise the minSpeed argument
//   overshoots or hits the wall         raise MOVEB_KD first, then lower MOVEB_KP
//   oscillates / hunts around target    raise MOVEB_KD; if it still hunts, lower MOVEB_KP
//   whole move too slow                 raise the maxSpeed argument, then MOVEB_KP
//   stops at a different place each run  raise MOVEB_SETTLE_MS (it is calling it done too eagerly)
//
// TUNE IN THIS ORDER: MOVEB_KP until it reaches the target without being painfully slow, then
// MOVEB_KD until it stops overshooting, then MOVEB_TOLERANCE_MM / MOVEB_SETTLE_MS for how fussy
// "arrived" is. maxSpeed and minSpeed are per-call ARGUMENTS, not constants -- change them in the
// route, not here.
inline constexpr double MOVEB_KP = 0.28;                     // TUNE: drive power per mm of error. UP = pulls harder / reaches further, too much = overshoot and hunting
inline constexpr double MOVEB_KD = 0.68;                     // TUNE: braking per mm/tick of closing speed. UP = less overshoot and less hunting, too much = sluggish and twitchy
inline constexpr double MOVEB_TOLERANCE_MM = 5.0;            // TUNE: how close counts as arrived. UP = stops sooner and sloppier, DOWN = more accurate but can time out
inline constexpr int MOVEB_SETTLE_MS = 55;                   // TUNE: must stay inside tolerance this long. UP = more repeatable, slower to finish
inline constexpr int MOVEB_DEFAULT_MIN_SPEED = 15;           // TUNE: floor power, used when the caller passes minSpeed <= 0
inline constexpr int MOVEB_OUT_OF_RANGE_MM = 9910;           // at/above this the sensor sees nothing (not a tuning knob)
inline constexpr int MOVEB_LOOP_MS = 20;                     // the loop period; leave it alone unless MOVEB_KD stops making sense

}  // namespace config
