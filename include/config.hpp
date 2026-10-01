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
// TODO(team): confirm ports, direction, and whether the 55W is 5x 11W or 4x 11W + 2x 5.5W.
inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS = {-8, -2, -1};   // TODO(team): confirm
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = {3, 9, 10};     // TODO(team): confirm

inline constexpr float DRIVE_WHEEL_DIAMETER_IN = 2.75f;      // notebook: 2.75" omni wheels
inline constexpr float DRIVE_RPM = 450.0f;                   // notebook: 450 RPM
inline constexpr float TRACK_WIDTH_IN = 11.5f;               // TODO(team): measure wheel-center to wheel-center
inline constexpr float HORIZONTAL_DRIFT = 2.0f;              // LemLib: 2 for a drive with traction wheels

// Odometry: one vertical tracking wheel pod. No inertial sensor is on the robot yet.
inline constexpr int8_t ODOM_VERTICAL_ROTATION_PORT = 17;     // TODO(team): confirm (negative to reverse)
inline constexpr float ODOM_WHEEL_DIAMETER_IN = 2.0f;        // TODO(team): confirm pod wheel (2" / 2.75")
inline constexpr float ODOM_VERTICAL_OFFSET_IN = 0.0f;       // TODO(team): measure left/right offset from center
// Inertial sensor: the robot doesn't have one mounted yet (port 8, the old value here, is a right
// drive motor -- there is no IMU on this robot to conflict with it). 0 = none fitted: robot.cpp
// builds no pros::Imu at all, LemLib gets a null IMU pointer (its own documented way to run on the
// tracking wheel alone), nothing calibrates it, and the brain shows "IMU  none" instead of junk.
// To add one later: set this to the IMU's own port (not a drive port) -- nothing else changes.
inline constexpr int8_t IMU_PORT = 18;                        // TODO(team): set once an IMU is mounted

// Chassis PID — TUNE. Starting values from the LemLib tutorial, not tuned on JOKR.
inline constexpr float LATERAL_KP = 10.0f, LATERAL_KI = 0.0f, LATERAL_KD = 3.0f;   // TUNE
inline constexpr float ANGULAR_KP = 2.0f, ANGULAR_KI = 0.0f, ANGULAR_KD = 10.0f;   // TUNE

// ---------------------------------------------------------------------------
// Lift (DR6-4B)
// ---------------------------------------------------------------------------
// Two motors on the 12T motor axle, geared into the 48T lift-arm axle: 12T -> 48T = 4:1 torque.
// Everywhere else in the code (presets, PID target/error, lift.getPosition(), the brain screen)
// works in LIFT-ARM DEGREES -- this ratio is what converts the motor/sensor side of the 4:1 down
// to that one shared unit. See Lift::getMotorPosition()/getSensorPosition() in subsystems/lift.cpp.
inline const std::vector<std::int8_t> LIFT_PORTS = {20, -13};             // TODO(team): confirm ports + which side is reversed
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

// Preset heights, in DEGREES OF THE LIFT ARM (what lift.getPosition() returns -- rotation sensor
// centidegrees already divided by LIFT_ROTATION_RATIO, see the lift gearing note above). 0 = fully
// down (where the robot is turned on). Measured straight off the brain's LIFT line in 7 reference
// photos (docs/robot/photos/levels/, 2026-09-30) with the robot powered on and the arm physically
// moved to each position by hand; see docs/robot/ROBOT.md "Lift levels" for the photo-by-photo readout.
//
// CAVEAT: those photos were taken on a build where Lift::zero() still reset_position()'d the sensor
// at every boot (startup-relative), before it was changed to seed from the sensor's own persistent
// absolute zero (see the comment in Lift::zero(), lift.cpp). If the arm wasn't sitting exactly at
// the true physical bottom when the robot was powered on for that photo session, every one of these
// numbers carries that same constant offset. Re-measure against the brain's LIFT line (now reading
// off the absolute zero) if anything here looks off on the real robot.
namespace lift_preset {
inline constexpr double DOWN = 0.0;                          // resting / start position (IMG_6088 read 0.1, i.e. ~0)
// TODO(team): RE-MEASURE. 1.1 arm deg is only 4.4 sensor deg -- about a millimetre of end-effector
// travel on a six-bar, which is not a visible "lift to the handoff". It is almost certainly a
// mis-read of IMG_6089 (or the photo was taken with the arm already resting at the bottom). It was
// also smaller than the old LIFT_SETTLE_DEG (1.5), which is why R1 reported "arrived at INTAKE"
// without the arm moving at all. To fix: run the program, D-pad Up until the claw lines up with the
// intake handoff, read the brain's LIFT line, and put that number here.
inline constexpr double INTAKE = 1.1;                        // TODO(team): re-measure, see above (photo IMG_6089)
inline constexpr double MATCHLOAD = 15.0;                    // TUNE: not covered by the reference photos yet
// L1 taps: tap 1 -> level 1, tap 2 -> level 2 ... tap 5 -> level 5 (stays at 5, the top). One entry
// per pin-stack height, measured from photos IMG_6090..IMG_6094 (score levels 1-5), 2026-09-30.
inline constexpr int STACK_LEVEL_COUNT = 5;
inline constexpr double STACK_LEVELS[STACK_LEVEL_COUNT] = {
    26.6,   // measured from photo IMG_6090, 2026-09-30 (level 1)
    54.4,   // measured from photo IMG_6091, 2026-09-30 (level 2)
    72.8,   // measured from photo IMG_6092, 2026-09-30 (level 3)
    116.3,  // measured from photo IMG_6093, 2026-09-30 (level 4)
    128.3,  // measured from photo IMG_6094, 2026-09-30 (level 5, the top)
};
// Software hard stop: top level (128.3, IMG_6094) + 5 deg of margin. setTarget()/moveTo() clamp to
// this, so it also doubles as "never command past here" -- raise it only once a higher level is
// actually measured and added above, never just to let a bad preset through.
inline constexpr double MAX = 133.3;
}  // namespace lift_preset

// ---------------------------------------------------------------------------
// Lift motion — DRIVE -> BRAKE -> HOLD. Read lift.hpp's class comment for the full picture.
// ---------------------------------------------------------------------------
// Both lift motors are RED (100 RPM) cartridges (see the MotorGears::red in Lift::Lift()). Through
// the 12T -> 48T (4:1) reduction the ARM's free speed is 100 / 4 = 25 arm RPM = 25 * 360 / 60 =
// 150 arm deg/s. That is the most the arm can do with the D-pad held, and it is what every preset move
// now does too: full voltage from the first tick, so the `vel` column of the trace (see LIFT_TRACE)
// should climb to about this number on any move longer than ~10 deg.
inline constexpr double LIFT_FREE_SPEED_DEG_PER_S = 150.0;   // red 100 RPM / 4:1 = 25 arm RPM = 150 deg/s
inline constexpr float LIFT_MAX_MV = 12000.0f;               // DRIVE voltage, and the manual override's full-send power
//
// HOW A MOVE WORKS. There is no position PID, no setpoint profile and no gravity feedforward any more.
// Every preset move is three phases, decided every 10 ms from the arm's measured POSITION and VELOCITY:
//   DRIVE   full +-LIFT_MAX_MV toward the goal (exactly what the D-pad does) until the arm is going
//           as fast as the braking curve below allows for the distance that is left,
//   BRAKE   a velocity loop (LIFT_VEL_KP / LIFT_VEL_KI) that follows that braking curve down to a
//           stop -- or, for INTAKE and DOWN, down to a slow CREEP that it holds until it is inside
//           the goal's tolerance,
//   HOLD    the motors' own brake-hold (MotorBrake::hold), the same thing that holds the arm when the
//           D-pad is let go. Arrival LATCHES here: nothing in this phase pushes, so nothing can hunt.
// The braking curve is the stopping-distance relation v = sqrt(2 * a * d) with a =
// LIFT_BRAKE_DECEL_DEG_PER_S2 and d = distance to go minus LIFT_BRAKE_MARGIN_DEG. It is computed from
// distance, not time, so an arm that is ahead of or behind the curve is corrected automatically.
//
// Why this and not the two designs before it: a jumped setpoint with kP = 600 oscillated (d2978c7); a
// trapezoidal profile + leash + feedforward + PID + gravity term + arrive/hold hysteresis (6b8165c) was
// slow off the line (the leash capped the net drive at ~3000 mV) and STILL oscillated, because its hold
// relied on a fixed gravity feedforward that is wrong at most angles of a six-bar, so the arm drifted
// out of the band, the loop re-chased with a D-term kick, and it drifted again -- a limit cycle. The
// design here cannot do that: HOLD is the firmware's brake-hold, and a move is re-engaged from HOLD at
// most LIFT_MAX_REDRIVES times, only for an error bigger than LIFT_REENGAGE_DEG.
//
// THE ONE NUMBER THAT SETS WHERE IT STOPS. How hard the arm can really brake, in arm deg/s^2. The
// loop starts braking when the arm is this many deg/s^2 away from stopping at the goal, so a value
// UNDER the real capability stops SHORT (safe), a value OVER it overshoots. Read it off the trace: in
// the BRAKE phase the `vel` column drops by about a * 0.02 per 20 ms line.
//   Stops short of every preset by the same few degrees: RAISE it (braking starts later).
//   Overshoots / arrives at a level still moving / taps the frame at DOWN: LOWER it.
inline constexpr double LIFT_BRAKE_DECEL_DEG_PER_S2 = 2000.0; // TUNE: conservative on purpose
// Where the braking curve aims to reach zero speed: this far before the goal, so tracking lag lands
// the arm on the goal rather than past it.
inline constexpr double LIFT_BRAKE_MARGIN_DEG = 2.0;         // TUNE
// The velocity loop that follows the curve (BRAKE and CREEP). Output mV = KP * (vref - v) + the
// integral of KI * (vref - v); the integral is what learns gravity (whatever voltage it takes to hold
// the arm's speed at this angle, up or down), so no gravity constant has to be right in advance.
//   `vel` swings above and below the curve in the trace (hunts): lower LIFT_VEL_KP.
//   CREEP takes a long time to reach LIFT_CREEP_DEG_PER_S, or sags before it gets going: raise LIFT_VEL_KI.
inline constexpr float LIFT_VEL_KP = 30.0f;                  // TUNE: mV per (deg/s) of speed error
inline constexpr float LIFT_VEL_KI = 600.0f;                 // TUNE: mV per (deg/s) of speed error, per second
// Most the velocity loop may push, either way. Braking from 150 deg/s against this much reverse
// voltage is already harsher than the firmware hold the D-pad release does, so don't raise it casually.
inline constexpr float LIFT_BRAKE_MAX_MV = 9000.0f;          // TUNE
// BRAKE hands over to HOLD once the arm is slower than this (and inside LIFT_TOLERANCE_DEG). The
// firmware hold grabs the position where it is called, so handing over at a crawl means no spring-back.
inline constexpr double LIFT_STOPPED_DEG_PER_S = 20.0;       // TUNE
// BRAKE/CREEP can't take longer than this; if it does (arm fighting something, loop badly tuned) the
// lift simply holds where it is and, if that is far off, re-drives per LIFT_REENGAGE_DEG below.
inline constexpr int LIFT_APPROACH_TIMEOUT_MS = 1000;        // TUNE
// Velocity estimate: position differenced every tick, then this much of each new sample is blended in
// (1.0 = raw, no filtering). 0.5 is one tick of lag, enough to take the edge off task timing jitter.
inline constexpr double LIFT_VEL_FILTER = 0.5;

// ---------------------------------------------------------------------------
// Lift arrival: tolerance, then hold. Precision traded for clean stops, on purpose.
// ---------------------------------------------------------------------------
// How close counts as arrived for the stack levels and MATCHLOAD. 5 deg of final error at a stack
// level is fine -- the pin only has to land in the cup -- and trading that for a stop with no hunting
// is a deliberate call. A goal already inside this band is "arrived" without moving at all.
inline constexpr double LIFT_TOLERANCE_DEG = 5.0;            // TUNE: stack levels, MATCHLOAD
// INTAKE is the exception: the claw has to line up with the handoff, so it is a PRECISE move -- it
// brakes down to LIFT_CREEP_DEG_PER_S by LIFT_CREEP_ZONE_DEG before the goal, creeps the rest, and
// hands over to HOLD inside this tight band. Lift::setTarget(h, true) marks a move precise, and any
// goal within half a degree of lift_preset::INTAKE is promoted automatically so no call site can forget.
inline constexpr double LIFT_PRECISE_TOLERANCE_DEG = 1.0;    // TUNE: INTAKE only
inline constexpr double LIFT_CREEP_DEG_PER_S = 25.0;         // TUNE: final-approach speed for INTAKE and DOWN
inline constexpr double LIFT_CREEP_ZONE_DEG = 6.0;           // TUNE: how far out the creep starts
// Heading to DOWN (0): the arm creeps (same LIFT_CREEP_* as above, so it cannot arrive at speed) and
// hands over to the brake-hold this many degrees above the hard stop. Nothing ever drives the arm
// into the frame -- an arm pushed onto its stop is what made the lift bounce at startup (17ab865).
inline constexpr double LIFT_REST_BAND_DEG = 3.0;            // TUNE: lower (1.5) to park closer to the frame
// Any goal at or below this counts as "a trip to the bottom": the DOWN creep applies, and Scoring's
// score() treats an arm already here as "nothing to score" (L2 is then a no-op). INTAKE (1.1) is inside it.
inline constexpr double LIFT_BOTTOM_GOAL_DEG = 5.0;          // TUNE
// From HOLD, the lift re-drives (DRIVE -> BRAKE -> HOLD again) only if the arm is this far from the
// goal -- a brake that stopped well short or well long -- and only LIFT_MAX_REDRIVES times per command.
// After that it holds wherever it is and the brain shows the residual error. Both together are the
// structural guarantee that a preset move cannot hunt: the number of pushes per button press is finite.
inline constexpr double LIFT_REENGAGE_DEG = 12.0;            // TUNE
inline constexpr int LIFT_MAX_REDRIVES = 2;                  // TUNE

// ---------------------------------------------------------------------------
// Lift guards
// ---------------------------------------------------------------------------
// Stall guard: a phase that WANTS motion (DRIVE at full voltage, or CREEP) but the arm moved less than
// STALL_DEG in STALL_MS = it can't get there (preset above the real top, or jammed). The lift gives up
// on that target, holds where it is (no re-drives) and the brain shows BLOCKED. It is not checked in
// BRAKE or HOLD, which are stopping on purpose. It cannot false-trip: at full voltage the arm covers
// 0.5 deg in a few tens of ms, and the creep covers it in 20 ms.
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
// C=CREEP, H=HOLD, M=manual D-pad, h=homing, F=fault, I=idle. Integers only (no float formatting on the
// control task), sampled every LIFT_TRACE_EVERY_TICKS, written from a separate low-priority task that
// drains a ring buffer -- the 10 ms control loop never calls printf. Keeps going LIFT_TRACE_TAIL_MS
// after the arm stops so the hold is visible too. Set false to silence it entirely (no task started).
inline constexpr bool LIFT_TRACE = true;
inline constexpr int LIFT_TRACE_EVERY_TICKS = 2;             // 2 x 10 ms = one line per 20 ms
inline constexpr int LIFT_TRACE_TAIL_MS = 500;

// ---------------------------------------------------------------------------
// Lift homing — find the hard stop and zero there, once per program run
// ---------------------------------------------------------------------------
// Why: the arm often powers on sitting well above the bottom. Lift::zero() can only seed the position
// counter from the rotation sensor's folded absolute angle, which is unambiguous only within +-45 arm
// deg (+-180 sensor deg) of the true bottom -- further up than that and every preset is off by a whole
// sensor revolution. Homing removes the guesswork: drive the arm down at a steady, moderate voltage
// until the sensor stops changing, confirm the sensor's absolute angle really is near its calibrated
// zero, and set_position(0) there.
//
// This is the ONE deliberate exception to "nothing moves at startup" (CLAUDE.md safety rule 1), and it
// is kept narrow: homing never runs in initialize(). It runs on the first ENABLED mode of a program
// run -- autonomous(), or opcontrol() if auton did not already do it -- so the field/competition
// switch, not power-on, is what makes the arm move. It runs inside Lift::update() as a mode, so the
// 10 ms task is still the only thing that writes the lift motors, nothing blocks on it, and holding
// D-pad Up/Down cancels it instantly (safety rule 3 is untouched). Lift targets asked for during
// homing are queued and run the moment it finishes.
inline constexpr float LIFT_HOME_MV = -4000.0f;              // TUNE: steady downward drive while homing (negative = down)
// Don't start looking for "stopped" until the arm has been driving this long, so a sticky start isn't
// mistaken for the bottom.
inline constexpr int LIFT_HOME_MIN_DRIVE_MS = 250;           // TUNE
// "Stopped" = the raw sensor moved less than LIFT_HOME_STILL_DEG (SENSOR degrees, not arm degrees)
// across a LIFT_HOME_STILL_MS window. Small wobble at the stop is expected and allowed.
inline constexpr double LIFT_HOME_STILL_DEG = 2.5;           // TUNE: sensor degrees
inline constexpr int LIFT_HOME_STILL_MS = 200;               // TUNE
// Plausibility window: at the stop, the sensor's own absolute angle (folded to +-180 sensor deg) must
// be within this much of its calibrated zero before we trust it and set_position(0). The user's own
// observation is that it settles around 358-359-0-1, i.e. within a degree or two. If the arm stopped
// somewhere else it is jammed on something, NOT at the bottom: homing then stops pushing, keeps the
// absolute-angle seed from zero(), and shows HOME FAIL instead of zeroing to a wrong place.
inline constexpr double LIFT_HOME_ZERO_WINDOW_DEG = 15.0;    // TUNE: sensor degrees either side of zero
// Hard stop on the whole attempt. Stops pushing and shows HOME FAIL.
inline constexpr int LIFT_HOME_TIMEOUT_MS = 3000;            // TUNE

// Default timeout for lift.moveTo() when the caller doesn't give one (autonomous safety net so a
// stuck lift can't hang a routine forever). Pass a different value per call, or 0 to wait forever.
// A blocking moveTo() called while homing waits for homing AND the move, so give it room for both.
inline constexpr int LIFT_DEFAULT_TIMEOUT_MS = 2500;         // TUNE

// ---------------------------------------------------------------------------
// Claw (pneumatic) + autoclamp sensor
// ---------------------------------------------------------------------------
inline constexpr char CLAW_PISTON_PORT = 'C';                // TODO(team): confirm ADI port
inline constexpr bool CLAW_EXTENDED_IS_CLOSED = true;        // TODO(team): does extending the cylinder close the jaws?
// Autoclamp: a distance sensor looking between the jaws. None is in the notebook yet.
inline constexpr int8_t CLAW_DISTANCE_PORT = 11;             // TODO(team): mount + confirm port
inline constexpr int CLAW_DETECT_MM = 40;                    // TODO(team): distance that means "pin is between the jaws"
inline constexpr int CLAW_DETECT_DEBOUNCE_MS = 60;           // pin must be seen this long before clamping
inline constexpr int CLAW_RELEASE_LOCKOUT_MS = 750;          // after a release, don't re-clamp for this long
inline constexpr bool CLAW_AUTOCLAMP_AT_START = false;       // Y toggles it in a match. Leave off until the sensor is mounted.

// ---------------------------------------------------------------------------
// Intake (ONE motor)
// ---------------------------------------------------------------------------
// Negative = reversed. If R1 spits cups out instead of pulling them in, flip the sign.
inline constexpr int8_t INTAKE_PORT = 12;
inline constexpr float INTAKE_MV = 12000.0f;

// ---------------------------------------------------------------------------
// Pivot + claw timing. See include/subsystems/scoring.hpp.
// ---------------------------------------------------------------------------
// If the claw/wrist flips with its own cylinder, set the port. ' ' = no pivot actuator.
inline constexpr char PIVOT_PISTON_PORT = 'D';
// THE PIVOT HAS TWO POSITIONS, named for where the end effector physically is (the driver's words):
//   OUT  = swung out. Where it sits at power-on, at every stack level while scoring, and in the
//          default down position after L2 / B. In the code, pivot OUT == true.
//   BACK = pulled in toward the intake handoff, for taking a pin from the intake (R1 / INTAKE).
//          In the code, pivot BACK == false.
// Which ADI level is OUT: at power-on the brain's ADI outputs are LOW and the pivot sits OUT, so the
// unpowered/LOW level is OUT and HIGH is BACK. That is also the only arrangement where Scoring can
// start with the pivot logically OUT without writing anything the ADI isn't already at -- so nothing
// moves at startup (CLAUDE.md rule 1). If pressing X the first time makes the pivot swing BACK and the
// brain's CLAW line says it went OUT, flip this one constant; nothing else needs to change.
inline constexpr bool PIVOT_OUT_IS_ADI_HIGH = false;         // TODO(team): confirm with X on the brain's CLAW line
// The matchload spot (scoreOrMatchload(), not on a driver button): take the pin with the pivot BACK
// like at the intake, or OUT like everywhere else?
inline constexpr bool PIVOT_BACK_AT_MATCHLOAD = false;       // TODO(team): confirm
// While L1 lifts a pin up out of the intake handoff, the pivot swings from BACK to OUT once the arm is
// this many degrees ABOVE lift_preset::INTAKE (or has arrived at the level, whichever is first, so a
// level can never be left waiting for it). It is a delay, not a gate: the lift starts rising the same
// tick L1's clamp finishes. 0 = swing OUT the instant the lift starts. This is the ONLY height the
// pivot's motion is tied to -- the old PIVOT_CLEAR interlock is gone (see scoring.hpp).
//   The pin swings into the intake as it leaves the handoff: raise it.
//   The pivot is still swinging when the arm reaches level 1: lower it toward 0.
inline constexpr double PIVOT_OUT_CLEARANCE_DEG = 10.0;      // TUNE
inline constexpr int PIVOT_TIME_MS = 250;                    // TUNE: time for the pivot to finish moving
// THIS IS THE L1 DELAY. The first L1 tap closes the jaws and waits here before the lift is given any
// target at all (Scoring::update(), case CLAMPING) -- interlock: the arm must not pull out of an open
// claw. So the gap between pressing L1 and the arm moving is at least this long, by design. 100 ms is
// about as low as a pneumatic claw can be trusted with.
//   Pin slips / the arm starts up before the jaws are shut: raise it back to 150.
// (Every L1 tap AFTER the first one commands the next level immediately -- no wait.)
inline constexpr int CLAW_CLOSE_TIME_MS = 100;               // TUNE: was 150. Jaws shut before the lift starts rising (L1)
// Jaws open before the arm lowers -- only for scoreOrMatchload()'s path (not currently on a button).
// L2's score() no longer waits on this: the claw opens, the pivot starts retracting, and the lift's
// own profiled descent all start in the same tick (see Scoring::enter()/update(), case RELEASING).
inline constexpr int CLAW_OPEN_TIME_MS = 200;                // TUNE

}  // namespace config
