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
inline constexpr double PIVOT_CLEAR = 25.0;                  // TUNE: lowest height the pivot can flip out without hitting anything
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
// Lift motion — what the hardware can do, and how a move is shaped
// ---------------------------------------------------------------------------
// Both lift motors are RED (100 RPM) cartridges (see the MotorGears::red in Lift::Lift()). Through
// the 12T -> 48T (4:1) reduction the ARM's free speed is 100 / 4 = 25 arm RPM = 25 * 360 / 60 =
// 150 arm deg/s. That number is the ceiling for every speed below; nothing should ask for more.
inline constexpr double LIFT_FREE_SPEED_DEG_PER_S = 150.0;   // red 100 RPM / 4:1 = 25 arm RPM = 150 deg/s
//
// HOW A MOVE IS SHAPED (Lift::update(), and read the "Motion" part of lift.hpp's class comment):
// EVERY move, up or down, is a TRAPEZOIDAL PROFILE on the setpoint -- accelerate at
// LIFT_ACCEL_DEG_PER_S2, cruise at LIFT_MAX_SPEED_DEG_PER_S, decelerate at LIFT_DECEL_DEG_PER_S2 so
// the setpoint arrives at the goal with zero speed. The velocity feedforward (LIFT_KV) supplies the
// voltage that speed needs, so the PID only ever corrects the small leftover tracking error, with
// gains low enough that it cannot ring. Then, once the arm is inside the goal's tolerance, the loop
// STOPS CHASING and switches to a gentle hold (LIFT_HOLD_KP) -- it does not keep a high-gain PID
// hunting around the target.
//
// This replaced a setpoint that JUMPED to the goal with kP = 600 doing all the work. That version
// oscillated and then tripped a guard, because a jumped setpoint means a huge instant error, full
// 12000 mV, and nothing in the loop that knows when to start slowing down: the only thing stopping
// the arm was the error shrinking, which happens after the arm is already there. A profile knows the
// stopping distance in advance. Do not go back to jumping the setpoint.
inline constexpr double LIFT_MAX_SPEED_DEG_PER_S = 140.0;    // TUNE: profile cruise, ~93% of free speed
// Accel/decel, arm deg/s^2. At 600 the profile reaches cruise in 0.23 s over 16 deg of travel, so a
// 0 -> top (128.3 deg) move is ~1.15 s. Decel is deliberately a little gentler than accel: an
// under-estimated decel just means braking starts earlier, which is the safe direction.
//   Overshoots the target / arrives still moving: LOWER LIFT_DECEL_DEG_PER_S2 (starts braking sooner).
//   Feels lazy leaving a height: raise LIFT_ACCEL_DEG_PER_S2.
inline constexpr double LIFT_ACCEL_DEG_PER_S2 = 600.0;       // TUNE
inline constexpr double LIFT_DECEL_DEG_PER_S2 = 450.0;       // TUNE
// The profile starts at this speed instead of 0 so the first 10 ms tick already puts real voltage on
// the motors -- that is what makes a button press feel instant. It is floored UNDER the deceleration
// limit, so it can never stop the profile braking properly near the goal.
//   Noticeable lag between the button and the arm moving: raise it.
//   Jerks/clunks at the start of a move: lower it.
inline constexpr double LIFT_START_SPEED_DEG_PER_S = 40.0;   // TUNE
// SETPOINT LEASH: the profiled setpoint is never allowed to get more than this far ahead of where the
// arm actually is. This is the single most important number for "snappy but never oscillates". If the
// arm cannot keep up (heavy pin, cold motors, a steep part of the six-bar's travel), the setpoint
// WAITS for it instead of running away, so the tracking error -- and therefore the PID's contribution
// -- is bounded at LIFT_MAX_LEAD_DEG * LIFT_KP no matter how far the goal is. No windup, no stored
// error to overshoot with, and the stall guard still sees a real jam because the arm stops moving.
//   Arm visibly stutters/pulses on a long move: raise it (the leash is binding; 12-15).
//   Any overshoot at all: lower it (6).
inline constexpr double LIFT_MAX_LEAD_DEG = 8.0;             // TUNE
// Volts (mV) per arm deg/s, i.e. LIFT_MAX_MV / LIFT_FREE_SPEED_DEG_PER_S = 12000/150 = 80. The
// VELOCITY FEEDFORWARD: while the profile's setpoint is moving at v deg/s, the loop adds LIFT_KV * v
// mV, so the motors are already turning at about the right speed and the PID is left with only the
// error. This is what lets the gains be low enough not to ring while the arm still moves fast.
//   Arm lags the setpoint the whole way (brain `err` holds steady well away from 0 at cruise): raise it.
//   Arm leads / runs ahead of the setpoint (`err` holds steady the other way): lower it.
inline constexpr float LIFT_KV = 80.0f;                      // TUNE: mV per arm deg/s of setpoint speed

// Lift PID — TUNE. Units: error in lift degrees, output in millivolts (-12000..12000).
// These are deliberately LOW. With the profile and LIFT_KV doing the driving, kP only has to pull in
// a few degrees of tracking error (bounded by LIFT_MAX_LEAD_DEG), so at the leash limit it adds
// 8 * 220 = 1760 mV on top of the feedforward. kP = 600 with a jumped setpoint is what oscillated.
//   Oscillates / rings at a preset: lower kP (150), raise kD a little.
//   Arm consistently lags the profile (steady `err` at cruise, slow to close the last few degrees):
//     raise LIFT_KV first -- that is the right knob for a lag. Only then raise kP (toward 320).
inline constexpr float LIFT_KP = 220.0f;                     // TUNE: was 600 (which oscillated)
inline constexpr float LIFT_KI = 0.0f;                       // TUNE (add last, only if it stalls short)
inline constexpr float LIFT_KD = 400.0f;                     // TUNE: light damping; the profile does the shaping
inline constexpr float LIFT_WINDUP_RANGE = 5.0f;             // integral only builds within 5 deg of target
inline constexpr float LIFT_KG = 1200.0f;                    // TUNE: mV that holds the lift still (gravity feedforward)
inline constexpr float LIFT_MAX_MV = 12000.0f;               // output limit (also the manual override's full-send power)

// ---------------------------------------------------------------------------
// Lift arrival: tolerance, then hold. Precision traded for clean stops, on purpose.
// ---------------------------------------------------------------------------
// How close counts as arrived. Once the arm is inside this band of the goal (and has stayed there for
// LIFT_HOLD_MS) the loop stops chasing the target and holds: that is what makes the stop clean instead
// of a hunt. 5 deg of final error at a stack level is fine -- the pin only has to land in the cup --
// and trading that precision for a non-oscillating stop is a deliberate call.
inline constexpr double LIFT_TOLERANCE_DEG = 5.0;            // TUNE: stack levels, DOWN, MATCHLOAD
// INTAKE is the exception: the claw has to line up with the handoff, so a precise goal keeps the full
// loop running inside a tight band and crawls the last LIFT_PRECISE_ZONE_DEG at LIFT_PRECISE_SPEED.
// Lift::setTarget(h, true) marks a move precise, and any goal within half a degree of
// lift_preset::INTAKE is promoted automatically so no call site can forget.
inline constexpr double LIFT_PRECISE_TOLERANCE_DEG = 1.0;    // TUNE: INTAKE only
inline constexpr double LIFT_PRECISE_SPEED_DEG_PER_S = 25.0; // TUNE: cruise for the last stretch of a precise move
inline constexpr double LIFT_PRECISE_ZONE_DEG = 12.0;        // TUNE: how far out the precise crawl starts
// Hysteresis: once holding, the loop only goes back to chasing if the arm drifts this much FURTHER
// than the tolerance. Without it the lift flickers between "moving" and "holding" right at the band
// edge, which is its own kind of oscillation.
inline constexpr double LIFT_HOLD_RELEASE_DEG = 3.0;         // TUNE
inline constexpr int LIFT_HOLD_MS = 60;                      // TUNE: must be in the band this long before holding
// The hold itself: gravity feedforward plus a low-gain P, capped well below the guards' thresholds so
// a hold can never look like a stall or a runaway. It is there to stop the arm sagging, not to close
// the last degree -- that is what the tolerance is for.
//   Arm sags out of the band after arriving: raise LIFT_HOLD_KP, or raise LIFT_KG.
//   Arm twitches/buzzes while holding: lower LIFT_HOLD_KP.
inline constexpr float LIFT_HOLD_KP = 120.0f;                // TUNE
inline constexpr float LIFT_HOLD_MAX_MV = 4000.0f;           // TUNE: must stay under LIFT_FAULT_MV and LIFT_STALL_MV
// Heading to DOWN (0): within this many degrees of 0 the motors just brake and hold. The one place the
// gravity feedforward must not be allowed to push UP -- an arm sitting on the hard stop with kG under
// it is what made the lift bounce at startup (17ab865).
inline constexpr double LIFT_REST_BAND_DEG = 3.0;
// Any goal at or below this counts as "a trip to the bottom", which is where Scoring's pivot-retract
// floor (Lift::setDescentFloor()) is allowed to hold the setpoint. INTAKE (1.1) is inside it.
inline constexpr double LIFT_BOTTOM_GOAL_DEG = 5.0;          // TUNE

// ---------------------------------------------------------------------------
// Lift guards
// ---------------------------------------------------------------------------
// Stall guard: the motion profile still has distance left to cover but the arm moved less than
// STALL_DEG in STALL_MS = it can't get there (preset above the real top, or jammed). The lift gives up
// on that target, holds where it is, and the brain shows BLOCKED.
// There is deliberately NO voltage threshold any more. The setpoint leash (LIFT_MAX_LEAD_DEG) makes a
// high output impossible during a jam: the setpoint stops dead against the stuck arm, the velocity
// feedforward goes to zero, and all that is left is kP * the pinned error -- about 3000 mV pushing up,
// and only a few hundred pushing down where kG nearly cancels it. The old "output over 8000 mV" test
// would simply never have fired, and the lift would have pushed quietly against a jam forever.
// "The profile wants to move and the arm doesn't" is the real condition. It cannot false-trip: the
// slowest the profile ever walks is LIFT_PRECISE_SPEED_DEG_PER_S, which still covers ~17 deg in a
// STALL_MS window, and a full-speed move covers ~1.4 deg in a single 10 ms tick.
//   Shows BLOCKED on a normal move: raise LIFT_STALL_MS, or raise LIFT_STALL_DEG if the arm genuinely
//   creeps somewhere in its travel. First, though, check it isn't a real jam or a preset above the top.
inline constexpr double LIFT_STALL_DEG = 0.5;                // TUNE: arm deg of progress that counts as moving
inline constexpr int LIFT_STALL_MS = 700;                    // TUNE
// Runaway guard: pushing over FAULT_MV in the direction that should CLOSE the gap to the goal, for
// FAULT_MS, and the distance to the goal ends up FAULT_DEG BIGGER than when that push started =
// sensor/motor direction mismatch -> cut power, "LIFT FAULT" on the brain.
// It watches the distance to the GOAL, not the tracking error against the walking setpoint. It has to:
// the setpoint leash (LIFT_MAX_LEAD_DEG) deliberately keeps the tracking error small by dragging the
// setpoint along with the arm, so a reversed sensor would hide behind it. Distance to the goal cannot
// hide -- a normal move always closes on the goal, and a reversed one runs away from it at 100+ deg/s,
// which blows past FAULT_DEG inside the FAULT_MS window with room to spare.
// It deliberately does NOT compare push direction against which way the arm is moving: braking a
// descending arm pushes up while the arm still moves down, which is correct, not a runaway.
// A FAULT is cleared by the manual override OR by a new explicit target (see Lift::setTarget()): the
// guard's job is to stop the motion happening now, not to brick every button until someone remembers
// to wiggle the D-pad. A real wiring fault simply re-faults within FAULT_MS.
//   LIFT FAULT on a normal move: raise LIFT_FAULT_DEG, and check LIFT_KV / the direction check in the README.
inline constexpr float LIFT_FAULT_MV = 6000.0f;
inline constexpr int LIFT_FAULT_MS = 300;
inline constexpr double LIFT_FAULT_DEG = 5.0;                // TUNE: growth in distance-to-goal that means runaway

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
// Pivot on/off at each pickup spot. It flips once the lift has arrived there
// (it always comes back in before the lift moves, so it never swings while travelling).
inline constexpr bool PIVOT_ON_AT_INTAKE = true;             // R1: pivot ON while intaking
inline constexpr bool PIVOT_ON_AT_MATCHLOAD = false;         // TODO(team): L2 matchload spot, on or off?
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
