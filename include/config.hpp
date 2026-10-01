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
inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS = {-2, -3, -1};   // TODO(team): confirm
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = {8, 9, 10};     // TODO(team): confirm

inline constexpr float DRIVE_WHEEL_DIAMETER_IN = 2.75f;      // notebook: 2.75" omni wheels
inline constexpr float DRIVE_RPM = 450.0f;                   // notebook: 450 RPM
inline constexpr float TRACK_WIDTH_IN = 11.5f;               // TODO(team): measure wheel-center to wheel-center
inline constexpr float HORIZONTAL_DRIFT = 2.0f;              // LemLib: 2 for a drive with traction wheels

// Odometry: one vertical tracking wheel pod. No inertial sensor is on the robot yet.
inline constexpr int8_t ODOM_VERTICAL_ROTATION_PORT = 7;     // TODO(team): confirm (negative to reverse)
inline constexpr float ODOM_WHEEL_DIAMETER_IN = 2.0f;        // TODO(team): confirm pod wheel (2" / 2.75")
inline constexpr float ODOM_VERTICAL_OFFSET_IN = 0.0f;       // TODO(team): measure left/right offset from center
// Inertial sensor: the robot doesn't have one mounted yet (port 8, the old value here, is a right
// drive motor -- there is no IMU on this robot to conflict with it). 0 = none fitted: robot.cpp
// builds no pros::Imu at all, LemLib gets a null IMU pointer (its own documented way to run on the
// tracking wheel alone), nothing calibrates it, and the brain shows "IMU  none" instead of junk.
// To add one later: set this to the IMU's own port (not a drive port) -- nothing else changes.
inline constexpr int8_t IMU_PORT = 0;                        // TODO(team): set once an IMU is mounted

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
inline constexpr double INTAKE = 1.1;                        // measured from photo IMG_6089, 2026-09-30
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

// Lift PID — TUNE. Units: error in lift degrees, output in millivolts (-12000..12000).
inline constexpr float LIFT_KP = 300.0f;                     // TUNE
inline constexpr float LIFT_KI = 0.0f;                       // TUNE (add last, only if it stalls short)
inline constexpr float LIFT_KD = 800.0f;                     // TUNE
inline constexpr float LIFT_WINDUP_RANGE = 5.0f;             // integral only builds within 5 deg of target
inline constexpr float LIFT_KG = 1200.0f;                    // TUNE: mV that holds the lift still (gravity feedforward)
inline constexpr float LIFT_MAX_MV = 12000.0f;               // output limit (also the manual override's full-send power)
inline constexpr float LIFT_SETTLE_DEG = 1.5f;               // "at target" band
inline constexpr int LIFT_SETTLE_MS = 150;                   // must stay in band this long
// Heading to the bottom (target 0): within this many degrees of 0 counts as "resting", motors just
// brake and hold -- see LIFT_SLOW_ZONE_DEG/LIFT_SLOW_SPEED_DPS below for the controlled approach
// into this band (this is just the final "stop fighting the frame" cutoff, keep it small).
inline constexpr double LIFT_REST_BAND_DEG = 3.0;
// Controlled descent into the bottom (Lift::update(), the goal_ <= DOWN branch). The WHOLE trip to
// 0 is profiled, not just the last stretch: the setpoint starts at wherever the arm is when the move
// is commanded and walks down at LIFT_LOWER_SPEED_DEG_PER_S (the cruise, below), then at
// LIFT_SLOW_SPEED_DPS for the last LIFT_SLOW_ZONE_DEG. The arm tracks the setpoint, so it is already
// slow by the time it gets near the frame instead of arriving at full speed with 20 deg left to stop
// in. A slow, actively-tracked setpoint is the only thing that controls fall speed -- gravity always
// beats a plain voltage cap, but it can't beat a PID that pushes back (upward, if that is what it
// takes) whenever the arm gets ahead of a setpoint that is only creeping down. Note the slow zone is
// measured on the SETPOINT, not the arm, so the handover happens before the arm is in the zone.
//   Rams into the bottom / still too fast: raise LIFT_SLOW_ZONE_DEG (hand over to the slow walk
//   earlier), lower LIFT_SLOW_SPEED_DPS, and/or lower LIFT_LOWER_SPEED_DEG_PER_S.
//   Takes too long to come down / looks like it's crawling: lower LIFT_SLOW_ZONE_DEG, raise
//   LIFT_SLOW_SPEED_DPS, and/or raise LIFT_LOWER_SPEED_DEG_PER_S.
inline constexpr double LIFT_SLOW_ZONE_DEG = 20.0;           // TUNE: start the slow walk this far above 0
inline constexpr double LIFT_SLOW_SPEED_DPS = 15.0;          // TUNE: arm degrees/sec during the slow walk
// Stall guard: output over STALL_MV but moved less than STALL_DEG in STALL_MS = can't get there
// (target above the real top, or jammed). The lift holds where it is and the brain shows BLOCKED.
inline constexpr float LIFT_STALL_MV = 8000.0f;
inline constexpr double LIFT_STALL_DEG = 0.5;
inline constexpr int LIFT_STALL_MS = 700;
// Descent cruise: the arm lowers at this speed (arm degrees per second) until the setpoint reaches
// the slow zone above, where LIFT_SLOW_SPEED_DPS takes over for the final approach. Lower = gentler.
// Every move to DOWN uses it -- the L2 score, the B stow, and auton -- because setTarget() fills it
// in for any trip to the bottom that asked for a jump (speed 0); nothing free-falls to 0 any more.
// Free reference: a red 100 RPM motor through the 4:1 is ~150 arm deg/s wide open, so this is the
// fraction of full speed the descent runs at.
//   Rams / reaches the slow zone still moving: lower it (and see LIFT_SLOW_* above).
//   Descent feels sluggish after a score: raise it -- up to ~90 is still well short of free-fall.
inline constexpr double LIFT_LOWER_SPEED_DEG_PER_S = 40.0;   // TUNE
// Descent ramp-up: the profiled trip to DOWN above doesn't start at full cruise (LIFT_LOWER_SPEED_DEG_PER_S)
// the instant it's commanded -- it starts at this slower "mid" speed and climbs LINEARLY up to cruise
// over the first LIFT_LOWER_RAMP_DEG of travel measured from wherever the move began (Lift::update(),
// the goal_ <= DOWN branch), so a score or a stow leaves the height smoothly instead of snapping
// straight to cruise. After the ramp it holds cruise until the slow zone near 0 (LIFT_SLOW_ZONE_DEG/
// LIFT_SLOW_SPEED_DPS above) takes over -- one continuous speed curve, mid -> high -> slow -> brake,
// with no stop or re-target in between (Scoring's pivot-retract floor on the walking setpoint, see
// Lift::setDescentFloor(), can briefly hold the LAST bit of that curve, never the ramp itself).
//   Leaves the top too slowly / looks sluggish at the start: raise LIFT_LOWER_START_SPEED_DEG_PER_S.
//   Snaps too hard into cruise: raise LIFT_LOWER_RAMP_DEG (ramps over more distance).
inline constexpr double LIFT_LOWER_START_SPEED_DEG_PER_S = 20.0;  // TUNE: starting ("mid") speed, deg/s
inline constexpr double LIFT_LOWER_RAMP_DEG = 15.0;               // TUNE: deg of travel to reach cruise
// Runaway guard: output over FAULT_MV, pushing in the direction that SHOULD shrink the error, for
// FAULT_MS, and |error| ends up FAULT_DEG BIGGER than when that push started = sensor/motor
// direction mismatch -> cut power, "LIFT FAULT" on the brain, controller rumbles. It deliberately
// does NOT compare push direction against which way the arm is moving: braking a descending arm
// pushes up while the arm still moves down, which is correct behaviour, not a runaway.
inline constexpr float LIFT_FAULT_MV = 6000.0f;
inline constexpr int LIFT_FAULT_MS = 300;
inline constexpr double LIFT_FAULT_DEG = 3.0;
// Default timeout for lift.moveTo() when the caller doesn't give one (autonomous safety net so a
// stuck lift can't hang a routine forever). Pass a different value per call, or 0 to wait forever.
inline constexpr int LIFT_DEFAULT_TIMEOUT_MS = 2000;         // TUNE

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
inline constexpr int CLAW_CLOSE_TIME_MS = 150;               // TUNE: jaws shut before the lift starts rising (L1)
// Jaws open before the arm lowers -- only for scoreOrMatchload()'s path (not currently on a button).
// L2's score() no longer waits on this: the claw opens, the pivot starts retracting, and the lift's
// own profiled descent all start in the same tick (see Scoring::enter()/update(), case RELEASING).
inline constexpr int CLAW_OPEN_TIME_MS = 200;                // TUNE

}  // namespace config
