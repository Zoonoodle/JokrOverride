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

// ---------------------------------------------------------------------------
// Lift speed budget — what the hardware can actually do
// ---------------------------------------------------------------------------
// Both lift motors are RED (100 RPM) cartridges (see the MotorGears::red in Lift::Lift()). Through
// the 12T -> 48T (4:1) reduction the ARM's free speed is 100 / 4 = 25 arm RPM = 25 * 360 / 60 =
// 150 arm deg/s. That number is the ceiling for every speed below; nothing in the code should ask
// for more than it, and anything far under it is leaving the lift slow on purpose.
inline constexpr double LIFT_FREE_SPEED_DEG_PER_S = 150.0;   // red 100 RPM / 4:1 = 25 arm RPM = 150 deg/s
// Volts (mV) per arm deg/s at the motor, i.e. LIFT_MAX_MV / LIFT_FREE_SPEED_DEG_PER_S = 12000/150 = 80.
// Used as a VELOCITY FEEDFORWARD on profiled (speed-limited) moves: while the setpoint is walking at
// v deg/s, the loop adds LIFT_KV * v mV so the motors are already turning at roughly the right speed
// instead of the PID having to build a big tracking error first to get there. Without it, a 130 deg/s
// descent needs ~-9000 mV, which with kP alone means the arm has to sit ~15-20 deg BELOW the setpoint
// the whole way down -- a lag that makes the slow zone near the bottom meaningless (the arm arrives
// there late and fast) and that looks exactly like a runaway to the fault guard. With it, the arm
// tracks the profile within a couple of degrees at any speed.
//   Arm lags the setpoint on a fast descent (brain `err` stays several deg negative): raise it.
//   Arm leads / dives ahead of the setpoint (`err` stays positive going down): lower it.
inline constexpr float LIFT_KV = 80.0f;                      // TUNE: mV per arm deg/s of setpoint speed
// Any goal at or below this height counts as "a trip to the bottom": it gets the profiled descent
// (ramp -> cruise -> slow zone) instead of a bare setpoint jump, so nothing can free-fall into the
// frame. INTAKE (1.1) is inside it on purpose -- R1 from the top level is a trip to the bottom in
// every way that matters mechanically. Keep it small; it is a "near the frame" test, not a preset.
inline constexpr double LIFT_BOTTOM_GOAL_DEG = 5.0;          // TUNE

// Lift PID — TUNE. Units: error in lift degrees, output in millivolts (-12000..12000).
// kP sizing: an UP move jumps the setpoint straight to the preset, so the PID is what decides how
// fast it gets there. Output saturates at 12000 mV while |error| > LIFT_MAX_MV/kP = 20 deg, so the
// arm runs wide open for all but the last 20 deg and then closes in at roughly kP/(LIFT_KV + kD/100)
// deg/s per deg of error = 600/92 ~ 6.5x error, i.e. ~65 deg/s with 10 deg to go, ~13 deg/s with 2.
// At the old kP = 300 that approach was half as fast and was most of the time a preset move took.
//   Overshoots / bounces at a preset: raise LIFT_KD first (it only resists fast error changes, it
//   barely slows the approach), then back kP off toward 450.
//   Still sluggish arriving at a preset: raise kP (700-800), keeping kD at about 2x kP.
inline constexpr float LIFT_KP = 600.0f;                     // TUNE: was 300, doubled for snappier arrivals
inline constexpr float LIFT_KI = 0.0f;                       // TUNE (add last, only if it stalls short)
inline constexpr float LIFT_KD = 1200.0f;                    // TUNE: was 800, raised with kP to damp the faster approach
inline constexpr float LIFT_WINDUP_RANGE = 5.0f;             // integral only builds within 5 deg of target
inline constexpr float LIFT_KG = 1200.0f;                    // TUNE: mV that holds the lift still (gravity feedforward)
inline constexpr float LIFT_MAX_MV = 12000.0f;               // output limit (also the manual override's full-send power)
inline constexpr float LIFT_SETTLE_DEG = 1.5f;               // "at target" band
inline constexpr int LIFT_SETTLE_MS = 150;                   // must stay in band this long
// Heading to the bottom (target 0): within this many degrees of 0 counts as "resting", motors just
// brake and hold -- see LIFT_SLOW_ZONE_DEG/LIFT_SLOW_SPEED_DPS below for the controlled approach
// into this band (this is just the final "stop fighting the frame" cutoff, keep it small).
inline constexpr double LIFT_REST_BAND_DEG = 3.0;
// Controlled descent into the bottom (Lift::update(), the headingToBottom() branch -- any goal at or
// below LIFT_BOTTOM_GOAL_DEG, which is DOWN and INTAKE). The WHOLE trip to
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
// SIZING (why 25, not 20 or 5): the slow zone has to be at least the arm's stopping distance from
// cruise, or the arm is still travelling fast when the setpoint runs out of room. Decelerating from
// LIFT_LOWER_SPEED_DEG_PER_S = 130 deg/s with the full 12000 mV of upward authority plus the rubber
// bands takes roughly 0.2 s, so v^2/(2a) with a ~ 650 deg/s^2 is ~13 deg of travel; add ~10 deg for
// the couple of ticks of tracking lag the arm still carries at cruise even with LIFT_KV, and round
// up: 25 deg. The last 25 deg then walks at LIFT_SLOW_SPEED_DPS = 35 deg/s (0.7 s) instead of the
// old 15 (1.3 s), and the arm is down to ~20 deg/s by the time it reaches LIFT_REST_BAND_DEG and
// brake-holds -- a tap, not a slam.
//   Taps/bangs the frame at the bottom: lower LIFT_SLOW_SPEED_DPS (toward 20) first, then raise
//   LIFT_SLOW_ZONE_DEG, then raise LIFT_REST_BAND_DEG (brake-hold starts higher).
//   Crawls the last bit: raise LIFT_SLOW_SPEED_DPS, lower LIFT_SLOW_ZONE_DEG.
inline constexpr double LIFT_SLOW_ZONE_DEG = 25.0;           // TUNE: start the slow walk this far above the goal
inline constexpr double LIFT_SLOW_SPEED_DPS = 35.0;          // TUNE: arm degrees/sec during the slow walk
// Stall guard: output over STALL_MV but moved less than STALL_DEG in STALL_MS = can't get there
// (target above the real top, or jammed). The lift holds where it is and the brain shows BLOCKED.
// UNCHANGED at the new speeds, on purpose: it only arms when the arm is barely moving (less than
// LIFT_STALL_DEG = 0.5 deg of travel in a whole LIFT_STALL_MS window), and a normal full-speed move
// covers 1.3 deg per 10 ms tick, so it re-arms every single tick and can never time out. Making the
// lift faster makes this guard harder to false-trip, not easier.
inline constexpr float LIFT_STALL_MV = 8000.0f;
inline constexpr double LIFT_STALL_DEG = 0.5;
inline constexpr int LIFT_STALL_MS = 700;
// Descent cruise: the arm lowers at this speed (arm degrees per second) until the setpoint reaches
// the slow zone above, where LIFT_SLOW_SPEED_DPS takes over for the final approach. Lower = gentler.
// Every move to DOWN uses it -- the L2 score, the B stow, and auton -- because setTarget() fills it
// in for any trip to the bottom that asked for a jump (speed 0); nothing free-falls to 0 any more.
// It is deliberately close to LIFT_FREE_SPEED_DEG_PER_S (150): 130 deg/s is ~87% of what the arm can
// physically do, which is as fast as a *controlled* descent can be while leaving the loop headroom to
// brake. Note the arm will NOT fall this fast on its own -- with the motors braking, gravity against
// back-EMF only drops it at ~15 deg/s -- so this is a driven descent, which is exactly why LIFT_KV
// exists (see it above). A full descent from the top level (128.3) is now ~1.5 s, not ~4 s.
//   Rams / reaches the slow zone still moving: lower it (and see LIFT_SLOW_* above).
//   Descent still feels sluggish: raise it toward 150, the arm's free speed. Above that is pointless;
//   the motors cannot turn faster, so the setpoint would just run away from the arm.
inline constexpr double LIFT_LOWER_SPEED_DEG_PER_S = 130.0;  // TUNE: was 40 (~27% of free speed)
// Descent ramp-up: the profiled trip to DOWN above doesn't start at full cruise (LIFT_LOWER_SPEED_DEG_PER_S)
// the instant it's commanded -- it starts at this slower "mid" speed and climbs LINEARLY up to cruise
// over the first LIFT_LOWER_RAMP_DEG of travel measured from wherever the move began (Lift::update(),
// the goal_ <= DOWN branch), so a score or a stow leaves the height smoothly instead of snapping
// straight to cruise. After the ramp it holds cruise until the slow zone near 0 (LIFT_SLOW_ZONE_DEG/
// LIFT_SLOW_SPEED_DPS above) takes over -- one continuous speed curve, mid -> high -> slow -> brake,
// with no stop or re-target in between (Scoring's pivot-retract floor on the walking setpoint, see
// Lift::setDescentFloor(), can briefly hold the LAST bit of that curve, never the ramp itself).
// The ramp is now only a few degrees long on purpose: its job is to take the jerk out of the first
// tick, not to be a slow phase of its own. 80 deg/s start over 5 deg of travel means cruise is
// reached in well under a tenth of a second.
//   Leaves the top too slowly / looks sluggish at the start: raise LIFT_LOWER_START_SPEED_DEG_PER_S
//   (up to the cruise speed, which removes the ramp entirely) or lower LIFT_LOWER_RAMP_DEG.
//   Snaps/jerks too hard at the start of a descent: raise LIFT_LOWER_RAMP_DEG (ramps over more
//   distance) or lower LIFT_LOWER_START_SPEED_DEG_PER_S.
inline constexpr double LIFT_LOWER_START_SPEED_DEG_PER_S = 80.0;  // TUNE: was 20. Starting ("mid") speed, deg/s
inline constexpr double LIFT_LOWER_RAMP_DEG = 5.0;                // TUNE: was 15. Deg of travel to reach cruise
// Runaway guard: output over FAULT_MV, pushing in the direction that SHOULD shrink the error, for
// FAULT_MS, and |error| ends up FAULT_DEG BIGGER than when that push started = sensor/motor
// direction mismatch -> cut power, "LIFT FAULT" on the brain, controller rumbles. It deliberately
// does NOT compare push direction against which way the arm is moving: braking a descending arm
// pushes up while the arm still moves down, which is correct behaviour, not a runaway.
// FAULT_DEG had to grow with the speeds. At the old 40 deg/s descent the loop only ever asked for
// ~2000 mV, below LIFT_FAULT_MV, so the guard was never even armed while lowering. A 130 deg/s
// descent is driven at ~9000-12000 mV, so it IS armed for the whole trip, and the few degrees of
// tracking error the arm legitimately builds while it accelerates would have tripped a 3 deg
// threshold. 10 deg is comfortably above any legitimate transient (LIFT_KV keeps steady-state
// tracking inside ~2 deg) and still far below what a real wiring reversal does: a runaway arm moves
// at 130+ deg/s in the wrong direction, so it blows past 10 deg of error growth in under 100 ms,
// well inside the 300 ms window.
//   LIFT FAULT on a normal fast move: raise LIFT_FAULT_DEG (and check LIFT_KV isn't too low).
inline constexpr float LIFT_FAULT_MV = 6000.0f;
inline constexpr int LIFT_FAULT_MS = 300;
inline constexpr double LIFT_FAULT_DEG = 10.0;               // TUNE: was 3, too tight for a 130 deg/s descent
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
