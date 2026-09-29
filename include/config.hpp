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
 *   - Lift: DR6-4B, 2 motors on a high-strength axle, 12T -> 36T (Build | Lift Drive, 9/10/26)
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

// Odometry: one vertical tracking wheel pod + inertial sensor.
inline constexpr int8_t ODOM_VERTICAL_ROTATION_PORT = 7;     // TODO(team): confirm (negative to reverse)
inline constexpr float ODOM_WHEEL_DIAMETER_IN = 2.0f;        // TODO(team): confirm pod wheel (2" / 2.75")
inline constexpr float ODOM_VERTICAL_OFFSET_IN = 0.0f;       // TODO(team): measure left/right offset from center
// WARNING: port 8 is also in RIGHT_DRIVE_PORTS. One device per port - one of these is wrong.
inline constexpr int8_t IMU_PORT = 8;                        // TODO(team): FIX - conflicts with right drive port 8

// Chassis PID — TUNE. Starting values from the LemLib tutorial, not tuned on JOKR.
inline constexpr float LATERAL_KP = 10.0f, LATERAL_KI = 0.0f, LATERAL_KD = 3.0f;   // TUNE
inline constexpr float ANGULAR_KP = 2.0f, ANGULAR_KI = 0.0f, ANGULAR_KD = 10.0f;   // TUNE

// ---------------------------------------------------------------------------
// Lift (DR6-4B)
// ---------------------------------------------------------------------------
// Two motors on the high-strength drive axle. 12T motor gear -> 36T lift gear.
inline const std::vector<std::int8_t> LIFT_PORTS = {20, -13};             // TODO(team): confirm ports + which side is reversed
inline constexpr double LIFT_GEAR_RATIO = 36.0 / 12.0;       // notebook: 12T -> 36T = 3:1 torque
// Rotation sensor on the lift. 0 = none fitted, use the motor encoders instead.
// When a port is set, the lift PID runs off THIS sensor, not the motor encoders.
inline constexpr int8_t LIFT_ROTATION_PORT = 16;
// DIRECTION CHECK (do this before anything else, see README "Lift direction check"):
// raise the lift by hand with the robot disabled and watch "rot" on the brain screen.
// It must go UP (positive). If it goes negative, flip this to true.
inline constexpr bool LIFT_ROTATION_REVERSED = false;        // TODO(team): check on the robot
// Sensor degrees per lift-arm degree. 1.0 if the sensor is on the 36T lift axle,
// 3.0 if it is on the 12T motor axle. The brain shows "rot" and "mot" side by side:
// raise the lift and they should read about the same. If rot reads ~3x mot, set 3.0.
inline constexpr double LIFT_ROTATION_RATIO = 1.0;           // TODO(team): check on the robot

// Preset heights, in DEGREES OF THE LIFT ARM. 0 = fully down (where the robot is turned on).
// How to measure: see README "Tuning the claw positions".
namespace lift_preset {
inline constexpr double DOWN = 0.0;                          // resting / start position
inline constexpr double INTAKE = 5.0;                        // TUNE: claw lined up with the intake handoff (R1)
inline constexpr double MATCHLOAD = 15.0;                    // TUNE: claw at matchload height (L2 with no cup)
inline constexpr double PIVOT_CLEAR = 25.0;                  // TUNE: lowest height the pivot can flip out without hitting anything
// L1 taps: tap 1 -> level 1, tap 2 -> level 2 ... tap 7 -> level 7. One entry per pin-stack height.
inline constexpr int STACK_LEVEL_COUNT = 7;
inline constexpr double STACK_LEVELS[STACK_LEVEL_COUNT] = {
    20.0,  // TUNE: level 1 (scoring the 1st cup)
    30.0,  // TUNE: level 2
    40.0,  // TUNE: level 3
    50.0,  // TUNE: level 4
    60.0,  // TUNE: level 5
    72.0,  // TUNE: level 6
    85.0,  // TUNE: level 7
};
inline constexpr double MAX = 105.0;                         // TODO(team): hard stop, never command past this
}  // namespace lift_preset

// Lift PID — TUNE. Units: error in lift degrees, output in millivolts (-12000..12000).
inline constexpr float LIFT_KP = 300.0f;                     // TUNE
inline constexpr float LIFT_KI = 0.0f;                       // TUNE (add last, only if it stalls short)
inline constexpr float LIFT_KD = 800.0f;                     // TUNE
inline constexpr float LIFT_WINDUP_RANGE = 5.0f;             // integral only builds within 5 deg of target
inline constexpr float LIFT_KG = 1200.0f;                    // TUNE: mV that holds the lift still (gravity feedforward)
inline constexpr float LIFT_MAX_MV = 12000.0f;               // output limit
inline constexpr float LIFT_SETTLE_DEG = 1.5f;               // "at target" band
inline constexpr int LIFT_SETTLE_MS = 150;                   // must stay in band this long
inline constexpr float LIFT_MANUAL_UP_MV = 12000.0f;         // Up arrow: 100% power (12000 mV is the V5 max)
inline constexpr float LIFT_MANUAL_DOWN_MV = 9000.0f;        // Down arrow power (gravity adds to it)
// Heading to the bottom (target 0): within this many degrees of 0 counts as "resting", motors just hold.
// Above it the lift is eased down with at least LIFT_DOWN_MV (never pushed up).
inline constexpr double LIFT_REST_BAND_DEG = 3.0;
// Stall guard: output over STALL_MV but moved less than STALL_DEG in STALL_MS = can't get there
// (target above the real top, or jammed). The lift holds where it is and the brain shows BLOCKED.
inline constexpr float LIFT_STALL_MV = 8000.0f;
inline constexpr double LIFT_STALL_DEG = 0.5;
inline constexpr int LIFT_STALL_MS = 700;
inline constexpr float LIFT_DOWN_MV = 2000.0f;
// After L2 scores, the arm lowers back to INTAKE at this speed (lift degrees per second). Lower = gentler.
inline constexpr double LIFT_LOWER_SPEED_DEG_PER_S = 40.0;   // TUNE
// Runaway guard: output over FAULT_MV for FAULT_MS while the lift moves FAULT_DEG the WRONG way
// = sensor/motor direction mismatch -> cut power, "LIFT FAULT" on the brain, controller rumbles.
inline constexpr float LIFT_FAULT_MV = 6000.0f;
inline constexpr int LIFT_FAULT_MS = 300;
inline constexpr double LIFT_FAULT_DEG = 3.0;

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
inline constexpr int CLAW_OPEN_TIME_MS = 200;                // TUNE: jaws open before the arm lowers (L2)

}  // namespace config
