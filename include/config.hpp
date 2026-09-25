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
inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS = {-1, -2, -3};   // TODO(team): confirm
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = {4, 5, 6};     // TODO(team): confirm

inline constexpr float DRIVE_WHEEL_DIAMETER_IN = 2.75f;      // notebook: 2.75" omni wheels
inline constexpr float DRIVE_RPM = 450.0f;                   // notebook: 450 RPM
inline constexpr float TRACK_WIDTH_IN = 11.5f;               // TODO(team): measure wheel-center to wheel-center
inline constexpr float HORIZONTAL_DRIFT = 2.0f;              // LemLib: 2 for a drive with traction wheels

// Odometry: one vertical tracking wheel pod + inertial sensor.
inline constexpr int8_t ODOM_VERTICAL_ROTATION_PORT = 7;     // TODO(team): confirm (negative to reverse)
inline constexpr float ODOM_WHEEL_DIAMETER_IN = 2.0f;        // TODO(team): confirm pod wheel (2" / 2.75")
inline constexpr float ODOM_VERTICAL_OFFSET_IN = 0.0f;       // TODO(team): measure left/right offset from center
inline constexpr int8_t IMU_PORT = 8;                        // TODO(team): confirm

// Chassis PID — TUNE. Starting values from the LemLib tutorial, not tuned on JOKR.
inline constexpr float LATERAL_KP = 10.0f, LATERAL_KI = 0.0f, LATERAL_KD = 3.0f;   // TUNE
inline constexpr float ANGULAR_KP = 2.0f, ANGULAR_KI = 0.0f, ANGULAR_KD = 10.0f;   // TUNE

// ---------------------------------------------------------------------------
// Lift (DR6-4B)
// ---------------------------------------------------------------------------
// Two motors on the high-strength drive axle. 12T motor gear -> 36T lift gear.
inline const std::vector<std::int8_t> LIFT_PORTS = {9, -10};             // TODO(team): confirm ports + which side is reversed
inline constexpr double LIFT_GEAR_RATIO = 36.0 / 12.0;       // notebook: 12T -> 36T = 3:1 torque
// Rotation sensor on the lift pivot. 0 = none fitted, use the motor encoders instead.
inline constexpr int8_t LIFT_ROTATION_PORT = 0;              // TODO(team): add a rotation sensor? (none in notebook)

// Preset heights, in DEGREES OF THE LIFT ARM (motor degrees / gear ratio). 0 = fully down.
// TODO(team): measure every preset on the robot (lower the lift, zero, raise to each spot, read the brain screen).
namespace lift_preset {
inline constexpr double DOWN = 0.0;                          // resting / start position
inline constexpr double INTAKE = 5.0;                        // TODO(team): claw level with intake handoff
inline constexpr double CARRY = 25.0;                        // TODO(team): holding a pin, clear of the field
inline constexpr double SCORE_LOW = 60.0;                    // TODO(team): low goal
inline constexpr double SCORE_HIGH = 95.0;                   // TODO(team): high goal
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
inline constexpr float LIFT_MANUAL_MV = 9000.0f;             // manual override power

// ---------------------------------------------------------------------------
// Claw (pneumatic) + autoclamp sensor
// ---------------------------------------------------------------------------
inline constexpr char CLAW_PISTON_PORT = 'A';                // TODO(team): confirm ADI port
inline constexpr bool CLAW_EXTENDED_IS_CLOSED = true;        // TODO(team): does extending the cylinder close the jaws?
// Autoclamp: a distance sensor looking between the jaws. None is in the notebook yet.
inline constexpr int8_t CLAW_DISTANCE_PORT = 11;             // TODO(team): mount + confirm port
inline constexpr int CLAW_DETECT_MM = 40;                    // TODO(team): distance that means "pin is between the jaws"
inline constexpr int CLAW_DETECT_DEBOUNCE_MS = 60;           // pin must be seen this long before clamping
inline constexpr int CLAW_RELEASE_LOCKOUT_MS = 750;          // after a release, don't re-clamp for this long

// ---------------------------------------------------------------------------
// Intake (three-stage rollers + bottom counter roller)
// ---------------------------------------------------------------------------
inline const std::vector<std::int8_t> INTAKE_PORTS = {12, -13};          // TODO(team): confirm motor count, ports and directions
inline constexpr float INTAKE_MV = 12000.0f;

// ---------------------------------------------------------------------------
// Pivot (intake -> scoring). See include/subsystems/scoring.hpp for the assumption.
// ---------------------------------------------------------------------------
// If the claw/wrist flips with its own cylinder, set the port. ' ' = no pivot actuator,
// the lift alone moves the claw from the intake handoff to the scoring height.
inline constexpr char PIVOT_PISTON_PORT = ' ';               // TODO(team): is there a separate pivot? which port?
inline constexpr int PIVOT_TIME_MS = 250;                    // TODO(team): time for the pivot to finish moving
inline constexpr int CLAW_OPEN_TIME_MS = 200;                // time for the jaws to open before lowering

}  // namespace config
