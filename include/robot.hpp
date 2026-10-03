#pragma once
/** Global robot objects, created once in robot.cpp. */

#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/lift.hpp"
#include "subsystems/scoring.hpp"

#include <memory>

namespace robot {
extern pros::Controller master;
extern lemlib::Chassis chassis;
// Exposed for the brain screen.
extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;
extern pros::Rotation verticalEncoder;
// The three field-facing distance sensors, all shown on the brain's DIST line. The rear one is also
// the feedback for auton::moveB(). (The claw's own sensor is inside subsystems::Claw.)
extern pros::Distance leftDistance;
extern pros::Distance rightDistance;
extern pros::Distance backDistance;
// Null when config::IMU_PORT == 0 (no IMU on the robot yet) -- always check before dereferencing.
extern std::unique_ptr<pros::Imu> imu;
extern subsystems::Lift lift;
extern subsystems::Claw claw;
extern subsystems::Intake intake;
extern subsystems::Scoring scoring;
}  // namespace robot
