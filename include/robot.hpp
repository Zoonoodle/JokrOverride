#pragma once
/** Global robot objects, created once in robot.cpp. */

#include "lemlib/api.hpp"
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
// Null when config::IMU_PORT == 0 (no IMU on the robot yet) -- always check before dereferencing.
extern std::unique_ptr<pros::Imu> imu;
extern subsystems::Lift lift;
extern subsystems::Claw claw;
extern subsystems::Intake intake;
extern subsystems::Scoring scoring;
}  // namespace robot
