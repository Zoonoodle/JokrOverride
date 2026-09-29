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

namespace robot {
extern pros::Controller master;
extern lemlib::Chassis chassis;
// Exposed for the brain screen.
extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;
extern pros::Rotation verticalEncoder;
extern pros::Imu imu;
extern subsystems::Lift lift;
extern subsystems::Claw claw;
extern subsystems::Intake intake;
extern subsystems::Scoring scoring;
}  // namespace robot
