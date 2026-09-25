#pragma once
/** Global robot objects, created once in robot.cpp. */

#include "lemlib/api.hpp"
#include "pros/misc.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/lift.hpp"
#include "subsystems/scoring.hpp"

namespace robot {
extern pros::Controller master;
extern lemlib::Chassis chassis;
extern subsystems::Lift lift;
extern subsystems::Claw claw;
extern subsystems::Intake intake;
extern subsystems::Scoring scoring;
}  // namespace robot
