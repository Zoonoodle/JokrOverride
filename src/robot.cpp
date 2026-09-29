#include "robot.hpp"

#include "config.hpp"

namespace robot {

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ---- Drivetrain (notebook: 55W, 2.75" omnis, 450 RPM, traction wheels) ----
static pros::MotorGroup leftDrive(config::LEFT_DRIVE_PORTS, pros::v5::MotorGears::blue);    // TODO(team): confirm cartridge
static pros::MotorGroup rightDrive(config::RIGHT_DRIVE_PORTS, pros::v5::MotorGears::blue);  // TODO(team): confirm cartridge

static lemlib::Drivetrain drivetrain(&leftDrive, &rightDrive, config::TRACK_WIDTH_IN, config::DRIVE_WHEEL_DIAMETER_IN,
                                     config::DRIVE_RPM, config::HORIZONTAL_DRIFT);

// ---- Odometry (notebook: ONE odometry pod + IMU) ----
static pros::Rotation verticalEncoder(config::ODOM_VERTICAL_ROTATION_PORT);
static lemlib::TrackingWheel verticalWheel(&verticalEncoder, config::ODOM_WHEEL_DIAMETER_IN,
                                           config::ODOM_VERTICAL_OFFSET_IN);
static pros::Imu imu(config::IMU_PORT);
static lemlib::OdomSensors sensors(&verticalWheel, nullptr, nullptr, nullptr, &imu);

// ---- Chassis PID (TUNE) ----
static lemlib::ControllerSettings lateral(config::LATERAL_KP, config::LATERAL_KI, config::LATERAL_KD,
                                          3,     // anti-windup range
                                          1,     // small error range (in)
                                          100,   // small error timeout (ms)
                                          3,     // large error range (in)
                                          500,   // large error timeout (ms)
                                          20);   // slew
static lemlib::ControllerSettings angular(config::ANGULAR_KP, config::ANGULAR_KI, config::ANGULAR_KD,
                                          3,     // anti-windup range
                                          1,     // small error range (deg)
                                          100,   // small error timeout (ms)
                                          3,     // large error range (deg)
                                          500,   // large error timeout (ms)
                                          0);    // slew

// Driver curves: small stick movements stay gentle, full stick is full speed.
static lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.009);
static lemlib::ExpoDriveCurve steerCurve(5, 15, 1.021);

lemlib::Chassis chassis(drivetrain, lateral, angular, sensors, &throttleCurve, &steerCurve);

// ---- Mechanisms ----
subsystems::Lift lift;
subsystems::Claw claw;
subsystems::Intake intake;
subsystems::Scoring scoring(lift, claw, intake);

}  // namespace robot
