#include "robot.hpp"

#include "config.hpp"

namespace robot {

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ---- Drivetrain (notebook: 55W, 2.75" omnis, 450 RPM, traction wheels) ----
pros::MotorGroup leftDrive(config::LEFT_DRIVE_PORTS, pros::v5::MotorGears::blue);    // TODO(team): confirm cartridge
pros::MotorGroup rightDrive(config::RIGHT_DRIVE_PORTS, pros::v5::MotorGears::blue);  // TODO(team): confirm cartridge

static lemlib::Drivetrain drivetrain(&leftDrive, &rightDrive, config::TRACK_WIDTH_IN, config::DRIVE_WHEEL_DIAMETER_IN,
                                     config::DRIVE_RPM, config::HORIZONTAL_DRIFT);

// ---- Odometry (ONE vertical tracking-wheel pod on port 17, plus the IMU on 18) ----
pros::Rotation verticalEncoder(config::ODOM_VERTICAL_ROTATION_PORT);
static lemlib::TrackingWheel verticalWheel(&verticalEncoder, config::ODOM_WHEEL_DIAMETER_IN,
                                           config::ODOM_VERTICAL_OFFSET_IN);
// An IMU IS fitted (config::IMU_PORT = 18, confirmed 2026-10-02), so this builds one and LemLib gets
// heading from it and distance from the tracking wheel. Setting IMU_PORT back to 0 builds nothing and
// hands LemLib a null IMU pointer instead, which is its own documented way to run on the tracking
// wheel alone -- nothing else here needs to change. Calibration happens in a background task from
// initialize(); nothing in driver control ever waits on it (CLAUDE.md safety rule 4).
std::unique_ptr<pros::Imu> imu = config::IMU_PORT != 0 ? std::make_unique<pros::Imu>(config::IMU_PORT) : nullptr;
static lemlib::OdomSensors sensors(&verticalWheel, nullptr, nullptr, nullptr, imu.get());

// ---- Chassis PID ----
// Drive PID: tune these numbers right here.
static lemlib::ControllerSettings lateral(6,    // proportional gain (kP)
                                          0,    // integral gain (kI)
                                          15,   // derivative gain (kD)
                                          0,    // anti windup
                                          1,    // small error range, in inches
                                          150,  // small error range timeout, ms
                                          4,    // large error range, in inches
                                          500,  // large error range timeout, ms
                                          0     // maximum acceleration (slew)
);

static lemlib::ControllerSettings angular(2.55, // proportional gain (kP)
                                          0,    // integral gain (kI)
                                          15,   // derivative gain (kD)
                                          0,    // anti windup
                                          1,    // small error range, in degrees
                                          150,  // small error range timeout, ms
                                          7,    // large error range, in degrees
                                          600,  // large error range timeout, ms
                                          0     // maximum acceleration (slew)
);

// Driver curves: small stick movements stay gentle, full stick is full speed.
static lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.009);
static lemlib::ExpoDriveCurve steerCurve(5, 15, 1.023);

lemlib::Chassis chassis(drivetrain, lateral, angular, sensors, &throttleCurve, &steerCurve);

// ---- Distance sensors (brain DIST line; the rear one also drives auton::moveB) ----
// One each side and one looking backward. The REAR one is the whole feedback loop for moveB(): drive
// until it reads a target mm from whatever is behind the robot. The left and right ones are read by
// the brain screen only, for lining the robot up while tuning a route. Each constructed ONCE, here,
// like every other device. (The claw's own sensor belongs to subsystems::Claw.)
pros::Distance leftDistance(config::LEFT_DISTANCE_PORT);
pros::Distance rightDistance(config::RIGHT_DISTANCE_PORT);
pros::Distance backDistance(config::BACK_DISTANCE_PORT);

// ---- Mechanisms ----
subsystems::Lift lift;
subsystems::Claw claw;
subsystems::Intake intake;
subsystems::Scoring scoring(lift, claw);

}  // namespace robot
