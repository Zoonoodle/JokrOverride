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
// The numbers live in config.hpp (LATERAL_* / ANGULAR_*), not here: these are the values tuned on the
// robot 2026-10-02, and every argument is named there -- the error ranges and timeouts were tuned too,
// not just the gains. Changing how the drive behaves means editing config.hpp and nothing else.
static lemlib::ControllerSettings lateral(config::LATERAL_KP, config::LATERAL_KI, config::LATERAL_KD,
                                          config::LATERAL_WINDUP,
                                          config::LATERAL_SMALL_ERROR_IN,
                                          config::LATERAL_SMALL_TIMEOUT_MS,
                                          config::LATERAL_LARGE_ERROR_IN,
                                          config::LATERAL_LARGE_TIMEOUT_MS,
                                          config::LATERAL_SLEW);
static lemlib::ControllerSettings angular(config::ANGULAR_KP, config::ANGULAR_KI, config::ANGULAR_KD,
                                          config::ANGULAR_WINDUP,
                                          config::ANGULAR_SMALL_ERROR_DEG,
                                          config::ANGULAR_SMALL_TIMEOUT_MS,
                                          config::ANGULAR_LARGE_ERROR_DEG,
                                          config::ANGULAR_LARGE_TIMEOUT_MS,
                                          config::ANGULAR_SLEW);

// Driver curves: small stick movements stay gentle, full stick is full speed.
static lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.009);
static lemlib::ExpoDriveCurve steerCurve(5, 15, 1.021);

lemlib::Chassis chassis(drivetrain, lateral, angular, sensors, &throttleCurve, &steerCurve);

// ---- Rear distance sensor (auton::moveB) ----
// Looks backward, and is the whole feedback loop for moveB(): drive until it reads a target mm from
// whatever is behind the robot. Constructed ONCE, here, like every other device.
pros::Distance backDistance(config::BACK_DISTANCE_PORT);

// ---- Mechanisms ----
subsystems::Lift lift;
subsystems::Claw claw;
subsystems::Intake intake;
subsystems::Scoring scoring(lift, claw);

}  // namespace robot
