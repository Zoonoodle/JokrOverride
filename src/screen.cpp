#include "screen.hpp"

#include "config.hpp"
#include "pros/device.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "robot.hpp"

#include <algorithm>
#include <cstdlib>

namespace screen {

DriverDebug driverDebug;
bool calibrated = false;

using namespace robot;

namespace {

/** nullptr if the right device is on the port, otherwise a short "what's wrong" message. */
const char* portProblem(int port, pros::DeviceType expected) {
    if (port == 0) return "no port set";
    const pros::DeviceType found = pros::Device::get_plugged_type(static_cast<std::uint8_t>(std::abs(port)));
    if (found == expected) return nullptr;
    if (found == pros::DeviceType::none) return "not connected";
    return "WRONG DEVICE on port";
}

double hottest(const pros::MotorGroup& group) {
    double t = 0.0;
    for (double v : group.get_temperature_all()) t = std::max(t, v);
    return t;
}

void draw() {
    // 0: odometry pose (tracking wheel + IMU through LemLib)
    const lemlib::Pose pose = chassis.getPose();
    pros::lcd::print(0, "POSE  X %.1f  Y %.1f  T %.1f", pose.x, pose.y, pose.theta);

    // 1: inertial sensor
    if (const char* p = portProblem(config::IMU_PORT, pros::DeviceType::imu))
        pros::lcd::print(1, "IMU   %s (port %d)", p, config::IMU_PORT);
    else if (imu.is_calibrating())
        pros::lcd::print(1, "IMU   calibrating... keep still");
    else
        pros::lcd::print(1, "IMU   heading %.1f  rotation %.1f", imu.get_heading(), imu.get_rotation());

    // 2: odometry tracking wheel
    if (const char* p = portProblem(config::ODOM_VERTICAL_ROTATION_PORT, pros::DeviceType::rotation))
        pros::lcd::print(2, "ODOM  %s (port %d)", p, std::abs(config::ODOM_VERTICAL_ROTATION_PORT));
    else
        pros::lcd::print(2, "ODOM  wheel %.1f deg", verticalEncoder.get_position() / 100.0);

    // 3: lift position / target / status
    pros::lcd::print(3, "LIFT  %.1f -> %.1f deg  %s", lift.getPosition(), lift.getTarget(),
                     lift.isFaulted() ? "LIFT FAULT" : lift.isBlocked() ? "BLOCKED" : lift.isSettled() ? "SETTLED" : "");

    // 4: lift rotation sensor vs motor encoders (direction check: both must go + when the lift goes up)
    if (const char* p = portProblem(config::LIFT_ROTATION_PORT, pros::DeviceType::rotation))
        pros::lcd::print(4, "ROT   %s (port %d)  mot %.1f", p, std::abs(config::LIFT_ROTATION_PORT),
                         lift.getMotorPosition());
    else
        // lift degrees, with the raw sensor/motor degrees in brackets (raw = before any gear ratio)
        pros::lcd::print(4, "ROT %.1f [%.0f]  MOT %.1f [%.0f]", lift.getSensorPosition(),
                         lift.getSensorPosition() * config::LIFT_ROTATION_RATIO, lift.getMotorPosition(),
                         lift.getMotorPosition() * config::LIFT_GEAR_RATIO);

    // 5: claw distance sensor
    if (const char* p = portProblem(config::CLAW_DISTANCE_PORT, pros::DeviceType::distance)) {
        pros::lcd::print(5, "DIST  %s (port %d)", p, config::CLAW_DISTANCE_PORT);
    } else {
        const int mm = claw.getDistanceMm();
        if (mm <= 0 || mm >= 9999) pros::lcd::print(5, "DIST  nothing in range");
        else
            pros::lcd::print(5, "DIST  %d mm  conf %d  %s", mm, claw.getDistanceConfidence(),
                             mm <= config::CLAW_DETECT_MM ? "PIN" : "");
    }

    // 6: driver loop heartbeat. "not started" = opcontrol never ran (not enabled / stuck before it).
    if (driverDebug.loops == 0)
        pros::lcd::print(6, "DRV   not started %s", calibrated ? "" : "(calibrating)");
    else
        pros::lcd::print(6, "DRV #%u fwd %d trn %d L1 %d L2 %d", static_cast<unsigned>(driverDebug.loops),
                         driverDebug.forward, driverDebug.turn, driverDebug.l1, driverDebug.l2);

    // 7: motor temps (C). Motors start losing power around 55 C.
    pros::lcd::print(7, "TEMP  L %.0f  R %.0f  lift %.0f  intake %.0f", hottest(leftDrive), hottest(rightDrive),
                     lift.getTemperature(), intake.getTemperature());
}

}  // namespace

void start() {
    pros::lcd::initialize();
    static pros::Task task([] {
        while (true) {
            draw();
            pros::delay(50);
        }
    });
}

}  // namespace screen
