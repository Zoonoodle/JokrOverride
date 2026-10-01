#include "screen.hpp"

#include "config.hpp"
#include "pros/device.hpp"
#include "pros/screen.hpp"
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

// This is a plain-text readout drawn with pros::screen (not the 8-line pros::lcd console) so the
// lift tuning section below has room without pushing the existing sensor lines off. Lines are
// pixel rows under the hood, not a fixed console -- there's nothing special about there being more
// than 8 of them.
void draw() {
    // 0: odometry pose (tracking wheel + IMU through LemLib)
    const lemlib::Pose pose = chassis.getPose();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "POSE  X %.1f  Y %.1f  T %.1f", pose.x, pose.y, pose.theta);

    // 1: inertial sensor -- not installed yet (config::IMU_PORT == 0), show that plainly instead of
    // treating it as a wiring problem. Odometry runs on the tracking wheel alone until one is added.
    if (!imu)
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "IMU   none (not installed)");
    else if (const char* p = portProblem(config::IMU_PORT, pros::DeviceType::imu))
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "IMU   %s (port %d)", p, config::IMU_PORT);
    else if (imu->is_calibrating())
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "IMU   calibrating... keep still");
    else
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "IMU   heading %.1f  rotation %.1f", imu->get_heading(),
                            imu->get_rotation());

    // 2: odometry tracking wheel
    if (const char* p = portProblem(config::ODOM_VERTICAL_ROTATION_PORT, pros::DeviceType::rotation))
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "ODOM  %s (port %d)", p, std::abs(config::ODOM_VERTICAL_ROTATION_PORT));
    else
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "ODOM  wheel %.1f deg", verticalEncoder.get_position() / 100.0);

    // 3: lift rotation sensor vs motor encoders, both converted to ARM degrees (the unit everything
    // else -- presets, PID target/error, the LIFT lines below -- uses), with the raw sensor/motor
    // degrees in brackets so the 4:1 (config::LIFT_ROTATION_RATIO / LIFT_GEAR_RATIO) can be checked
    // by hand. Direction check: both must go + when the lift goes up (see README).
    if (const char* p = portProblem(config::LIFT_ROTATION_PORT, pros::DeviceType::rotation))
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "ROT   %s (port %d)  mot %.1f", p,
                            std::abs(config::LIFT_ROTATION_PORT), lift.getMotorPosition());
    else
        // arm degrees, with the raw sensor/motor degrees in brackets (raw = before the 4:1)
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "ROT %.1f [%.0f]  MOT %.1f [%.0f]", lift.getSensorPosition(),
                            lift.getSensorPosition() * config::LIFT_ROTATION_RATIO, lift.getMotorPosition(),
                            lift.getMotorPosition() * config::LIFT_GEAR_RATIO);

    // 4: the rotation sensor's own absolute zero (calibrated on the brain's device screen, outside
    // this code -- see Lift::zero()). ANGLE is the LIVE raw get_angle(), in sensor degrees, so the
    // calibrated zero can be checked by eye any time (should read ~0 with the arm at the true
    // bottom). SEED is the one-time signed angle zero() seeded the position counter from at startup.
    // HOME is appended: HOMING while it is driving down to the hard stop, HOMED once it zeroed the
    // sensor there, HOME FAIL if it stopped somewhere implausible or timed out (the seed is kept and
    // nothing was zeroed), HOME CANCELLED if the driver took manual control, HOME SKIPPED if no
    // rotation sensor answered. "@ N" is the absolute angle it saw at the stop -- that is the number
    // the plausibility window (LIFT_HOME_ZERO_WINDOW_DEG) was checked against, so a FAIL can be
    // diagnosed by eye.
    {
        char home[28] = "";
        if (lift.getHomeState() == subsystems::Lift::Home::RUNNING)
            std::snprintf(home, sizeof(home), "  %s", lift.homeStateName());
        else if (lift.getHomeState() != subsystems::Lift::Home::IDLE)
            std::snprintf(home, sizeof(home), "  %s @ %.1f", lift.homeStateName(), lift.getHomeAngle());
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "ANGLE raw %.1f  seed %.1f%s", lift.getRawAngle(),
                            lift.getStartupAngle(), home);
    }

    // 5: CLAW is the claw's own state (OPEN/ARMED/CLAMPED/RELEASED) and the intake macro's state and
    // stack level, then the distance sensor. Without this line there was no way to see that the jaws
    // were still CLAMPED, or that the macro was parked in STACKING -- and both of those silently
    // change what R1 and L2 do (interlock 4 makes R1 a no-op while a pin is held, and a claw left
    // clamped by B or the manual override is what made L2 fire the jaws at the bottom). If a button
    // seems to do nothing, read this line first.
    {
        char dist[26] = "";
        if (const char* p = portProblem(config::CLAW_DISTANCE_PORT, pros::DeviceType::distance)) {
            std::snprintf(dist, sizeof(dist), "%s p%d", p, config::CLAW_DISTANCE_PORT);
        } else {
            const int mm = claw.getDistanceMm();
            if (mm <= 0 || mm >= 9999) std::snprintf(dist, sizeof(dist), "--");
            else std::snprintf(dist, sizeof(dist), "%dmm%s", mm, mm <= config::CLAW_DETECT_MM ? " PIN" : "");
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "CLAW %s  MACRO %s L%d  DIST %s", claw.stateName(),
                            scoring.stateName(), scoring.getLevel(), dist);
    }

    // --- Lift tuning section (lines 6-10): everything needed to tune the PID at a glance. ---

    // 6: position -> goal, which named preset (if any) is active, and settled/blocked/fault status.
    // During a speed-limited move (every descent to DOWN is one) the PID is not chasing the goal yet
    // but a setpoint walking toward it, so that gets its own "(sp N)" so the two can't be mixed up:
    //   LIFT 61.4 -> 0.0 (sp 58.2)  DOWN
    // means "goal is 0, the loop is currently asking for 58.2, the arm is at 61.4 (3.2 deg behind)".
    // Once the setpoint lands on the goal the "(sp ...)" disappears and the error is the real error.
    {
        char status[12] = "";
        if (lift.isFaulted()) std::snprintf(status, sizeof(status), "FAULT");
        else if (lift.isBlocked()) std::snprintf(status, sizeof(status), "BLOCKED");
        else if (lift.isSettled()) std::snprintf(status, sizeof(status), "SETTLED");
        char setpoint[16] = "";
        if (lift.isProfiling()) std::snprintf(setpoint, sizeof(setpoint), "(sp %.1f) ", lift.getSetpoint());
        if (lift.hasActiveTarget())
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "LIFT %.1f -> %.1f %s %s %s", lift.getPosition(),
                                lift.getTarget(), setpoint, lift.activePresetName(), status);
        else
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "LIFT %.1f -> none  %s", lift.getPosition(), status);
    }

    // 7: error and total PID output (mV), so you can see the loop working in real time.
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "LIFT err %.1f deg   out %.0f mV", lift.getError(), lift.getOutput());

    // 8: P / I / D terms separately (mV each) -- watch these while tuning kP/kD/kI in that order.
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "LIFT P %.0f   I %.0f   D %.0f", lift.getPTerm(), lift.getITerm(),
                        lift.getDTerm());

    // 9: the gains themselves, read straight from config.hpp -- tuning happens in code, not here.
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "GAIN kP %.0f  kI %.0f  kD %.0f  kG %.0f", config::LIFT_KP,
                        config::LIFT_KI, config::LIFT_KD, config::LIFT_KG);

    // 10: lift motor health -- temp (both readouts lose power around 55 C) and current draw.
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, "LIFT temp %.0fC   current %.0f mA", lift.getTemperature(),
                        lift.getCurrentDraw());

    // 11: overall motor temps (C), all mechanisms.
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, "TEMP  L %.0f  R %.0f  lift %.0f  intake %.0f", hottest(leftDrive),
                        hottest(rightDrive), lift.getTemperature(), intake.getTemperature());

    // 12: driver loop heartbeat. "not started" = opcontrol never ran (not enabled / stuck before it).
    if (driverDebug.loops == 0)
        pros::screen::print(pros::E_TEXT_MEDIUM, 12, "DRV   not started %s", calibrated ? "" : "(calibrating)");
    else
        pros::screen::print(pros::E_TEXT_MEDIUM, 12, "DRV #%u fwd %d trn %d Up %d Dn %d",
                            static_cast<unsigned>(driverDebug.loops), driverDebug.forward, driverDebug.turn,
                            driverDebug.up, driverDebug.down);
}

}  // namespace

void start() {
    static pros::Task task([] {
        while (true) {
            draw();
            pros::delay(50);
        }
    });
}

}  // namespace screen
