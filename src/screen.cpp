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
RMode rMode = RMode::IDLE;
bool autoIntake = false;
bool autoOuttake = false;
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

/** The word for the brain's `R:` field -- which mode R1/R2 are in. */
const char* rModeName() {
    switch (rMode) {
        case RMode::IDLE: return "-";
        case RMode::INTAKE: return "INTAKE";
        case RMode::LIFT: return "LIFT";
        case RMode::INERT: return "wait";
    }
    return "?";
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

    // 1: inertial sensor. One IS fitted now (config::IMU_PORT = 18, confirmed 2026-10-02): heading
    // comes from it and distance from the tracking wheel. "none (not installed)" only appears if
    // IMU_PORT is set back to 0, which is the supported way to run on the pod alone -- it is not a
    // wiring problem. "calibrating... keep still" is normal for the first couple of seconds after
    // power-on; nothing in driver control waits for it (CLAUDE.md safety rule 4).
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

    // 5: CLAW is the claw's own state (OPEN/ARMED/CLAMPED/RELEASED) and the pivot (OUT/BACK), each
    // with its ADI PORT LETTER and the raw level it is driven to, so config::CLAW_CLOSED_IS_ADI_HIGH
    // and PIVOT_OUT_IS_ADI_HIGH can each be confirmed in seconds with A / X: if the word says one
    // thing and the mechanism did the other, flip that one constant. The port letters are on the line
    // because the two cylinders were wired to each other's ports in code until 2026-10-01, and nothing
    // on the screen would have shown it. Then the intake macro's state and stack level, and which mode
    // R1/R2 are in (see screen::RMode). Without this line there was no way to see that the jaws were
    // still CLAMPED or that the macro was parked in STACKING -- and both silently change what R1 and
    // L2 do. If a button seems to do nothing, read this line first.
    // The AC: field is the AUTOCLAMP: OFF (disabled, or the jaws are shut by hand), ARMED (watching,
    // nothing seen), SEEING (something in range -- the AUTOCLAMP_CONFIRM_MS window is running),
    // CLEAR? (disarmed after an open, waiting for a clear reading before it may re-arm, see
    // AUTOCLAMP_REARM_MM) or HELD. Tune the range and the debounce against this plus the DIST mm on
    // line 8: SEEING that never becomes a clamp means the reading is not staying inside the range for
    // long enough. See README "Autoclamp".
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "CLAW %s %c[%d] AC:%s  PIV %s %c[%d]  %s L%d  R:%s",
                        claw.stateName(), config::CLAW_PISTON_PORT, claw.adiHigh() ? 1 : 0,
                        claw.autoclampStateName(), scoring.isPivotOut() ? "OUT" : "BACK",
                        config::PIVOT_PISTON_PORT, scoring.pivotAdiHigh() ? 1 : 0, scoring.stateName(),
                        scoring.getLevel(), rModeName());

    // --- Lift section (lines 6-10): everything needed to judge a move at a glance. ---

    // 6: position -> goal, which named preset (if any) is active, which phase the move is in
    // (DRIVE / BRAKE / APPROACH / HOLD / DESCEND / RETURN / FINAL / SEAT / SEAT(short) / PRESS / REL /
    // UNSTK), and SCORE STUCK / FAULT / BLOCKED / SETTLED. SCORE STUCK is shown ahead of
    // BLOCKED/SETTLED because it is the more specific answer to "why did that not do what I expected".
    // There is no NO CONTACT any more: L2_AUTO_RELEASE_DEG guarantees a release by then, so a score
    // always ends with the pin off (see REL(32) on the RELEASE phase and CLAUDE.md's L2 note).
    //   LIFT 61.4 -> 128.3  STACK 5  DRIVE
    // means "goal is level 5, the arm is at 61.4 and still under full voltage".
    {
        char status[14] = "";
        if (lift.isFaulted()) std::snprintf(status, sizeof(status), "FAULT");
        else if (lift.isScoreStuck()) std::snprintf(status, sizeof(status), "SCORE STUCK");
        else if (lift.isBlocked()) std::snprintf(status, sizeof(status), "BLOCKED");
        else if (lift.isSettled()) std::snprintf(status, sizeof(status), "SETTLED");
        if (lift.hasActiveTarget())
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "LIFT %.1f -> %.1f  %s  %s %s", lift.getPosition(),
                                lift.getTarget(), lift.activePresetName(), lift.phaseName(), status);
        else
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "LIFT %.1f -> none  %s", lift.getPosition(), status);
    }

    // 7: error, measured arm speed, and what the loop sent to the motors (mV, 0 = brake-hold).
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "LIFT err %.1f  vel %.0f/s  out %.0f mV", lift.getError(),
                        lift.getVelocity(), lift.getOutput());

    // 8: the speed reference the velocity loop is tracking this tick (0 in HOLD/SEAT), then the hold
    // loop's learned gravity voltage OR -- while an L2 score is running -- ALL THREE things the contact
    // detector is watching: `lag` the accumulated position shortfall in arm degrees (fires at
    // L2_CONTACT_LAG_DEG), `eff` the extra downward push over the free-descent baseline in mV (fires at
    // L2_CONTACT_EFFORT_MV), and `v/b` the measured/baseline speed ratio as a percentage (fires under
    // L2_CONTACT_SPEED_FRAC). Watching these three while scoring is how you tell which signal is
    // actually moving when the pin lands -- and the phase name on line 6 says which one fired.
    {
        char dist[26] = "";
        if (const char* p = portProblem(config::CLAW_DISTANCE_PORT, pros::DeviceType::distance)) {
            std::snprintf(dist, sizeof(dist), "%s p%d", p, config::CLAW_DISTANCE_PORT);
        } else {
            const int mm = claw.getDistanceMm();
            if (mm <= 0 || mm >= 9999) std::snprintf(dist, sizeof(dist), "--");
            else std::snprintf(dist, sizeof(dist), "%dmm%s", mm, mm <= config::AUTOCLAMP_RANGE_MM ? " PIN" : "");
        }
        if (lift.isScoring())
            pros::screen::print(pros::E_TEXT_MEDIUM, 8, "L2 lag %.2f eff %.0f v/b %.0f%%   DIST %s",
                                lift.getContactLagDeg(), lift.getContactEffortMv(),
                                lift.getSpeedRatio() * 100.0, dist);
        else
            pros::screen::print(pros::E_TEXT_MEDIUM, 8, "LIFT ref %.0f/s  grav %.0f mV   DIST %s",
                                lift.getSpeedRef(), lift.getGravityMv(), dist);
    }

    // 9: the headline motion constants, read straight from config.hpp's "LIFT TUNING -- start here"
    // block -- tuning happens in code, not here. This line is just so you can see which build is on
    // the brain while you iterate.
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "TUNE decel %.0f  appr %.0f/%.0f  holdD %.0f  tol %.0f",
                        config::LIFT_DECEL_DEG_PER_S2, config::LIFT_APPROACH_ZONE_DEG,
                        config::LIFT_APPROACH_DEG_PER_S, static_cast<double>(config::LIFT_HOLD_KD),
                        config::LIFT_TOLERANCE_DEG);

    // 9b: the rear distance sensor -- the entire feedback loop for auton::moveB(). Watch this before
    // trusting a route to it: "--" means nothing in range, and moveB drives BLIND at full commanded
    // speed while that is true (kept from the old library, see auton::moveB()).
    {
        char back[28] = "";
        if (const char* p = portProblem(config::BACK_DISTANCE_PORT, pros::DeviceType::distance)) {
            std::snprintf(back, sizeof(back), "%s p%d", p, config::BACK_DISTANCE_PORT);
        } else {
            const int mm = backDistance.get();
            if (mm <= 0 || mm >= config::MOVEB_OUT_OF_RANGE_MM) std::snprintf(back, sizeof(back), "--");
            else std::snprintf(back, sizeof(back), "%dmm", mm);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 13, "BACK  %s (moveB, port %d)", back,
                            config::BACK_DISTANCE_PORT);
    }

    // 10: lift motor health -- temp (both readouts lose power around 55 C) and current draw.
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, "LIFT temp %.0fC   current %.0f mA", lift.getTemperature(),
                        lift.getCurrentDraw());

    // 11: overall motor temps (C), all mechanisms.
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, "TEMP  L %.0f  R %.0f  lift %.0f  intake %.0f", hottest(leftDrive),
                        hottest(rightDrive), lift.getTemperature(), intake.getTemperature());

    // 12: driver loop heartbeat, plus INT:AUTO while L1's auto-intake is sweeping during a rise off
    // the floor, or INT:OUT while an L2 score descent is auto-outtaking near the bottom (the two never
    // overlap -- one needs the claw empty and rising, the other a pin held and descending). "not
    // started" = opcontrol never ran (not enabled / stuck before it).
    if (driverDebug.loops == 0)
        pros::screen::print(pros::E_TEXT_MEDIUM, 12, "DRV   not started %s", calibrated ? "" : "(calibrating)");
    else
        pros::screen::print(pros::E_TEXT_MEDIUM, 12, "DRV #%u fwd %d trn %d Up %d Dn %d%s",
                            static_cast<unsigned>(driverDebug.loops), driverDebug.forward, driverDebug.turn,
                            driverDebug.up, driverDebug.down, autoOuttake ? "  INT:OUT" : autoIntake ? "  INT:AUTO" : "");
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
