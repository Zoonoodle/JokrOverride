#include "screen.hpp"

#include "config.hpp"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "auton.hpp"
#include "autons.hpp"
#include "robot.hpp"

#include <cstdio>
#include <cstdlib>

namespace screen {

RMode rMode = RMode::IDLE;
bool calibrated = false;

using namespace robot;

namespace {

/** How many `!!` warning lines may be drawn before the rest are dropped. A wall of them is no more
 *  useful than three, and the four real lines must never be pushed off. */
constexpr int kMaxWarnings = 3;

/** nullptr if the right device is on the port, otherwise a short "what's wrong" message. */
const char* portProblem(int port, pros::DeviceType expected) {
    if (port == 0) return "no port set";
    const pros::DeviceType found = pros::Device::get_plugged_type(static_cast<std::uint8_t>(std::abs(port)));
    if (found == expected) return nullptr;
    if (found == pros::DeviceType::none) return "not connected";
    return "WRONG DEVICE";
}

/** The word for the `R:` field -- which mode R1/R2 are in. */
const char* rModeName() {
    switch (rMode) {
        case RMode::IDLE: return "-";
        case RMode::INTAKE: return "INTAKE";
        case RMode::LIFT: return "LIFT";
        case RMode::INERT: return "wait";
    }
    return "?";
}

/** One distance sensor, as millimetres or `--`. `--` covers every unusable reading the same way: the
 *  sensor's own "nothing in range" (>= MOVEB_OUT_OF_RANGE_MM), a 0 (it cannot measure), and PROS_ERR
 *  from an unplugged port. The port itself gets a `!!` warning line, so this stays short. */
void formatMm(char* out, std::size_t n, int mm) {
    if (mm <= 0 || mm >= config::MOVEB_OUT_OF_RANGE_MM) std::snprintf(out, n, "--");
    else std::snprintf(out, n, "%d", mm);
}

void draw() {
    // --- 0: POSE. The odometry pose (tracking wheel for distance, IMU for heading, through LemLib).
    // While the IMU is still calibrating the numbers are meaningless, so say so instead of showing
    // them. Nothing waits on this in driver control (CLAUDE.md safety rule 4).
    if (!calibrated) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "POSE  calibrating...            ");
    } else {
        const lemlib::Pose pose = chassis.getPose();
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "POSE  X %.1f  Y %.1f  T %.1f      ", pose.x, pose.y,
                            pose.theta);
    }

    // --- 1: LIFT. Position -> target, which named preset that is, the phase (DRIVE / BRAKE /
    // APPROACH / HOLD / PARK / PARK(t/o) / GRAB / PRESS / REL(...) / PUSH / SCORED UP / DESCEND /
    // RETURN / FINAL / SEAT / REST), then anything wrong (FAULT / BLOCKED / SCORE STUCK), then the
    // homing state (HOMING / HOMED / HOME FAIL @ <angle>) and the R1/R2 mode. If a lift button seems
    // to do nothing, this line says why.
    {
        char status[20] = "";
        // SENSOR BAD comes FIRST: it is the one fault that makes every other number on this line
        // stale, so it must not be hidden behind a generic FAULT.
        if (lift.isSensorBad()) std::snprintf(status, sizeof(status), " SENSOR BAD");
        else if (lift.isFaulted()) std::snprintf(status, sizeof(status), " FAULT");
        else if (lift.isScoreStuck()) std::snprintf(status, sizeof(status), " SCORE STUCK");
        else if (lift.isBlocked()) std::snprintf(status, sizeof(status), " BLOCKED");


        char home[24] = "";
        switch (lift.getHomeState()) {
            case subsystems::Lift::Home::IDLE: break;
            case subsystems::Lift::Home::RUNNING:
            case subsystems::Lift::Home::DONE:
                std::snprintf(home, sizeof(home), " %s", lift.homeStateName());
                break;
            default:  // FAILED / CANCELLED / SKIPPED: the angle it stopped at is the whole diagnosis
                std::snprintf(home, sizeof(home), " %s @ %.1f", lift.homeStateName(), lift.getHomeAngle());
                break;
        }

        if (lift.hasActiveTarget()) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "LIFT  %.1f -> %.1f  %s  %s%s%s  R:%s     ",
                                lift.getPosition(), lift.getTarget(), lift.activePresetName(),
                                lift.phaseName(), status, home, rModeName());
        } else {
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "LIFT  %.1f -> none%s%s  R:%s          ",
                                lift.getPosition(), status, home, rModeName());
        }
    }

    // --- 2: CLAW. The jaws, what the autoclamp is doing (OFF / ARMED / SEEING / CLEAR? / HELD -- see
    // README "Autoclamp"), and which way the pivot is. The ADI port letters and raw levels that used to
    // be here did their job: both polarities are confirmed, so they are gone.
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "CLAW  %s  AC:%s  PIV %s          ", claw.stateName(),
                        claw.autoclampStateName(), scoring.isPivotOut() ? "OUT" : "BACK");

    // --- 3: DIST. The three field-facing distance sensors and the one in the claw, in millimetres.
    // BACK is the whole feedback loop for auton::moveB(), and `--` there means moveB would drive BLIND
    // at full commanded speed (see auton::moveB()), so it is worth a glance before trusting a route.
    {
        char l[12], r[12], b[12], c[12];
        formatMm(l, sizeof(l), leftDistance.get());
        formatMm(r, sizeof(r), rightDistance.get());
        formatMm(b, sizeof(b), backDistance.get());
        formatMm(c, sizeof(c), claw.getDistanceMm());  // the claw's own sensor lives inside Claw
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "DIST  L %s  R %s  BACK %s  CLAW %s mm     ", l, r, b, c);
    }

    // --- 4: START. Before the match (robot disabled) this is the LIVE alignment preview: what the
    // left sensor is averaging right now against the SELECTED route's expected reading, so the drive
    // team can see how the robot is placed BEFORE auton runs. Once auton has run startPose() it shows
    // what was actually applied, or says it was skipped (a reading it could not trust -- the nominal
    // pose was used instead). Blank when neither applies.
    // A route may align X off the left sensor, Y off the back sensor, both or neither. The BACK-Y
    // case gets the line to itself when it is the one in use, because that is the number the drive
    // team has to sanity-check against where the robot is actually standing.
    {
        const double expected = autons::expectedLeftMm();
        const double expectedBack = autons::expectedBackMm();
        if (auton::startPoseRan() && auton::startBackRequested()) {
            // Back-Y was asked for: report what it did with the reading.
            const double bmm = auton::startBackMm();
            if (auton::startBackApplied()) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "START BACK %.0f -> y %+.2f in          ",
                                    bmm, auton::startShiftYIn());
            } else {
                char back[12];
                formatMm(back, sizeof(back), static_cast<int>(bmm));
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "!! START BACK %s ignored, y 0          ",
                                    back);
            }
        } else if (auton::startPoseRan()) {
            const double mm = auton::startLeftMm();
            char left[12];
            formatMm(left, sizeof(left), static_cast<int>(mm));
            if (auton::startAligned()) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "START dx %+.1f in (L %s/%.0f)          ",
                                    auton::startShiftIn(), left, expected);
            } else {
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "!! START align skipped (L %s)          ", left);
            }
        } else if (expectedBack > 0.0) {
            // LIVE pre-match preview for a back-Y route: what Y this placement would start at, and
            // whether the reading is even inside the window that would let it be used.
            const double liveBack = auton::startBackPreview();
            if (liveBack > 0.0) {
                const bool usable = liveBack >= config::MATCHLOAD_BACK_MIN_MM &&
                                    liveBack <= config::MATCHLOAD_BACK_MAX_MM;
                if (usable) {
                    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "START BACK %.0f/%.0f  y %+.2f in     ",
                                        liveBack, expectedBack, (liveBack - expectedBack) / 25.4);
                } else {
                    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "!! BACK %.0f out of %.0f-%.0f        ",
                                        liveBack, config::MATCHLOAD_BACK_MIN_MM,
                                        config::MATCHLOAD_BACK_MAX_MM);
                }
            } else {
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "!! BACK no reading, y will be 0        ");
            }
        } else {
            const double live = auton::startAlignPreview();
            if (live > 0.0 && expected > 0.0) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "START L %.0f exp %.0f  dx %+.1f in     ", live,
                                    expected, (live - expected) / 25.4);
            } else if (live > 0.0) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "START L %.0f (route has no expected)   ", live);
            } else {
                pros::screen::print(pros::E_TEXT_MEDIUM, 4, "                                        ");
            }
        }
    }

    // --- Warnings, and ONLY when something is actually wrong: a missing device, or the wrong kind of
    // device on a port. One line each, newest ports last, capped at kMaxWarnings so the four lines
    // above can never be pushed off. A clean robot shows nothing here at all.
    int line = 5;
    const auto warn = [&line](const char* what, int port) {
        if (!what || line >= 5 + kMaxWarnings) return;
        pros::screen::print(pros::E_TEXT_MEDIUM, line++, "!! %s (port %d)            ", what, std::abs(port));
    };
    for (std::int8_t port : config::LEFT_DRIVE_PORTS) warn(portProblem(port, pros::DeviceType::motor), port);
    for (std::int8_t port : config::RIGHT_DRIVE_PORTS) warn(portProblem(port, pros::DeviceType::motor), port);
    for (std::int8_t port : config::LIFT_PORTS) warn(portProblem(port, pros::DeviceType::motor), port);
    warn(portProblem(config::INTAKE_PORT, pros::DeviceType::motor), config::INTAKE_PORT);
    warn(portProblem(config::LIFT_ROTATION_PORT, pros::DeviceType::rotation), config::LIFT_ROTATION_PORT);
    warn(portProblem(config::ODOM_VERTICAL_ROTATION_PORT, pros::DeviceType::rotation),
         config::ODOM_VERTICAL_ROTATION_PORT);
    if (imu) warn(portProblem(config::IMU_PORT, pros::DeviceType::imu), config::IMU_PORT);
    warn(portProblem(config::CLAW_DISTANCE_PORT, pros::DeviceType::distance), config::CLAW_DISTANCE_PORT);
    warn(portProblem(config::LEFT_DISTANCE_PORT, pros::DeviceType::distance), config::LEFT_DISTANCE_PORT);
    warn(portProblem(config::RIGHT_DISTANCE_PORT, pros::DeviceType::distance), config::RIGHT_DISTANCE_PORT);
    warn(portProblem(config::BACK_DISTANCE_PORT, pros::DeviceType::distance), config::BACK_DISTANCE_PORT);
    // Blank out whatever a previous, worse-off pass left behind.
    while (line < 5 + kMaxWarnings) {
        pros::screen::print(pros::E_TEXT_MEDIUM, line++, "                                        ");
    }

    // --- 8: THE LIFT'S ZERO. Did homing actually reach the hard stop and write the zero there, and
    // what does the arm read right now? This is the line to look at when "every height lands too low":
    // the frame is only trustworthy after HOMED. Anything else -- HOME FAIL (timed out before reaching
    // the stop), HOME CANCELLED (the D-pad interrupted it), HOME SKIPPED, or blank (never requested) --
    // means the readings are still in the RAW sensor frame and every preset is off by whatever the
    // bottom happens to read. `bottom` is the live position, so with a good home it should sit at
    // about lift_preset::DOWN (0.0) with the arm resting down.
    {
        const char* hs = lift.homeStateName();
        if (lift.isFrameZeroed()) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 8, "LIFT0 HOMED   pos %.1f  frame OK        ",
                                lift.getPosition());
        } else {
            pros::screen::print(pros::E_TEXT_MEDIUM, 8, "!! LIFT0 %-14s pos %.1f  frame RAW  ",
                                hs[0] ? hs : "NOT HOMED", lift.getPosition());
        }
    }

    // --- 10: SCORE. THE LAST THING THAT HAPPENED TO A SCORE, and where the arm was when it did:
    // PRESS, then whichever signal released the pin (REL(lag) / REL(spd) / REL(eff) / REL(stall), or
    // REL(32) = the L2_AUTO_RELEASE_DEG travel rule, REL(floor) = too low for that), then RUN DOWN
    // (the uncancellable, constant-voltage trip to 0 -- there is no FINAL on this path any more),
    // CLAMP ARMED (L2_AUTOCLAMP_LOCKOUT_MS is up, the autoclamp is live again and the arm is still on
    // its way down), then SEAT. If the arm ever stops above 0 after an L2 again, THIS
    // LINE SAYS WHY, with the height: RETARGET (something commanded a new height), CANCEL (the D-pad,
    // R2 or a route let go), REFUSE clamp (the autoclamp's 37.5 hop was blocked -- the arm kept
    // going), BLOCKED (physically caught, holding rather than grinding), FAULT down (the runaway
    // guard), SEAT(short) (FINAL's window ran out short of 0). The same lines go to the terminal as
    // `SCORE <ms> <what> pos <p> goal <g> vel <v>`, one per event. (Line 9 is MOVEB; 5-7 are warnings.)
    {
        const char* note = lift.lastNote();
        if (note[0]) pros::screen::print(pros::E_TEXT_MEDIUM, 10, "LIFT  %-30s", note);
    }
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
