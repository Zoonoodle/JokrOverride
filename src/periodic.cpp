#include "periodic.hpp"

#include "config.hpp"
#include "robot.hpp"
#include "screen.hpp"

#include <cstdint>
#include <mutex>

namespace periodic {

using subsystems::Intake;
using subsystems::Scoring;

namespace {
// L1's auto-intake window: the millisecond the rise began, or 0 when it isn't running.
std::uint32_t autoIntakeStart = 0;
// L2's auto-outtake window: the millisecond the rollers started, or 0 when it isn't running, plus a
// latch so L2_OUTTAKE_MAX_MS's timeout can't just re-arm itself on the very next tick -- it stays off
// until the score descent ends and a fresh one starts.
std::uint32_t autoOuttakeStart = 0;
bool autoOuttakeExpired = false;

pros::Mutex gate;
}  // namespace

pros::Mutex& mutex() { return gate; }

void reset() {
    std::lock_guard<pros::Mutex> guard(gate);
    autoIntakeStart = 0;
    autoOuttakeStart = 0;
    autoOuttakeExpired = false;
    screen::autoIntake = false;
    screen::autoOuttake = false;
}

void tick(const Inputs& in) {
    std::lock_guard<pros::Mutex> guard(gate);

    const double liftPos = robot::lift.getPosition();

    // --- L2 auto-outtake: while an L2 SCORE descent (not the at-the-bottom clamp toggle, not
    // scoreOrMatchload()'s INTAKE lower) is under L2_OUTTAKE_BELOW_DEG, eject through the rest of the
    // trip down -- PRESS, REL, UNSTK, RETURN -- so anything still in the intake doesn't ride along.
    // Stack level 1 (31.0 deg) is already under 40, so scoring from level 1 outtakes from the start of
    // the press; that is this constant as specified, not a bug (see config.hpp). Stops as soon as the
    // lift reaches L2_OUTTAKE_STOP_DEG (tied directly to LIFT_FINAL_START_DEG) -- the lift's own FINAL
    // phase takes real downward authority from there, so the roller need not run into it. ---
    const bool scoringDown = robot::scoring.isScoreDescent();
    if (!scoringDown) autoOuttakeExpired = false;  // a finished score re-arms the timeout for the next one
    if (scoringDown && !autoOuttakeExpired && autoOuttakeStart == 0 &&
        liftPos < config::L2_OUTTAKE_BELOW_DEG) {
        autoOuttakeStart = pros::millis();
    }
    if (autoOuttakeStart != 0) {
        // Done once the arm reaches the stop height. Cancelled by anything that would otherwise fight
        // it -- the manual override, B, a stuck score, or a lift that gave up on the move entirely
        // (BLOCKED/FAULT).
        const bool done = liftPos <= config::L2_OUTTAKE_STOP_DEG;
        const bool cancelled = in.manualLift || in.abort || robot::lift.isScoreStuck() ||
                               robot::lift.isBlocked() || robot::lift.isFaulted();
        const bool timedOut =
            pros::millis() - autoOuttakeStart >= static_cast<std::uint32_t>(config::L2_OUTTAKE_MAX_MS);
        if (timedOut) autoOuttakeExpired = true;
        if (done || cancelled || timedOut) autoOuttakeStart = 0;
    }
    screen::autoOuttake = autoOuttakeStart != 0;

    // --- L1 auto-intake: sweep the floor while the arm is rising off it anyway. An L1 press that
    // starts a rise FROM THE FLOOR -- arm at or below LIFT_BOTTOM_GOAL_DEG, seated, or parked at the
    // INTAKE handoff after R1 -- runs the rollers IN from the press itself (so they are already going
    // during CLAMPING's claw-close wait) until the arm climbs past L1_AUTO_INTAKE_UNTIL_DEG. Extra L1
    // taps during the same rise neither restart nor extend it (the window only opens while it is
    // closed), and an L1 from a stack level never opens it at all. This is ONE INPUT to the single
    // intake.set() below, never a second writer. ---
    if (in.stepUpPressed && autoIntakeStart == 0 && !in.manualLift &&
        robot::scoring.getState() != Scoring::State::RELEASING &&
        (robot::lift.isSeated() || liftPos <= config::LIFT_BOTTOM_GOAL_DEG ||
         robot::scoring.getState() == Scoring::State::INTAKE_POS)) {
        autoIntakeStart = pros::millis();
    }
    if (autoIntakeStart != 0) {
        // Done when the arm is clear of the floor; cancelled by anything that abandons the rise -- the
        // manual override, clamped-mode R1/R2 (which take the lift over), B, L2, or a lift that gave
        // up; and timed out so a rise that never gets there can't leave the rollers spinning.
        const bool done = liftPos > config::L1_AUTO_INTAKE_UNTIL_DEG;
        const bool cancelled = in.manualLift || in.abort || in.scorePressed || in.liftButtonMode ||
                               robot::lift.isBlocked() || robot::lift.isFaulted();
        const bool timedOut =
            pros::millis() - autoIntakeStart >= static_cast<std::uint32_t>(config::L1_AUTO_INTAKE_MAX_MS);
        if (done || cancelled || timedOut) autoIntakeStart = 0;
    }
    screen::autoIntake = autoIntakeStart != 0;

    // The direct request always wins: R1/R2 in R:INTAKE mode are the roller buttons, and auton's
    // intakeIn()/intakeOut() land in the same place. Then L2's auto-outtake (so a score always ejects),
    // then L1's auto-intake -- they can never both be active (one needs the claw empty and rising, the
    // other a pin held and descending) -- then stop. ONE intake.set() call, the single writer decision.
    if (in.intakeRequest != Intake::Mode::STOP) robot::intake.set(in.intakeRequest, in.intakePower);
    else if (screen::autoOuttake) robot::intake.set(Intake::Mode::OUT, config::L2_OUTTAKE_PCT / 100.0);
    else if (screen::autoIntake) robot::intake.set(Intake::Mode::IN);
    else robot::intake.set(Intake::Mode::STOP);

    // The autoclamp must never fire during an L2 score: the arm is pressing a pin onto a stack with
    // the jaws already shut, and the moment they open the stack is a few millimetres away. Claw's own
    // re-arm rule (wait for a clear reading) already covers the aftermath; this covers the press
    // itself, explicitly, every tick. See include/subsystems/autoclamp_detector.hpp.
    robot::claw.setInhibit(robot::scoring.isScoreDescent() || robot::lift.isScoring());

    robot::claw.update();
    robot::scoring.update();
}

}  // namespace periodic
