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
// An L1 that opened the auto-intake sweep while the GRAB push was running: the sweep waits and starts
// from the RISE, never during the push (the rollers must not drag the pin out from under the jaws).
bool sweepAfterGrab = false;
// Are the macro windows actually driving the rollers this tick? Plain file-scope state now -- these
// used to live in namespace screen for the brain's old DRV line, which is gone.
bool autoIntakeOn = false;
bool autoOuttakeOn = false;

pros::Mutex gate;
}  // namespace

pros::Mutex& mutex() { return gate; }

void reset() {
    std::lock_guard<pros::Mutex> guard(gate);
    autoIntakeStart = 0;
    autoOuttakeStart = 0;
    autoOuttakeExpired = false;
    sweepAfterGrab = false;
    autoIntakeOn = false;
    autoOuttakeOn = false;
    // ...and drop any clamp nobody has reacted to yet. The one that matters is
    // competition_initialize()'s pre-match grip of the preloaded pin: that clamp happens while the robot
    // is DISABLED, with no tick running, so without this it would still be sitting there when auton
    // starts and the arm would hop CLAMP_RISE_DEG on the first tick of the match (CLAUDE.md safety
    // rule 1 -- the field switch must not make the arm move for a clamp that happened before it).
    robot::claw.clearClampEvent();
}

void tick(const Inputs& in) {
    std::lock_guard<pros::Mutex> guard(gate);

    const double liftPos = robot::lift.getPosition();
    // THE GRAB: while the arm is forcing the claw down onto a pin, the intake STOPS -- every input,
    // including the driver's R1/R2 and L1's own sweep. The rollers would otherwise drag the pin out
    // from under the jaws exactly as they close. See "THE GRAB" in lift.hpp and config::L1_GRAB_*.
    const bool grabbing = robot::lift.isGrabbing();

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
        // ...and the moment the descent it belongs to is over. With the two-press L2 that is the end
        // of PRESS 1 (the arm stops L2_CLEAR_RISE_DEG above the stack rather than carrying on to 0,
        // so the stop-height test below would never be met), and the end of press 2's run home.
        const bool cancelled = in.manualLift || in.abort || !scoringDown || robot::lift.isScoreStuck() ||
                               robot::lift.isBlocked() || robot::lift.isFaulted();
        const bool timedOut =
            pros::millis() - autoOuttakeStart >= static_cast<std::uint32_t>(config::L2_OUTTAKE_MAX_MS);
        if (timedOut) autoOuttakeExpired = true;
        if (done || cancelled || timedOut) autoOuttakeStart = 0;
    }
    autoOuttakeOn = autoOuttakeStart != 0;

    // --- L1 auto-intake: sweep the floor while the arm is rising off it anyway. An L1 press that
    // starts a rise FROM THE FLOOR -- arm at or below LIFT_BOTTOM_GOAL_DEG, seated, or parked at the
    // INTAKE handoff after R1 -- runs the rollers IN from the press itself (so they are already going
    // during CLAMPING's claw-close wait) until the arm climbs past L1_AUTO_INTAKE_UNTIL_DEG. Extra L1
    // taps during the same rise neither restart nor extend it (the window only opens while it is
    // closed), and an L1 from a stack level never opens it at all. This is ONE INPUT to the single
    // intake.set() below, never a second writer. ---
    if (in.stepUpPressed && autoIntakeStart == 0 && !in.manualLift && !sweepAfterGrab &&
        robot::scoring.getState() != Scoring::State::RELEASING &&
        (robot::lift.isSeated() || liftPos <= config::LIFT_BOTTOM_GOAL_DEG ||
         robot::scoring.getState() == Scoring::State::INTAKE_POS ||
         robot::scoring.getState() == Scoring::State::GRABBING)) {
        // An L1 from the handoff starts the GRAB push first; the sweep then belongs to the RISE that
        // follows it, not to the push.
        if (grabbing) sweepAfterGrab = true;
        else autoIntakeStart = pros::millis();
    }
    if (sweepAfterGrab) {
        const bool cancelled = in.manualLift || in.abort || in.scorePressed || in.liftButtonMode ||
                               robot::lift.isBlocked() || robot::lift.isFaulted();
        if (cancelled) sweepAfterGrab = false;
        else if (!grabbing) {  // the push is over and the rise is starting: sweep now
            sweepAfterGrab = false;
            autoIntakeStart = pros::millis();
        }
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
    autoIntakeOn = autoIntakeStart != 0;

    // The direct request always wins: R1/R2 in R:INTAKE mode are the roller buttons, and auton's
    // intakeIn()/intakeOut() land in the same place. Then L2's auto-outtake (so a score always ejects),
    // then L1's auto-intake -- they can never both be active (one needs the claw empty and rising, the
    // other a pin held and descending) -- then stop. ONE intake.set() call, the single writer decision.
    if (grabbing) robot::intake.set(Intake::Mode::STOP);  // the grab push beats every other input
    else if (in.intakeRequest != Intake::Mode::STOP) robot::intake.set(in.intakeRequest, in.intakePower);
    else if (autoOuttakeOn) robot::intake.set(Intake::Mode::OUT, config::L2_OUTTAKE_PCT / 100.0);
    else if (autoIntakeOn) robot::intake.set(Intake::Mode::IN);
    else robot::intake.set(Intake::Mode::STOP);

    // The autoclamp must never fire during an L2 score: the arm is pressing a pin onto a stack with
    // the jaws already shut, and the moment they open the stack is a few millimetres away. Claw's own
    // re-arm rule (wait for a clear reading) already covers the aftermath; this covers the press
    // itself, explicitly, every tick. See include/subsystems/autoclamp_detector.hpp.
    // ...and for the WHOLE two-press score cycle (Lift::isScoreCycle(): press 1, the hold above the
    // stack, press 2's descent, until the arm is seated). Without that, the autoclamp would re-arm as
    // soon as the arm rose clear of the pin and then grab it again on the way back down past it.
    robot::claw.setInhibit(robot::scoring.isScoreDescent() || robot::lift.isScoring() ||
                           robot::lift.isScoreCycle());

    // AT THE R1 INTAKE HANDOFF the autoclamp fires on the FIRST in-range reading rather than after the
    // ordinary AUTOCLAMP_CONFIRM_MS debounce -- the driver wants the pin gripped "at the instant it sees
    // the object", and Scoring then pushes the claw down onto it instead of hopping up. Passed every
    // tick, like the inhibit above, so it can never be left latched on.
    robot::claw.setIntakeMode(robot::scoring.isAtIntakeHandoff());

    // Claw::update() first, then Scoring::update(): a clamp the autoclamp makes this tick is therefore
    // reacted to (the rise, or the handoff push) in the SAME tick rather than the next one.
    robot::claw.update();
    robot::scoring.update();
}

}  // namespace periodic
