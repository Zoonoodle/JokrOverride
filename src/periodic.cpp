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
    // ...and drop any clamp nobody has reacted to yet. THIS PREVENTS MOTION, which is why it stays
    // (and matters more than ever now that no mode start resets anything): a clamp that happened with
    // no tick running would otherwise still be queued when the next enabled mode starts ticking, and
    // Scoring would run the clamp-grab hop for it -- the field switch making the arm move for a
    // clamp that happened before it, which is exactly CLAUDE.md safety rule 1.
    // competition_initialize() used to be the source of one of these (it gripped the preload with
    // claw.clamp()); that call is gone as of 2026-10-07, but a clamp can still arrive while disabled
    // from the autoclamp, so the guard is still load-bearing.
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
        // of the one-press score (the arm carries on to the bottom at the scoring speed,
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
    // THE R1 ROLLER GATE (driver, 2026-10-07): while the intake macro is still getting the arm into
    // position -- climbing to the pivot height, waiting for the pivot, or at the top of the descent --
    // holding R1 must NOT spin the rollers. Scoring::intakeRollersBlocked() is that test (the sequence
    // is running AND the arm is above INTAKE_ROLLERS_ENABLE_BELOW_DEG). Gated on !in.auton so it only
    // ever touches the DRIVER's R1: an auton route's auton::intakeIn()/intakeFor() arrives through this
    // same field and is deliberately left alone.
    const bool driverRollerBlocked = !in.auton && robot::scoring.intakeRollersBlocked();
    if (grabbing) robot::intake.set(Intake::Mode::STOP);  // the grab push beats every other input
    else if (in.intakeRequest != Intake::Mode::STOP && !driverRollerBlocked)
        robot::intake.set(in.intakeRequest, in.intakePower);
    else if (autoOuttakeOn) robot::intake.set(Intake::Mode::OUT, config::L2_OUTTAKE_PCT / 100.0);
    else if (autoIntakeOn) robot::intake.set(Intake::Mode::IN);
    else robot::intake.set(Intake::Mode::STOP);

    // The autoclamp must never fire during an L2 score: the arm is pressing a pin onto a stack with
    // the jaws already shut, and the moment they open the stack is a few millimetres away. Claw's own
    // re-arm rule (wait for a clear reading) already covers the aftermath; this covers the press
    // itself, explicitly, every tick. See include/subsystems/autoclamp_detector.hpp.
    // THE INHIBIT ENDS WHEN PRESS 2 IS PRESSED, not when the arm seats (the driver, 2026-10-03: "from
    // the moment the second l2 ... auto clamp should be back on, should not wait for that at all"), so
    // Lift::isScoreCycle() -- which startScoreReturn() now clears -- is what bounds it, together with
    // isScoring() for the press itself.
    // NOTE Scoring::isScoreDescent() is deliberately NOT in here any more. It stays true through the
    // RETURNING state, i.e. all of press 2's descent, so leaving it in would have kept the inhibit on for
    // exactly the window the driver asked to have the autoclamp live in -- clearing the lift's own flag
    // alone would have changed nothing. It is still the right signal for the auto-outtake window above,
    // which DOES want to run through that descent, so the two uses have been split rather than merged.
    robot::claw.setInhibit(robot::lift.isScoring() || robot::lift.isScoreCycle());

    // AT THE R1 INTAKE HANDOFF the autoclamp fires on the FIRST in-range reading rather than after the
    // ordinary AUTOCLAMP_CONFIRM_MS debounce -- the driver wants the pin gripped "at the instant it sees
    // the object", and Scoring then pushes the claw down onto it instead of hopping up. Passed every
    // tick, like the inhibit above, so it can never be left latched on.
    robot::claw.setIntakeMode(robot::scoring.isAtIntakeHandoff());
    // ...and which mode we are in, because the trigger range differs: 65 mm in auton, 55 in driver (the
    // driver's measured numbers -- a human aiming the claw wants the shorter reach, a route that drives
    // to a surveyed spot needs to commit from further out). Same deal: passed, never latched.
    robot::claw.setAutonMode(in.auton);
    // Scoring needs the same fact, for the clamp hop's per-mode delay (0 in auton, 200 ms in driver --
    // see CLAMP_RISE_DELAY_AUTON_MS). From here rather than is_autonomous() so a brain-menu run counts.
    robot::scoring.setAutonMode(in.auton);

    // ...AND THE ENABLE ITSELF, every tick, for the same reason all three above are passed every tick.
    // THE BUG THIS FIXES (the driver, 2026-10-08): "when I start driver (raw, without doing auton),
    // autoclamp is never armed and autoclamp doesn't happen. Ensure auto clamp is always on in driver."
    // The enable + arm used to be the ONE piece of the claw's per-mode state that was latched: a single
    // scoring.setAutoclamp(config::AUTOCLAMP_ON_IN_DRIVER) at the top of opcontrol(), and nothing
    // anywhere re-asserted it for the rest of the match. PROS enters opcontrol() once, so if that one
    // call did not land on the Claw -- it early-returns while the jaws are CLAMPED (Scoring), and
    // Claw::arm() early-returns from CLAMPED/RELEASED -- the autoclamp stayed dead for the whole of
    // driver control, showing `AC:OFF` with nothing to explain why and no way back.
    // Now it is a MIRROR of the mode's intent (Scoring::autoclampOn(), which is what opcontrol's
    // AUTOCLAMP_ON_IN_DRIVER, auton's AUTOCLAMP_ON_IN_AUTON, Y and auton::autoclamp() all set), pushed
    // every tick exactly like the inhibit. Nothing here moves a cylinder: it is the flag only, and
    // Claw::update()'s own OPEN branch is what turns an enabled-but-open claw back into an armed one.
    robot::claw.setAutoclampEnabled(robot::scoring.autoclampOn());

    // Claw::update() first, then Scoring::update(): a clamp the autoclamp makes this tick is therefore
    // reacted to (the rise, or the handoff push) in the SAME tick rather than the next one.
    robot::claw.update();
    robot::scoring.update();
}

}  // namespace periodic
