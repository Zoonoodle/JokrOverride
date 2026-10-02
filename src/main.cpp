#include "main.h"

#include "config.hpp"
#include "robot.hpp"
#include "screen.hpp"


using namespace robot;
using subsystems::Intake;
using subsystems::Scoring;

namespace {
/** R1/R2 have two meanings and the one in force is LATCHED at the moment the button goes down, so a
 *  clamp that changes while the button is held can never swap a lift move for a spinning intake
 *  mid-press. See the button map above and README "Driver guide".
 *    INTAKE  nothing clamped: R1 = roller in (+ tap sends the arm to the handoff), R2 = roller out.
 *    LIFT    a pin is clamped: R1 = lift up, R2 = controlled lift down. Neither runs the intake.
 *    INERT   the claw clamped or unclamped while the button was held: the lift move and the intake are
 *            both stopped and nothing happens until the button is released and pressed again.
 */
screen::RMode rModeFor(bool held, bool clamped) {
    if (!held) return screen::RMode::IDLE;  // released: the next press re-latches
    switch (screen::rMode) {
        case screen::RMode::IDLE: return clamped ? screen::RMode::LIFT : screen::RMode::INTAKE;
        case screen::RMode::LIFT: return clamped ? screen::RMode::LIFT : screen::RMode::INERT;
        case screen::RMode::INTAKE: return clamped ? screen::RMode::INERT : screen::RMode::INTAKE;
        case screen::RMode::INERT: return screen::RMode::INERT;
    }
    return screen::RMode::INERT;
}
}  // namespace

/**
 * Button map (driver). Full walkthrough in README "Driver guide".
 *   (At the start of the first enabled mode the lift homes itself: it drives down to its hard stop
 *    and zeroes the rotation sensor there. Watch "HOMING"/"HOMED" on the brain's ANGLE line. Holding
 *    D-pad Up or Down cancels it. Nothing waits for it -- you can drive straight away.)
 *
 *   Left stick Y / Right stick X   arcade drive
 *   D-pad Up (hold)    lift up, full power, plain manual control (same feel as plain motor power)
 *   D-pad Down (hold)  lift down, full power, plain manual control
 *               Holding either immediately cancels the intake macro / any preset target and
 *               takes manual control. Letting go brakes and holds right there -- it does NOT
 *               resume whatever preset or macro step was active before.
 *   L1          intake macro: step up a stack level (1st tap clamps + goes to level 1; each
 *               tap after goes up one level, staying at the top once reached). An L1 that starts a
 *               rise FROM THE FLOOR (at the bottom/seated, or parked at the INTAKE handoff) also runs
 *               the intake rollers IN from the press until the arm passes L1_AUTO_INTAKE_UNTIL_DEG --
 *               sweeping the floor while the arm is going up anyway. `INT:AUTO` on the brain's DRV
 *               line while it does. Extra taps don't restart or extend it.
 *   L2          intake macro: THE SENSING SCORER (above the bottom, holding a pin). Presses the pin
 *               DOWN onto the stack with the claw still SHUT at L2_SCORE_SPEED_PCT, watches the
 *               rotation sensor for the landing (at least two of LAG/EFFORT/SPEED must agree -- see
 *               contact_detector.hpp -- a lone signal is no longer enough since this six-bar's gravity
 *               torque swing can false-fire one on its own). GUARANTEED RELEASE: if nothing is detected
 *               by the time the arm crosses L2_AUTO_RELEASE_DEG (32 deg) it releases anyway, right
 *               there (shown as REL(32)) -- there is no more "pressed all the way down with nothing
 *               released". Level 1 (31 deg) is already below this, so L2 at level 1 releases on the
 *               very first tick, with no press-and-sense feel at all. Either way: releases the claw
 *               and keeps pushing so the jaws slide off, checks the arm is really free (lifting
 *               L2_UNSTICK_UP_DEG and retrying if the claw is caught), then runs home fast, forces the
 *               last few degrees onto the hard stop (Lift's FINAL phase -- see lift.hpp -- so a cup or
 *               friction can't leave it parked short of true 0), and SEATs. Pivot stays OUT. Resets
 *               the level count. At the bottom instead, L2 toggles the claw (clamp / release) and
 *               moves nothing.
 *               Once this descent passes below L2_OUTTAKE_BELOW_DEG (40 deg), the intake rollers run
 *               OUT at L2_OUTTAKE_PCT on their own -- through REL/UNSTK/RETURN, whichever phase the
 *               arm is in -- and stop as soon as the arm reaches L2_OUTTAKE_STOP_DEG (tied to the
 *               lift's own FINAL handover height, 3 deg), where FINAL's own authority takes over
 *               forcing the rest of the way down. `INT:OUT` on the brain's DRV line while it runs.
 *
 *   R1 / R2 have TWO modes, and which one is in force is latched when the button goes down (see
 *   rModeFor() below; the brain's CLAW line shows `R:INTAKE` / `R:LIFT` / `R:wait`):
 *   nothing clamped ->
 *     R1 (hold)   intake roller in; (new-press) also sends the lift to the INTAKE handoff position
 *                 with the jaws open and the pivot BACK
 *     R2 (hold)   intake roller out
 *   a pin CLAMPED ->
 *     R1 (hold)   lift UP at R_LIFT_UP_PCT. Plain voltage, same feel as the D-pad.
 *     R2 (hold)   lift DOWN, controlled: closed-loop on the measured arm speed at R_LIFT_DOWN_PCT of
 *                 the free speed, slowing to the final-approach crawl near the bottom and SEATING on
 *                 the hard stop, exactly like L2's descent. Neither runs the intake.
 *   Letting go of either brakes and holds where it is (no old target resumes); D-pad Up/Down still
 *   win over both.
 *
 *   B           stow: pivot OUT, lift straight down to DOWN (0 deg) and seated, cancel the macro
 *   A           claw clamp / open (the claw is physically OPEN at power-on -- it needs air to grip:
 *               the CLAW line shows `OPEN D[0]`, and A closes it)
 *   X           pivot OUT / BACK (the brain's CLAW line shows which, with the port letter and level)
 *   Y           autoclamp on / off
 *
 * The lift PID task (subsystems/lift.*) starts once in initialize() and runs for the whole
 * program; it is the only code that ever writes to the lift motors (see the class comment in
 * include/subsystems/lift.hpp for why that matters). L1/L2/R1/B and the intake macro
 * (subsystems/scoring.*) all command it through lift.setTarget()/moveTo() -- never raw voltage.
 * D-pad Up/Down only ever reach the lift through lift.driverPower(), which preempts the PID from
 * inside that same task. Nothing else touches the motors.
 */

void initialize() {
    screen::start();  // brain readout first: if you only see the program name, initialize() never ran

    // Startup signal: if the controller doesn't buzz once and show "PROGRAM ON" at power-up,
    // the controller isn't linked to the brain, or this program isn't the one running.
    master.rumble(".");
    master.print(0, 0, "PROGRAM ON      ");

    // The lift must start at (or within 45 arm degrees of) DOWN. zero() does NOT re-zero the rotation
    // sensor -- its absolute zero is calibrated on the brain's device screen and survives power
    // cycles on its own; zero() just seeds the position counter from it (see the comment in
    // lift.cpp). It doesn't move anything either way. The PID task starts idle: no target is active
    // until a preset button, the intake macro, or autonomous explicitly commands a height (see lift.hpp).
    lift.zero();
    lift.startTask();

    // Calibrate in the background so nothing here can delay driver control.
    // chassis.calibrate(false) skips IMU calibration entirely when robot::imu is null (no IMU
    // fitted yet, see config::IMU_PORT) -- LemLib runs on the tracking wheel alone until one is added.
    static pros::Task calibrate([] {
        chassis.calibrate(imu != nullptr);
        screen::calibrated = true;
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Home the lift FIRST, before the odometry wait: requestHome() returns immediately and the homing
    // itself runs inside the lift's own 10 ms task, so the arm starts finding its hard stop while the
    // tracking wheel is still coming up and the rest of this routine is free to drive. Nothing below
    // has to wait for it -- lift.moveTo(..., true) queues behind homing on its own (see Lift). This is
    // the first ENABLED mode of a match, so this is the one place homing happens; opcontrol()'s call
    // below is only for a driver-skills run with no auton.
    scoring.reset();
    scoring.pivotOut();  // home in the power-on configuration (pivot OUT); a no-op at power-on
    lift.requestHome();

    while (!screen::calibrated) pros::delay(10);  // odometry must be ready before moving
    // TODO(team): autonomous routines. Example of a LemLib move:
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 24, 2000);
    //
    // Lift: named heights live in config::lift_preset (include/config.hpp) -- the same constants
    // the driver's preset buttons use. moveTo(height, timeoutMs, waitUntilSettled) blocks THIS
    // task (not the lift's own background task, which keeps running the PID regardless):
    //   lift.moveTo(config::lift_preset::INTAKE, 1500, true);
    //   lift.moveTo(config::lift_preset::STACK_LEVELS[2], 2000, true);  // 3rd stack level
    // lift.isSettled() / lift.isBlocked() / lift.cancel() are also available for non-blocking use.
}

void opcontrol() {
    master.print(0, 0, "DRIVER ON       ");

    // Home the lift if autonomous() didn't already (driver skills, or a practice run straight into
    // driver control). requestHome() is a no-op once a run has homed, failed or been cancelled, so a
    // real match homes exactly once, during auton. It never blocks: the drive loop below starts
    // immediately and the arm homes underneath it. Holding D-pad Up/Down cancels it.
    scoring.pivotOut();
    lift.requestHome();

    // L1's auto-intake window: the millisecond the rise began, or 0 when it isn't running. A plain
    // local, not static, so a second opcontrol() in one program run starts clean.
    std::uint32_t autoIntakeStart = 0;
    // L2's auto-outtake window: the millisecond the rollers started, or 0 when it isn't running, plus
    // a latch so L2_OUTTAKE_MAX_MS's timeout can't just re-arm itself on the very next tick -- it stays
    // off until the score descent ends and a fresh one starts.
    std::uint32_t autoOuttakeStart = 0;
    bool autoOuttakeExpired = false;

    while (true) {
        // --- Drive ---
        const int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(forward, turn);

        // --- Lift manual override: D-pad Up/Down, full power, plain manual control (see button map
        // above). Holding either cancels the intake macro / any preset target so they can never
        // fight over the motors; scoring.reset() below just stops the macro from re-issuing lift
        // commands, it doesn't touch the claw or pivot where they physically are. ---
        const bool up = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        const bool down = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        if (up) {
            scoring.reset();
            lift.driverPower(1.0);
        } else if (down) {
            scoring.reset();
            lift.driverPower(-1.0);
        }

        // --- R1 / R2: intake rollers, or (while a pin is clamped) manual lift up/down. The new-press
        // edges are read exactly once per loop, here, so latching below can't swallow one. ---
        const bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        const bool r1New = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);
        const bool l1New = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
        const bool l2New = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
        const bool bNew = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
        screen::rMode = rModeFor(r1 || r2, claw.isClamped());
        const bool rLift = screen::rMode == screen::RMode::LIFT && !up && !down;  // D-pad wins

        // --- L1 auto-intake: sweep the floor while the arm is rising off it anyway. An L1 press that
        // starts a rise FROM THE FLOOR -- arm at or below LIFT_BOTTOM_GOAL_DEG, seated, or parked at
        // the INTAKE handoff after R1 -- runs the rollers IN from the press itself (so they are already
        // going during CLAMPING's claw-close wait) until the arm climbs past
        // L1_AUTO_INTAKE_UNTIL_DEG. Extra L1 taps during the same rise neither restart nor extend it
        // (the window only opens while it is closed), and an L1 from a stack level never opens it at
        // all. This is ONE INPUT to the single intake.set() below, never a second writer. ---
        const double liftPos = lift.getPosition();

        // --- L2 auto-outtake: while an L2 SCORE descent (not the at-the-bottom clamp toggle, not
        // scoreOrMatchload()'s INTAKE lower) is under L2_OUTTAKE_BELOW_DEG, eject through the rest of
        // the trip down -- PRESS, REL, UNSTK, RETURN -- so anything still in the intake doesn't ride
        // along. Stack level 1 (31.0 deg) is already under 40, so scoring from level 1 outtakes from
        // the start of the press; that is this constant as specified, not a bug (see config.hpp).
        // Stops as soon as the lift reaches L2_OUTTAKE_STOP_DEG (2026-10-02 retune: tied directly to
        // LIFT_FINAL_START_DEG, see config.hpp) -- the lift's own FINAL phase takes real downward
        // authority from there, so the roller does not need to keep running into it. ---
        const bool scoringDown = scoring.isScoreDescent();
        if (!scoringDown) autoOuttakeExpired = false;  // a finished score re-arms the timeout for the next one
        if (scoringDown && !autoOuttakeExpired && autoOuttakeStart == 0 &&
            liftPos < config::L2_OUTTAKE_BELOW_DEG) {
            autoOuttakeStart = pros::millis();
        }
        if (autoOuttakeStart != 0) {
            // Done once the arm reaches the stop height. Cancelled by anything that would otherwise
            // fight it -- the manual override, B, a stuck score, or a lift that gave up on the move
            // entirely (BLOCKED/FAULT).
            const bool done = liftPos <= config::L2_OUTTAKE_STOP_DEG;
            const bool cancelled =
                up || down || bNew || lift.isScoreStuck() || lift.isBlocked() || lift.isFaulted();
            const bool timedOut =
                pros::millis() - autoOuttakeStart >= static_cast<std::uint32_t>(config::L2_OUTTAKE_MAX_MS);
            if (timedOut) autoOuttakeExpired = true;
            if (done || cancelled || timedOut) autoOuttakeStart = 0;
        }
        screen::autoOuttake = autoOuttakeStart != 0;

        if (l1New && autoIntakeStart == 0 && !up && !down &&
            scoring.getState() != Scoring::State::RELEASING &&
            (lift.isSeated() || liftPos <= config::LIFT_BOTTOM_GOAL_DEG ||
             scoring.getState() == Scoring::State::INTAKE_POS)) {
            autoIntakeStart = pros::millis();
        }
        if (autoIntakeStart != 0) {
            // Done when the arm is clear of the floor; cancelled by anything that abandons the rise --
            // the manual override, clamped-mode R1/R2 (which take the lift over), B, L2, or a lift that
            // gave up; and timed out so a rise that never gets there can't leave the rollers spinning.
            const bool done = liftPos > config::L1_AUTO_INTAKE_UNTIL_DEG;
            const bool cancelled = up || down || bNew || l2New || screen::rMode == screen::RMode::LIFT ||
                                   lift.isBlocked() || lift.isFaulted();
            const bool timedOut =
                pros::millis() - autoIntakeStart >= static_cast<std::uint32_t>(config::L1_AUTO_INTAKE_MAX_MS);
            if (done || cancelled || timedOut) autoIntakeStart = 0;
        }
        screen::autoIntake = autoIntakeStart != 0;

        // The driver always wins: R1/R2 in R:INTAKE mode are the roller buttons. Then L2's auto-outtake
        // (so a score always ejects), then L1's auto-intake -- they can never both be active (one needs
        // the claw empty and rising, the other a pin held and descending) -- then stop. ONE intake.set()
        // call, the single writer decision.
        if (screen::rMode == screen::RMode::INTAKE && r1) intake.set(Intake::Mode::IN);
        else if (screen::rMode == screen::RMode::INTAKE && r2) intake.set(Intake::Mode::OUT);
        else if (screen::autoOuttake) intake.set(Intake::Mode::OUT, config::L2_OUTTAKE_PCT / 100.0);
        else if (screen::autoIntake) intake.set(Intake::Mode::IN);
        else intake.set(Intake::Mode::STOP);

        // Clamped mode. R1 is plain voltage through the same manual path the D-pad uses (so it cancels
        // any target/macro and brake-holds on release); R2 is the lift's own banded descent, which
        // seats the arm on its hard stop if it is held all the way down. Both are no-ops the moment
        // the claw unclamps, because rModeFor() has already dropped the mode to INERT.
        if (rLift && r1) {
            scoring.reset();
            lift.driverPower(config::R_LIFT_UP_PCT / 100.0);
        } else if (!up && !down) {
            lift.driverPower(0.0);
        }
        if (rLift && r2 && !r1) scoring.reset();
        lift.descendManual(rLift && r2 && !r1);

        // --- Lift height presets / intake macro. Locked out while the manual override is held so a
        // tap here can never re-command the lift out from under a manual nudge. ---
        if (!up && !down) {
            if (r1New && screen::rMode == screen::RMode::INTAKE) scoring.goToIntake();
            if (l1New) scoring.stepUp();
            if (l2New) scoring.score();
            if (bNew) scoring.stow();  // pivot OUT + lift to DOWN, seated
        }

        // --- Claw (A), pivot (X), autoclamp (Y) ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) scoring.toggleClaw();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) scoring.togglePivot();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) scoring.setAutoclamp(!scoring.autoclampOn());

        claw.update();
        scoring.update();

        // --- Brain "DRV" line: proves this loop is running and what it reads ---
        screen::driverDebug = {screen::driverDebug.loops + 1, forward, turn, up, down};

        pros::delay(10);
    }
}
