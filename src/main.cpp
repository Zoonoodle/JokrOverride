#include "main.h"

#include "auton.hpp"
#include "autons.hpp"
#include "config.hpp"
#include "periodic.hpp"
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
 *               FROM THE INTAKE HANDOFF (the R1 position) L1 does THE GRAB first: the arm forces
 *               itself DOWN onto the pin at L1_GRAB_PUSH_PCT (40% of volts), the jaws close
 *               L1_GRAB_CLOSE_AT_MS (200 ms) into that push, the push holds until both
 *               L1_GRAB_PUSH_MS (300 ms) and the jaws' own CLAW_CLOSE_TIME_MS are satisfied -- so
 *               about 350 ms of pressing down -- and only then does the arm rise to level 1. The
 *               INTAKE IS STOPPED for the whole push (it would drag the pin out from under the jaws),
 *               and the floor sweep above starts from the rise instead. Brain `LIFT` line: `GRAB`.
 *   L2          intake macro: THE SENSING SCORER, IN TWO PRESSES (above the bottom, holding a pin).
 *               PRESS 1 presses the pin
 *               DOWN onto the stack with the claw still SHUT at L2_SCORE_SPEED_PCT, watches the
 *               rotation sensor for the landing (at least two of LAG/EFFORT/SPEED must agree -- see
 *               contact_detector.hpp -- a lone signal is no longer enough since this six-bar's gravity
 *               torque swing can false-fire one on its own; a loaded arm also gets ARM ON MOTION +
 *               SPLIT STALL so it isn't misread as already resting on something while it is still just
 *               slowly out-running gravity under load). GUARANTEED RELEASE: if nothing is detected once
 *               the arm has travelled L2_AUTO_RELEASE_DEG (32 deg) DOWN FROM where L2 was first pressed
 *               (RELATIVE travel, not an absolute reading), it releases anyway, right there (shown as
 *               REL(32)) -- there is no more "pressed all the way down with nothing released". A start
 *               too low for that much travel before the floor (level 1, 31 deg) instead falls back to
 *               releasing at L2_RELEASE_FLOOR_DEG (shown as REL(floor)), so level 1 still gets real
 *               sensing time rather than releasing on the first tick. Either way: releases the claw
 *               and keeps pushing so the jaws slide off, checks the arm is really free (lifting
 *               L2_UNSTICK_UP_DEG and retrying if the claw is caught), keeps pressing down a further
 *               L2_POST_RELEASE_PUSH_MS (100 ms) so the jaws slide clear, then RISES
 *               L2_CLEAR_RISE_DEG (20 deg) above wherever it stopped and HOLDS there, indefinitely --
 *               brain: `SCORED UP`. It does NOT go to the bottom any more: the point is to drive away
 *               with the claw above the pins without knocking any over.
 *               PRESS 2 (L2 again from that held position) is the run home: the fast descent at
 *               L2_RETURN_SPEED_PCT, FINAL forcing the last few degrees onto the hard stop, and SEAT
 *               at 0 -- exactly the old ending, just on a second press. B does the same thing from
 *               there. Pivot stays OUT. The level count resets after press 1. While held above: R1
 *               still runs the rollers and sends the arm to the handoff, L1 still starts a new stack
 *               from level 1, and D-pad manual still wins over everything.
 *               At the bottom instead, L2 toggles the claw (clamp / release) and moves nothing.
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

void disabled() {
    // A field switch from auton to driver kills the autonomous() task wherever it happens to be, so
    // this is the first place that can retire the auton tick. end() is idempotent and blocks until the
    // tick has really exited, so the auton tick and opcontrol()'s loop can never both be running (see
    // include/periodic.hpp). It does not touch the lift: whatever height auton left the arm holding,
    // or the hard stop it left it seated on, stays held by the lift's own task.
    auton::end();
}

void competition_initialize() {}

void autonomous() {
    // COMMON SETUP, in one place, so no route can forget any of it. The route itself is in
    // src/autons.cpp -- that is the only file to edit when writing routes (see README "Writing an
    // auton route"). Everything here is deliberately non-blocking or bounded: nothing in autonomous()
    // can eat the match period waiting on a sensor.

    // 1. The auton tick. The lift PID has always run in auton (its own 10 ms task from
    //    initialize()), but Scoring's state machine, the claw's autoclamp and the intake decision all
    //    used to live inside opcontrol()'s while loop, so NONE of them ran here -- an auton L2 score
    //    could never have its jaws opened, and the rollers never turned. begin() starts a 10 ms task
    //    that runs THE SAME per-loop logic the driver loop runs, in one copy (periodic::tick()). It
    //    commands no motion.
    auton::begin();

    // 2. The autoclamp is OFF for auton by default; a route turns it on where it wants it with
    //    auton::autoclamp(true). (In driver control it is on by default -- see opcontrol().)
    auton::autoclamp(false);

    // 3. Home the lift, at the start of EVERY route. This is the first ENABLED mode of a match, so
    //    this is the one place it happens (opcontrol()'s own call is only for a driver-skills run with
    //    no auton). requestHome() returns immediately and the homing runs inside the lift's own task,
    //    so the arm finds its hard stop while the route drives; a lift command from the route queues
    //    behind it on its own. On success the arm ends SEATED on the stop, not on a passive brake.
    auton::homeLift();

    // 4. Odometry/IMU calibration, BOUNDED. A drive move needs a heading, so wait for it -- but never
    //    longer than AUTON_CALIBRATE_TIMEOUT_MS, so an unplugged IMU costs a few seconds instead of
    //    the whole autonomous period, and a route that never drives still runs.
    const std::uint32_t calibrateStart = pros::millis();
    while (!screen::calibrated &&
           pros::millis() - calibrateStart < static_cast<std::uint32_t>(config::AUTON_CALIBRATE_TIMEOUT_MS)) {
        pros::delay(10);
    }

    // 5. The route. ONE call. Which one runs is the SELECTED line at the top of src/autons.cpp.
    autons::run();

    // Retire the auton tick. The lift keeps holding -- or stays seated on -- whatever state the route
    // left it in; end() never touches it.
    auton::end();
}

void opcontrol() {
    master.print(0, 0, "DRIVER ON       ");

    // Take the per-loop logic back from the auton tick BEFORE anything else: end() blocks until that
    // task has really left periodic::tick(), so from here on this loop is its only caller and there
    // is exactly one intake decision and one Scoring::update() per tick (see include/periodic.hpp).
    // It never touches the lift, so a seated or held arm stays exactly where auton left it.
    auton::end();
    periodic::reset();  // a second enabled mode never inherits a half-open macro window

    // Home the lift if autonomous() didn't already (driver skills, or a practice run straight into
    // driver control). requestHome() is a no-op once a run has homed, failed or been cancelled, so a
    // real match homes exactly once, during auton. It never blocks: the drive loop below starts
    // immediately and the arm homes underneath it. Holding D-pad Up/Down cancels it.
    scoring.pivotOut();
    lift.requestHome();

    // The autoclamp is ON for the whole of driver control by default, every time opcontrol starts:
    // whenever the jaws are open, anything that sits at or inside config::AUTOCLAMP_RANGE_MM for
    // AUTOCLAMP_CONFIRM_MS gets gripped. Y still toggles it mid-match, and the brain's CLAW line shows
    // what it is doing (OFF / ARMED / SEEING / CLEAR?). See README "Autoclamp".
    scoring.setAutoclamp(config::AUTOCLAMP_ON_IN_DRIVER);

    // L1's auto-intake sweep window, L2's auto-outtake window, the single intake decision, and the
    // Claw/Scoring ticks all live in periodic::tick() now -- ONE copy of that logic, shared with the
    // auton tick (see include/periodic.hpp). This loop's only job there is to fill in what the driver
    // is asking for, in `in`, and call it once at the bottom of every pass.
    while (true) {
        periodic::Inputs in;
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

        // --- What the driver is asking of the shared per-loop logic this tick. R1/R2 in R:INTAKE mode
        // are the roller buttons and always beat both macro windows; the rest are the flags those
        // windows open and close on (see periodic::Inputs). periodic::tick() at the bottom of this loop
        // makes the ONE intake decision and ticks the claw and Scoring. ---
        if (screen::rMode == screen::RMode::INTAKE && r1) in.intakeRequest = Intake::Mode::IN;
        else if (screen::rMode == screen::RMode::INTAKE && r2) in.intakeRequest = Intake::Mode::OUT;
        in.manualLift = up || down;
        in.liftButtonMode = screen::rMode == screen::RMode::LIFT;
        in.abort = bNew;
        in.scorePressed = l2New;
        in.stepUpPressed = l1New;

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

        // --- The shared per-loop logic, once: both macro windows, the single intake decision, then
        // Claw::update() and Scoring::update(). The auton tick runs this exact call (see
        // include/periodic.hpp), and auton::end() above guarantees it is not running now. ---
        periodic::tick(in);

        // --- Brain "DRV" line: proves this loop is running and what it reads ---
        screen::driverDebug = {screen::driverDebug.loops + 1, forward, turn, up, down};

        pros::delay(10);
    }
}
