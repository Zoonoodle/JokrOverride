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
 *   D-pad Up (hold)    lift up at LIFT_MANUAL_UP_PCT (100%), plain manual control
 *   D-pad Down (hold)  lift down at LIFT_MANUAL_DOWN_PCT (50%) -- HALF SPEED on purpose, the driver's
 *                      call 2026-10-03: gravity already helps on the way down and it ends against the
 *                      hard stop, so full voltage down is a drop rather than a nudge
 *               Holding either immediately cancels the intake macro / any preset target and
 *               takes manual control. Letting go brakes and holds right there -- it does NOT
 *               resume whatever preset or macro step was active before.
 *   L1 (hold)   LIFT UP, steady and fast, for as long as it is held: plain manual voltage at
 *               L1_LIFT_UP_PCT (99% of LIFT_MAX_MV), through the same Lift::driverPower() path the
 *               D-pad uses. THIS IS THE OLD CLAMPED-MODE R1-UP, moved to L1 and 10% quicker (it was
 *               90%) -- the driver, 2026-10-09: "l1 hold moves the arm up like our old r1 did when we
 *               had clamp active, it moves arm up steady and fast with hold, holds it after release".
 *               Holding it CANCELS the intake macro and any preset target, exactly as the D-pad does,
 *               and LETTING GO BRAKE-HOLDS the arm right where it is -- no old target resumes. It
 *               works with the claw clamped or empty, and like every manual override it is not
 *               clamped by lift_preset::MAX (the mechanism's own top stop is the limit). D-pad
 *               Up/Down still win over it; it wins over everything else, a score included.
 *               THERE ARE NO STACK LEVELS IN DRIVER ANY MORE (the driver's call, same day): no ladder,
 *               no rung counter, no +20 step, no grab-at-the-handoff on L1. The level system is
 *               AUTON'S -- auton::stepUp() still walks the ladder and auton::liftTo(Level::N) still
 *               goes to a measured level. Nothing about auton changed.
 *   L2          DURING THE INTAKE SEQUENCE (R1's approach / pivot wait / descent / holding the
 *               handoff) IT IS "GO HOME" (2026-10-09): rise just far enough for the pivot to retract
 *               (INTAKE_RESET_CLEAR_DEG), swing the pivot OUT, then force straight down to 0 and SEAT.
 *               L2 during the APPROACH, where the pivot has not engaged yet, skips the rise and simply
 *               goes down. The autoclamp is inhibited for the whole manoeuvre -- the cup being taken
 *               is right in front of the open jaws, going up and coming back down -- and is armed
 *               again the moment the arm seats. Brain line 10: `INTAKE RESET` -> `SEAT`.
 *               EVERYWHERE ELSE L2 is unchanged:
 *               intake macro: THE SENSING SCORER, IN TWO PRESSES (above the bottom, holding a pin).
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
 *   R1 (hold)   THE INTAKE ROLLERS IN, ALWAYS -- clamped or not, macro or no macro, for as long as
 *               it is held (the driver, 2026-10-09: "have r1 intake always"). There is no roller gate
 *               any more: the rollers run through the whole of the macro below, approach included.
 *               The only thing that still silences them is the grab push itself, which would drag the
 *               pin out from under the closing jaws.
 *     (new-press, JAWS EMPTY)  ...and start the INTAKE MACRO: pivot disengaged and up to
 *               INTAKE_PIVOT_TRIGGER_DEG, engage the pivot, wait INTAKE_PIVOT_SETTLE_MS, then creep
 *               down to lift_preset::INTAKE and hold there with the jaws open for the handoff. The
 *               autoclamp grabs what arrives and the automatic grab sequence lifts it to level 1.
 *     (new-press, HOLDING A PIN)  nothing but the rollers. No lift move, no pivot swing.
 *   R2 (hold)   TWO modes, latched when the button goes down (see rModeFor() below; the brain's CLAW
 *               line shows `R:INTAKE` / `R:LIFT` / `R:wait`):
 *                 nothing clamped -> intake roller OUT.
 *                 a pin CLAMPED   -> lift DOWN, controlled: closed-loop on the measured arm speed at
 *                 R_LIFT_DOWN_PCT of the free speed, slowing to the final-approach crawl near the
 *                 bottom and SEATING on the hard stop, exactly like L2's descent. Letting go
 *                 part-way brakes and holds where it is (no old target resumes).
 *               R1'S OLD CLAMPED-MODE "lift up at R_LIFT_UP_PCT" MOVED TO L1 (hold), above.
 *   D-pad Up/Down still win over both.
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

    // THE LIFT MAY START ANYWHERE. Nothing here resets, seeds, folds or tares the lift's position --
    // initState() only puts the control state in a known place, and it reads no sensor at all (the
    // driver, 2026-10-07: "don't reset the position ANYWHERE in the code ... we can have the lift up
    // and it DOESN'T count that starting position as 0"). getPosition() is the rotation sensor's own
    // all-time count, so an arm left part-way up reports its real height.
    // Neither call moves anything. The PID task starts idle: no target is active until a preset
    // button, the intake macro, or an auton route explicitly commands a height (see lift.hpp).
    lift.initState();
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

    // PRE-MATCH ALIGNMENT PREVIEW. While the robot sits disabled on the tiles, keep averaging the left
    // distance sensor: auton::startPose() then uses that long, settled average instead of spending
    // auton time on one, and the brain's START line shows the drive team the alignment live BEFORE the
    // match starts. PROS kills this task the moment the robot is enabled, and is_disabled() ends the
    // loop cleanly if it does not. Nothing here moves anything.
    while (pros::competition::is_disabled()) {
        auton::startAlignSample();
        pros::delay(20);
    }
}

void competition_initialize() {
    // NOTHING MOVES HERE. This used to grip the preloaded pin with claw.clamp() (2026-10-03), but the
    // driver's standing instruction from 2026-10-07 is "no longer reset position at the beginning of
    // auton or driver, just keep arm claw where it is" -- and that applies to the pre-match hook too:
    // the jaws stay wherever they physically are, and no cylinder fires on its own.
    //
    // A ROUTE THAT NEEDS THE PRELOAD GRIPPED MUST SAY SO. auton::clawGrip() is the call, and both
    // MATCH and MATCHLOAD_FINISH already begin with it, so nothing regressed there. A route that
    // forgets it now starts with open jaws instead of silently inheriting a pre-match grip.
}

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

    // 2. The autoclamp is ON for auton by default too (config::AUTOCLAMP_ON_IN_AUTON, 2026-10-03 -- it
    //    used to be hard-off here and every route had to remember auton::autoclamp(true)). It is
    //    asynchronous: the auton tick above is what watches the sensor, so a route never blocks on it and
    //    nothing has to be called to get it. A route that does not want it calls auton::autoclamp(false).
    //    (Driver control enables it the same way from AUTOCLAMP_ON_IN_DRIVER -- see opcontrol().)
    auton::autoclamp(config::AUTOCLAMP_ON_IN_AUTON);

    // 3. HOME THE LIFT, NON-BLOCKING (the driver, 2026-10-07: "go back to how we reset our position to
    //    0 by moving lift down at the start. have this be a non blocking thing so rest of auton can
    //    still run while it does this"). The arm drives down onto its hard stop and the sensor is
    //    zeroed THERE, which is what makes lift_preset::DOWN 0.0 and every other height a positive
    //    number measured up from the stop -- the same figures on every boot, however the brain was
    //    started. It replaces the software startup offset, which could not work (see "THE STARTUP
    //    UNWRAP IS GONE" in config.hpp).
    //
    //    THIS RETURNS AT ONCE. Homing runs as a mode inside the lift's own task, so the route below
    //    starts immediately and the arm finds its stop underneath it: drive moves, the claw and the
    //    intake all carry on. A LIFT command issued while it is still homing is queued and starts the
    //    moment homing finishes, so a route never silently loses one. Holding the D-pad cancels it.
    //    Only the LIFT moves -- homeLift() does not touch the claw or the pivot.
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
    // chassis.moveToPoint(0, -14, 1000, {.forwards = false, .maxSpeed = 60});
    // auton::autoclamp(true);  // driver control starts with the autoclamp ON, just like auton (config::AUTOCLAMP_ON_IN_DRIVER)
    // pros::delay(100000); 
    
    // NOTHING MOVES AT THE START OF DRIVER CONTROL. This used to call auton::liftDownFast(), which ran
    // the arm down to its hard stop the instant driver control began -- including on the auton ->
    // driver switch in a real match, where it threw away whatever height auton had left the arm at.
    // Removed on the driver's standing instruction, 2026-10-07: "no longer reset position at the
    // beginning of auton or driver, just keep arm claw where it is".
    master.print(0, 0, "DRIVER ON       ");

    // Take the per-loop logic back from the auton tick BEFORE anything else: end() blocks until that
    // task has really left periodic::tick(), so from here on this loop is its only caller and there
    // is exactly one intake decision and one Scoring::update() per tick (see include/periodic.hpp).
    // It never touches the lift, so a seated or held arm stays exactly where auton left it.
    auton::end();
    periodic::reset();  // a second enabled mode never inherits a half-open macro window
    // ...and neither does the intake macro. Autonomous is killed by the field switch wherever it
    // happens to be, so Scoring can be left mid-sequence -- STACKING, RELEASING, MOVING, SCORED. Its
    // state machine would then carry straight on the moment this loop starts ticking it and command
    // the lift with nobody touching a button (a STACKING with the claw already open, for instance,
    // immediately heads for the INTAKE handoff). reset() drops it back to IDLE; it touches neither
    // the claw nor the pivot nor the lift, so whatever height auton left the arm holding is kept.
    scoring.reset();

    // HOME HERE TOO, BUT NEVER TWICE IN ONE MATCH (the driver, 2026-10-07: "have it run during
    // opcontrol for now BUT if it ran in auton, dont run it in opcontrol"). This needs no mode flag
    // and no bookkeeping: Lift::requestHome() only acts from Home::IDLE, and robot::lift is one
    // long-lived object whose home_ state survives the auton -> driver switch. So a real match homes
    // in autonomous(), leaves home_ at DONE (or FAILED), and this call is a no-op; a practice run
    // straight into driver control finds home_ still IDLE and homes here instead.
    //
    // Non-blocking, exactly as in auton: it returns at once and the arm finds its stop underneath the
    // drive loop below. Holding the D-pad cancels it. It moves ONLY the lift -- no pivot, no claw.
    // (If homing was cancelled during auton it is not retried here either: home_ is CANCELLED, not
    // IDLE. Press the D-pad down to put the arm on its stop by hand if that ever happens.)
    //
    // Called straight on the lift rather than through auton::homeLift(), which is the same request
    // wrapped in the auton mutex: driver control has no business taking that lock, and this keeps the
    // driver path independent of the auton API.
    lift.requestHome();
    // THE REST OF THE SOFTWARE STATE NEEDS NO RE-SEEDING, and nothing else moves: Scoring's pivotOut_
    // and Claw's state_/level_ live on one long-lived object each, so they already describe where the
    // cylinders physically are when driver control takes over from auton. Only scoring.reset() above
    // was needed, and that moves nothing.

    // The autoclamp is ON for the whole of driver control by default, every time opcontrol starts:
    // whenever the jaws are open, anything that sits at or inside config::AUTOCLAMP_RANGE_DRIVER_MM for
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

        // --- Lift manual override: D-pad Up/Down, plain manual control at LIFT_MANUAL_UP_PCT /
        // LIFT_MANUAL_DOWN_PCT (100% / 50% -- down is half speed on purpose, see config.hpp) (see button map
        // above). Holding either cancels the intake macro / any preset target so they can never
        // fight over the motors; scoring.reset() below just stops the macro from re-issuing lift
        // commands, it doesn't touch the claw or pivot where they physically are. ---
        const bool up = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        const bool down = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        if (up) {
            scoring.reset();
            lift.driverPower(config::LIFT_MANUAL_UP_PCT / 100.0);
        } else if (down) {
            scoring.reset();
            lift.driverPower(-config::LIFT_MANUAL_DOWN_PCT / 100.0);
        }

        // --- R1: ALWAYS the intake rollers. R2: rollers out, or (while a pin is clamped) the
        // controlled lift down. The new-press edges are read exactly once per loop, here, so latching
        // below can't swallow one. ---
        const bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        const bool r1New = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);
        const bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);  // HELD, not a new press
        const bool l2New = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
        const bool bNew = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
        // THE MODE LATCH IS R2'S ALONE NOW (2026-10-09). It used to be latched off either button,
        // because either one changed meaning when a pin was clamped; R1 does not change meaning any
        // more -- it is the intake, clamped or not -- so including it here would only let an R1 press
        // flip R2's meaning out from under it. The brain's `R:` field therefore describes R2.
        screen::rMode = rModeFor(r2, claw.isClamped());
        const bool rLift = screen::rMode == screen::RMode::LIFT && !up && !down;  // D-pad wins

        // --- What the driver is asking of the shared per-loop logic this tick. periodic::tick() at the
        // bottom of this loop makes the ONE intake decision and ticks the claw and Scoring.
        // R1 RUNS THE ROLLERS WHENEVER IT IS HELD -- clamped or not, macro or no macro, from the press
        // until it is let go (the driver, 2026-10-09: "have r1 intake always"). It is not a mode any
        // more and it has no gate: the whole of the unclamped macro's approach, pivot wait and descent
        // now runs with the rollers going, which is the point of pressing it.
        // in.stepUpPressed is NOT set any more: L1 is a bare step up (see below), so it no longer opens
        // the auto-intake floor sweep -- R1 is the roller button and the driver holds it when they want
        // rollers. The sweep's code is still there and still works for auton's stepUp(). ---
        if (r1) in.intakeRequest = Intake::Mode::IN;
        else if (screen::rMode == screen::RMode::INTAKE && r2) in.intakeRequest = Intake::Mode::OUT;
        // L1 counts as a manual lift override, like the D-pad: it is the same plain-voltage path, and
        // the macro windows (the floor sweep, the L2 outtake) must let go of the intake when the
        // driver takes the arm over.
        in.manualLift = up || down || l1;
        in.liftButtonMode = screen::rMode == screen::RMode::LIFT;
        in.abort = bNew;
        in.scorePressed = l2New;

        // L1 HELD = LIFT UP, steady and fast, for as long as it is held (the driver, 2026-10-09: "l1
        // hold moves the arm up like our old r1 did when we had clamp active ... holds it after
        // release"). THIS IS THE OLD CLAMPED-MODE R1-UP, MOVED TO L1 AND 10% QUICKER: the identical
        // plain-voltage path through Lift::driverPower() that the D-pad uses, at config::L1_LIFT_UP_PCT
        // (99%, up from R1's 90). So it cancels any target or macro on the way in -- scoring.reset()
        // stops the state machine re-issuing a height; driverPower() itself cancels the lift's target
        // and any homing -- and letting go BRAKE-HOLDS exactly where it is, resuming nothing. It works
        // clamped or not, because L1 now just means "raise", and it is not clamped by lift_preset::MAX:
        // manual voltage never was (the arm's own hard stop at the top is the limit, same as the D-pad).
        // The D-pad still wins over it, and it wins over everything else -- including a score or the
        // automatic intake grab, which is what a manual override is for and exactly what the old R1-up
        // did from the same line of code.
        const bool l1Lift = l1 && !up && !down;  // D-pad wins
        if (l1Lift) {
            scoring.reset();
            lift.driverPower(config::L1_LIFT_UP_PCT / 100.0);
        } else if (!up && !down) {
            lift.driverPower(0.0);
        }
        // Clamped mode, R2 ONLY: the lift's own banded descent, which seats the arm on its hard stop
        // if it is held all the way down. It is a no-op the moment the claw unclamps, because
        // rModeFor() has already dropped the mode to INERT, and it gives way to L1 going up.
        if (rLift && r2 && !l1Lift) scoring.reset();
        lift.descendManual(rLift && r2 && !l1Lift);

        // --- Lift height presets / intake macro. Locked out while the manual override is held so a
        // tap here can never re-command the lift out from under a manual nudge. ---
        if (!up && !down) {
            // R1's new press ALSO starts the intake macro -- but only with the jaws empty. Holding a
            // pin, R1 is rollers and nothing else (goToIntake() refuses a clamped claw on its own
            // account too, interlock 4; this is the same rule said out loud at the call site).
            if (r1New && !claw.isClamped()) scoring.goToIntake();
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

        pros::delay(10);
    }
}
