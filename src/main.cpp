#include "main.h"

#include "config.hpp"
#include "robot.hpp"
#include "screen.hpp"


using namespace robot;
using subsystems::Intake;
using subsystems::Scoring;

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
 *               tap after goes up one level, staying at the top once reached)
 *   L2          intake macro: SCORE (open the jaws and lower all the way to DOWN in one motion,
 *               pivot stays OUT) and reset the level count so the next L1 starts at level 1 again
 *   R1 (hold)   intake roller in; (new-press) also sends the lift to the INTAKE handoff position
 *               with the jaws open and the pivot BACK (a no-op while a pin is CLAMPED)
 *   R2 (hold)   intake roller out
 *   B           stow: pivot OUT, lift straight down to DOWN (0 deg), cancel the macro
 *   A           claw open / clamp (the claw is physically shut at power-on; the CLAW line shows
 *               CLOSED[0], and A opens it)
 *   X           pivot OUT / BACK (the brain's CLAW line shows which, and the raw ADI level)
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
        } else {
            lift.driverPower(0.0);
        }

        // --- Intake roller: R1 in, R2 out (hold) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intake.set(Intake::Mode::IN);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) intake.set(Intake::Mode::OUT);
        else intake.set(Intake::Mode::STOP);

        // --- Lift height presets / intake macro. Locked out while the manual override is held so a
        // tap here can never re-command the lift out from under a manual nudge. ---
        if (!up && !down) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) scoring.goToIntake();
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) scoring.stepUp();
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) scoring.score();
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) scoring.stow();  // pivot OUT + lift to DOWN
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
