#include "main.h"

#include "config.hpp"
#include "robot.hpp"
#include "screen.hpp"


using namespace robot;
using subsystems::Intake;
using subsystems::Scoring;

/**
 * Button map (driver). Full walkthrough in README "Driver guide".
 *   Left stick Y / Right stick X  arcade drive
 *   L1 (hold)   lift up, full power, same feel as plain motor power
 *   L2 (hold)   lift down, full power
 *               Pressing either immediately cancels the intake macro / any preset target and
 *               takes manual control. Letting go brakes and holds right there -- it does NOT
 *               resume whatever preset or macro step was active before.
 *   R1 (hold)   intake in
 *   R2 (hold)   intake out
 *   Up          intake macro: go to the INTAKE handoff position
 *   Left        intake macro: clamp + stack (tap again to go up a level, max 7)
 *   Down        intake macro: SCORE if holding a pin, else go to MATCHLOAD
 *   Right       intake macro: back to stack level 1
 *   B           stow: lift straight down to DOWN (0 deg) and cancel the macro
 *   A           claw clamp / open
 *   X           pivot in / out
 *   Y           autoclamp on / off
 *
 * The lift PID task (subsystems/lift.*) starts once in initialize() and runs for the whole
 * program; it is the only code that ever writes to the lift motors (see the class comment in
 * include/subsystems/lift.hpp for why that matters). Up/Left/Down/Right/B and the intake macro
 * (subsystems/scoring.*) all command it through lift.setTarget()/moveTo() -- never raw voltage.
 * L1/L2 only ever reach the lift through lift.driverPower(), which preempts the PID from inside
 * that same task. Nothing else touches the motors.
 */

void initialize() {
    screen::start();  // brain readout first: if you only see the program name, initialize() never ran

    // Startup signal: if the controller doesn't buzz once and show "PROGRAM ON" at power-up,
    // the controller isn't linked to the brain, or this program isn't the one running.
    master.rumble(".");
    master.print(0, 0, "PROGRAM ON      ");

    // The lift must start fully down (0 = lift_preset::DOWN). zero() only tares the sensor/encoders --
    // it doesn't move anything. The PID task starts idle: no target is active until a preset button,
    // the intake macro, or autonomous explicitly commands a height (see lift.hpp).
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
    while (!screen::calibrated) pros::delay(10);  // odometry must be ready before moving
    scoring.reset();
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

    while (true) {
        // --- Drive ---
        const int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(forward, turn);

        // --- Lift: L1 up / L2 down, full power, plain manual control (see button map above).
        // Holding either cancels the intake macro / any preset target so they can never fight over
        // the motors; scoring.reset() below just stops the macro from re-issuing lift commands, it
        // doesn't touch the claw or pivot where they physically are. ---
        const bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        if (l1) {
            scoring.reset();
            lift.driverPower(1.0);
        } else if (l2) {
            scoring.reset();
            lift.driverPower(-1.0);
        } else {
            lift.driverPower(0.0);
        }

        // --- Intake: R1 in, R2 out (hold) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intake.set(Intake::Mode::IN);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) intake.set(Intake::Mode::OUT);
        else intake.set(Intake::Mode::STOP);

        // --- Lift height presets / intake macro. Locked out while L1/L2 are held so a tap here
        // can never re-command the lift out from under a manual nudge. ---
        if (!l1 && !l2) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) scoring.goToIntake();
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) scoring.stepUp();
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) scoring.scoreOrMatchload();
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) scoring.resetLevel();
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                scoring.reset();
                lift.moveTo(config::lift_preset::DOWN);
            }
        }

        // --- Claw (A), pivot (X), autoclamp (Y) ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) scoring.toggleClaw();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) scoring.togglePivot();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) scoring.setAutoclamp(!scoring.autoclampOn());

        claw.update();
        scoring.update();

        // --- Brain "DRV" line: proves this loop is running and what it reads ---
        screen::driverDebug = {screen::driverDebug.loops + 1, forward, turn, l1, l2};

        pros::delay(10);
    }
}
