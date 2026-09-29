#include "main.h"

#include "config.hpp"
#include "robot.hpp"
#include "screen.hpp"

#include <cstdio>
#include <string>

using namespace robot;
using subsystems::Intake;
using subsystems::Scoring;

/**
 * Button map (driver). Full walkthrough in README "Driver guide".
 *   Left stick Y / Right stick X  arcade drive
 *   R1 (hold)   intake in. First press also sends the claw to the INTAKE position (open)
 *   L1 (tap)    clamp + lift to stack level 1; each extra tap = one level higher (max 7)
 *   L2 (tap)    holding a cup: SCORE (open claw, arm lowers slowly to INTAKE)
 *               no cup: claw goes to the MATCHLOAD position (open)
 *   R2          holding a cup: tap = back to stack level 1
 *               no cup: hold = intake out (extake)
 *   B (hold)    intake out, always (unjam)
 *   Up / Down   lift manual fine-tune (hold); the lift holds where you let go
 *   A           manual claw clamp / open
 *   Y           autoclamp on / off
 */

void initialize() {
    screen::start();      // live sensor readout on the brain (see src/screen.cpp)
    chassis.calibrate();  // IMU + odometry. Keep the robot still.
    lift.zero();          // the lift must start fully down
    lift.startTask();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // TODO(team): autonomous routines. Example of a LemLib move:
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 24, 2000);
    scoring.reset();
}

void opcontrol() {
    // Nothing moves until the driver presses a button.
    scoring.reset();
    intake.set(Intake::Mode::STOP);
    lift.manual(0);

    std::string lastScreen;
    std::uint32_t lastScreenAt = 0;
    bool faultRumbled = false;

    while (true) {
        // --- Drive ---
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        const bool holdingCup = claw.isClamped();

        // --- Lift / claw sequence ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) scoring.goToIntake();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) scoring.stepUp();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) scoring.scoreOrMatchload();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) scoring.resetLevel();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) scoring.toggleClaw();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) scoring.setAutoclamp(!scoring.autoclampOn());

        // --- Intake: only ever runs while a button is held ---
        const bool in = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const bool out = master.get_digital(pros::E_CONTROLLER_DIGITAL_B) ||
                         (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !holdingCup);
        if (in) intake.set(Intake::Mode::IN);
        else if (out) intake.set(Intake::Mode::OUT);
        else intake.set(Intake::Mode::STOP);

        // --- Lift manual fine-tune (PID holds wherever you let go) ---
        double liftInput = 0.0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) liftInput = 1.0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) liftInput = -1.0;
        lift.manual(liftInput);

        claw.update();
        scoring.update();

        // --- Controller screen: level + state, so the driver knows which tap they're on ---
        const std::uint32_t now = pros::millis();
        if (now - lastScreenAt >= 100) {
            char buf[20];
            if (lift.isFaulted()) std::snprintf(buf, sizeof(buf), "LIFT FAULT       ");
            else if (scoring.getLevel() > 0) std::snprintf(buf, sizeof(buf), "LVL %d  %-10s", scoring.getLevel(),
                                                            lift.isBlocked() ? "BLOCKED" : scoring.stateName());
            else std::snprintf(buf, sizeof(buf), "%-16s", scoring.stateName());
            if (buf != lastScreen) {
                master.print(0, 0, "%s", buf);
                lastScreen = buf;
            }
            lastScreenAt = now;
        }
        if (lift.isFaulted() && !faultRumbled) {
            master.rumble("---");
            faultRumbled = true;
        }
        if (!lift.isFaulted()) faultRumbled = false;

        pros::delay(10);
    }
}
