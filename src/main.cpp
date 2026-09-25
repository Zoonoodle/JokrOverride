#include "main.h"

#include "config.hpp"
#include "robot.hpp"

using namespace robot;
namespace preset = config::lift_preset;

/**
 * Button map (driver):
 *   Left stick Y / Right stick X  arcade drive
 *   R1          score HIGH  (only works while holding a pin)
 *   R2          score LOW
 *   L1          drop the pin (only once the lift has settled at the goal)
 *   L2          claw manual override: clamp / release
 *   A           cancel -> back to intaking
 *   Up / Down   lift manual override (hold)
 *   B           intake reverse (hold)
 *   Y           autoclamp on / off
 */

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();  // IMU + odometry. Keep the robot still.
    lift.zero();          // the lift must start fully down
    lift.startTask();

    // Brain screen: live debug values for tuning and for notebook screenshots.
    pros::Task screen([] {
        while (true) {
            const lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X %.1f  Y %.1f  H %.1f", pose.x, pose.y, pose.theta);
            pros::lcd::print(1, "Lift %.1f / %.1f deg %s", lift.getPosition(), lift.getTarget(),
                             lift.isSettled() ? "SETTLED" : "");
            pros::lcd::print(2, "Claw %s", claw.stateName());
            pros::lcd::print(3, "Scoring %s", scoring.stateName());
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // TODO(team): autonomous routines. Example of a LemLib move:
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 24, 2000);
    scoring.begin();
}

void opcontrol() {
    scoring.begin();
    bool autoclampOn = true;

    while (true) {
        // --- Drive ---
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // --- Pivot / scoring sequence ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) scoring.requestScore(preset::SCORE_HIGH);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) scoring.requestScore(preset::SCORE_LOW);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) scoring.release();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) scoring.cancel();

        // --- Claw: manual override + autoclamp switch ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) claw.toggle();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            autoclampOn = !autoclampOn;
            if (autoclampOn) claw.arm();
            else claw.disarm();
        }

        // --- Lift manual override (PID holds wherever you let go) ---
        double liftInput = 0.0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) liftInput = 1.0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) liftInput = -1.0;
        lift.manual(liftInput);

        // --- Intake reverse (unjam) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) intake.set(subsystems::Intake::Mode::OUT);
        else if (intake.getMode() == subsystems::Intake::Mode::OUT)
            intake.set(scoring.getState() == subsystems::Scoring::State::INTAKING ? subsystems::Intake::Mode::IN
                                                                                  : subsystems::Intake::Mode::STOP);

        claw.update();
        scoring.update();
        pros::delay(10);
    }
}
