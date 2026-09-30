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
 * Button map (driver):
 *   Left stick Y / Right stick X  arcade drive
 *   L1 (hold)   lift up, full power
 *   L2 (hold)   lift down, full power
 *               let go of both = lift motors HOLD where they are
 *   R1 (hold)   intake in
 *   R2 (hold)   intake out
 *   A           claw clamp / open
 *   X           pivot in / out
 *
 * During driver control nothing else touches the lift: the PID task, presets, stall/runaway
 * guards and the scoring macros are all off (lift.setDirect(true)).
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
    lift.setDirect(false);  // autonomous may use the lift PID / presets
    scoring.reset();
}

void opcontrol() {
    // Driver control: the lift is plain motor power. No PID, no presets, no guards, no macros.
    lift.setDirect(true);
    intake.set(Intake::Mode::STOP);

    while (true) {
        // --- Drive ---
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // --- Lift: L1 up, L2 down, full power. Let go = motors HOLD in place. ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move(-127);
        } else {
            lift.move(0);
        }

        // --- Intake: R1 in, R2 out (hold) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intake.set(Intake::Mode::IN);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) intake.set(Intake::Mode::OUT);
        else intake.set(Intake::Mode::STOP);

        // --- Claw (A) and pivot (X) ---
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (claw.isClamped()) claw.disarm();  // open
            else claw.clamp();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) scoring.togglePivot();

        pros::delay(10);
    }
}
