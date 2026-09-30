#include "main.h"

#include "config.hpp"
#include "robot.hpp"
#include "screen.hpp"


using namespace robot;
using subsystems::Intake;
using subsystems::Scoring;

// Driver-control lift: plain motor group, driven only by L1/L2 in opcontrol.
pros::MotorGroup liftMotors({-13, 20}, pros::v5::MotorGears::red);  // keep in sync with config::LIFT_PORTS

/**
 * Button map (driver):
 *   Left stick Y / Right stick X  arcade drive
 *   L1 (hold)   lift up, full power
 *   L2 (hold)   lift down, full power     (neither = lift motors stop and HOLD)
 *   R1 (hold)   intake in
 *   R2 (hold)   intake out
 *   A           claw clamp / open
 *   X           pivot in / out
 *
 * During driver control nothing else touches the lift: the lift PID task is off
 * (lift.setDirect(true)) and the scoring macros aren't called.
 */

void initialize() {
    // Startup signal: if the controller doesn't buzz once and show "PROGRAM ON" at power-up,
    // the controller isn't linked to the brain, or this program isn't the one running.
    master.rumble(".");
    master.print(0, 0, "PROGRAM ON      ");

    screen::start();  // brain readout, incl. "DRV" line showing the driver loop is alive
    liftMotors.set_brake_mode_all(pros::v5::MotorBrake::hold);

    // Calibrate in the background so nothing here can delay driver control.
    // Only calibrate the IMU if an IMU is really on IMU_PORT (port 8 is also a drive motor).
    static pros::Task calibrate([] {
        const bool imuPresent = pros::Device::get_plugged_type(config::IMU_PORT) == pros::DeviceType::imu;
        chassis.calibrate(imuPresent);
        screen::calibrated = true;
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    while (!screen::calibrated) pros::delay(10);  // odometry must be ready before moving
    lift.zero();          // lift must start fully down
    lift.startTask();     // PID only runs in autonomous
    lift.setDirect(false);
    scoring.reset();
    // TODO(team): autonomous routines. Example of a LemLib move:
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 24, 2000);
}

void opcontrol() {
    lift.setDirect(true);  // if autonomous started the lift PID, it stops touching the motors now
    master.print(0, 0, "DRIVER ON       ");

    while (true) {
        // --- Drive ---
        const int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(forward, turn);

        // --- Lift: L1 up, L2 down, full power. Nothing else touches the lift. ---
        const bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        if (l1) {
            liftMotors.move(127);
        } else if (l2) {
            liftMotors.move(-127);
        } else {
            liftMotors.move(0);
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

        // --- Brain "DRV" line: proves this loop is running and what it reads ---
        screen::driverDebug = {screen::driverDebug.loops + 1, forward, turn, l1, l2};

        pros::delay(10);
    }
}
