#pragma once
/**
 * Brain screen — live sensor readout while the program runs (disabled, auton and driver).
 * Unplugged or wrong devices show which port to check instead of garbage numbers.
 *
 * Drawn with pros::screen (not the 8-line pros::lcd console) so there's room for a full lift
 * tuning section (position, target, error, output, P/I/D terms, active preset, settled/blocked/
 * fault, gains, temp and current) alongside the existing pose/IMU/odom/distance readouts, plus an
 * ANGLE line showing the rotation sensor's live raw absolute angle and the one-time startup seed
 * (see Lift::zero()) so the sensor's own calibrated zero can be checked by eye.
 */

#include <cstdint>

namespace screen {
/** Written by opcontrol every loop so the brain shows the driver loop is alive and what it sees. */
struct DriverDebug {
    std::uint32_t loops = 0;  // 0 = opcontrol has not started
    int forward = 0, turn = 0;
    bool up = false, down = false;  // D-pad Up/Down, the lift's manual override
};
extern DriverDebug driverDebug;
/** What R1/R2 are doing right now, written by opcontrol every loop so the brain can show it:
 *  INTAKE = roller in/out (nothing clamped), LIFT = lift up/down (a pin is clamped), INERT = the
 *  clamp state changed while the button was held, so they do nothing until it is released. */
enum class RMode { IDLE, INTAKE, LIFT, INERT };
extern RMode rMode;
/** True once chassis.calibrate() has finished (it runs in the background). */
extern bool calibrated;

/** Start the background task that redraws the brain screen every 50 ms. Call once from initialize(). */
void start();
}  // namespace screen
