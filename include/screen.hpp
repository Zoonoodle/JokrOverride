#pragma once
/**
 * Brain screen — live sensor readout while the program runs (disabled, auton and driver).
 * Unplugged or wrong devices show which port to check instead of garbage numbers.
 */

#include <cstdint>

namespace screen {
/** Written by opcontrol every loop so the brain shows the driver loop is alive and what it sees. */
struct DriverDebug {
    std::uint32_t loops = 0;  // 0 = opcontrol has not started
    int forward = 0, turn = 0;
    bool l1 = false, l2 = false;
};
extern DriverDebug driverDebug;
/** True once chassis.calibrate() has finished (it runs in the background). */
extern bool calibrated;

/** Start the background task that redraws the brain screen every 50 ms. Call once from initialize(). */
void start();
}  // namespace screen
