#pragma once
/**
 * Brain screen — live sensor readout while the program runs (disabled, auton and driver).
 * Unplugged or wrong devices show which port to check instead of garbage numbers.
 */

namespace screen {
/** Start the background task that redraws the brain screen every 50 ms. Call once from initialize(). */
void start();
}  // namespace screen
