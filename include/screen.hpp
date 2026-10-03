#pragma once
/**
 * Brain screen — four lines, and nothing else unless something is broken.
 *
 *   POSE  X 0.0  Y 0.0  T 0.0
 *   LIFT  31.0 -> 31.0  STACK 1  PARK  HOMED  R:INTAKE
 *   CLAW  CLAMPED  AC:HELD  PIV OUT
 *   DIST  L 152  R 148  BACK 300  CLAW 12 mm
 *
 * That is the whole readout (the driver's own choice, 2026-10-03: "display left right and back
 * distance sensor values to brain please, remove all temp and that random stuff"). The old tuning
 * block -- ROT, ANGLE, err/vel/out, the velocity reference, the learned gravity voltage, the L2
 * contact signals, the TUNE constants echo, motor temperatures and currents, the DRV heartbeat -- is
 * gone. Nothing is lost that mattered: every one of those numbers is in the LIFT_TRACE terminal
 * output at 20 ms resolution (VS Code -> PROS -> Brain Terminal; see README "Capturing a trace"),
 * which is a better place to read a moving number than a screen refreshed every 50 ms anyway.
 *
 * WARNING LINES appear only when a device is missing or is the wrong type -- one line per problem,
 * at most kMaxWarnings of them, under the four lines above:
 *
 *   !! IMU not connected (port 18)
 *
 * Everything the four lines show comes from the subsystems themselves; this header only carries the
 * few flags the driver loop has to hand over.
 */

#include <cstdint>

namespace screen {

/** What R1/R2 are doing right now, written by opcontrol every loop so the brain can show it:
 *  INTAKE = roller in/out (nothing clamped), LIFT = lift up/down (a pin is clamped), INERT = the
 *  clamp state changed while the button was held, so they do nothing until it is released. The
 *  safety rules in CLAUDE.md reference this token, so it stays on the screen. */
enum class RMode { IDLE, INTAKE, LIFT, INERT };
extern RMode rMode;

/** True once chassis.calibrate() has finished (it runs in the background). autonomous() waits on it,
 *  bounded; the POSE line says `calibrating...` until then. */
extern bool calibrated;

/** Start the background task that redraws the brain screen every 50 ms. Call once from initialize(). */
void start();

}  // namespace screen
