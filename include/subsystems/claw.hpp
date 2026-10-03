#pragma once
/**
 * Claw — pneumatic claw with autoclamp.
 *
 * Why autoclamp: the driver has to line the claw up with a pin AND press the
 * clamp button at the right moment. A distance sensor between the jaws can see
 * the pin arrive and fire the cylinder the instant it is in place, which saves
 * reaction time on every cycle and stops early "air grabs".
 *
 * State machine:
 *
 *   OPEN ──(arm)──> ARMED ──(pin seen for DEBOUNCE ms)──> CLAMPED
 *    ^ (power-on)     ^                                      │
 *    │                └──(lockout over)── RELEASED <──(release)
 *    └──(disarm)───────────────────────────────────────────────
 *
 *   OPEN     jaws open, autoclamp DISABLED (driver in full manual control). This is the power-on
 *            state, because the cylinder's unpowered position IS open -- see below.
 *   ARMED    jaws open, autoclamp enabled, watching the sensor through AutoclampDetector
 *   CLAMPED  jaws closed on a pin (L1, A, the autoclamp, or an auton call)
 *   RELEASED jaws just opened with the autoclamp still enabled. It may NOT re-arm until the sensor
 *            has read CLEAR (further than AUTOCLAMP_REARM_MM for AUTOCLAMP_REARM_CLEAR_MS) and
 *            CLAW_RELEASE_LOCKOUT_MS has passed -- then -> ARMED. The clear-reading half is the
 *            important one: the pin just let go of is still millimetres from the sensor, so a
 *            timer-only re-arm (which is what this used to be) re-grabs it every single time and no
 *            score can ever finish. -> OPEN instead if the autoclamp has since been switched off.
 *
 * THE AUTOCLAMP RULE itself is not here: it is subsystems::AutoclampDetector (its own header), which
 * is PURE -- no sensor, no clock, no cylinder -- and host-tested in test/autoclamp_detector_test.cpp,
 * the same arrangement as the lift's ContactDetector. This class reads the sensor once per tick,
 * hands it over, and owns the cylinder. Every threshold is in config.hpp's AUTOCLAMP_* block. In
 * short: something at or inside AUTOCLAMP_RANGE_MM, continuously for AUTOCLAMP_CONFIRM_MS, over at
 * least AUTOCLAMP_MIN_SAMPLES fresh readings -> clamp. On in driver by default, off in auton until a
 * route asks, off at power-on, and inhibited (setInhibit()) for the whole of an L2 score.
 *
 * THE CLAW IS ON ADI config::CLAW_PISTON_PORT, which is 'D'. Until 2026-10-01 the code had the claw
 * and the pivot on each other's ports; the derivation of which level closes the jaws
 * (config::CLAW_CLOSED_IS_ADI_HIGH, true on this robot: the jaws need air to grip, so the unpowered
 * LOW level is OPEN) is written out in full in config.hpp. At power-on the pin is written to the level
 * it already sits at, so nothing moves at startup. There is deliberately no "shut at power-on" state
 * any more -- unpowered is open, so power-on is State::OPEN, and R1 straight after power-on works
 * without anything having to special-case it. The brain's CLAW line shows the state, the port letter
 * and the raw level, so one press of A confirms the constant.
 */

#include "config.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "subsystems/autoclamp_detector.hpp"

#include <cstdint>

namespace subsystems {

class Claw {
  public:
    enum class State { OPEN, ARMED, CLAMPED, RELEASED };

    Claw();

    /** Turn autoclamp ON. The jaws open (if they weren't already) and the detector starts watching:
     *  OPEN -> ARMED. Called from RELEASED it only sets the enable -- that state has to see the sensor
     *  read CLEAR first (see the re-arm note in autoclamp_detector.hpp), and then arms itself. A no-op
     *  while CLAMPED: it never opens a claw that is holding something. */
    void arm();
    /** Turn autoclamp OFF; the jaws open and stay open (-> OPEN). */
    void disarm();
    /** Is the autoclamp enabled? (Y toggles it; opcontrol turns it on, auton leaves it off.) */
    bool autoclampEnabled() const { return enabled_; }
    /** Block the autoclamp from firing this tick without disabling it -- used while an L2 score
     *  sequence owns the claw, so a stack it is pressing into can never trigger a clamp. Pass it
     *  every tick (periodic::tick() does). */
    void setInhibit(bool inhibit) { inhibit_ = inhibit; }
    /** Driver override: close now, no sensor needed (-> CLAMPED). */
    void clamp();
    /** Open the jaws (CLAMPED -> RELEASED, starts the lockout). */
    void release();
    /** One button for the driver: open if clamped, clamp if open. */
    void toggle();

    /** Run one step of the state machine. Call every loop (10 ms). */
    void update();

    State getState() const { return state_; }
    /** True when the jaws were closed ON PURPOSE (L1 / A / autoclamp): a pin is presumed held. */
    bool isClamped() const { return state_ == State::CLAMPED; }
    /** True whenever the jaws are physically shut. Unpowered is OPEN on this robot, so this is the
     *  same thing as isClamped() -- both are kept because they mean different things to a reader. */
    bool isClosed() const { return state_ == State::CLAMPED; }
    /** The raw level the claw's ADI pin was last driven to (for the brain's CLAW line). */
    bool adiHigh() const { return level_; }
    /** True if the sensor currently reads something at or inside config::AUTOCLAMP_RANGE_MM. This is
     *  the RAW look, with no debounce at all -- the debounce lives in AutoclampDetector. */
    bool seesPin();
    /** What the autoclamp is doing, for the brain's CLAW line: "OFF" (disabled, or the jaws are shut
     *  by hand), "ARMED" (watching, nothing seen), "SEEING" (something in range, the confirm window is
     *  running), "CLEAR?" (disarmed after an open, waiting for a clear reading before re-arming), or
     *  "HELD" (clamped). */
    const char* autoclampStateName() const;
    /** The last reading the detector accepted as a real object, mm (-1 if there hasn't been one). */
    int getAutoclampMm() const { return detect_.lastValidMm(); }
    /** Raw distance sensor reading in mm (large value / error when nothing is in range or it's unplugged). */
    int getDistanceMm() { return sensor_.get_distance(); }
    int getDistanceConfidence() { return sensor_.get_confidence(); }
    const char* stateName() const;

  private:
    void setJaws(bool closed);
    static bool levelFor(bool closed);
    /** The AUTOCLAMP_* block from config.hpp, packed for the pure detector. */
    static AutoclampDetector::Config detectorConfig();

    pros::adi::DigitalOut piston_;
    pros::Distance sensor_;
    AutoclampDetector detect_;       // the debounce + re-arm rule, pure (see its header)
    State state_ = State::OPEN;      // unpowered = open, see the class comment
    bool enabled_ = config::CLAW_AUTOCLAMP_AT_START;  // autoclamp on/off (Y, opcontrol, auton)
    bool inhibit_ = false;           // something else owns the claw this tick (an L2 score)
    bool level_ = false;             // the level the pin is actually driven to, for the brain screen
    std::uint32_t releasedAt_ = 0;   // when the jaws were last opened
};

}  // namespace subsystems
