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
 *   OPEN     jaws open, autoclamp off (driver in full manual control). This is the power-on state,
 *            because the cylinder's unpowered position IS open -- see below.
 *   ARMED    jaws open, watching the sensor
 *   CLAMPED  jaws closed on a pin (L1, A, or the autoclamp)
 *   RELEASED jaws just opened; autoclamp is locked out for RELEASE_LOCKOUT ms
 *            so the pin we just let go of is not grabbed again, then -> ARMED
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

#include "pros/adi.hpp"
#include "pros/distance.hpp"

#include <cstdint>

namespace subsystems {

class Claw {
  public:
    enum class State { OPEN, ARMED, CLAMPED, RELEASED };

    Claw();

    /** Turn autoclamp on (OPEN -> ARMED, jaws open). */
    void arm();
    /** Turn autoclamp off; the jaws open and stay open (-> OPEN). */
    void disarm();
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
    /** True if the sensor currently sees something between the jaws. */
    bool seesPin();
    /** Raw distance sensor reading in mm (large value / error when nothing is in range or it's unplugged). */
    int getDistanceMm() { return sensor_.get_distance(); }
    int getDistanceConfidence() { return sensor_.get_confidence(); }
    const char* stateName() const;

  private:
    void setJaws(bool closed);
    static bool levelFor(bool closed);

    pros::adi::DigitalOut piston_;
    pros::Distance sensor_;
    State state_ = State::OPEN;      // unpowered = open, see the class comment
    bool level_ = false;             // the level the pin is actually driven to, for the brain screen
    std::uint32_t seenSince_ = 0;    // when the pin was first seen (0 = not seen)
    std::uint32_t releasedAt_ = 0;   // when the jaws were last opened
};

}  // namespace subsystems
