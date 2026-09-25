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
 *     ^               ^                                     │
 *     │               └──(lockout over)── RELEASED <──(release)┘
 *     └──(disarm)──────────────────────────────────────────────
 *
 *   OPEN     jaws open, autoclamp off (driver in full manual control)
 *   ARMED    jaws open, watching the sensor
 *   CLAMPED  jaws closed on a pin
 *   RELEASED jaws just opened; autoclamp is locked out for RELEASE_LOCKOUT ms
 *            so the pin we just let go of is not grabbed again, then -> ARMED
 */

#include "pros/adi.hpp"
#include "pros/distance.hpp"

#include <cstdint>

namespace subsystems {

class Claw {
  public:
    enum class State { OPEN, ARMED, CLAMPED, RELEASED };

    Claw();

    /** Turn autoclamp on (OPEN -> ARMED). */
    void arm();
    /** Turn autoclamp off; the jaws open and stay open (-> OPEN). */
    void disarm();
    /** Driver override: close now, no sensor needed (-> CLAMPED). */
    void clamp();
    /** Open the jaws (CLAMPED -> RELEASED, starts the lockout). */
    void release();
    /** One button for the driver: clamp if open, release if clamped. */
    void toggle();

    /** Run one step of the state machine. Call every loop (10 ms). */
    void update();

    State getState() const { return state_; }
    bool isClamped() const { return state_ == State::CLAMPED; }
    /** True if the sensor currently sees something between the jaws. */
    bool seesPin();
    const char* stateName() const;

  private:
    void setJaws(bool closed);

    pros::adi::Pneumatics piston_;
    pros::Distance sensor_;
    State state_ = State::OPEN;
    std::uint32_t seenSince_ = 0;    // when the pin was first seen (0 = not seen)
    std::uint32_t releasedAt_ = 0;   // when the jaws were last opened
};

}  // namespace subsystems
