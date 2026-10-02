#include "subsystems/claw.hpp"

#include "config.hpp"
#include "pros/rtos.hpp"

namespace subsystems {

bool Claw::levelFor(bool closed) { return closed == config::CLAW_CLOSED_IS_ADI_HIGH; }

Claw::Claw()
    // The jaws are physically OPEN at power-on: the claw needs air to grip, so the cylinder's
    // unpowered position is open (see the derivation on CLAW_CLOSED_IS_ADI_HIGH in config.hpp). Start
    // in State::OPEN and write the pin to the level OPEN means -- with CLAW_CLOSED_IS_ADI_HIGH == true
    // that is LOW, the level the ADI already sits at, so nothing moves at startup (CLAUDE.md rule 1).
    : piston_(static_cast<std::uint8_t>(config::CLAW_PISTON_PORT), levelFor(false)),
      sensor_(config::CLAW_DISTANCE_PORT),
      level_(levelFor(false)) {}

void Claw::setJaws(bool closed) {
    level_ = levelFor(closed);
    piston_.set_value(level_ ? HIGH : LOW);
}

bool Claw::seesPin() {
    const int mm = sensor_.get_distance();
    // The sensor returns a large value (or error) when nothing is in range.
    return mm > 0 && mm <= config::CLAW_DETECT_MM;
}

void Claw::arm() {
    if (state_ == State::OPEN) {
        setJaws(false);
        seenSince_ = 0;
        state_ = State::ARMED;
    }
}

void Claw::disarm() {
    setJaws(false);
    state_ = State::OPEN;
}

void Claw::clamp() {
    setJaws(true);
    state_ = State::CLAMPED;
}

void Claw::release() {
    setJaws(false);
    releasedAt_ = pros::millis();
    state_ = State::RELEASED;
}

void Claw::toggle() {
    if (isClosed()) release();
    else clamp();
}

void Claw::update() {
    const std::uint32_t now = pros::millis();
    switch (state_) {
        case State::OPEN:
        case State::CLAMPED:
            break;  // nothing automatic happens here; the driver or the scoring sequence decides

        case State::ARMED:
            // Debounce: the pin must stay in view for DEBOUNCE ms. One noisy reading won't fire the claw.
            if (seesPin()) {
                if (seenSince_ == 0) seenSince_ = now;
                if (now - seenSince_ >= static_cast<std::uint32_t>(config::CLAW_DETECT_DEBOUNCE_MS)) clamp();
            } else {
                seenSince_ = 0;
            }
            break;

        case State::RELEASED:
            // Lockout: ignore the sensor until the released pin has had time to leave the jaws.
            if (now - releasedAt_ >= static_cast<std::uint32_t>(config::CLAW_RELEASE_LOCKOUT_MS)) {
                seenSince_ = 0;
                state_ = State::ARMED;
            }
            break;
    }
}

const char* Claw::stateName() const {
    switch (state_) {
        case State::OPEN: return "OPEN";
        case State::ARMED: return "ARMED";
        case State::CLAMPED: return "CLAMPED";
        case State::RELEASED: return "RELEASED";
    }
    return "?";
}

}  // namespace subsystems
