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
      level_(levelFor(false)) {
    // Start the autoclamp detector in its "waiting for a clear reading" state, so a robot powered on
    // with a pin already sitting in the open jaws cannot fire the cylinder the first time update()
    // runs (CLAUDE.md safety rule 1). It is disabled at power-on anyway (CLAW_AUTOCLAMP_AT_START).
    detect_.reset(true);
}

/** The AUTOCLAMP_* block in config.hpp, in the pure detector's own words. FOUR of the numbers are not
 *  fixed: at the R1 INTAKE handoff (setIntakeMode()) the trigger range, its matching clear threshold and
 *  the confirm window all swap for the AUTOCLAMP_INTAKE_* pair -- a longer reach (75 mm against 50) and
 *  no debounce at all, because there the intake pushes the pin ACROSS the jaws and the claw has to
 *  commit while it is still arriving. The detector's own code is identical either way.
 *
 *  THE RANGE AND THE CLEAR THRESHOLD MOVE TOGETHER, ALWAYS. Re-arming wants a reading FURTHER than
 *  rearmMm and clamping wants one at or inside rangeMm, so a rearmMm at or below rangeMm makes anything
 *  between them read as clear AND in range -- the autoclamp re-arms and instantly re-grabs what it just
 *  released, forever. That is why there is a separate AUTOCLAMP_INTAKE_REARM_MM rather than one shared
 *  clear threshold: 80 is correct above a 50 mm range and WRONG above a 75 mm one. */
AutoclampDetector::Config Claw::detectorConfig() const {
    const int rangeMm = intakeMode_ ? config::AUTOCLAMP_INTAKE_RANGE_MM : config::AUTOCLAMP_RANGE_MM;
    const int rearmMm = intakeMode_ ? config::AUTOCLAMP_INTAKE_REARM_MM : config::AUTOCLAMP_REARM_MM;
    const int confirmMs = intakeMode_ ? config::AUTOCLAMP_INTAKE_CONFIRM_MS : config::AUTOCLAMP_CONFIRM_MS;
    const int minSamples =
        intakeMode_ ? config::AUTOCLAMP_INTAKE_MIN_SAMPLES : config::AUTOCLAMP_MIN_SAMPLES;
    static_assert(config::AUTOCLAMP_REARM_MM > config::AUTOCLAMP_RANGE_MM,
                  "AUTOCLAMP_REARM_MM must stay ABOVE AUTOCLAMP_RANGE_MM -- see the note above: equal or "
                  "below makes the autoclamp re-grab whatever it just released, forever.");
    static_assert(config::AUTOCLAMP_INTAKE_REARM_MM > config::AUTOCLAMP_INTAKE_RANGE_MM,
                  "AUTOCLAMP_INTAKE_REARM_MM must stay ABOVE AUTOCLAMP_INTAKE_RANGE_MM, same reason.");
    return AutoclampDetector::Config{
        rangeMm,                          confirmMs,
        minSamples,                       config::AUTOCLAMP_STALE_GRACE_MS,
        rearmMm,                          config::AUTOCLAMP_REARM_CLEAR_MS,
        config::AUTOCLAMP_MAX_VALID_MM,   config::AUTOCLAMP_MIN_VALID_MM,
        config::AUTOCLAMP_MIN_CONFIDENCE,
    };
}

void Claw::setJaws(bool closed) {
    level_ = levelFor(closed);
    piston_.set_value(level_ ? HIGH : LOW);
}

bool Claw::seesPin() {
    const int mm = sensor_.get_distance();
    // Raw look, no debounce: valid reading, something really there, inside WHICHEVER range is in force
    // (the handoff's longer reach while intakeMode_ is set -- see detectorConfig()).
    const int rangeMm = intakeMode_ ? config::AUTOCLAMP_INTAKE_RANGE_MM : config::AUTOCLAMP_RANGE_MM;
    return mm >= config::AUTOCLAMP_MIN_VALID_MM && mm < config::AUTOCLAMP_MAX_VALID_MM && mm <= rangeMm;
}

void Claw::arm() {
    enabled_ = true;
    if (state_ == State::CLAMPED) return;  // never open a claw that is holding something
    if (state_ == State::RELEASED) return;  // it re-arms itself once the sensor reads clear
    setJaws(false);
    detect_.reset(false);  // armed fresh with empty jaws: no clear-reading wait needed
    state_ = State::ARMED;
}

void Claw::disarm() {
    enabled_ = false;
    setJaws(false);
    detect_.reset(true);
    state_ = State::OPEN;
}

void Claw::clamp() { clampFrom(ClampSource::MANUAL); }

void Claw::clampFrom(ClampSource source) {
    // Only a transition into CLAMPED is an EVENT. Clamping an already-clamped claw (L1's grab asking
    // for jaws the autoclamp already shut, a re-tap of A) must not raise a second one, or the arm would
    // get a second clamp-rise for one grip.
    const bool wasClamped = state_ == State::CLAMPED;
    setJaws(true);
    // Whatever is in the jaws now is NOT a reason to clamp again later: the next open starts from
    // "wait for a clear reading" (see autoclamp_detector.hpp).
    detect_.reset(true);
    state_ = State::CLAMPED;
    if (!wasClamped) clampEvent_ = source;
}

void Claw::release() {
    setJaws(false);
    releasedAt_ = pros::millis();
    detect_.reset(true);  // the pin we just let go of is still millimetres away -- wait for clear
    state_ = State::RELEASED;
}

void Claw::toggle() {
    if (isClosed()) release();
    else clamp();
}

void Claw::update() {
    const std::uint32_t now = pros::millis();
    // One sensor read per tick, shared by both states that care. The detector is PURE -- it is handed
    // the raw numbers and the clock and decides on its own (see autoclamp_detector.hpp).
    const int mm = sensor_.get_distance();
    const int confidence = sensor_.get_confidence();
    const AutoclampDetector::Config cfg = detectorConfig();

    switch (state_) {
        case State::OPEN:
        case State::CLAMPED:
            break;  // autoclamp off, or already holding: the driver / the scoring sequence decides

        case State::ARMED:
            // THE AUTOCLAMP. Something at or inside AUTOCLAMP_RANGE_MM, continuously for
            // AUTOCLAMP_CONFIRM_MS, over at least AUTOCLAMP_MIN_SAMPLES fresh readings -> grip it. A
            // spike that comes and goes resets the window and can never fire. inhibit_ (an L2 score
            // sequence) keeps the window at zero.
            // AT THE R1 HANDOFF this window is AUTOCLAMP_INTAKE_CONFIRM_MS / _MIN_SAMPLES (see
            // detectorConfig()), i.e. the first valid in-range reading fires. Everywhere else it is the
            // ordinary debounce. Either way Scoring is told it was the AUTOCLAMP that shut the jaws, so
            // it can push DOWN at the handoff and hop UP anywhere else.
            if (detect_.step(now, mm, confidence, cfg, inhibit_)) clampFrom(ClampSource::AUTOCLAMP);
            break;

        case State::RELEASED: {
            // RE-ARM, deliberately NOT a plain timer: the pin that was just let go of is still right
            // there, so the detector must see a genuinely CLEAR reading (further than
            // AUTOCLAMP_REARM_MM for AUTOCLAMP_REARM_CLEAR_MS) before the autoclamp may look for
            // anything again -- otherwise it re-grabs what it just put down, every time. The old
            // CLAW_RELEASE_LOCKOUT_MS is kept on top of that as a floor, so the jaws also get time to
            // physically swing open past the sensor.
            detect_.step(now, mm, confidence, cfg, inhibit_);
            const bool lockoutOver =
                now - releasedAt_ >= static_cast<std::uint32_t>(config::CLAW_RELEASE_LOCKOUT_MS);
            if (lockoutOver && detect_.state() == AutoclampDetector::State::ARMED) {
                // Autoclamp off? Then a released claw just becomes a plain open one -- it must NOT
                // silently end up ARMED, which is what the old version did unconditionally.
                state_ = enabled_ ? State::ARMED : State::OPEN;
            }
            break;
        }
    }
}

const char* Claw::autoclampStateName() const {
    if (state_ == State::CLAMPED) return "HELD";
    if (!enabled_) return "OFF";
    if (state_ == State::OPEN) return "OFF";
    if (state_ == State::RELEASED) return "CLEAR?";
    return AutoclampDetector::stateName(detect_.state());
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
