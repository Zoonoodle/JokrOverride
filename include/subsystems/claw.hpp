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
 *   OPEN ──(arm, or update() while ENABLED)──> ARMED ──(pin seen for DEBOUNCE ms)──> CLAMPED
 *    ^ (power-on)                              ^                                      │
 *    │                                         └──(lockout over)── RELEASED <──(release)
 *    └──(disarm)──────────────────────────────────────────────────────────────────────────
 *
 *   OPEN     jaws open, autoclamp DISABLED (driver in full manual control). This is the power-on
 *            state, because the cylinder's unpowered position IS open -- see below.
 *            IT IS NOT A DEAD END ANY MORE: update() arms an OPEN claw by itself the moment the
 *            autoclamp is enabled (and nothing is inhibiting it), without touching the cylinder.
 *            It used to need somebody to CALL arm(), so an enable that did not reach a call -- the
 *            one-shot at the top of opcontrol() being the only one in the whole program run -- left
 *            the autoclamp showing `AC:OFF` while switched ON, for good. The enable is now mirrored
 *            onto this class every tick by periodic::tick(), and this is the half that acts on it.
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
 * least AUTOCLAMP_MIN_SAMPLES fresh readings -> clamp. On by default in driver AND in auton
 * (AUTOCLAMP_ON_IN_DRIVER / AUTOCLAMP_ON_IN_AUTON, 2026-10-03 -- a route no longer has to ask for it),
 * off at power-on, and inhibited (setInhibit()) for the whole of an L2 score.
 *
 * TWO THINGS THIS CLASS REPORTS TO THE REST OF THE ROBOT, both for Scoring:
 *   setIntakeMode()    while the macro is at the R1 INTAKE handoff the confirm window is swapped for
 *                      AUTOCLAMP_INTAKE_CONFIRM_MS / _MIN_SAMPLES, so the FIRST valid in-range reading
 *                      fires -- "trigger claw at the instant it sees the object".
 *   takeClampEvent()   a one-shot "the jaws just shut, and here is why" (MANUAL vs AUTOCLAMP), which
 *                      Scoring turns into the clamp-grab hop everywhere, or the
 *                      AUTOCLAMP_INTAKE_PUSH_MS forced push down at the handoff. It is ALWAYS consumed
 *                      and never queues: a clamp made while disabled is dropped by clearClampEvent().
 * Neither of them touches a cylinder or the lift. This class still owns the one, Scoring the other.
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

    /** WHY the jaws last shut, for the one consumer that has to tell those apart: Scoring's
     *  clamp-rise / intake-push decision (see Scoring::handleClampEvent()). NONE = no unconsumed
     *  clamp. */
    enum class ClampSource { NONE, MANUAL, AUTOCLAMP };

    Claw();

    /** Turn autoclamp ON. The jaws open (if they weren't already) and the detector starts watching:
     *  OPEN -> ARMED. Called from RELEASED it only sets the enable -- that state has to see the sensor
     *  read CLEAR first (see the re-arm note in autoclamp_detector.hpp), and then arms itself. A no-op
     *  while CLAMPED: it never opens a claw that is holding something. */
    void arm();
    /** Turn autoclamp OFF; the jaws open and stay open (-> OPEN). */
    void disarm();
    /** JUST THE ENABLE FLAG -- never the cylinder, never the state machine. For the one caller that has
     *  to record "the autoclamp is on" while the jaws are CLAMPED on a pin, where arm() would be a no-op
     *  and disarm() would open the jaws and drop it. Without this, a match that grips its preload in
     *  competition_initialize() and then enables the autoclamp in autonomous() set the flag on Scoring
     *  but never here, so the first time the route opened the jaws they re-armed to OPEN (autoclamp off)
     *  instead of ARMED and the autoclamp could not fire for the rest of the match.
     *
     *  PASSED EVERY TICK by periodic::tick() as well, from Scoring::autoclampOn() -- the same deal as
     *  setInhibit() / setIntakeMode() / setAutonMode(). That is what makes "the autoclamp is always on
     *  in driver" true by construction: the enable is a mirror of the mode's intent rather than latched
     *  state that one call at mode start has to land for. */
    void setAutoclampEnabled(bool on) { enabled_ = on; }
    /** Is the autoclamp enabled? (Y toggles it; opcontrol turns it on, auton leaves it off.) */
    bool autoclampEnabled() const { return enabled_; }
    /** Block the autoclamp from firing this tick without disabling it -- used while an L2 score
     *  sequence owns the claw, so a stack it is pressing into can never trigger a clamp. Pass it
     *  every tick (periodic::tick() does). */
    void setInhibit(bool inhibit) { inhibit_ = inhibit; }
    /** THE R1 INTAKE HANDOFF MODE. While true the autoclamp uses config::AUTOCLAMP_INTAKE_CONFIRM_MS /
     *  _MIN_SAMPLES instead of the ordinary confirm window, i.e. it fires on the FIRST valid in-range
     *  reading -- the driver wants the pin gripped the instant it arrives at the handoff. Pass it every
     *  tick (periodic::tick() does, from Scoring::isAtIntakeHandoff()). It changes nothing else: the
     *  pure detector is the same code, just handed different numbers. */
    void setIntakeMode(bool atIntakeHandoff) { intakeMode_ = atIntakeHandoff; }
    /** WHICH MODE THE ROBOT IS IN, because the trigger range differs: config::AUTOCLAMP_RANGE_AUTON_MM
     *  (65) against AUTOCLAMP_RANGE_DRIVER_MM (55). In driver control a human is aiming the claw, so the
     *  shorter range is the safer default; in autonomous nothing is aiming and the claw has to commit
     *  from further out. Passed every tick by periodic::tick() from periodic::Inputs::auton -- NOT
     *  latched state -- so a field switch can never leave it in the wrong mode. */
    void setAutonMode(bool auton) { autonMode_ = auton; }
    /** The trigger range in force right now, mm: the handoff's reach if setIntakeMode(), else this
     *  mode's. Public so the brain/README can be checked against it. */
    int activeRangeMm() const;
    /** Driver override: close now, no sensor needed (-> CLAMPED). */
    void clamp();

    /** TAKE the one-shot "the jaws just shut" event, and WHY they shut. Returns ClampSource::NONE if
     *  there is nothing to take, and always clears it -- so an event can never queue up and fire late
     *  (notably: a clamp made while the robot is DISABLED, e.g. competition_initialize()'s pre-match
     *  grip, is dropped by clearClampEvent() at the start of each enabled mode rather than making the
     *  arm move the moment auton starts). Exactly one consumer: Scoring::handleClampEvent(). */
    ClampSource takeClampEvent() {
        const ClampSource s = clampEvent_;
        clampEvent_ = ClampSource::NONE;
        return s;
    }
    /** Throw away any unconsumed clamp event (periodic::reset(), at the start of each enabled mode). */
    void clearClampEvent() { clampEvent_ = ClampSource::NONE; }
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
    /** The AUTOCLAMP_* block from config.hpp, packed for the pure detector. NOT static any more: the
     *  confirm window depends on intakeMode_ (see setIntakeMode()). */
    AutoclampDetector::Config detectorConfig() const;
    /** Shut the jaws and record WHY, for Scoring's clamp-rise / intake-push decision. */
    void clampFrom(ClampSource source);

    pros::adi::DigitalOut piston_;
    pros::Distance sensor_;
    AutoclampDetector detect_;       // the debounce + re-arm rule, pure (see its header)
    State state_ = State::OPEN;      // unpowered = open, see the class comment
    bool enabled_ = config::CLAW_AUTOCLAMP_AT_START;  // autoclamp on/off (Y, opcontrol, auton)
    bool inhibit_ = false;           // something else owns the claw this tick (an L2 score)
    bool wasInhibited_ = false;      // last tick's inhibit_, to catch the falling edge (see update())
    bool intakeMode_ = false;        // at the R1 handoff: the instant-fire confirm (see setIntakeMode())
    bool autonMode_ = false;         // autonomous: a longer trigger range (see setAutonMode())
    ClampSource clampEvent_ = ClampSource::NONE;  // one-shot: the jaws just shut, and why
    bool level_ = false;             // the level the pin is actually driven to, for the brain screen
    std::uint32_t releasedAt_ = 0;   // when the jaws were last opened
};

}  // namespace subsystems
