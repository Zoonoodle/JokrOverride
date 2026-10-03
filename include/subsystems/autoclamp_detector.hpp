#pragma once
/**
 * AutoclampDetector — "is there a pin (or a cup) in the jaws, really?" for the claw's autoclamp.
 *
 * PURE ON PURPOSE, like subsystems::ContactDetector next to it: no PROS, no sensor, no clock, no
 * cylinder. It is fed one raw distance reading and a timestamp per tick and returns whether the claw
 * should fire. That is what lets the debounce be reasoned about (and tested on a laptop) instead of
 * discovered on the robot, where a false clamp costs a dropped pin and a missed grab costs a cycle.
 * Claw::update() owns the cylinder and calls step() once per 10 ms tick.
 *
 * THE RULE. While ARMED, the reading must be VALID and at or inside rangeMm CONTINUOUSLY for
 * confirmMs, over at least minSamples FRESH samples. Any reading outside the range, or an invalid
 * one, resets the window to zero -- so the spike the user described (something appears for one
 * reading, is gone 50-100 ms later, then appears again) can never clamp: each spike starts the window
 * over, and a spike shorter than confirmMs never completes one.
 *
 * WHY "FRESH" SAMPLES ARE COUNTED SEPARATELY FROM TIME. The V5 Distance Sensor does not produce a new
 * measurement every 10 ms -- it refreshes on the order of 30 ms, so reading it from a 10 ms loop hands
 * back the SAME number two or three times in a row. A pure time window would therefore be satisfiable
 * by a SINGLE real measurement that happens to be repeated for 50 ms, which is exactly the "random
 * spike" case this is supposed to reject. So samples are counted, not just milliseconds: minSamples
 * in-range readings have to be seen as well as confirmMs of elapsed time.
 *
 * A reading counts as a new sample when EITHER the value changed (a new measurement, obviously) OR
 * staleGraceMs has passed since the last counted one. The second half matters: the sensor can report
 * exactly the same millimetre twice in a row for a pin that is sitting still, and without it a
 * perfectly steady reading would never accumulate a second sample and the claw would never close.
 * staleGraceMs is set to about two refresh periods, so an identical reading is only credited once the
 * sensor must have genuinely re-measured. The failure this cannot distinguish is a sensor whose
 * reading has FROZEN in range -- there is no timestamp in the PROS API to tell that from a real static
 * object -- and a frozen in-range sensor will eventually clamp. That is why "unplugged / cannot
 * measure" (which does have a signature: PROS_ERR, or 0) is handled explicitly instead.
 *
 * WHY WAIT_CLEAR NEEDS A WAY OUT (added 2026-10-03, after this took the autoclamp out completely).
 * The clear rule below is right, but on its own it is an UNBOUNDED wait, and "wait forever" is a
 * terrible failure mode for a feature with no indication beyond one word on a screen. The trap is that
 * rearmMm is derived as rangeMm + a gap, so raising the trigger range raises the distance the world has
 * to be at before the autoclamp will arm again -- and that has a hard physical ceiling: whatever the
 * claw habitually has in front of it. At a 15 mm range the clear threshold was 30 mm and essentially
 * always satisfiable; at 60-75 mm it became 90-105 mm, and anything permanently inside 10 cm of the
 * sensor -- the claw's own structure, a cup, a wall, the pin it just put down and has not driven away
 * from -- pinned the detector in WAIT_CLEAR for the rest of the match. The jaws then never clamped
 * again and the only clue was `AC:CLEAR?` on the brain.
 * So rearmMaxMs is a bounded escape hatch: after that long waiting, arm anyway. It is deliberately much
 * longer than rearmClearMs (seconds, not milliseconds) so it is never the normal path -- a genuinely
 * clear reading still arms in rearmClearMs -- and the risk it reintroduces is bounded and different in
 * kind from the bug the clear rule exists to prevent: that one re-grabbed the pin within ~150 ms, every
 * time, so no score could ever finish. Re-grabbing something still sitting in the jaws SECONDS later,
 * after the robot has had every chance to drive away, costs one button press.
 *
 * WHY RE-ARMING NEEDS A CLEAR READING, NOT A TIMER. The instant the jaws open -- A, the L2 score's
 * release, an auton release -- the pin they just let go of is still sitting right there, a few mm
 * away. An autoclamp that re-arms on a plain timer re-grabs it, every time, and the driver can never
 * put anything down and a score can never finish. So after any open, the detector starts in
 * WAIT_CLEAR: it will not look for a pin at all until the sensor has read CLEAR -- further away than
 * rearmMm, which is deliberately bigger than rangeMm so the two cannot chatter at one distance -- for
 * rearmMs. "No object in range" counts as clear. Only then does it arm.
 *
 * VALIDITY. maxMm and above (the sensor's own "nothing there" value, 9999) is treated as CLEAR, not
 * as an error. Anything below minMm (a 0, which this sensor emits when it cannot measure at all) and
 * any negative value (PROS_ERR, or an unplugged port) is treated as NOT DETECTED and resets the
 * confirm window -- but it does NOT count as clear either, so a sensor that has died cannot re-arm a
 * disarmed autoclamp and cannot fire a clamp. A dead sensor makes the autoclamp do nothing, which is
 * the correct failure.
 *
 * rangeMm AND rearmMm MOVE TOGETHER, ALWAYS -- the one invariant a caller can break from outside. Arming
 * wants a reading FURTHER than rearmMm; clamping wants one at or inside rangeMm. Hand this class a
 * rearmMm at or below rangeMm and anything sitting between them is BOTH clear and in range, so it arms
 * and instantly re-fires on the object it just announced, forever. Claw::detectorConfig() static_asserts
 * both of the real pairs (config::AUTOCLAMP_RANGE_MM / _REARM_MM, 50 / 80, and the handoff's
 * _INTAKE_RANGE_MM / _INTAKE_REARM_MM, 75 / 105), and test/autoclamp_detector_test.cpp has a case that
 * proves what the broken ordering does, so this cannot be rediscovered the hard way on the robot.
 *
 * RANGE, worth knowing before tuning: 50 mm (and 75 at the handoff) is well clear of the V5 Distance
 * Sensor's ~20 mm specified minimum, so nothing here is reading below spec. If the clamp never fires,
 * check on the brain's DIST line whether the CLAW reading actually falls inside the range with a pin in
 * the jaws at all, or whether it reads 0 (an object below the sensor's floor -- that is treated as
 * "cannot measure", never as "very close", so move the sensor back a few mm).
 */

#include <cstdint>

namespace subsystems {

class AutoclampDetector {
  public:
    /** Every threshold comes from config.hpp (the AUTOCLAMP_* block); nothing is baked in here. */
    struct Config {
        int rangeMm;        // at or inside this = a pin is in the jaws
        int confirmMs;      // ...continuously for this long
        int minSamples;     // ...over at least this many FRESH readings
        int staleGraceMs;   // an unchanged reading counts as a new sample again after this long
        int rearmMm;        // further than this = clear (hysteresis above rangeMm)
        int rearmClearMs;   // ...for this long, before a disarmed detector arms again
        int rearmMaxMs;     // ...OR this long in WAIT_CLEAR with no clear reading at all: arm anyway.
                            // The escape hatch -- see "WHY WAIT_CLEAR NEEDS A WAY OUT" in the header.
        int maxValidMm;     // at or above this = the sensor's "nothing in range" value: CLEAR
        int minValidMm;     // below this (e.g. a 0) = cannot measure: NOT DETECTED, and NOT clear
        int minConfidence;  // 0 = ignore the sensor's confidence; otherwise require at least this
    };

    /** What it is doing, for the brain screen. */
    enum class State {
        WAIT_CLEAR,  // disarmed after an open: waiting for the sensor to read clear
        ARMED,       // watching for a pin, nothing seen
        SEEING,      // something is in range, the confirm window is running
    };

    /** Start over. needsClear = true after any claw open (see the class comment); false when the
     *  autoclamp is being armed fresh with the jaws already empty. */
    void reset(bool needsClear) {
        state_ = needsClear ? State::WAIT_CLEAR : State::ARMED;
        since_ = 0;
        haveSince_ = false;
        haveWaitSince_ = false;  // the escape-hatch clock latches on the first WAIT_CLEAR tick
        waitSince_ = 0;
        freshInRange_ = 0;
        lastRawMm_ = kNoReading;
        lastCountedAt_ = 0;
        haveCounted_ = false;
        lastValidMm_ = -1;
    }

    /**
     * One tick. mm is the raw sensor reading (negative for PROS_ERR / no device), confidence is the
     * sensor's own 0..63 confidence (pass 64 if it isn't available), and nowMs is a millisecond clock.
     * Returns true on the ONE tick the claw should close. inhibit = true (e.g. an L2 score sequence is
     * running) keeps the window reset and can never return true.
     */
    bool step(std::uint32_t nowMs, int mm, int confidence, const Config& cfg, bool inhibit) {
        // New measurement, or the same number long enough after the last counted one that the sensor
        // must have re-measured (see the class comment).
        const bool changed = mm != lastRawMm_;
        const bool staleButDue =
            haveCounted_ && nowMs - lastCountedAt_ >= static_cast<std::uint32_t>(cfg.staleGraceMs);
        const bool fresh = changed || staleButDue;
        lastRawMm_ = mm;

        const bool confident = cfg.minConfidence <= 0 || confidence >= cfg.minConfidence;
        const bool readable = mm >= cfg.minValidMm && confident;  // a 0 / PROS_ERR / low-confidence read
        const bool noObject = mm >= cfg.maxValidMm;               // the sensor's own "nothing there"
        if (readable && !noObject) lastValidMm_ = mm;

        const bool inRange = readable && !noObject && mm <= cfg.rangeMm;
        const bool clear = noObject || (readable && mm > cfg.rearmMm);

        if (inhibit) {
            // Something else owns the claw right now (an L2 score). Hold the window at zero so the
            // tick the inhibit lifts cannot inherit a nearly-complete confirm.
            haveSince_ = false;
            freshInRange_ = 0;
            if (state_ == State::SEEING) state_ = State::ARMED;
            return false;
        }

        if (state_ == State::WAIT_CLEAR) {
            // THE ESCAPE HATCH. Start the clock on the first tick we are actually waiting (not in
            // reset(), which has no clock) and give up waiting after rearmMaxMs. Without this, a sensor
            // whose view is permanently occupied inside rearmMm -- the claw's own structure, a cup, the
            // field a few centimetres away -- leaves the autoclamp disarmed FOREVER, silently. See the
            // class comment; this is the bug that took the feature out entirely once rangeMm (and so the
            // derived rearmMm) grew.
            if (!haveWaitSince_) {
                haveWaitSince_ = true;
                waitSince_ = nowMs;
            }
            if (cfg.rearmMaxMs > 0 && nowMs - waitSince_ >= static_cast<std::uint32_t>(cfg.rearmMaxMs)) {
                state_ = State::ARMED;
                haveSince_ = false;
                freshInRange_ = 0;
                haveCounted_ = false;
                return false;  // armed as of now; this tick does not also get to fire
            }
            if (!clear) {
                haveSince_ = false;
                return false;
            }
            if (!haveSince_) {
                haveSince_ = true;
                since_ = nowMs;
            }
            if (nowMs - since_ >= static_cast<std::uint32_t>(cfg.rearmClearMs)) {
                state_ = State::ARMED;
                haveSince_ = false;
                freshInRange_ = 0;
                haveCounted_ = false;
            }
            return false;
        }

        // ARMED / SEEING: the confirm window.
        if (!inRange) {
            state_ = State::ARMED;
            haveSince_ = false;
            freshInRange_ = 0;
            return false;
        }
        state_ = State::SEEING;
        if (!haveSince_) {
            haveSince_ = true;
            since_ = nowMs;
            freshInRange_ = 0;
            haveCounted_ = false;  // this window counts its own samples from scratch
        }
        // A repeated identical reading is the SAME measurement read twice (see the class comment): it
        // keeps the window open but is not new evidence.
        if (fresh) {
            if (freshInRange_ < 1000) ++freshInRange_;
            lastCountedAt_ = nowMs;
            haveCounted_ = true;
        }
        if (nowMs - since_ >= static_cast<std::uint32_t>(cfg.confirmMs) && freshInRange_ >= cfg.minSamples) {
            // Fired. Whatever happens to the cylinder is the Claw's business; this detector goes back
            // to waiting for a clear reading, so the pin it just announced cannot announce itself twice.
            reset(true);
            return true;
        }
        return false;
    }

    State state() const { return state_; }
    /** The last reading that was a real measurement of a real object, mm (-1 if there hasn't been one). */
    int lastValidMm() const { return lastValidMm_; }
    /** How many fresh in-range readings the open confirm window has seen. */
    int freshInRange() const { return freshInRange_; }
    /** "OFF" is the Claw's word, not this class's -- see Claw::autoclampStateName(). */
    static const char* stateName(State s) {
        switch (s) {
            case State::WAIT_CLEAR: return "CLEAR?";
            case State::ARMED: return "ARMED";
            case State::SEEING: return "SEEING";
        }
        return "?";
    }

  private:
    static constexpr int kNoReading = -32768;

    State state_ = State::WAIT_CLEAR;
    std::uint32_t since_ = 0;
    bool haveSince_ = false;
    std::uint32_t waitSince_ = 0;   // when this WAIT_CLEAR started (the escape hatch's clock)
    bool haveWaitSince_ = false;
    int freshInRange_ = 0;
    std::uint32_t lastCountedAt_ = 0;
    bool haveCounted_ = false;
    int lastRawMm_ = kNoReading;
    int lastValidMm_ = -1;
};

}  // namespace subsystems
