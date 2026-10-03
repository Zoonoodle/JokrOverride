// Host-side test for subsystems::AutoclampDetector -- the claw's "is a pin really in the jaws?"
// debounce. Like contact_detector_test.cpp, this does NOT build with pros make (the PROS makefile
// only globs $(ROOT)/src). Run it on the Mac:
//
//     test/run.sh
//
// Why it exists: the whole point of this class is to reject something the robot cannot be made to do
// on demand -- a sensor spike. Driving it with synthetic sample streams proves the rule before
// anything is uploaded, including the two awkward parts: a V5 Distance Sensor that refreshes slower
// than the 10 ms tick (so the same reading is handed back two or three times running), and the re-arm
// rule that stops the claw re-grabbing the pin it has just released.

#include "../include/subsystems/autoclamp_detector.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using subsystems::AutoclampDetector;

namespace {

// The real config values (config.hpp's AUTOCLAMP_* block), restated so this test needs no PROS
// headers. If they are changed there, change them here and re-run.
// DRIVER CONTROL: config::AUTOCLAMP_RANGE_DRIVER_MM (60) + AUTOCLAMP_REARM_GAP_MM (30). kCfg is the
// default for every general-purpose case below.
constexpr AutoclampDetector::Config kCfg{
    /*rangeMm*/ 60,
    /*confirmMs*/ 50,
    /*minSamples*/ 2,
    /*staleGraceMs*/ 60,
    /*rearmMm*/ 90,
    /*rearmClearMs*/ 150,
    /*rearmMaxMs*/ 2000,
    /*maxValidMm*/ 9999,
    /*minValidMm*/ 1,
    /*minConfidence*/ 0,
};

// THE R1 INTAKE HANDOFF variant: the same config with AUTOCLAMP_INTAKE_CONFIRM_MS / _MIN_SAMPLES in
// place of the ordinary confirm window, which is exactly what Claw::detectorConfig() hands the detector
// while Scoring::isAtIntakeHandoff() is true. The detector's own code is identical in both modes -- the
// "fire the instant it sees the object" behaviour is purely these two numbers, which is why it can be
// proved here instead of on the robot.
// AUTONOMOUS: the same rule, more reach (AUTOCLAMP_RANGE_AUTON_MM 75 + the 30 mm gap). Nothing is aiming
// the claw in a route, so it has to commit from further out than a driver's would.
constexpr AutoclampDetector::Config kAutonCfg{
    /*rangeMm*/ 75,
    /*confirmMs*/ 50,
    /*minSamples*/ 2,
    /*staleGraceMs*/ 60,
    /*rearmMm*/ 105,
    /*rearmClearMs*/ 150,
    /*rearmMaxMs*/ 2000,
    /*maxValidMm*/ 9999,
    /*minValidMm*/ 1,
    /*minConfidence*/ 0,
};

constexpr AutoclampDetector::Config kIntakeCfg{
    /*rangeMm*/ 75,
    /*confirmMs*/ 0,
    /*minSamples*/ 1,
    /*staleGraceMs*/ 60,
    /*rearmMm*/ 105,
    /*rearmClearMs*/ 150,
    /*rearmMaxMs*/ 2000,
    /*maxValidMm*/ 9999,
    /*minValidMm*/ 1,
    /*minConfidence*/ 0,
};

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("%-62s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

/** Run a stream of readings at 10 ms per tick, holding each one for `holdTicks` ticks to model the
 *  sensor's slower refresh. Returns the millisecond the detector fired, or -1. */
int runStream(AutoclampDetector& d, const std::vector<int>& readings, int holdTicks, int startMs = 1000,
              const AutoclampDetector::Config& cfg = kCfg) {
    int now = startMs;
    for (int mm : readings) {
        for (int t = 0; t < holdTicks; ++t) {
            if (d.step(static_cast<std::uint32_t>(now), mm, 64, cfg, false)) return now;
            now += 10;
        }
    }
    return -1;
}

}  // namespace

int main() {
    std::printf("AutoclampDetector\n");

    // --- a real pin, held in the jaws: must fire, and not instantly ---------------------------
    {
        AutoclampDetector d;
        d.reset(false);  // armed, empty jaws
        // 10 fresh readings of 9 mm, each held 3 ticks (30 ms) like the real sensor.
        const int fired = runStream(d, {9, 9, 9, 9, 9, 9, 9, 9, 9, 9}, 3);
        check(fired > 0, "a pin held in the jaws fires");
        check(fired >= 1000 + 50, "...not before AUTOCLAMP_CONFIRM_MS has elapsed");
        // 2 fresh samples at 30 ms apart is the earliest the rule can be satisfied.
        check(fired <= 1000 + 130, "...and within ~2 sensor refreshes of that");
    }

    // --- ONE lucky in-range measurement, repeated by the stale read: must NOT fire ------------
    // This is the case a pure time window gets wrong: one sample held for 100 ms looks like 100 ms of
    // evidence. minSamples is what rejects it.
    {
        AutoclampDetector d;
        d.reset(false);
        // 4 ticks = 40 ms, under AUTOCLAMP_STALE_GRACE_MS, so this really is one credited sample.
        const int fired = runStream(d, {9}, 4);
        check(fired < 0, "one measurement, not yet re-measured, does NOT fire");
    }

    // --- the user's spike: in range, gone, back again -- never long enough ---------------------
    {
        AutoclampDetector d;
        d.reset(false);
        // 9 mm for 30 ms, then clear for 100 ms, then 9 mm for 30 ms, repeatedly.
        std::vector<int> stream;
        for (int i = 0; i < 6; ++i) {
            stream.push_back(9);
            stream.push_back(400);
            stream.push_back(500);
            stream.push_back(600);
        }
        const int fired = runStream(d, stream, 3);
        check(fired < 0, "a spike in / out / in never fires");
    }

    // --- invalid readings are not detections, and not clear either -----------------------------
    {
        AutoclampDetector d;
        d.reset(false);
        check(runStream(d, {0, 0, 0, 0, 0, 0, 0, 0}, 3) < 0, "a 0 reading (cannot measure) never fires");
        AutoclampDetector e;
        e.reset(false);
        check(runStream(e, {-1, -1, -1, -1, -1, -1}, 3) < 0, "PROS_ERR / unplugged never fires");
    }
    {
        // An unplugged sensor must not be able to re-arm a disarmed detector either: otherwise a dead
        // sensor would quietly turn the autoclamp back on.
        AutoclampDetector d;
        d.reset(true);  // just released: waiting for a clear reading
        runStream(d, {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}, 3);
        check(d.state() == AutoclampDetector::State::WAIT_CLEAR,
              "an unplugged sensor cannot re-arm a disarmed detector");
    }

    // --- the re-arm rule: the pin it just let go of must not be re-grabbed --------------------
    {
        AutoclampDetector d;
        d.reset(true);  // the state after any claw open
        // The released pin is still sitting at 8 mm for a good while.
        const int fired = runStream(d, {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8}, 3);
        check(fired < 0, "a released pin still in front of the sensor is not re-grabbed");
        check(d.state() == AutoclampDetector::State::WAIT_CLEAR, "...the detector is still WAIT_CLEAR");
        // Now it is taken away: clear (above AUTOCLAMP_REARM_MM) for AUTOCLAMP_REARM_CLEAR_MS.
        runStream(d, {200, 200, 200, 200, 200, 200, 200}, 3);
        check(d.state() == AutoclampDetector::State::ARMED, "...clear for the confirm time re-arms it");
        // And now a new pin arrives and IS gripped.
        check(runStream(d, {9, 9, 9, 9, 9, 9}, 3) > 0, "...the next pin fires normally");
    }
    {
        // 65 mm is inside the hysteresis gap (above rangeMm 50, below rearmMm 80): it is neither a pin
        // nor clear, so a disarmed detector stays disarmed. This is what stops the two thresholds
        // chattering at one distance.
        AutoclampDetector d;
        d.reset(true);
        runStream(d, {75, 75, 75, 75, 75, 75, 75, 75, 75, 75}, 3);
        check(d.state() == AutoclampDetector::State::WAIT_CLEAR,
              "a reading inside the 60-90 mm hysteresis gap does not re-arm");
    }

    // --- "nothing in range" counts as clear ---------------------------------------------------
    {
        AutoclampDetector d;
        d.reset(true);
        runStream(d, {9999, 9999, 9999, 9999, 9999, 9999, 9999}, 3);
        check(d.state() == AutoclampDetector::State::ARMED, "the sensor's 9999 'no object' counts as clear");
    }

    // --- inhibit (an L2 score is running) can never fire, and does not bank progress ----------
    {
        AutoclampDetector d;
        d.reset(false);
        std::uint32_t now = 1000;
        bool fired = false;
        for (int i = 0; i < 40; ++i) {  // 400 ms of a pin right in the jaws, inhibited
            fired = fired || d.step(now, 9, 64, kCfg, true);
            now += 10;
        }
        check(!fired, "inhibit never fires");
        // The tick the inhibit lifts must not inherit a nearly-complete window: a fresh confirm has
        // to run from zero, so a single tick cannot clamp.
        check(!d.step(now, 9, 64, kCfg, false), "...and the tick it lifts cannot fire immediately");
    }

    // --- after firing, it disarms itself (the pin it announced cannot announce itself twice) ---
    {
        AutoclampDetector d;
        d.reset(false);
        check(runStream(d, {9, 9, 9, 9, 9, 9}, 3) > 0, "fires once");
        check(d.state() == AutoclampDetector::State::WAIT_CLEAR, "...then disarms itself");
    }

    // --- THE RANGE / RE-ARM ORDERING, as a regression test ------------------------------------------
    // The trap the 15 -> 50 mm range change walked straight into (2026-10-03): re-arming wants a reading
    // FURTHER than rearmMm, clamping wants one at or inside rangeMm. Leave rearmMm at or below rangeMm
    // and a pin between them is BOTH clear and in range, so the detector arms and instantly re-fires on
    // the thing it just released -- forever. Config::rearmMm must always be above Config::rangeMm; the
    // real pair is static_assert()ed in Claw::detectorConfig(), and this proves what goes wrong if it
    // ever is not.
    {
        AutoclampDetector::Config bad = kCfg;
        bad.rangeMm = 50;
        bad.rearmMm = 30;  // WRONG: below the range
        AutoclampDetector d;
        d.reset(true);  // just released; the pin it let go of is sitting at ~40 mm
        bool refired = false;
        std::uint32_t now = 1000;
        for (int i = 0; i < 60; ++i) {
            // 40/41 alternating, because a real sensor's reading always jitters by a millimetre or two.
            // A perfectly CONSTANT reading would not re-fire here -- not because the thresholds are
            // safe, but because the detector only credits a fresh sample when the value changes or
            // staleGraceMs passes, and a window that opens on an unchanged value has nothing to credit.
            // That is an artifact of a synthetic stream, not a property of the robot, so don't "fix" the
            // thresholds on the strength of it.
            refired = refired || d.step(now, 40 + (i % 2), 64, bad, false);
            now += 10;
        }
        check(refired, "rearmMm BELOW rangeMm re-grabs the released pin (why the assert exists)");
    }
    {
        // ...and the shipped pair does not, at the same distance.
        AutoclampDetector d;
        d.reset(true);
        bool refired = false;
        std::uint32_t now = 1000;
        for (int i = 0; i < 60; ++i) {
            refired = refired || d.step(now, 40 + (i % 2), 64, kCfg, false);
            now += 10;
        }
        check(!refired, "...the shipped 60/90 pair does not");
        check(d.state() == AutoclampDetector::State::WAIT_CLEAR, "...and stays WAIT_CLEAR");
    }
    {
        // The same, at the handoff's longer 75/105 pair, with a pin at 90 mm -- inside ITS gap.
        AutoclampDetector d;
        d.reset(true);
        runStream(d, {90, 90, 90, 90, 90, 90, 90, 90, 90, 90}, 3, 1000, kIntakeCfg);
        check(d.state() == AutoclampDetector::State::WAIT_CLEAR,
              "the handoff's 75-105 mm gap does not re-arm either");
    }
    {
        // THE RANGES, as the one thing a reading at the wrong distance would silently get wrong.
        // 65 mm is a PIN at the handoff and in auton (both <= 75), and NOT one in driver control (> 60).
        // That ladder is the point of having separate numbers, so pin every rung down.
        AutoclampDetector d;
        d.reset(false);
        check(d.step(1000, 65, 64, kIntakeCfg, false), "65 mm fires at the handoff (range 75)");
        AutoclampDetector e;
        e.reset(false);
        check(runStream(e, {65, 66, 65, 66, 65, 66, 65, 66}, 3, 1000, kAutonCfg) > 0,
              "65 mm fires in auton too (range 75)");
        AutoclampDetector f;
        f.reset(false);
        check(runStream(f, {65, 66, 65, 66, 65, 66, 65, 66}, 3) < 0,
              "...but NOT in driver control (range 60)");
        // ...and 50 mm is a pin in all three, so the ladder is about 60-75, not about everything.
        AutoclampDetector g;
        g.reset(false);
        check(runStream(g, {50, 51, 50, 51, 50, 51, 50, 51}, 3) > 0, "50 mm fires in driver control");
    }
    {
        // Each mode's clear threshold tracks ITS OWN range (range + AUTOCLAMP_REARM_GAP_MM), which is
        // what stops a reading that is "clear" in one mode being a pin in another. 90 mm is inside
        // auton's 75-105 gap.
        AutoclampDetector d;
        d.reset(true);
        runStream(d, {90, 90, 90, 90, 90, 90, 90, 90, 90, 90}, 3, 1000, kAutonCfg);
        check(d.state() == AutoclampDetector::State::WAIT_CLEAR,
              "auton's 75-105 mm gap does not re-arm either");
        // 100 mm IS clear in driver control (> 90) but NOT in auton (< 105) -- the same reading means
        // different things in the two modes, which is exactly why the threshold is derived per mode.
        AutoclampDetector e;
        e.reset(true);
        runStream(e, {100, 100, 100, 100, 100, 100, 100}, 3);
        check(e.state() == AutoclampDetector::State::ARMED, "100 mm re-arms in driver control");
        AutoclampDetector f;
        f.reset(true);
        runStream(f, {100, 100, 100, 100, 100, 100, 100}, 3, 1000, kAutonCfg);
        check(f.state() == AutoclampDetector::State::WAIT_CLEAR, "...but not in auton (needs 105)");
    }

    // --- THE ESCAPE HATCH out of WAIT_CLEAR -- the bug that took the autoclamp out entirely ---------
    // Because rearmMm is derived as rangeMm + a gap, raising the trigger range raised how far away the
    // world has to be before the detector will arm again (90 mm in driver control). Anything permanently
    // inside that -- the claw's own structure, a cup, the pin it just put down -- used to pin it in
    // WAIT_CLEAR for the rest of the match, with `AC:CLEAR?` on the brain as the only clue. rearmMaxMs
    // bounds the wait.
    {
        AutoclampDetector d;
        d.reset(true);  // just released; the view is permanently occupied at 40 mm (inside the range)
        std::uint32_t now = 1000;
        bool firedEarly = false;
        // Well before the hatch it must still be waiting -- the hatch is a backstop, not the normal path.
        for (int i = 0; i < 100; ++i) {  // 1.0 s
            firedEarly = firedEarly || d.step(now, 40 + (i % 2), 64, kCfg, false);
            now += 10;
        }
        check(!firedEarly && d.state() == AutoclampDetector::State::WAIT_CLEAR,
              "an occupied view still waits well before rearmMaxMs");
        bool firedLate = false;
        for (int i = 0; i < 150; ++i) {  // past 2.0 s in total
            firedLate = firedLate || d.step(now, 40 + (i % 2), 64, kCfg, false);
            now += 10;
        }
        // Once the hatch arms it, the object that was occupying the view IS a pin as far as the rule is
        // concerned, so it gets gripped -- which is precisely the bounded trade the hatch makes, and the
        // reason it is seconds rather than milliseconds. Asserting on `fired` says that out loud;
        // asserting on state() would not, because firing resets straight back to WAIT_CLEAR.
        check(firedLate, "...but arms anyway once rearmMaxMs has passed (and then grips what is there)");
    }
    {
        // A GENUINELY clear reading must still arm in rearmClearMs (150 ms), nowhere near the hatch --
        // otherwise the hatch has quietly become the normal path and the clear rule stops meaning
        // anything.
        AutoclampDetector d;
        d.reset(true);
        runStream(d, {200, 200, 200, 200, 200, 200, 200}, 3);  // 210 ms of clear
        check(d.state() == AutoclampDetector::State::ARMED,
              "a real clear reading still arms in rearmClearMs, not at the hatch");
    }
    {
        // And the hatch must not resurrect a DEAD sensor into clamping on junk: it arms, but invalid
        // readings still never fire, so an unplugged sensor stays harmless.
        AutoclampDetector d;
        d.reset(true);
        std::uint32_t now = 1000;
        bool fired = false;
        for (int i = 0; i < 400; ++i) {  // 4 s of PROS_ERR, well past the hatch
            fired = fired || d.step(now, -1, 64, kCfg, false);
            now += 10;
        }
        check(!fired, "the hatch cannot make a dead sensor clamp");
    }

    // --- the R1 HANDOFF mode: the FIRST valid in-range reading fires, with no debounce at all -------
    {
        AutoclampDetector d;
        d.reset(false);  // armed at the handoff, empty jaws
        check(d.step(1000, 9, 64, kIntakeCfg, false),
              "handoff mode fires on the FIRST in-range reading");
    }
    {
        // ...and that really is the intake config doing it, not the detector: the SAME single reading
        // under the ordinary config must not fire.
        AutoclampDetector d;
        d.reset(false);
        check(!d.step(1000, 9, 64, kCfg, false), "...the ordinary config does not fire on one reading");
    }
    {
        // What instant firing costs, stated as a test so it is not a surprise later: one junk in-range
        // sample is enough. That is the accepted trade at the handoff (see AUTOCLAMP_INTAKE_CONFIRM_MS
        // in config.hpp) -- a false clamp there costs one button press, a missed pin costs a cycle.
        AutoclampDetector d;
        d.reset(false);
        check(runStream(d, {400, 400}, 3, 1000, kIntakeCfg) < 0,
              "handoff mode ignores out-of-range readings");
        AutoclampDetector e;
        e.reset(false);
        check(!e.step(1000, 0, 64, kIntakeCfg, false), "...and a 0 (cannot measure) still never fires");
        AutoclampDetector f;
        f.reset(false);
        check(!f.step(1000, -1, 64, kIntakeCfg, false), "...nor PROS_ERR / unplugged");
    }
    {
        // The re-arm rule is untouched by the fast confirm: a just-released pin is still not re-grabbed.
        AutoclampDetector d;
        d.reset(true);
        bool fired = false;
        std::uint32_t now = 1000;
        for (int i = 0; i < 30; ++i) {
            fired = fired || d.step(now, 8, 64, kIntakeCfg, false);
            now += 10;
        }
        check(!fired, "handoff mode still cannot re-grab a just-released pin");
    }
    {
        // ...and so is the inhibit (an L2 score owning the claw).
        AutoclampDetector d;
        d.reset(false);
        check(!d.step(1000, 9, 64, kIntakeCfg, true), "handoff mode still respects the inhibit");
    }

    std::printf("%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
