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
constexpr AutoclampDetector::Config kCfg{
    /*rangeMm*/ 15,
    /*confirmMs*/ 50,
    /*minSamples*/ 2,
    /*staleGraceMs*/ 60,
    /*rearmMm*/ 30,
    /*rearmClearMs*/ 150,
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
int runStream(AutoclampDetector& d, const std::vector<int>& readings, int holdTicks, int startMs = 1000) {
    int now = startMs;
    for (int mm : readings) {
        for (int t = 0; t < holdTicks; ++t) {
            if (d.step(static_cast<std::uint32_t>(now), mm, 64, kCfg, false)) return now;
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
        // 25 mm is inside the hysteresis gap (above rangeMm, below rearmMm): it is neither a pin nor
        // clear, so a disarmed detector stays disarmed. This is what stops the two thresholds
        // chattering at one distance.
        AutoclampDetector d;
        d.reset(true);
        runStream(d, {25, 25, 25, 25, 25, 25, 25, 25, 25, 25}, 3);
        check(d.state() == AutoclampDetector::State::WAIT_CLEAR,
              "a reading inside the 15-30 mm hysteresis gap does not re-arm");
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

    std::printf("%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
