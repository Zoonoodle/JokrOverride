// Host-side test for subsystems::ContactDetector -- the L2 scorer's landing detector.
//
// This does NOT build with pros make: the PROS makefile only globs $(ROOT)/src (see Makefile's
// SRCDIR), so this whole directory is invisible to the robot build. Run it on the Mac:
//
//     test/run.sh
//
// Why it exists: the first version of the detector never fired on the robot, and finding that out
// cost a real scoring attempt. The detector is a pure function of (dt, reference speed, measured
// speed, output mV), so it can be driven with synthetic 10 ms traces that stand in for the cases
// that matter -- free descent, spin-up, and three kinds of landing -- and the thresholds can be
// proven to fire (or not fire) before anything is uploaded.
//
// The scenarios are modelled on how the real loop behaves, not on arbitrary numbers:
//   - a free descent settles BELOW the nominal 90 deg/s, because gravity resists over much of this
//     six-bar's arc and the press cap limits the downward push. 55 deg/s is the pessimistic case
//     that broke the old detector.
//   - the motor output while descending freely is some steady value; on contact the velocity loop's
//     P term reacts within a tick (KP = 30 mV per deg/s of error) and its integral winds up after,
//     so the output ramps downward (harder) over ~100-200 ms.
//   - measurement noise is +-3 deg/s, which is far more than the rotation sensor's real quantisation
//     (0.25 deg/s per count) -- a deliberately hostile margin.

#include "../include/subsystems/contact_detector.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using subsystems::ContactDetector;

namespace {

constexpr double kDt = 0.01;  // the lift task's 10 ms tick

// The real config, mirrored from include/config.hpp's L2_* block. Kept in sync by hand on purpose:
// if someone changes a threshold in config.hpp and this test still passes with the old one, the
// mismatch is the thing worth noticing.
ContactDetector::Config realConfig() {
    return ContactDetector::Config{
        /*settleMs*/ 200.0,
        /*ignoreMs*/ 60.0,
        /*confirmMs*/ 20.0,
        /*baselineTauMs*/ 200.0,
        /*outputTauMs*/ 50.0,
        /*lagDeg*/ 1.5,
        /*lagLeakMs*/ 200.0,
        /*effortMv*/ 2000.0,
        /*speedFrac*/ 0.70,
        /*freezeFrac*/ 0.90,
        /*stallDegPerS*/ 6.0,
    };
}

// A deterministic, repeatable "noise" source -- a fixed LCG, so a failure is always reproducible.
struct Noise {
    unsigned state = 12345u;
    double next(double amplitude) {
        state = state * 1664525u + 1013904223u;
        const double unit = static_cast<double>((state >> 8) & 0xFFFF) / 65535.0;  // 0..1
        return (unit * 2.0 - 1.0) * amplitude;
    }
};

struct Sample {
    double measured;  // unsigned descent speed, deg/s
    double outMv;     // signed, negative = pushing down
};

struct Result {
    bool fired = false;
    ContactDetector::Trigger trigger = ContactDetector::Trigger::NONE;
    int tickOfFire = -1;
    double lagAtFire = 0.0, effortAtFire = 0.0, ratioAtFire = 0.0;
};

Result run(const std::vector<Sample>& trace) {
    ContactDetector det;
    det.reset();
    const ContactDetector::Config cfg = realConfig();
    Result r;
    for (std::size_t i = 0; i < trace.size(); ++i) {
        const ContactDetector::Trigger t = det.step(kDt, trace[i].measured, trace[i].outMv, cfg);
        if (t != ContactDetector::Trigger::NONE && !r.fired) {
            r.fired = true;
            r.trigger = t;
            r.tickOfFire = static_cast<int>(i);
            r.lagAtFire = det.lagDeg();
            r.effortAtFire = det.effortMv();
            r.ratioAtFire = det.speedRatio();
            break;
        }
    }
    return r;
}

// --- scenario builders ------------------------------------------------------------------------

/** Spin-up from a standstill to `cruise` deg/s at `accel` deg/s^2, then hold it, with noise.
 *  outMv settles at `holdMv` (signed). This is the shape of every press before anything happens. */
std::vector<Sample> freeDescent(double cruise, double holdMv, int ticks, Noise& n, double accel = 800.0) {
    std::vector<Sample> out;
    double v = 0.0;
    for (int i = 0; i < ticks; ++i) {
        v = std::min(cruise, v + accel * kDt);
        // While accelerating the loop pushes harder than it will at cruise.
        const double mv = v < cruise ? holdMv - 2500.0 * (1.0 - v / cruise) : holdMv;
        out.push_back({std::max(0.0, v + n.next(3.0)), mv + n.next(250.0)});
    }
    return out;
}

/** Append a landing: the speed decays from cruise to `endSpeed` over `fallMs`, and the loop pushes
 *  down harder -- `extraMv` more by `effortMs` after the landing starts (P term immediately, the
 *  integral winding in after). */
void appendLanding(std::vector<Sample>& out, double cruise, double holdMv, double endSpeed, double fallMs,
                   double extraMv, double effortMs, int ticks, Noise& n) {
    for (int i = 0; i < ticks; ++i) {
        const double t = i * kDt * 1000.0;
        const double fall = std::min(1.0, t / std::max(fallMs, 1.0));
        const double v = cruise + (endSpeed - cruise) * fall;
        const double eff = std::min(1.0, t / std::max(effortMs, 1.0));
        const double mv = holdMv - extraMv * eff;
        out.push_back({std::max(0.0, v + n.next(3.0)), mv + n.next(250.0)});
    }
}

struct Case {
    std::string name;
    std::vector<Sample> trace;
    bool expectFire;
    int landingTick;  // -1 when there is no landing
    double maxLatencyMs;
};

int failures = 0;

void report(const Case& c) {
    const Result r = run(c.trace);
    const double latency = r.fired && c.landingTick >= 0 ? (r.tickOfFire - c.landingTick) * kDt * 1000.0 : 0.0;
    bool ok = r.fired == c.expectFire;
    if (ok && c.expectFire && c.landingTick >= 0) {
        if (r.tickOfFire < c.landingTick) ok = false;  // fired BEFORE the landing: a false trigger
        if (latency > c.maxLatencyMs) ok = false;
    }
    std::printf("%-42s %-9s %-7s", c.name.c_str(), r.fired ? "FIRED" : "no fire",
                ContactDetector::triggerName(r.trigger));
    if (r.fired && c.landingTick >= 0) std::printf(" %6.0f ms", latency);
    else std::printf("          ");
    if (r.fired)
        std::printf("   lag %4.2f  eff %5.0f  ratio %4.2f", r.lagAtFire, r.effortAtFire, r.ratioAtFire);
    std::printf("   %s\n", ok ? "PASS" : "**FAIL**");
    if (!ok) ++failures;
}

}  // namespace

int main() {
    std::printf("ContactDetector — synthetic 10 ms traces (lag 1.5 deg leak 200 ms, effort 2000 mV,\n");
    std::printf("                  frac 0.70, stall 6 deg/s, settle 200 ms, confirm 20 ms)\n\n");
    std::printf("%-42s %-9s %-7s %-9s   %-34s %s\n", "scenario", "result", "trigger", "latency", "at fire",
                "verdict");
    std::printf("%s\n", std::string(118, '-').c_str());

    // 1. A free descent that settles at only 55 deg/s -- well under the nominal 90. This is the case
    //    the OLD detector could never fire on, and the one it must never FALSELY fire on either.
    {
        Noise n;
        Case c{"free descent 55 deg/s, 2.0 s, noisy", freeDescent(55.0, -1500.0, 200, n), false, -1, 0.0};
        report(c);
    }
    // 2. A free descent at the full 90 deg/s, for completeness.
    {
        Noise n;
        Case c{"free descent 90 deg/s, 2.0 s, noisy", freeDescent(90.0, -800.0, 200, n), false, -1, 0.0};
        report(c);
    }
    // 3. Spin-up only: 300 ms of accelerating from a standstill. Must not fire.
    {
        Noise n;
        Case c{"spin-up from 0, 300 ms only", freeDescent(55.0, -1500.0, 30, n), false, -1, 0.0};
        report(c);
    }
    // 4. HARD landing: the pin hits a solid stack and the arm stops dead within 50 ms, the loop
    //    slamming into the press cap (an extra 3500 mV of downward push within 100 ms).
    {
        Noise n;
        std::vector<Sample> t = freeDescent(55.0, -1500.0, 80, n);
        const int landing = static_cast<int>(t.size());
        appendLanding(t, 55.0, -1500.0, 0.0, 50.0, 3500.0, 100.0, 120, n);
        Case c{"hard landing: stop in 50 ms", t, true, landing, 40.0};
        report(c);
    }
    // 5. SOFT landing: the stack gives, so the arm only drops to ~50% and then creeps. No dramatic
    //    stall, a moderate extra push.
    {
        Noise n;
        std::vector<Sample> t = freeDescent(55.0, -1500.0, 80, n);
        const int landing = static_cast<int>(t.size());
        appendLanding(t, 55.0, -1500.0, 27.0, 120.0, 1800.0, 250.0, 150, n);
        Case c{"soft landing: 50% + creep", t, true, landing, 250.0};
        report(c);
    }
    // 6. CREEP landing: barely a speed change at all -- down to ~70% -- but the loop is clearly
    //    working harder (2000 mV extra). This is the one only the effort/lag signals can catch, and
    //    the case the user described as "that bump/dip in position vs motor work".
    {
        Noise n;
        std::vector<Sample> t = freeDescent(55.0, -1500.0, 80, n);
        const int landing = static_cast<int>(t.size());
        appendLanding(t, 55.0, -1500.0, 38.5, 150.0, 2000.0, 200.0, 200, n);
        Case c{"creep landing: 70% + 2000 mV effort", t, true, landing, 400.0};
        report(c);
    }
    // 7. A pin ALREADY resting on the stack when L2 is pressed: the arm never gets going at all.
    //    Only the absolute stall trigger can see this, and it must not need the baseline.
    {
        Noise n;
        std::vector<Sample> t;
        for (int i = 0; i < 100; ++i) t.push_back({std::max(0.0, 1.0 + n.next(1.5)), -4500.0 + n.next(250.0)});
        Case c{"pin already resting: never moves", t, true, 0, 120.0};
        report(c);
    }
    // 8. A slow, genuine drift in what a free descent costs (gravity torque changing with angle):
    //    the speed sags from 70 to 45 deg/s over 2 s and the push grows by 2500 mV over the same
    //    time. The baselines must track it, so NOTHING fires.
    {
        Noise n;
        std::vector<Sample> t;
        for (int i = 0; i < 20; ++i) {  // spin-up
            const double v = std::min(70.0, 800.0 * i * kDt);
            t.push_back({std::max(0.0, v + n.next(3.0)), -1500.0 + n.next(250.0)});
        }
        for (int i = 0; i < 200; ++i) {
            const double f = i / 200.0;
            t.push_back({std::max(0.0, 70.0 - 25.0 * f + n.next(3.0)), -1500.0 - 2500.0 * f + n.next(250.0)});
        }
        Case c{"slow gravity drift 70->45 deg/s over 2 s", t, false, -1, 0.0};
        report(c);
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASS" : "FAILURES PRESENT");
    return failures == 0 ? 0 : 1;
}
