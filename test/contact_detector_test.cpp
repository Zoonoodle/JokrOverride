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

#include <algorithm>
#include <cmath>
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
        /*ignoreMs*/ 80.0,
        /*confirmMs*/ 30.0,
        /*baselineTauMs*/ 200.0,
        /*outputTauMs*/ 50.0,
        /*lagDeg*/ 2.2,
        /*lagLeakMs*/ 200.0,
        /*effortMv*/ 1800.0,
        /*speedFrac*/ 0.55,
        /*freezeFrac*/ 0.82,
        /*stallDegPerS*/ 6.0,
        /*armTravelDeg*/ 3.5,
        /*noStartMs*/ 350.0,
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

/** Spin-up from a standstill to `cruise` deg/s at `accel` deg/s^2, then hold it, with noise (`noiseDeg`
 *  on the speed, fixed 250 mV on the output). outMv settles at `holdMv` (signed). This is the shape of
 *  every press before anything happens. */
std::vector<Sample> freeDescent(double cruise, double holdMv, int ticks, Noise& n, double accel = 800.0,
                                double noiseDeg = 3.0) {
    std::vector<Sample> out;
    double v = 0.0;
    for (int i = 0; i < ticks; ++i) {
        v = std::min(cruise, v + accel * kDt);
        // While accelerating the loop pushes harder than it will at cruise.
        const double mv = v < cruise ? holdMv - 2500.0 * (1.0 - v / cruise) : holdMv;
        out.push_back({std::max(0.0, v + n.next(noiseDeg)), mv + n.next(250.0)});
    }
    return out;
}

/** Spin-up that does NOT start from a clean standstill: a residual few deg/s left over from HOLD's
 *  damped settle (a real HOLD bounces to a near-still, not a true dead stop), then the same ramp shape
 *  as freeDescent(). Starts just above stallDegPerS on purpose, so the absolute near-stop trigger is
 *  never even a question here -- this scenario is about the BASELINE-relative signals surviving an
 *  untidy handover, not about STALL. Must never fire. */
std::vector<Sample> spinUpFromHold(double cruise, double holdMv, int ticks, Noise& n, double accel = 800.0) {
    std::vector<Sample> out;
    double v = 8.0;
    for (int i = 0; i < ticks; ++i) {
        v = std::min(cruise, v + accel * kDt);
        const double mv = v < cruise ? holdMv - 2500.0 * (1.0 - v / cruise) : holdMv;
        out.push_back({std::max(0.0, v + n.next(3.0)), mv + n.next(250.0)});
    }
    return out;
}

/** Append a gravity-torque swing: speed ramps from `v0` down to `vDip` and back up to `v0` over
 *  `periodMs` (a triangle wave), with the loop's output swinging the OPPOSITE way -- down to
 *  `mv0 - mvSwing` at the dip, back to `mv0` -- modelling the six-bar passing vertical mid-press, where
 *  gravity resists harder for a while then lets up. Real motion the whole time, no landing: must never
 *  fire, however far either signal swings on its own. */
void appendGravitySwing(std::vector<Sample>& out, double v0, double mv0, double vDip, double mvSwing,
                        double periodMs, int ticks, Noise& n) {
    for (int i = 0; i < ticks; ++i) {
        const double t = i * kDt * 1000.0;
        const double phase = std::fmod(t, periodMs) / periodMs;
        const double tri = phase < 0.5 ? phase * 2.0 : (1.0 - phase) * 2.0;  // 0 -> 1 -> 0
        const double v = v0 + (vDip - v0) * tri;
        const double mv = mv0 - mvSwing * tri;
        out.push_back({std::max(0.0, v + n.next(3.0)), mv + n.next(250.0)});
    }
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
    std::printf("ContactDetector — synthetic 10 ms traces (lag 2.2 deg leak 200 ms, effort 1800 mV,\n");
    std::printf("                  frac 0.55, stall 6 deg/s, settle 200 ms, confirm 30 ms, AGREEMENT >=2,\n");
    std::printf("                  arm travel 3.5 deg, no-start 350 ms)\n\n");
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
        Case c{"hard landing: stop in 50 ms", t, true, landing, 120.0};
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
        Case c{"creep landing: 70% + 2000 mV effort", t, true, landing, 650.0};
        report(c);
    }
    // 7. A pin ALREADY resting on the stack when L2 is pressed: the arm never gets going at all.
    //    Only the absolute stall trigger can see this, and it must not need the baseline. SPLIT STALL
    //    (2026-10-02): the arm never arms_ here (it never clears stallDegPerS), so this now takes the
    //    much longer noStartMs window (350 ms) instead of the fast confirmMs -- the deliberate trade
    //    that lets a loaded-but-moving arm prove it isn't resting before STALL gives up on it. See the
    //    "loaded claw" scenarios below for why.
    {
        Noise n;
        std::vector<Sample> t;
        for (int i = 0; i < 100; ++i) t.push_back({std::max(0.0, 1.0 + n.next(1.5)), -4500.0 + n.next(250.0)});
        Case c{"pin already resting: never moves", t, true, 0, 420.0};
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
    // 9. THE REAL FALSE-FIRE: a gravity-torque swing, not a drift. This six-bar swings up and back
    //    PAST VERTICAL, so over a press of a few tens of degrees the torque can go from resisting to
    //    assisting (or back) well inside one second -- faster than the 200 ms baselines can track, and
    //    exactly the shape that used to cross SPEED_FRAC (then 0.70) on its own with nothing under the
    //    arm at all. 90 -> 50 -> 90 over 1 s: 50/90 = 0.56, which the OLD threshold (0.70) would have
    //    called contact; the new one (0.50) and AGREEMENT must not.
    {
        Noise n;
        std::vector<Sample> t = freeDescent(90.0, -800.0, 25, n);
        appendGravitySwing(t, 90.0, -800.0, 50.0, 2000.0, 1000.0, 150, n);
        Case c{"gravity speed swing 90->50->90 over 1.0 s", t, false, -1, 0.0};
        report(c);
    }
    // 10. The other half of the same mechanism: the OUTPUT swings by several thousand mV as gravity
    //     flips (the velocity loop fighting harder to hold the SAME speed), with the speed itself
    //     barely moving. This used to be able to fire EFFORT alone; now it needs a second signal that
    //     isn't there.
    {
        Noise n;
        std::vector<Sample> t = freeDescent(90.0, -800.0, 25, n);
        appendGravitySwing(t, 90.0, -800.0, 85.0, 3800.0, 800.0, 120, n);
        Case c{"gravity effort swing, several 1000 mV, flat speed", t, false, -1, 0.0};
        report(c);
    }
    // 11. Heavier noise than the sensor could ever actually produce (+-8 deg/s, vs the +-3 used
    //     everywhere else and the sensor's real 0.25 deg/s quantisation): a plain free descent, long
    //     enough to accumulate if the noise alone could ever add up to anything.
    {
        Noise n;
        Case c{"free descent 55 deg/s, 2.0 s, +-8 deg/s noise", freeDescent(55.0, -1500.0, 200, n, 800.0, 8.0),
               false, -1, 0.0};
        report(c);
    }
    // 12. Spin-up straight out of HOLD -- not a clean standstill (see spinUpFromHold()). Must not fire,
    //     including on the absolute near-stop: the residual speed starts above stallDegPerS on purpose.
    {
        Noise n;
        Case c{"spin-up from HOLD (residual speed, not 0)", spinUpFromHold(55.0, -1500.0, 30, n), false, -1, 0.0};
        report(c);
    }
    // 13-16. LOADED CLAW (pin + cup): the user's field report -- empty claw was correct both ways, but
    // holding a pin+cup it auto-released almost immediately, every time, over bare floor. The weight
    // adds directly to the gravity resistance over the part of the arc where this six-bar's torque
    // already resists descending, and L2_PRESS_MAX_MV caps how hard the loop may push back -- so a
    // loaded arm can sit under stallDegPerS for a lot longer than an empty one while it is still
    // perfectly free, which is exactly what the OLD fixed-time STALL window read as "resting on a
    // stack". ARM ON MOTION (armTravelDeg) + SPLIT STALL (noStartMs) are the fix; these four scenarios
    // are what would have caught it before it ever reached the robot.
    //
    // 13. Loaded slow spin-up: only reaches ~45 deg/s, and not until ~200-250 ms in (an unloaded press
    //     is at cruise within ~70 ms -- see freeDescent's default 800 deg/s^2). Must NOT fire.
    {
        Noise n;
        Case c{"loaded slow spin-up (45 deg/s by ~200 ms)", freeDescent(45.0, -3500.0, 200, n, 200.0), false, -1,
               0.0};
        report(c);
    }
    // 14. Loaded free descent with a much higher EFFORT baseline (-4500 mV just to cruise, vs -1500 for
    //     an empty claw) but otherwise ordinary and steady. EFFORT measures DEVIATION from the learned
    //     baseline, not the raw mV, so a high but steady cost must not fire on its own.
    {
        Noise n;
        Case c{"loaded free descent, high effort baseline", freeDescent(70.0, -4500.0, 200, n), false, -1, 0.0};
        report(c);
    }
    // 15. Loaded LANDING: the same slow loaded spin-up as #13, then a real (if not especially hard)
    //     landing. Must still fire, and in reasonable time -- the fix must not cost real detections.
    {
        Noise n;
        std::vector<Sample> t = freeDescent(45.0, -3500.0, 60, n, 200.0);
        const int landing = static_cast<int>(t.size());
        appendLanding(t, 45.0, -3500.0, 10.0, 80.0, 2200.0, 150.0, 150, n);
        Case c{"loaded landing after slow spin-up", t, true, landing, 500.0};
        report(c);
    }
    // 16. Loaded, NEVER starts -- a pin+cup held against something immovable from the first tick
    //     (distinct from #7's unloaded resting case only in the effort baseline it pushes at). Must
    //     still fire eventually -- noStartMs is a delay, never a disable -- just not before it.
    {
        Noise n;
        std::vector<Sample> t;
        for (int i = 0; i < 100; ++i) t.push_back({std::max(0.0, 1.0 + n.next(1.5)), -5000.0 + n.next(250.0)});
        Case c{"loaded, never starts (held against something)", t, true, 0, 420.0};
        report(c);
    }

    std::printf("\n%s\n", failures == 0 ? "ALL PASS" : "FAILURES PRESENT");
    return failures == 0 ? 0 : 1;
}
