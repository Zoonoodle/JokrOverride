#pragma once
/**
 * ContactDetector — "has the pin landed on the stack yet?" for the L2 scorer.
 *
 * PURE ON PURPOSE. No PROS, no motors, no clock, no I/O: it is fed one tick at a time and returns
 * what it thinks. That is what lets it be compiled and tested on a laptop against synthetic traces
 * (test/contact_detector_test.cpp, run with test/run.sh) instead of only on the robot, where a
 * mis-tuned threshold costs a knocked-over stack to find out about. Lift::updateScore() owns the
 * motors and calls step() once per 10 ms tick.
 *
 * WHY THE FIRST VERSION NEVER FIRED. It compared the measured speed against the NOMINAL commanded
 * speed: a relative trigger that only armed once the arm reached 85% of ~90 deg/s, and an absolute
 * trigger at 8 deg/s. Both are dead if the arm simply cannot make 90 deg/s -- and on this six-bar it
 * often cannot, because the arm swings up and BACK, so over much of the arc gravity RESISTS the
 * descent and the loop has to push down against it with only L2_PRESS_MAX_MV of authority. A free
 * descent settling at, say, 50 deg/s never arms the relative trigger (50 < 76.5) and never trips the
 * absolute one (50 > 8); then the landing slows it to 15 deg/s, which is still over 8. Nothing fires,
 * and the arm presses on until it reaches the bottom and reports NO CONTACT.
 *
 * So EVERY signal here is measured against what the arm was ACTUALLY DOING a moment ago -- a learned
 * baseline -- not against what it was told to do. Three signals, any one of which fires:
 *
 *   LAG     a LEAKY one-sided integral of the speed SHORTFALL against the baseline, in degrees: how
 *           far behind where it would have been the arm now is. This is the "position vs expected
 *           position" signal, and it is the sensitive one: it catches a landing that only slows the
 *           arm by a quarter, because the shortfall accumulates over time rather than having to show
 *           up in one sample. It is clamped at 0 whenever the arm is at or above the baseline, and it
 *           LEAKS with time constant lagLeakMs -- which matters: a plain integrator accumulates
 *           without bound on any persistent shortfall, and a free descent does have one, because the
 *           gravity torque on this six-bar changes with angle so the speed sags slowly all the way
 *           down and the baseline EMA always trails it a little. The host test proved that: a 70 ->
 *           45 deg/s sag over 2 s false-fired a non-leaky integrator. With the leak, LAG measures a
 *           CHANGE rather than a drift: a persistent small shortfall settles at shortfall x lagLeakMs
 *           and stays under the threshold, while a landing's much bigger shortfall crosses it in a
 *           few tens of milliseconds.
 *   EFFORT  the loop is pushing DOWN harder than the free descent needed, by more than a threshold.
 *           This is the "motor work" signal -- it catches a pin that is being forced into the tower
 *           even while it still creeps, and it is usually the fastest on a hard landing.
 *   SPEED   measured below a fraction of the baseline, or an absolute near-stop. The backstop. The
 *           absolute one needs no baseline at all, so it still works for a pin that is already
 *           resting on the stack when L2 is pressed.
 *
 * THE BASELINES ARE A TWO-LEVEL GATE, and that detail is load-bearing. They are EMAs, so they track
 * the slow genuine changes in what a free descent costs (the gravity torque on this six-bar changes a
 * lot with angle) -- that tracking is what keeps LAG and EFFORT from false-firing on the way down.
 * But if they keep tracking while a GENTLE landing is developing, they follow the arm down and mask
 * exactly the thing they are supposed to catch. The host test caught this too: a landing that only
 * slowed the arm to 70% went undetected because the baseline had quietly moved to meet it.
 * So there are two levels:
 *   freezeFrac  (loose, 0.90) -- the moment the arm is even slightly below baseline, STOP LEARNING.
 *               Cheap, and wrong only in the harmless direction: a brief dip just pauses adaptation.
 *   speedFrac   (strict, 0.70) -- and the LAG / EFFORT thresholds. These FIRE.
 * A slow gravity sag never reaches freezeFrac (the baseline trails a 12 deg/s-per-second sag by only
 * a few percent), so adaptation continues; a landing crosses it immediately, adaptation stops, and the
 * shortfall then accumulates against a baseline that is holding still.
 */

#include <algorithm>
#include <cmath>

namespace subsystems {

class ContactDetector {
  public:
    enum class Trigger { NONE, STALL, LAG, EFFORT, SPEED };

    /** All of these come from config::L2_* -- see Lift::updateScore(). Kept as a plain struct so the
     *  host test can sweep them without touching config.hpp. */
    struct Config {
        double settleMs;       // spin-up allowance before the baseline is snapshotted and trusted
        double ignoreMs;       // dead time before ANY trigger may fire
        double confirmMs;      // how long a candidate must hold before it counts
        double baselineTauMs;  // baseline EMA time constant (speed and effort)
        double outputTauMs;    // low-pass on the motor output, for the effort signal
        double lagDeg;         // shortfall-integral threshold, arm degrees
        double lagLeakMs;      // the lag integrator's leak -- see the note on LAG above
        double effortMv;       // extra downward push over the baseline that counts as contact
        double speedFrac;      // measured < this fraction of the baseline speed -> SPEED trigger
        double freezeFrac;     // measured < this fraction of the baseline -> stop LEARNING (see below)
        double stallDegPerS;   // absolute near-stop, needs no baseline
    };

    void reset() {
        elapsedMs_ = 0.0;
        armed_ = false;
        baseSpeed_ = 0.0;
        baseMv_ = 0.0;
        outFilt_ = 0.0;
        haveOut_ = false;
        lag_ = 0.0;
        effort_ = 0.0;
        ratio_ = 1.0;
        candidate_ = false;
        candidateMs_ = 0.0;
        pending_ = Trigger::NONE;
        fired_ = Trigger::NONE;
    }

    /**
     * One control tick.
     *   dt        seconds since the last call.
     *   measured  the arm's measured descent speed, UNSIGNED deg/s, + = going down. Pass
     *             max(-velocity, 0): an arm moving UP is not descending at all, it is stopped or worse.
     *   outMv     the voltage actually sent, SIGNED, negative = pushing down.
     * Returns the trigger that fired on this tick, or NONE. Once something has fired it keeps
     * returning it, so a caller that misses a tick cannot miss the landing.
     */
    Trigger step(double dt, double measured, double outMv, const Config& cfg) {
        if (fired_ != Trigger::NONE) return fired_;
        if (dt <= 0.0) dt = 0.01;
        const double ms = dt * 1000.0;
        elapsedMs_ += ms;

        // Low-pass the motor output: it is KP * error plus an integral, so it is the noisiest input
        // here, and the effort signal is a comparison of two slow averages.
        if (!haveOut_) {
            outFilt_ = outMv;
            haveOut_ = true;
        } else {
            outFilt_ += std::min(1.0, ms / std::max(cfg.outputTauMs, 1.0)) * (outMv - outFilt_);
        }

        // The absolute near-stop needs no baseline, so it is live from the first tick past the dead
        // time. This is what catches a pin that is already resting on the stack when L2 is pressed.
        Trigger which = Trigger::NONE;
        if (measured < cfg.stallDegPerS) which = Trigger::STALL;

        if (!armed_) {
            // Spin-up: learn, don't judge (beyond the absolute stop above). Arming needs the settle
            // time AND an arm that is actually descending, so a stuck arm never baselines at zero --
            // if it is stuck, the STALL trigger is what fires, not a bogus baseline.
            if (elapsedMs_ >= cfg.settleMs && measured >= cfg.stallDegPerS) {
                armed_ = true;
                baseSpeed_ = measured;
                baseMv_ = outFilt_;
                lag_ = 0.0;
            }
        } else {
            // THE LAG SIGNAL: a LEAKY one-sided integral of the shortfall against the baseline.
            // Degrees of "where it should be by now minus where it is". Clamped at zero, so running at
            // or above the baseline discharges it; and leaking, so the slow speed sag of a normal
            // descent settles at a harmless steady value instead of integrating into a false trigger.
            lag_ += (baseSpeed_ - measured) * dt - lag_ * (ms / std::max(cfg.lagLeakMs, 1.0));
            if (lag_ < 0.0) lag_ = 0.0;
            // THE EFFORT SIGNAL: how much harder the loop is pushing DOWN than the free descent
            // needed. outFilt_ is signed with down negative, so a bigger push makes this grow.
            effort_ = baseMv_ - outFilt_;
            ratio_ = baseSpeed_ > 1.0 ? measured / baseSpeed_ : 1.0;

            if (which == Trigger::NONE && lag_ >= cfg.lagDeg) which = Trigger::LAG;
            if (which == Trigger::NONE && effort_ >= cfg.effortMv) which = Trigger::EFFORT;
            if (which == Trigger::NONE && ratio_ < cfg.speedFrac) which = Trigger::SPEED;

            // Learn the baselines only while nothing looks even slightly wrong -- the loose freeze
            // gate, well below any firing threshold. See "TWO-LEVEL GATE" in the class comment: this
            // is what stops the baseline chasing a gentle landing and masking it.
            if (which == Trigger::NONE && measured >= cfg.freezeFrac * baseSpeed_) {
                const double a = std::min(1.0, ms / std::max(cfg.baselineTauMs, 1.0));
                baseSpeed_ += a * (measured - baseSpeed_);
                baseMv_ += a * (outFilt_ - baseMv_);
            }
        }

        if (which != Trigger::NONE) {
            if (!candidate_) {
                candidate_ = true;
                candidateMs_ = 0.0;
                pending_ = which;  // the FIRST signal to notice is the one reported
            } else {
                candidateMs_ += ms;
            }
            if (elapsedMs_ >= cfg.ignoreMs && candidateMs_ >= cfg.confirmMs) {
                fired_ = pending_;
                return fired_;
            }
        } else {
            candidate_ = false;
            candidateMs_ = 0.0;
            pending_ = Trigger::NONE;
        }
        return Trigger::NONE;
    }

    /** True once a landing has been confirmed; stays true until reset(). */
    bool hasFired() const { return fired_ != Trigger::NONE; }
    Trigger firedTrigger() const { return fired_; }
    /** True while at least one signal is over its threshold but the confirm window hasn't elapsed. */
    bool isCandidate() const { return candidate_; }
    bool isArmed() const { return armed_; }

    // Telemetry, for the brain screen and the trace.
    double lagDeg() const { return lag_; }
    double effortMv() const { return effort_; }
    double speedRatio() const { return ratio_; }
    double baselineSpeed() const { return baseSpeed_; }

    /** 1..4 for the trace's `trg` column (0 = nothing has fired). */
    static int triggerCode(Trigger t) {
        switch (t) {
            case Trigger::NONE: return 0;
            case Trigger::STALL: return 1;
            case Trigger::LAG: return 2;
            case Trigger::EFFORT: return 3;
            case Trigger::SPEED: return 4;
        }
        return 0;
    }
    /** Short word for the brain's LIFT line. */
    static const char* triggerName(Trigger t) {
        switch (t) {
            case Trigger::NONE: return "";
            case Trigger::STALL: return "stall";
            case Trigger::LAG: return "lag";
            case Trigger::EFFORT: return "eff";
            case Trigger::SPEED: return "spd";
        }
        return "";
    }

  private:
    double elapsedMs_ = 0.0;
    bool armed_ = false;
    double baseSpeed_ = 0.0;
    double baseMv_ = 0.0;
    double outFilt_ = 0.0;
    bool haveOut_ = false;
    double lag_ = 0.0;
    double effort_ = 0.0;
    double ratio_ = 1.0;
    bool candidate_ = false;
    double candidateMs_ = 0.0;
    Trigger pending_ = Trigger::NONE;
    Trigger fired_ = Trigger::NONE;
};

}  // namespace subsystems
