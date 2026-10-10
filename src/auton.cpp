#include "auton.hpp"

#include "config.hpp"
#include "periodic.hpp"
#include "robot.hpp"
#include "screen.hpp"

#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <atomic>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <memory>
#include <algorithm>
#include <mutex>

namespace auton {

using subsystems::Intake;
using subsystems::Scoring;
namespace preset = config::lift_preset;

namespace {

// --- the auton tick task ---------------------------------------------------
std::unique_ptr<pros::Task> tickTask;
std::atomic<bool> tickRun{false};
std::atomic<bool> tickExited{true};

// --- what the routine has asked the tick to do -----------------------------
// The intake request is LATCHED (it stays until changed), exactly like holding R1; the three button
// "presses" are one-shot pulses consumed by the next tick, exactly like a new-press edge.
std::atomic<int> intakeRequest{static_cast<int>(Intake::Mode::STOP)};
std::atomic<int> intakePowerPermille{1000};
std::atomic<bool> pendingStepUp{false};
std::atomic<bool> pendingScore{false};
std::atomic<bool> pendingAbort{false};

/** Shorthand for "take the lock that keeps Scoring/Claw transitions atomic against the tick task"
 *  (see periodic.hpp). Every command below that touches Scoring or the Claw holds it. */
#define AUTON_LOCK std::lock_guard<pros::Mutex> autonGuard(periodic::mutex())

/** Nothing but `done` and the timeout may end a wait during its first few hundred milliseconds.
 *  WHY: the macro operations (stepUp / goToIntake / stow) do not command the lift themselves -- they
 *  change Scoring's state, and Scoring::update() issues the height on a later tick (CLAMPING alone
 *  holds it for CLAW_CLOSE_TIME_MS, and a pivot swing for PIVOT_TIME_MS). Judging "no active target"
 *  or a stale BLOCKED in that gap would report failure before the move had even been asked for. */
constexpr std::uint32_t kJudgeAfterMs = 400;

/** A lift wait that cannot hang: it ends on `done`, on a lift that has given up, or on the timeout.
 *  Returns what `done` said at the moment it stopped. timeoutMs <= 0 just samples once. */
template <typename DonePredicate>
bool waitLiftUntil(DonePredicate done, int timeoutMs) {
    const std::uint32_t start = pros::millis();
    while (true) {
        if (done()) return true;
        // While homing, there is no active target (it is queued) and BLOCKED/FAULT are not meaningful,
        // so only judge those once homing is out of the way -- the same rule Lift::moveTo() uses.
        if (!robot::lift.isHoming() && pros::millis() - start >= kJudgeAfterMs) {
            if (robot::lift.isBlocked() || robot::lift.isFaulted()) return false;
            if (!robot::lift.hasActiveTarget() && !robot::lift.isScoring()) return false;
        }
        if (timeoutMs <= 0 || pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) return done();
        pros::delay(10);
    }
}

/** Settled on this exact commanded height (not an older one) -- the same test Scoring uses. */
bool settledAt(double degrees) {
    return robot::lift.isSettled() && std::fabs(robot::lift.getTarget() - degrees) < 0.01;
}

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void begin() {
    // Clear last run's requests first, so a task that is already running can never act on them.
    intakeRequest.store(static_cast<int>(Intake::Mode::STOP));
    intakePowerPermille.store(1000);
    pendingStepUp.store(false);
    pendingScore.store(false);
    pendingAbort.store(false);
    periodic::reset();
    screen::rMode = screen::RMode::IDLE;  // R1/R2 belong to the driver; auton owns neither

    if (tickTask) return;  // already ticking: begin() is idempotent
    tickRun.store(true);
    tickExited.store(false);
    tickTask = std::make_unique<pros::Task>(
        [] {
            while (tickRun.load()) {
                periodic::Inputs in;
                in.intakeRequest = static_cast<Intake::Mode>(intakeRequest.load());
                in.intakePower = intakePowerPermille.load() / 1000.0;
                // One-shot edges: exchange, so exactly one tick ever sees each press.
                in.stepUpPressed = pendingStepUp.exchange(false);
                in.scorePressed = pendingScore.exchange(false);
                in.abort = pendingAbort.exchange(false);
                in.manualLift = false;      // there is no D-pad in auton
                in.liftButtonMode = false;  // nor clamped-mode R1/R2
                in.auton = true;            // the autoclamp's auton trigger range (65 mm, not 55)
                periodic::tick(in);
                pros::delay(config::AUTON_TICK_MS);
            }
            tickExited.store(true);
        },
        TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "auton-tick");
}

void end() {
    if (!tickTask) return;
    tickRun.store(false);
    // BLOCK until the task has really left periodic::tick(). This is what makes "never two writers"
    // true at the auton -> driver switch: opcontrol()'s loop does not start until this returns.
    const std::uint32_t start = pros::millis();
    while (!tickExited.load() && pros::millis() - start < 200) pros::delay(5);
    if (!tickExited.load()) tickTask->remove();  // backstop only; a tick that never returns is a bug
    tickTask.reset();
    // Leave the lift's TARGET exactly as it is (its own task keeps holding/seating it -- CLAUDE.md
    // rule 2 and "every state holds"), but do not hand the driver a spinning roller...
    intakeRequest.store(static_cast<int>(Intake::Mode::STOP));
    robot::intake.set(Intake::Mode::STOP);
    // ...and do not hand them an arm still leaning on the floor either. liftPush() is LATCHED plain
    // voltage, so a route that forgot to stop it -- or one the field switch cut off partway -- would
    // otherwise keep pressing down for the rest of the match. Releasing it brake-holds where the arm is
    // (Lift::update()'s manual path), which is the same thing letting go of the D-pad does; it does not
    // resume an old target and it does not move the arm.
    robot::lift.driverPower(0.0);
}

bool running() { return tickTask != nullptr && !tickExited.load(); }

void wait(int ms) {
    if (ms > 0) pros::delay(static_cast<std::uint32_t>(ms));
}

// ---------------------------------------------------------------------------
// Lift
// ---------------------------------------------------------------------------
double heightOf(Level level) {
    // ZERO is its own constant, not an array entry -- checked BEFORE the clamp below, which would
    // otherwise fold it onto level 1 (its index is -1).
    if (level == Level::ZERO) return config::lift_preset::STACK_LEVEL_ZERO;
    int i = static_cast<int>(level) - 1;
    if (i < 0) i = 0;
    if (i >= preset::STACK_LEVEL_COUNT) i = preset::STACK_LEVEL_COUNT - 1;
    return preset::STACK_LEVELS[i];
}

void homeLift() {
    AUTON_LOCK;
    // Only the lift moves: no pivot swing, no claw. Returns at once; the lift task does the work.
    robot::lift.requestHome();
}

bool waitHome(int timeoutMs) {
    const std::uint32_t start = pros::millis();
    while (robot::lift.isHoming()) {
        if (timeoutMs <= 0 || pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) break;
        pros::delay(10);
    }
    return robot::lift.isHomed();
}

void liftTo(double degrees, bool precise) {
    AUTON_LOCK;
    robot::scoring.reset();  // a direct height command takes the macro out of the way first
    // A ROUTE'S TRIP TO THE BOTTOM ALWAYS FORCES ITSELF DOWN, and says so here rather than relying on
    // Lift's auton mirror having been pushed already (periodic::tick() sets it, and the very first
    // lift call of a route can beat the tick task's first pass). This is the same test setTarget()
    // uses to decide a goal is a banded descent rather than a position move, so liftDown(),
    // liftToMatchload() and a liftTo(0) in a route all get it, and nothing above the bottom does.
    if (!precise && degrees <= config::LIFT_BOTTOM_GOAL_DEG) robot::lift.forceNextDescent();
    robot::lift.setTarget(degrees, precise);
}

void liftTo(Level level) { liftTo(heightOf(level)); }
void liftToIntake() { liftTo(preset::INTAKE, true); }
void liftToMatchload() { liftTo(preset::MATCHLOAD); }
void liftToFirstScore() { liftTo(preset::FIRST_SCORE); }
void liftToFrontToggle() {
    AUTON_LOCK;
    robot::scoring.reset();
    // COARSE on purpose: this height only has to be TOUCHED before the arm comes straight back down,
    // so it brakes late, keeps a quick tail and is allowed to overshoot -- noticeably faster than an
    // ordinary rise to a stack level. See Lift::setTargetFast() and "A COARSE move" in config.hpp.
    robot::lift.setTargetFast(preset::FRONT_TOGGLE);
}
void liftDown() { liftTo(preset::DOWN); }

void liftDownFast() {
    AUTON_LOCK;
    robot::scoring.reset();  // a direct height command takes the macro out of the way first
    // Forced, for the same reason as liftTo() above -- and the one-shot survives descendFast()'s own
    // wait for a FRONT_TOGGLE rise to finish, because only setTarget() consumes it.
    robot::lift.forceNextDescent();
    robot::lift.descendFast();
}

void liftStop() {
    AUTON_LOCK;
    robot::scoring.reset();
    robot::lift.cancel();
}

void liftPush(double percent) {
    AUTON_LOCK;
    // The macro must let go first: this is manual voltage, and Scoring would otherwise keep re-issuing
    // the height it thinks the arm should be at. (Lift::driverPower() cancels the TARGET on its own; it
    // cannot cancel the state machine that keeps setting one.)
    robot::scoring.reset();
    if (percent < 0.0) percent = -percent;   // "-40%" is a direction, not a sign the caller has to get right
    if (percent > 100.0) percent = 100.0;
    robot::lift.driverPower(-percent / 100.0);  // negative = down, the same path the D-pad feeds
}

void liftPushStop() {
    AUTON_LOCK;
    robot::lift.driverPower(0.0);  // brake-holds right here; no old target resumes
}

void liftPushFor(int ms, double percent) {
    liftPush(percent);
    wait(ms);
    liftPushStop();
}

void liftPushUp(double percent) {
    AUTON_LOCK;
    robot::scoring.reset();  // same reason as liftPush(): the macro must stop re-issuing a height
    if (percent < 0.0) percent = -percent;  // a magnitude; up is the only direction
    if (percent > 100.0) percent = 100.0;
    robot::lift.driverPower(percent / 100.0);  // positive = up, the same path the D-pad feeds
}

void liftPushUpFor(int ms, double percent) {
    liftPushUp(percent);
    wait(ms);
    liftPushStop();
}

bool waitLift(int timeoutMs) {
    return waitLiftUntil([] { return robot::lift.isSettled(); }, timeoutMs);
}

bool waitLiftSeated(int timeoutMs) {
    return waitLiftUntil([] { return robot::lift.isSeated(); }, timeoutMs);
}

bool liftToAndWait(double degrees, int timeoutMs, bool precise) {
    liftTo(degrees, precise);
    const double goal = robot::lift.getTarget();  // the clamped value the lift actually took
    return waitLiftUntil([goal] { return settledAt(goal); }, timeoutMs);
}

bool liftToAndWait(Level level, int timeoutMs) { return liftToAndWait(heightOf(level), timeoutMs); }

bool liftToIntakeAndWait(int timeoutMs) { return liftToAndWait(preset::INTAKE, timeoutMs, true); }

bool liftDownAndWait(int timeoutMs) {
    liftDown();
    return waitLiftSeated(timeoutMs);
}

bool liftToFrontToggleAndWait(int timeoutMs) { return liftToAndWait(preset::FRONT_TOGGLE, timeoutMs); }

bool liftToFirstScoreAndWait(int timeoutMs) { return liftToAndWait(preset::FIRST_SCORE, timeoutMs); }

bool liftDownFastAndWait(int timeoutMs) {
    liftDownFast();
    return waitLiftSeated(timeoutMs);
}

double liftPosition() { return robot::lift.getPosition(); }
double liftTarget() { return robot::lift.getTarget(); }
bool liftSettled() { return robot::lift.isSettled(); }
bool liftSeated() { return robot::lift.isSeated(); }
bool liftSeatedShort() { return robot::lift.isSeated() && robot::lift.isSeatShort(); }
bool liftBlocked() { return robot::lift.isBlocked(); }
bool liftFaulted() { return robot::lift.isFaulted(); }
bool liftHoming() { return robot::lift.isHoming(); }
bool liftHomed() { return robot::lift.isHomed(); }
const char* liftPhase() { return robot::lift.phaseName(); }

// ---------------------------------------------------------------------------
// Claw and pivot
// ---------------------------------------------------------------------------
void clawGrip() {
    AUTON_LOCK;
    if (!robot::claw.isClosed()) robot::scoring.toggleClaw();  // the exact A path
}

void clawOpen() {
    AUTON_LOCK;
    if (robot::claw.isClosed()) robot::scoring.toggleClaw();
}

void clawToggle() {
    AUTON_LOCK;
    robot::scoring.toggleClaw();
}

bool clawClamped() { return robot::claw.isClamped(); }

bool gripAndWait(int timeoutMs) {
    clawGrip();
    // The cylinder is commanded instantly; what has to be waited out is the jaws physically shutting,
    // which is CLAW_CLOSE_TIME_MS -- the same interlock CLAMPING enforces before the arm pulls away.
    const int settle = config::CLAW_CLOSE_TIME_MS;
    wait(timeoutMs > 0 && timeoutMs < settle ? timeoutMs : settle);
    return robot::claw.isClamped();
}

bool openAndWait(int timeoutMs) {
    clawOpen();
    const int settle = config::CLAW_OPEN_TIME_MS;
    wait(timeoutMs > 0 && timeoutMs < settle ? timeoutMs : settle);
    return !robot::claw.isClosed();
}

void grab() {
    AUTON_LOCK;
    robot::scoring.grabHere();
}

bool grabAndWait(int timeoutMs) {
    grab();
    const std::uint32_t start = pros::millis();
    // The push, then the arm settling into the hold it ends in.
    while (robot::scoring.isGrabbing() || robot::lift.isGrabbing() || !robot::lift.isSettled()) {
        if (robot::lift.isFaulted()) return false;
        if (timeoutMs <= 0 || pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) break;
        pros::delay(10);
    }
    return robot::claw.isClamped();
}

bool waitUntilClamped(int timeoutMs, bool stopChassis) {
    const std::uint32_t start = pros::millis();
    while (!robot::claw.isClamped()) {
        if (timeoutMs <= 0 || pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) {
            return false;  // never clamped: leave whatever motion is running alone
        }
        pros::delay(10);
    }
    // Clamped. Stop the drive FIRST if asked -- the point of the option is that the robot does not
    // keep pushing into the pin it has just gripped -- then let the jaws finish closing.
    if (stopChassis) robot::chassis.cancelMotion();
    wait(config::CLAW_CLOSE_TIME_MS);
    return true;
}

void pivotOut(bool waitForIt) {
    {
        AUTON_LOCK;
        robot::scoring.pivotOut();
    }
    if (waitForIt) wait(config::PIVOT_TIME_MS);
}

void pivotBack(bool waitForIt) {
    {
        AUTON_LOCK;
        robot::scoring.pivotBack();
    }
    if (waitForIt) wait(config::PIVOT_TIME_MS);
}

bool pivotIsOut() { return robot::scoring.isPivotOut(); }

void autoclamp(bool on) {
    AUTON_LOCK;
    robot::scoring.setAutoclamp(on);
}

bool autoclampOn() { return robot::scoring.autoclampOn(); }

// ---------------------------------------------------------------------------
// Intake
// ---------------------------------------------------------------------------
namespace {
void requestIntake(Intake::Mode mode, double power) {
    if (power < 0.0) power = 0.0;
    if (power > 1.0) power = 1.0;
    intakePowerPermille.store(static_cast<int>(power * 1000.0));
    intakeRequest.store(static_cast<int>(mode));
}
}  // namespace

void intakeIn(double power) { requestIntake(Intake::Mode::IN, power); }
void intakeOut(double power) { requestIntake(Intake::Mode::OUT, power); }
void intakeStop() { requestIntake(Intake::Mode::STOP, 1.0); }

void intakeFor(int ms, Spin dir, double power) {
    // Direction first, so a 0 ms call is an explicit "stop" rather than a brief twitch.
    if (ms <= 0) {
        intakeStop();
        return;
    }
    if (dir == Spin::IN) intakeIn(power);
    else intakeOut(power);
    wait(ms);
    intakeStop();  // never leave the request latched: the macros decide again from here
}

void intakeInFor(int ms, double power) { intakeFor(ms, Spin::IN, power); }

void intakeOutFor(int ms, double power) { intakeFor(ms, Spin::OUT, power); }

// ---------------------------------------------------------------------------
// The driver's macro operations
// ---------------------------------------------------------------------------
void goToIntake() {
    AUTON_LOCK;
    robot::scoring.goToIntake();
}

bool goToIntakeAndWait(int timeoutMs) {
    goToIntake();
    return waitLiftUntil(
        [] { return robot::scoring.getState() == Scoring::State::INTAKE_POS; }, timeoutMs);
}

void stepUp() {
    {
        AUTON_LOCK;
        robot::scoring.stepUp();
    }
    pendingStepUp.store(true);  // lets the tick open L1's auto-intake sweep, exactly like the button
}

bool stepUpAndWait(int timeoutMs) {
    stepUp();
    const int lvl = robot::scoring.getLevel();
    const double goal = lvl >= 1 ? heightOf(static_cast<Level>(lvl)) : preset::STACK_LEVELS[0];
    return waitLiftUntil([goal] { return settledAt(goal); }, timeoutMs);
}

void score() {
    {
        AUTON_LOCK;
        robot::scoring.score();
    }
    pendingScore.store(true);
}

bool waitScore(int timeoutMs) {
    const std::uint32_t start = pros::millis();
    // THE SCORE IS DONE WHEN THE PIN IS OFF AND THE ARM IS ON ITS WAY DOWN -- not when it reaches the
    // bottom (driver, 2026-10-07: "score and wait takes too long rn"). Lift::isScoring() covers
    // PRESS + RELEASE + the L2_RELEASE_SETTLE_MS pause and goes false the tick the descent is
    // commanded, so this returns there and the route can drive away while the arm runs home on its own.
    //
    // WHAT WAS WAITED ON BEFORE, and why it got slow: isScoreDescent() stays true for the whole of
    // Scoring::RELEASING, which since the one-press score (2026-10-07) runs all the way to DOWN --
    // so this used to block for the full descent AND the FINAL forcing window AND the seat. The
    // isScoredAbove() term is dead too: nothing holds above a stack any more.
    // A route that really does want the arm parked at the bottom first calls auton::waitLiftSeated().
    // (An L2 at the bottom is just a claw toggle: both tests are false at once and this returns true
    // immediately, as it always did.)
    while (robot::scoring.isScorePending() || robot::lift.isScoring()) {
        if (robot::lift.isScoreStuck() || robot::lift.isFaulted()) return false;
        if (timeoutMs <= 0 || pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) return false;
        pros::delay(10);
    }
    return !robot::lift.isScoreStuck() && !robot::lift.isFaulted();
}

void scoreReturn() {
    AUTON_LOCK;
    robot::scoring.score();  // from the held-above state, Scoring::score() IS press 2
}

bool scoreReturnAndWait(int timeoutMs) {
    scoreReturn();
    return waitLiftSeated(timeoutMs);
}

bool scoredUp() { return robot::scoring.isScoredUp(); }

bool scoreAndWait(int timeoutMs) {
    score();
    return waitScore(timeoutMs);
}

bool scoreStuck() { return robot::lift.isScoreStuck(); }

void matchload() {
    AUTON_LOCK;
    robot::scoring.scoreOrMatchload();
}

void stow() {
    {
        AUTON_LOCK;
        robot::scoring.stow();
    }
    pendingAbort.store(true);  // B cancels both macro windows, exactly like the button
}

bool stowAndWait(int timeoutMs) {
    stow();
    return waitLiftSeated(timeoutMs);
}

void resetMacro() {
    AUTON_LOCK;
    robot::scoring.reset();
}

void resetLevel() {
    AUTON_LOCK;
    robot::scoring.resetLevel();
}

int level() { return robot::scoring.getLevel(); }
const char* macroState() { return robot::scoring.stateName(); }

// ---------------------------------------------------------------------------
// Start-of-auton alignment (see auton.hpp for the full description)
// ---------------------------------------------------------------------------
namespace {

/** One validated reading, mm, or -1. The same rule the team's old accumulateReading() used: reject
 *  anything past START_ALIGN_MAX_MM or under START_ALIGN_MIN_CONFIDENCE. PROS_ERR (negative), the
 *  sensor's "nothing there" 9999 and a 0 (cannot measure) all fall out of that test on their own. */
double validMm(pros::Distance& sensor) {
    const std::int32_t mm = sensor.get();
    if (mm <= 0 || mm > config::START_ALIGN_MAX_MM) return -1.0;
    const std::int32_t conf = sensor.get_confidence();
    if (conf < config::START_ALIGN_MIN_CONFIDENCE) return -1.0;
    return static_cast<double>(mm);
}

// The pre-match rolling history, filled by startAlignSample() from disabled() and read by the auton
// task and the brain screen -- hence the mutex.
constexpr int kHistorySize = 64;  // START_ALIGN_HISTORY_MS / 20 ms, with room to spare
struct Sample {
    std::uint32_t ms;
    double mm;
};
Sample history[kHistorySize];
int historyCount = 0;
int historyNext = 0;
pros::Mutex historyGate;

// What the last startPose() did, for the brain's START line.
std::atomic<bool> poseRan{false};
std::atomic<bool> poseAligned{false};
std::atomic<int> poseShiftThou{0};  // thousandths of an inch, so it fits an atomic int
std::atomic<int> poseLeftMm{-1};
// ...and the same three for the back sensor's Y correction.
std::atomic<bool> poseBackApplied{false};
std::atomic<bool> poseBackRequested{false};
std::atomic<int> poseShiftYThou{0};
std::atomic<int> poseBackMm{-1};

/** The settled average of the history, or -1. "Settled" = enough samples, recent enough, and tight
 *  enough that nothing was moving while they were taken (a hand still placing the robot, or someone
 *  walking through the beam, spreads the window and disqualifies it). */
double historyAverage() {
    std::lock_guard<pros::Mutex> guard(historyGate);
    if (historyCount < 5) return -1.0;
    const std::uint32_t now = pros::millis();
    double sum = 0.0, lo = 1e9, hi = -1e9;
    int n = 0;
    for (int i = 0; i < historyCount; ++i) {
        const Sample& sm = history[i];
        if (now - sm.ms > static_cast<std::uint32_t>(config::START_ALIGN_HISTORY_MS)) continue;
        sum += sm.mm;
        lo = std::min(lo, sm.mm);
        hi = std::max(hi, sm.mm);
        ++n;
    }
    if (n < 5) return -1.0;
    if (hi - lo > config::START_ALIGN_HISTORY_SPREAD_MM) return -1.0;  // not settled: something moved
    return sum / n;
}

}  // namespace

double leftDistanceMm() { return validMm(robot::leftDistance); }
double rightDistanceMm() { return validMm(robot::rightDistance); }

double readLeftAveraged(int ms) {
    const std::uint32_t start = pros::millis();
    double sum = 0.0;
    int n = 0;
    do {
        const double mm = validMm(robot::leftDistance);
        if (mm > 0.0) {
            sum += mm;
            ++n;
        }
        pros::delay(10);
    } while (ms > 0 && pros::millis() - start < static_cast<std::uint32_t>(ms));
    return n > 0 ? sum / n : -1.0;
}

double startBackPreview() { return validMm(robot::backDistance); }

double readBackAveraged(int ms) {
    // Identical to readLeftAveraged() but on the rear sensor. There is no disabled-mode rolling
    // history for this one, so this fresh average is the only source and it always costs its ms.
    const std::uint32_t start = pros::millis();
    double sum = 0.0;
    int n = 0;
    do {
        const double mm = validMm(robot::backDistance);
        if (mm > 0.0) {
            sum += mm;
            ++n;
        }
        pros::delay(10);
    } while (ms > 0 && pros::millis() - start < static_cast<std::uint32_t>(ms));
    return n > 0 ? sum / n : -1.0;
}

void startAlignSample() {
    const double mm = validMm(robot::leftDistance);
    if (mm <= 0.0) return;
    std::lock_guard<pros::Mutex> guard(historyGate);
    history[historyNext] = Sample{pros::millis(), mm};
    historyNext = (historyNext + 1) % kHistorySize;
    if (historyCount < kHistorySize) ++historyCount;
}

double startAlignPreview() {
    {
        std::lock_guard<pros::Mutex> guard(historyGate);
        if (historyCount == 0) return -1.0;
        const int last = (historyNext + kHistorySize - 1) % kHistorySize;
        if (pros::millis() - history[last].ms > static_cast<std::uint32_t>(config::START_ALIGN_HISTORY_STALE_MS)) {
            return -1.0;  // nobody has sampled for a while: this is not this match's setup
        }
    }
    return historyAverage();
}

double startShiftIn() { return poseShiftThou.load() / 1000.0; }
double startLeftMm() { return poseLeftMm.load(); }
bool startAligned() { return poseAligned.load(); }
bool startPoseRan() { return poseRan.load(); }
double startShiftYIn() { return poseShiftYThou.load() / 1000.0; }
double startBackMm() { return poseBackMm.load(); }
bool startBackApplied() { return poseBackApplied.load(); }
bool startBackRequested() { return poseBackRequested.load(); }

bool startPose(double x, double y, double thetaDeg, double expectedLeftMm, BackY backY) {
    // 1. CALIBRATION FIRST. chassis.calibrate() resets the pose, so applying one before it finishes
    //    would simply be erased. autonomous() already waits (bounded) before the route runs; this is
    //    the backstop for the case where that wait timed out.
    const std::uint32_t waitStart = pros::millis();
    while (!screen::calibrated &&
           pros::millis() - waitStart < static_cast<std::uint32_t>(config::START_ALIGN_CALIBRATE_WAIT_MS)) {
        pros::delay(10);
    }
    const bool calibratedInTime = screen::calibrated;

    // 2. THE READING. Prefer the long, settled average taken while the robot sat disabled -- it costs
    //    no auton time and is a far better average than a route could afford -- but confirm it with a
    //    short fresh sample, because the robot may have been moved since. If they disagree, the fresh
    //    one is the newer truth and wins.
    //
    //    SKIPPED ENTIRELY when expectedLeftMm <= 0, i.e. the caller asked for a plain pose with no
    //    start alignment. The sensor is not read AT ALL in that case -- which matters because
    //    readLeftAveraged() BLOCKS for START_ALIGN_SAMPLE_MS (300 ms), and spending 300 ms of the
    //    autonomous period averaging a reading that step 3 then discards is pure waste. Before
    //    2026-10-07 the sample happened regardless and only the correction was gated.
    double measured = -1.0;
    if (expectedLeftMm > 0.0) {
        const double preview = startAlignPreview();
        const double confirm = readLeftAveraged(preview > 0.0 ? config::START_ALIGN_CONFIRM_MS
                                                              : config::START_ALIGN_SAMPLE_MS);
        if (preview > 0.0 && confirm > 0.0) {
            measured =
                std::fabs(preview - confirm) <= config::START_ALIGN_CONFIRM_TOL_MM ? preview : confirm;
        } else if (confirm > 0.0) {
            measured = confirm;
        } else if (preview > 0.0) {
            measured = preview;  // the sensor just went invalid; the pre-match average is all there is
        }
    }

    // 3. THE CORRECTION, and the guard. Further than expected = shifted +x (to the robot's right),
    //    closer = -x. Anything bigger than START_ALIGN_MAX_IN is the sensor looking at a goal or the
    //    wrong wall, not a hand-placement error, so it is dropped rather than trusted.
    double dxIn = 0.0;
    bool applied = false;
    if (measured > 0.0 && expectedLeftMm > 0.0) {
        const double candidate = (measured - expectedLeftMm) / 25.4;
        if (std::fabs(candidate) <= config::START_ALIGN_MAX_IN) {
            dxIn = candidate;
            applied = true;
        }
    }

    // 3b. THE Y CORRECTION, from the back sensor, and its own gate. Independent of the X alignment
    //     above: this route may use one, both or neither. The window is a HARD GATE -- a reading
    //     outside it is dropped entirely and Y stays nominal, rather than being clamped to the edge,
    //     because a sensor aimed at a robot instead of the wall must not drag the pose part-way.
    //     Reading FURTHER than expected = +y, CLOSER = -y, equal = 0. The driver's own convention.
    double backMeasured = -1.0;
    double dyIn = 0.0;
    bool backApplied = false;
    if (backY.expectedMm > 0.0) {
        backMeasured = readBackAveraged(config::MATCHLOAD_BACK_SAMPLE_MS);
        if (backMeasured > 0.0 && backMeasured >= backY.minMm && backMeasured <= backY.maxMm) {
            dyIn = (backMeasured - backY.expectedMm) / 25.4;
            backApplied = true;
        }
    }

    // 4. Apply it along the ROBOT'S RIGHT AXIS. In LemLib's frame theta is clockwise from +y, so the
    //    robot's right unit vector is (cos theta, -sin theta); at theta = 0 this is exactly x + dx,
    //    which is the driver's own description. Writing it this way keeps it physically right if a
    //    route ever starts facing another way.
    //    The back sensor's dy goes straight onto Y with NO rotation: it is only meaningful at a known
    //    heading, and every route using it starts at theta 0, where the field Y axis and the robot's
    //    own fore-aft axis are the same line.
    const double rad = thetaDeg * M_PI / 180.0;
    const double px = x + dxIn * std::cos(rad);
    const double py = y - dxIn * std::sin(rad) + dyIn;
    robot::chassis.setPose(px, py, thetaDeg);

    poseRan.store(true);
    poseAligned.store(applied);
    poseShiftThou.store(static_cast<int>(dxIn * 1000.0));
    poseLeftMm.store(measured > 0.0 ? static_cast<int>(measured) : -1);
    poseBackRequested.store(backY.expectedMm > 0.0);
    poseBackApplied.store(backApplied);
    poseShiftYThou.store(static_cast<int>(dyIn * 1000.0));
    poseBackMm.store(backMeasured > 0.0 ? static_cast<int>(backMeasured) : -1);

    // 5. If calibration had NOT finished when we got here, it will reset the pose when it does. Put it
    //    back, once, from a short-lived task so the route is not held up.
    if (!calibratedInTime) {
        static std::unique_ptr<pros::Task> reapply;
        reapply = std::make_unique<pros::Task>(
            [px, py, thetaDeg] {
                const std::uint32_t start = pros::millis();
                while (!screen::calibrated &&
                       pros::millis() - start < static_cast<std::uint32_t>(config::START_ALIGN_REAPPLY_WINDOW_MS)) {
                    pros::delay(10);
                }
                if (screen::calibrated) {
                    pros::delay(20);  // let calibrate()'s own reset land first
                    robot::chassis.setPose(px, py, thetaDeg);
                }
            },
            TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "start-pose-reapply");
    }
    // True if EITHER axis was corrected. px/py captured by the re-apply task above already carry both,
    // so a late calibration cannot wipe the Y correction either.
    return applied || backApplied;
}

// ---------------------------------------------------------------------------
// moveB — the rear-distance-sensor PD drive, ported 1:1 (see auton.hpp)
// ---------------------------------------------------------------------------
void moveB(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeOutMs) {
    // NEVER two writers on the drive: this function writes leftDrive/rightDrive itself, so no LemLib
    // motion may be in flight underneath it. And it is an AUTON move -- the driver loop owns the drive
    // in opcontrol, so refuse unless the auton tick is up.
    if (!running()) {
        // SAY SO ON THE BRAIN, not just the terminal. This refusal looks exactly like a dead robot --
        // moveB returns before it writes a single volt, and before the result line below -- so without
        // this the only symptom is "the bot doesn't move" with a blank line 9. The usual cause is a
        // moveB call placed ABOVE auton::begin() in autonomous(): the tick task does not exist yet, so
        // running() is false. The guard itself is deliberate and stays: moveB writes leftDrive and
        // rightDrive directly, so it must never run while the driver loop owns the drive.
        pros::screen::print(pros::E_TEXT_MEDIUM, 9,
                            "!! MOVEB REFUSED: auton::begin() first    ");
        std::printf("moveB refused: not in autonomous (auton::begin() has not run)\n");
        return;
    }
    robot::chassis.cancelAllMotions();
    while (robot::chassis.isInMotion()) pros::delay(10);

    const double kP = config::MOVEB_KP;
    const double kD = config::MOVEB_KD;
    const double tolerance = config::MOVEB_TOLERANCE_MM;  // mm
    const int settleTime = config::MOVEB_SETTLE_MS;       // ms
    double prevError = 0;
    double error = 0;
    const std::uint32_t startTime = pros::millis();
    std::uint32_t settleStartTime = 0;
    bool isSettled = false;
    // WHY THE MOVE ENDED, for the report on the brain when it is over. "SETTLED" is the only good one.
    const char* outcome = "TIMED OUT";
    double lastSeen = -1.0;  // the final sensor reading, mm; -1 if it never got a valid one
    while (true) {
        if (pros::millis() - startTime > static_cast<std::uint32_t>(timeOutMs)) break;
        const std::int32_t currentDistance = robot::backDistance.get();
        if (currentDistance == PROS_ERR) {
            std::printf("Back distance sensor error\n");
            outcome = "SENSOR ERR";
            break;
        }
        lastSeen = static_cast<double>(currentDistance);
        if (currentDistance >= config::MOVEB_OUT_OF_RANGE_MM) {
            // KEPT AS-IS from the old library: nothing in range, so drive blind at full commanded
            // speed until something comes into range or timeOutMs ends the move.
            const int approachPower = forwards ? maxSpeed : -maxSpeed;
            robot::leftDrive.move(approachPower);
            robot::rightDrive.move(approachPower);
            prevError = 0;
            std::printf("Back sensor out of range (%ldmm), driving at max speed\n",
                        static_cast<long>(currentDistance));
            pros::delay(config::MOVEB_LOOP_MS);
            continue;
        }
        if (decreasing) error = currentDistance - distance;
        else error = distance - currentDistance;
        if (std::fabs(error) <= tolerance) {
            if (!isSettled) {
                settleStartTime = pros::millis();
                isSettled = true;
            } else if (pros::millis() - settleStartTime >= static_cast<std::uint32_t>(settleTime)) {
                outcome = "SETTLED";
                break;
            }
        } else {
            isSettled = false;
        }
        const double derivative = error - prevError;
        double motorPower = (kP * error) + (kD * derivative);
        if (motorPower > maxSpeed) motorPower = maxSpeed;
        else if (motorPower < -maxSpeed) motorPower = -maxSpeed;
        const int effectiveMinSpeed = (minSpeed > 0) ? minSpeed : config::MOVEB_DEFAULT_MIN_SPEED;
        if (std::fabs(motorPower) > 0 && std::fabs(motorPower) < effectiveMinSpeed)
            motorPower = (motorPower > 0) ? effectiveMinSpeed : -effectiveMinSpeed;
        const int finalMotorPower = static_cast<int>(forwards ? motorPower : -motorPower);
        robot::leftDrive.move(finalMotorPower);
        robot::rightDrive.move(finalMotorPower);
        prevError = error;
        std::printf("Distance: %ldmm, Target: %.0fmm, Error: %.0fmm, Power: %d\n",
                    static_cast<long>(currentDistance), distance, error, finalMotorPower);
        pros::delay(config::MOVEB_LOOP_MS);
    }
    robot::leftDrive.move(0);
    robot::rightDrive.move(0);

    // THE RESULT, where it can actually be read. Brain line 9 (LIFT0 owns 8, warnings own 5-7) and
    // the terminal get the same figures, and the line STAYS until the next moveB -- so after a run at
    // the wall the miss is still on screen to tune against. See "TUNING moveB" in config.hpp.
    //   err is SIGNED in the same sense the loop uses: negative = stopped SHORT, positive = went PAST.
    const double finalError = lastSeen < 0.0 ? 0.0
                              : (decreasing ? lastSeen - distance : distance - lastSeen);
    const std::uint32_t took = pros::millis() - startTime;
    char seen[12];
    if (lastSeen < 0.0) std::snprintf(seen, sizeof(seen), "--");
    else std::snprintf(seen, sizeof(seen), "%.0f", lastSeen);
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "MOVEB tgt %.0f got %s err %+.0fmm %lums %s    ",
                        distance, seen, finalError, static_cast<unsigned long>(took), outcome);
    std::printf("moveB done: target %.0fmm got %smm error %+.0fmm in %lums %s\n", distance, seen,
                finalError, static_cast<unsigned long>(took), outcome);
}

int backDistanceMm() { return robot::backDistance.get(); }

}  // namespace auton

#undef AUTON_LOCK
