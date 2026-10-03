#include "auton.hpp"

#include "config.hpp"
#include "periodic.hpp"
#include "robot.hpp"
#include "screen.hpp"

#include "pros/error.h"
#include "pros/rtos.hpp"

#include <atomic>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <memory>
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
    // Leave the lift exactly as it is (its own task keeps holding/seating it -- CLAUDE.md rule 2 and
    // "every state holds"), but do not hand the driver a spinning roller.
    intakeRequest.store(static_cast<int>(Intake::Mode::STOP));
    robot::intake.set(Intake::Mode::STOP);
}

bool running() { return tickTask != nullptr && !tickExited.load(); }

void wait(int ms) {
    if (ms > 0) pros::delay(static_cast<std::uint32_t>(ms));
}

// ---------------------------------------------------------------------------
// Lift
// ---------------------------------------------------------------------------
double heightOf(Level level) {
    int i = static_cast<int>(level) - 1;
    if (i < 0) i = 0;
    if (i >= preset::STACK_LEVEL_COUNT) i = preset::STACK_LEVEL_COUNT - 1;
    return preset::STACK_LEVELS[i];
}

void homeLift() {
    AUTON_LOCK;
    robot::scoring.pivotOut();  // home in the power-on configuration, same as main.cpp does
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
    robot::lift.setTarget(degrees, precise);
}

void liftTo(Level level) { liftTo(heightOf(level)); }
void liftToIntake() { liftTo(preset::INTAKE, true); }
void liftToMatchload() { liftTo(preset::MATCHLOAD); }
void liftDown() { liftTo(preset::DOWN); }

void liftStop() {
    AUTON_LOCK;
    robot::scoring.reset();
    robot::lift.cancel();
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

void intakeInFor(int ms, double power) {
    intakeIn(power);
    wait(ms);
    intakeStop();
}

void intakeOutFor(int ms, double power) {
    intakeOut(power);
    wait(ms);
    intakeStop();
}

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
    while (robot::scoring.isScoreDescent() || robot::lift.isScoring()) {
        if (robot::lift.isScoreStuck() || robot::lift.isFaulted()) return false;
        if (timeoutMs <= 0 || pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) return false;
        pros::delay(10);
    }
    return !robot::lift.isScoreStuck() && !robot::lift.isFaulted();
}

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
// moveB — the rear-distance-sensor PD drive, ported 1:1 (see auton.hpp)
// ---------------------------------------------------------------------------
void moveB(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeOutMs) {
    // NEVER two writers on the drive: this function writes leftDrive/rightDrive itself, so no LemLib
    // motion may be in flight underneath it. And it is an AUTON move -- the driver loop owns the drive
    // in opcontrol, so refuse unless the auton tick is up.
    if (!running()) {
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
    while (true) {
        if (pros::millis() - startTime > static_cast<std::uint32_t>(timeOutMs)) break;
        const std::int32_t currentDistance = robot::backDistance.get();
        if (currentDistance == PROS_ERR) {
            std::printf("Back distance sensor error\n");
            break;
        }
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
    std::printf("moveB complete\n");
}

int backDistanceMm() { return robot::backDistance.get(); }

}  // namespace auton

#undef AUTON_LOCK
