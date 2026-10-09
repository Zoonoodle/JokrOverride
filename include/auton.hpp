#pragma once
/**
 * auton — one readable call per lift / claw / pivot / intake operation, for autonomous().
 *
 * THE POINT. A routine should read like a script and never need to know how any of it works:
 *
 *     auton::begin();                       // starts the 10 ms subsystem tick
 *     auton::liftTo(auton::Level::THREE);   auton::waitLift();
 *     auton::score();                       auton::waitScore();
 *     auton::liftDown();                    auton::waitLiftSeated();
 *
 * Every operation goes through EXACTLY the code path the matching driver button uses -- stepUp() is
 * L1, score() is L2, goToIntake() is R1's new-press, stow() is B, clawGrip()/clawOpen() are A --
 * so auton and driver can never drift apart, and tuning one tunes both.
 *
 * BLOCKING VS NON-BLOCKING -- read this before writing a route.
 *   - A COMMAND returns immediately. `auton::liftTo(Level::TWO);` starts the arm moving and hands
 *     control straight back, so the very next line can drive. That is how the lift and the
 *     drivetrain run at the same time.
 *   - An `*AndWait` call blocks THIS task until the thing has happened (or its timeout). Use it when
 *     the next step physically depends on the result.
 *   - To wait LATER instead, put `auton::waitLift(ms)` immediately before the step that needs the arm
 *     to be there. That is the whole trick: command early, wait late.
 *
 *     chassis.moveToPoint(-17.5, -24, 1500, {.forwards = false});
 *     auton::liftTo(Level::TWO);     // rises WHILE the robot drives
 *     chassis.waitUntilDone();
 *     auton::waitLift();             // ...and only now insist it has arrived
 *     auton::scoreAndWait();
 *
 *   - THE LEMLIB GOTCHA: `chassis.move...` calls are async, but asking for a SECOND one while the
 *     first is still running BLOCKS THIS TASK until the first finishes. So a lift command written
 *     after two chassis moves does not start until the first move ends. Put the lift command between
 *     them, or use `chassis.waitUntil(inches)` to let the drive get partway before firing the lift.
 *   - You do not have to wait before scoring or grabbing: `score()`, `grab()` and L1's handoff grab
 *     all wait for an in-flight lift move to ARRIVE before they start pressing (Scoring's LIFT_SETTLE
 *     state, bounded by config::SCORING_SETTLE_WAIT_MS). Starting a press from a moving arm would
 *     feed the contact detector the move's own deceleration and measure the release-travel budget
 *     from the wrong place, so the macro simply refuses to start early.
 *   - `startPose()`, `moveB()`, `intakeInFor/OutFor()` and `wait()` block by their nature; everything
 *     else that is not named `*AndWait` returns at once.
 *
 * TWO HALVES, ON PURPOSE.
 *   Commands (liftTo, score, intakeIn, ...) never block. They return the instant the request is in.
 *   Waits (waitLift, waitScore, waitClamped, ...) block the CALLING task only, ALWAYS take a
 *   timeout, and ALWAYS return true/false -- true = the thing really happened, false = the timeout
 *   ran out (or the lift reported BLOCKED / FAULT / SCORE STUCK). A routine therefore cannot hang
 *   forever on a jammed mechanism, and it can also just not wait: command the lift, drive, and check
 *   liftSettled() when it suits. Nothing in here blocks the lift's own PID task or the tick task.
 *
 * WHY begin()/end() EXIST (CLAUDE.md safety rules 2 and 4). The lift PID has always run in auton --
 * it is its own 10 ms task from initialize(). The rest of the per-loop logic did not: Scoring's state
 * machine, the claw autoclamp and the single intake decision all lived in opcontrol()'s while loop.
 * begin() starts a 10 ms task that runs that same logic, in one copy, from periodic::tick() (see
 * include/periodic.hpp). end() stops it and BLOCKS until it has really exited, and opcontrol() and
 * disabled() both call it before touching anything -- so the auton tick and the driver loop can never
 * both be running, and the field switch from auton to driver hands ownership over rather than sharing
 * it. end() is idempotent and safe to call when nothing was ever started.
 *
 * STATE HOLDS. Nothing in here ever leaves the lift limp. A level ends in the damped HOLD, the bottom
 * ends SEATED (pressed into the hard stop at config::LIFT_SEAT_MV, not parked above it), a score ends
 * seated at the bottom, and successful homing ends seated too (Lift::updateHoming() adopts a DOWN
 * target already in SEAT). The one call that deliberately leaves a passive brake-hold with no target
 * is liftStop() -- it is for abandoning a move, not for ending a routine.
 */

#include "config.hpp"
#include "subsystems/intake.hpp"

namespace auton {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
/** Start the 10 ms auton tick (Scoring + claw autoclamp + the one intake decision) and clear the
 *  macro windows. FIRST line of autonomous(). Idempotent. Commands no motion. */
void begin();
/** Stop the auton tick and block until it has actually exited, then stop the intake. Called
 *  automatically by opcontrol() and disabled(); safe to call twice or when begin() never ran. Never
 *  touches the lift, so whatever state auton left the arm holding is still held. */
void end();
/** True while the auton tick task is running. */
bool running();

/** pros::delay(), with a name that reads in a routine. */
void wait(int ms);

// ---------------------------------------------------------------------------
// Lift — presets, arbitrary angles, and the waits
// ---------------------------------------------------------------------------
/** The five pin-stack heights, config::lift_preset::STACK_LEVELS[0..4]. */
enum class Level { ONE = 1, TWO = 2, THREE = 3, FOUR = 4, FIVE = 5 };
/** The arm degrees a Level means. */
double heightOf(Level level);

/**
 * HOME THE LIFT: drive it down onto its hard stop and zero the sensor there, which is what defines
 * the whole frame (lift_preset::DOWN is 0.0 and every height is positive, measured up from the stop).
 * Back as of 2026-10-07 at the driver's request.
 *
 * NON-BLOCKING, and that is the point: it returns the instant the request is in, and the homing
 * itself runs as a mode inside the lift's own task. So a route can call this first and carry straight
 * on -- chassis moves, the claw, the intake all run while the arm is still finding its stop. A LIFT
 * command issued during homing is not lost either: it is QUEUED and starts the moment homing ends.
 *
 * Use auton::waitHome() only if a route genuinely must not move until the frame is known.
 * A no-op once a run has homed, failed or been cancelled, so a program homes exactly once.
 * It does NOT touch the claw or the pivot -- only the lift moves.
 */
void homeLift();
/** Block until homing finishes. True only if it really HOMED (so the arm is seated on a trustworthy
 *  zero); false on HOME FAIL / CANCELLED / SKIPPED or the timeout. */
bool waitHome(int timeoutMs = config::AUTON_HOME_TIMEOUT_MS);

/**
 * MOVE THE ARM TO A HEIGHT, in arm degrees, zero at the hard stop. This is the clean "put the arm at
 * X" call -- the same move a preset uses: the ordinary fast profile, braking, and settling into a
 * hold. Prefer it over the timed liftPush*For() helpers, which are raw voltage for a duration and
 * have no idea where the arm ends up.
 *
 *     auton::liftTo(40);            // up to 40 deg, returns at once
 *     auton::liftToAndWait(40);     // ...or block until it is there
 *     auton::liftTo(0);             // all the way down (see below)
 *
 * ASKING FOR 0 (or anything at or below config::LIFT_BOTTOM_GOAL_DEG, 5) IS NOT A POSITION HOLD AT 0:
 * it is the real trip to the bottom -- the banded descent, then FINAL forcing onto the hard stop, then
 * SEAT, where the arm keeps driving down at config::LIFT_DOWN_PUSH_MV for as long as the target stays
 * there. That is the "always trying to go below 0" behaviour, and it is what you want for matchload and
 * cup pickup. Same path as auton::liftDown() and the driver's trip home.
 *
 * The target is CLAMPED to [config::lift_preset::DOWN, config::lift_preset::MAX], currently [0, 179.5],
 * so an out-of-range number is pinned rather than refused.
 *
 * Non-blocking: it returns the moment the request is in. It also drops the scoring macro
 * (Scoring::reset(), which moves nothing) so a direct height command is never fought by a macro
 * mid-sequence.
 *
 * precise = true asks for the tight LIFT_PRECISE_TOLERANCE_DEG band instead of LIFT_TOLERANCE_DEG;
 * lift_preset::INTAKE is promoted to precise automatically whoever asks for it.
 */
void liftTo(double degrees, bool precise = false);
/** Command a stack level. Non-blocking. */
void liftTo(Level level);
/** Command the precise INTAKE handoff height (slow, exact final approach -- it does NOT seat). */
void liftToIntake();
/** Command the MATCHLOAD height. */
void liftToMatchload();
/** The front-toggle height (config::lift_preset::FRONT_TOGGLE, 30 deg): an ordinary position move.
 *  Non-blocking. */
void liftToFrontToggle();
/**
 * THE FIRST-PIN SCORE HEIGHT (config::lift_preset::FIRST_SCORE, 9 deg): a low "just off the floor" lift
 * for putting the first pin down, where there is no stack under it yet and stack level 1 (31 deg) is far
 * too high. An ordinary, fast position move -- non-blocking.
 *
 * CHANGED 2026-10-07: this used to warn that auton::score() could sense nothing from here, because 9 deg
 * was below L2_RELEASE_FLOOR_DEG when that floor was a bare 10. The floor is now lift_preset::DOWN + 10
 * = -21.9, so 9.0 is ~31 deg above it and the scorer has a real travel budget to detect a landing in.
 * `REL(floor)` is no longer the guaranteed outcome. Scoring onto BARE FLOOR still has nothing to sense a
 * landing on, though, so for the first pin the explicit sequence is still the predictable one:
 *     auton::liftToFirstScoreAndWait();
 *     auton::clawOpen();
 *     auton::liftPushFor(300);   // keep pressing down so the jaws slide clear of the pin
 */
void liftToFirstScore();
/** The trip to the bottom: the banded descent that ends SEATED on the hard stop, the same one L2's
 *  run home, B and R2 use. Non-blocking. */
void liftDown();
/**
 * The FAST trip to the bottom -- the front toggle's way back down. The same banded descent, FINAL and
 * SEAT/REST as liftDown(), with only the two fast bands raised (config::LIFT_FAST_DESCENT_*), so it
 * comes down noticeably harder but still slows into the approach cruise and settles on the stop the
 * normal way. Non-blocking.
 *
 * QUEUED BEHIND A TOGGLE RISE: called while a move to FRONT_TOGGLE is still climbing, it waits (inside
 * the lift task, never blocking the route) until the arm TOUCHES the toggle height and then turns
 * straight around -- so a route can ask for the way down a few hundred ms after the way up and still
 * get a real toggle. Brain: `TOGGLE ^`, then `DESC FAST`.
 */
void liftDownFast();
/** Abandon whatever the lift is doing and brake-hold right here, with NO target active. The one call
 *  in this header that leaves a passive hold -- use it to stop a move, not to end a routine. */
void liftStop();

/**
 * PUSH THE LIFT DOWN at a constant percentage, until something stops it. Non-blocking and LATCHED, like
 * holding a button down:
 *
 *     auton::liftPush();        // presses down at config::AUTON_LIFT_PUSH_PCT (85%)
 *     auton::clawOpen();        // ...do whatever needed the arm pressed down
 *     auton::wait(300);
 *     auton::liftPushStop();    // let go: the lift brake-holds exactly where it is
 *
 * FOR SCORING A PIN AT THE VERY BOTTOM, which is the case no height command covers: there is nothing
 * left to command a move TO, the arm just has to lean on the pin while the jaws open. The driver's own
 * request (2026-10-03): "a lift movement when scoring pin at very bottom that moves lift at constant
 * -40% speed down until we stop it. just easy way to toggle in auto."
 *
 * It is plain open-loop voltage through Lift::driverPower() -- the SAME path the D-pad uses -- so:
 * it cancels any active target and the Scoring macro the instant it starts, nothing profiles or tapers
 * it, liftPushStop() brake-holds where the arm is rather than resuming an old target, and the lift's one
 * task is still the only thing writing the motors. `percent` is a MAGNITUDE: the direction is always
 * down, so liftPush(40) and liftPush(-40) do the same thing.
 *
 * YOU DO NOT HAVE TO REMEMBER TO STOP IT. auton::end() releases it too, so a route that forgets -- or
 * one the field switch cuts off mid-push -- cannot hand driver control an arm still pressing the floor.
 * But a route that leaves it running will press for as long as the route has left, so stop it.
 */
void liftPush(double percent = config::AUTON_LIFT_PUSH_PCT);
/** Let go of liftPush(): the lift brake-holds right where it is. Safe to call when nothing is pushing. */
void liftPushStop();
/** liftPush() + wait(ms) + liftPushStop(), the one-liner form. BLOCKS for ms. */
void liftPushFor(int ms, double percent = config::AUTON_LIFT_PUSH_PCT);

/**
 * THE SAME THING UPWARD -- a brief open-loop nudge UP, at config::AUTON_LIFT_PUSH_UP_PCT (40%).
 *
 * This is the direct translation of the driver's old-library lines:
 *     liftMotors.move(50);  pros::delay(300);  liftMotors.move(0);
 * becomes
 *     auton::liftPushUpFor(300);
 * (move() was on the -127..127 scale, so 50 there is ~39% of volts -- see AUTON_LIFT_PUSH_UP_PCT.)
 * Same path, same rules as liftPush(): plain voltage through Lift::driverPower(), it cancels any target
 * and the Scoring macro, liftPushStop() brake-holds where the arm is, auton::end() releases it, and
 * `percent` is a magnitude -- the direction is always UP.
 *
 * PREFER liftToFirstScore() IF WHAT YOU WANT IS A HEIGHT. A timed open-loop push lands wherever battery
 * voltage, the load in the claw and the arc put it that run; a commanded height lands on the height.
 * This exists because it is the behaviour the driver already has confidence in.
 */
void liftPushUp(double percent = config::AUTON_LIFT_PUSH_UP_PCT);
/** liftPushUp() + wait(ms) + liftPushStop(). BLOCKS for ms. `liftPushUpFor(300)` is the old
 *  move(50)/delay(300)/move(0) in one call. */
void liftPushUpFor(int ms, double percent = config::AUTON_LIFT_PUSH_UP_PCT);

/** Block until the lift arrives (HOLD at a level, or SEAT at the bottom). True = arrived; false =
 *  BLOCKED, FAULT, or the timeout. Returns false immediately if no target is active. */
bool waitLift(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);
/** Block until the arm is pressed onto its hard stop (phase SEAT, including SEAT(short)). */
bool waitLiftSeated(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);

/** liftTo() + waitLift(), the one-liner form. BLOCKS until the arm is settled on the height it
 *  actually took (the clamped one), or the timeout. True = arrived; false = BLOCKED, FAULT or timeout.
 *  liftToAndWait(0) returns once the arm is SEATED on the hard stop, not merely near 0.
 *      auton::liftToAndWait(40);   // up to 40 and wait
 *      auton::liftToAndWait(0);    // down to the stop and wait for the seat */
bool liftToAndWait(double degrees, int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS, bool precise = false);
bool liftToAndWait(Level level, int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);
/** liftToIntake() + waitLift(). The precise band is tight, so give it room. */
bool liftToIntakeAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);
/** liftDown() + waitLiftSeated(). True = really seated at the bottom. */
bool liftDownAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);
/** liftToFrontToggle() + waitLift(). */
bool liftToFrontToggleAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);
/** liftToFirstScore() + waitLift(). */
bool liftToFirstScoreAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);
/** liftDownFast() + waitLiftSeated() -- including any queued wait for the toggle height. */
bool liftDownFastAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);

/** Non-blocking status, for a routine that wants to drive while the arm moves. */
double liftPosition();
double liftTarget();
bool liftSettled();
bool liftSeated();
bool liftSeatedShort();
bool liftBlocked();
bool liftFaulted();
bool liftHoming();
bool liftHomed();
/** The brain's own phase word ("DRIVE" / "BRAKE" / "APPROACH" / "HOLD" / "DESCEND" / "FINAL" /
 *  "SEAT" / "SEAT(short)" / "PRESS" / "REL(...)" / "UNSTK", "" with no target). */
const char* liftPhase();

// ---------------------------------------------------------------------------
// Claw and pivot (Scoring owns the cylinders; these are the driver's own calls)
// ---------------------------------------------------------------------------
/** A, when the jaws are open: shut them on a pin. */
void clawGrip();
/** A, when the jaws are shut: open them (starts the autoclamp lockout). */
void clawOpen();
/** A, exactly: shut if open, open if shut. */
void clawToggle();
bool clawClamped();
/** clawGrip() + block until the jaws report CLAMPED and CLAW_CLOSE_TIME_MS has passed, so the arm
 *  never pulls out of a claw that is still closing. True = gripped. */
bool gripAndWait(int timeoutMs = config::AUTON_CLAW_TIMEOUT_MS);

/**
 * THE GRAB -- "grab it right here". The same force-down-and-close sequence L1 does at the intake
 * handoff: the arm presses DOWN at config::L1_GRAB_PUSH_PCT, the jaws shut L1_GRAB_CLOSE_AT_MS into
 * the push, the push continues until both L1_GRAB_PUSH_MS and the jaws' own CLAW_CLOSE_TIME_MS are
 * satisfied -- and then the arm HOLDS exactly where the push ended, instead of rising to a level the
 * way L1 does. The intake is stopped for the whole push. Use this when the claw is already lined up
 * on a pin; use clawGrip() when all that is wanted is the cylinder.
 */
void grab();
/** grab() + block until the push is over and the arm is holding. True = a pin is actually clamped. */
bool grabAndWait(int timeoutMs = config::AUTON_GRAB_TIMEOUT_MS);
/** clawOpen() + block for CLAW_OPEN_TIME_MS (or until the jaws read open). True = open. */
bool openAndWait(int timeoutMs = config::AUTON_CLAW_TIMEOUT_MS);
/**
 * THE AUTOCLAMP WAIT. Block until the claw is actually CLAMPED -- by the autoclamp seeing a pin, or
 * by anything else that shuts the jaws -- then wait out config::CLAW_CLOSE_TIME_MS so the grip is
 * real before the routine moves on. True = clamped; false = the timeout ran out.
 *
 * stopChassis = true cancels the chassis motion that is running the instant the clamp happens
 * (lemlib::Chassis::cancelMotion()), which is the "drive into a pin and stop when you have it"
 * pattern:
 *
 *     auton::autoclamp(true);
 *     chassis.moveToPoint(24, 0, 2000, {.async = true});
 *     if (auton::waitUntilClamped(2000, true)) auton::stepUp();   // got it, and the drive is stopped
 *
 * With stopChassis = false it just waits and leaves the motion alone.
 */
bool waitUntilClamped(int timeoutMs = config::AUTON_CLAMP_TIMEOUT_MS, bool stopChassis = false);

/** X: the pivot's two positions. OUT = swung out (power-on, every stack level, the default down
 *  position); BACK = pulled in toward the intake handoff. Each returns after PIVOT_TIME_MS if
 *  waitForIt, else immediately. */
void pivotOut(bool waitForIt = false);
void pivotBack(bool waitForIt = false);
bool pivotIsOut();
/** Y: autoclamp on/off. */
void autoclamp(bool on);
bool autoclampOn();

// ---------------------------------------------------------------------------
// Intake — the same single-writer decision the driver's R1/R2 feed
// ---------------------------------------------------------------------------
/** Latch a roller request. It is applied by the auton tick, as the one intake write per tick, and it
 *  beats both the L1/L2 macro windows exactly as the driver's R1/R2 do. power is 0..1 of
 *  config::INTAKE_MV. */
void intakeIn(double power = 1.0);
void intakeOut(double power = 1.0);
/** Drop the request: the macros (or nothing) decide again. */
void intakeStop();

/** Which way the rollers turn, for intakeFor(). IN pulls a pin in (the R1 direction); OUT ejects. */
enum class Spin { IN, OUT };
/**
 * SPIN THE ROLLERS ONE WAY FOR A FIXED TIME, then stop them. The driver's 2026-10-07 request: one
 * function with a direction and a duration, to kick the intake at the start of a route.
 *
 *     auton::intakeFor(1500, auton::Spin::IN);    // 1.5 s inward, e.g. first thing in auto
 *     auton::intakeFor(400, auton::Spin::OUT);    // a short eject
 *     auton::intakeFor(1500, auton::Spin::IN, 0.6);  // same, at 60% of config::INTAKE_MV
 *
 * BLOCKS for ms, then stops the rollers -- so the next thing in the route starts with the intake
 * already idle and nothing latched. ms <= 0 just stops them and returns immediately. power is 0..1 of
 * config::INTAKE_MV and is clamped into that range.
 *
 * It goes through the SAME latched request the driver's R1/R2 feed, so it beats the L1/L2 macro
 * windows exactly as the driver's own buttons do, and it is applied by the auton tick as the one
 * intake write per tick (CLAUDE.md's single-writer rule).
 *
 * To overlap the spin with a drive move, don't use this -- latch it and stop it yourself:
 *     auton::intakeIn();                 // returns at once
 *     chassis.moveToPoint(...);          // rollers keep turning through the move
 *     auton::intakeStop();
 */
void intakeFor(int ms, Spin dir, double power = 1.0);
/** intakeFor() with the direction baked in. Kept because existing routes call them. */
void intakeInFor(int ms, double power = 1.0);
void intakeOutFor(int ms, double power = 1.0);

// ---------------------------------------------------------------------------
// The driver's macro operations, through the identical code paths
// ---------------------------------------------------------------------------
/** R1 new-press: send the claw to the INTAKE handoff -- jaws open, pivot BACK, precise approach.
 *  Ignored while a pin is clamped (interlock 4). */
void goToIntake();
/** goToIntake() + block until Scoring reports it is parked there. */
bool goToIntakeAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);

/** L1: the first call clamps and rises to stack level 1; each later call goes up one level, staying
 *  at the top. Also opens L1's auto-intake sweep when the rise starts from the floor, exactly like
 *  the button (so the rollers sweep while the arm climbs). */
void stepUp();
/** stepUp() + block until the arm is holding the level it ended up at. */
bool stepUpAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);

/**
 * L2 PRESS 1 of two. Above the bottom holding a pin -> the sensing scorer: press the pin onto the
 * stack with the jaws shut, release at the detected landing (or at L2_AUTO_RELEASE_DEG's travel
 * limit), keep pushing L2_POST_RELEASE_PUSH_MS so the jaws slide off, then RISE L2_CLEAR_RISE_DEG and
 * HOLD there, indefinitely -- so the robot can drive away with the claw above the pins. It does NOT
 * descend to the bottom any more; scoreReturn() is the second press that does. At the bottom instead,
 * this toggles the claw in place and moves nothing.
 */
void score();
/** Block until press 1 has finished -- the arm holding L2_CLEAR_RISE_DEG above the stack with the pin
 *  off. False on SCORE STUCK (the claw stayed hooked), FAULT, or the timeout. */
bool waitScore(int timeoutMs = config::AUTON_SCORE_TIMEOUT_MS);
/** score() + waitScore(). */
bool scoreAndWait(int timeoutMs = config::AUTON_SCORE_TIMEOUT_MS);
/** L2 PRESS 2: from the held-above position, the fast run home at L2_RETURN_SPEED_PCT that ends
 *  SEATED on the hard stop. The autoclamp stays inhibited for the whole descent, so the pin that was
 *  just released cannot be re-grabbed on the way past it. A no-op anywhere else. */
void scoreReturn();
/** scoreReturn() + waitLiftSeated(). True = seated at the bottom. */
bool scoreReturnAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);
/** True while the arm is holding above a finished score, waiting for press 2. */
bool scoredUp();
/** True if the last score gave up with the claw still caught. */
bool scoreStuck();

/** Scoring::scoreOrMatchload(): holding a pin -> score and lower to INTAKE instead of the bottom;
 *  empty -> go to the MATCHLOAD position. Not on any driver button. */
void matchload();
/** B: stow -- cancel the macro, pivot OUT, lift straight down to the bottom and seat. */
void stow();
/** stow() + waitLiftSeated(). */
bool stowAndWait(int timeoutMs = config::AUTON_LIFT_TIMEOUT_MS);
/** Scoring::reset(): back to IDLE, level back to 1, nothing moves, the lift keeps holding. */
void resetMacro();
/** Scoring::resetLevel(): back to stack level 1 without scoring. */
void resetLevel();
/** The stack level the macro is on (0 when it isn't stacking), and its state word for the brain. */
int level();
const char* macroState();

// ---------------------------------------------------------------------------
// Start-of-auton alignment, and the raw distance sensors
// ---------------------------------------------------------------------------
/** The side sensors' current readings, mm. Negative = nothing usable (out of range, 0, PROS_ERR, or
 *  below the confidence floor). The rear one is auton::backDistanceMm(), below. */
double leftDistanceMm();
double rightDistanceMm();
/** Average the LEFT sensor for ms, rejecting invalid samples (see config::START_ALIGN_*). Returns a
 *  negative number if nothing valid came back. Blocks the calling task for ms. */
double readLeftAveraged(int ms);

/**
 * THE START-OF-AUTON X CORRECTION. First line of every route.
 *
 * The robot is set against the front wall, so the distance from the front and the heading are
 * repeatable; how far ALONG that wall it ends up is not. This measures that one unknown with the left
 * sensor and folds it into the starting pose:
 *
 *     dx_in = (measured left mm - expectedLeftMm) / 25.4
 *
 * READING FURTHER than expected = the robot is shifted +x (to the robot's right); CLOSER = -x. That is
 * the driver's own sign convention and it matches LemLib's frame at heading 0, where the robot's right
 * is +x. The shift is applied along the ROBOT'S RIGHT AXIS -- (cos theta, -sin theta) * dx -- so it
 * stays physically correct if a route ever starts at another heading; at theta = 0 that is simply
 * x + dx. Because the heading is fixed and known, the change in the reading IS the lateral shift: no
 * sensor-mounting offset enters the maths at all.
 *
 * WHERE THE READING COMES FROM. While the robot sits disabled before the match, the left sensor is
 * averaged continuously (see startAlignSample()); startPose() uses that long, settled average and
 * spends no auton time on it, confirming it with one short fresh sample in case the robot was moved.
 * With no disabled history -- autonomous launched straight from the brain menu -- it falls back to
 * sampling for config::START_ALIGN_SAMPLE_MS.
 *
 * IT CAN REFUSE. If nothing valid is read, or the correction would be bigger than
 * config::START_ALIGN_MAX_IN, the nominal (x, y, theta) is used unchanged and the brain says
 * `!! START align skipped`. A sensor that is looking at a goal instead of the wall must never be able
 * to teleport the pose. Returns true if EITHER axis's correction was applied.
 *
 * It also handles the one ordering trap: chassis.calibrate() RESETS the pose, so this waits for
 * calibration (bounded) and re-applies the pose once more if calibration lands late.
 *
 *     auton::startPose(0, 0, 0, MATCH_START_LEFT_MM);
 *
 * THE Y AXIS, OPTIONALLY, FROM THE BACK SENSOR. Pass a BackY and the rear sensor
 * (config::BACK_DISTANCE_PORT) sets the starting Y the same way the left sensor sets X:
 *
 *     dy_inches = (reading_mm - backY.expectedMm) / 25.4
 *
 * so a reading FURTHER than expected gives +y and CLOSER gives -y. It is applied straight to Y with
 * no heading rotation, because this correction is only meaningful at a known heading and every route
 * using it starts at theta 0. The two axes are INDEPENDENT: X alignment is driven by expectedLeftMm,
 * Y by backY.expectedMm, and either can be off (0) while the other is on.
 *
 * BackY REFUSES OUTSIDE ITS WINDOW. A reading below minMm or above maxMm, or one the sensor itself
 * rejects, is not used at all and Y stays nominal -- it is a gate, not a clamp, so a sensor aimed at
 * a robot instead of the wall cannot drag the pose part-way. startBackApplied() says which happened.
 *
 *     auton::startPose(0, 0, 0, 0,
 *                      {.expectedMm = config::MATCHLOAD_BACK_EXPECTED_MM,
 *                       .minMm = config::MATCHLOAD_BACK_MIN_MM,
 *                       .maxMm = config::MATCHLOAD_BACK_MAX_MM});
 */
struct BackY {
    double expectedMm = 0.0;  // <= 0 disables the Y correction entirely (the sensor is not even read)
    double minMm = 0.0;       // readings below this are ignored
    double maxMm = 0.0;       // readings above this are ignored
};
bool startPose(double x, double y, double thetaDeg, double expectedLeftMm, BackY backY = {});
/** The shift startPose() applied, inches (0 if it was skipped or never ran). */
double startShiftIn();
/** The averaged left reading startPose() used, mm (negative if it had none). */
double startLeftMm();
/** The Y shift startPose() applied from the back sensor, inches (0 if skipped or never ran). */
double startShiftYIn();
/** The averaged back reading startPose() took, mm (negative if it had none or never asked). */
double startBackMm();
/** True if the last startPose() applied a back-sensor Y correction. */
bool startBackApplied();
/** True if the last startPose() was ASKED for a back-sensor Y correction, whether or not it used the
 *  reading. Lets the brain distinguish "ignored a reading" from "never wanted one". */
bool startBackRequested();
/** True if the last startPose() applied its correction; false if it was skipped or hasn't run. */
bool startAligned();
/** True once startPose() has run at all this program run (so the brain can show its result). */
bool startPoseRan();

/** Sample the left sensor into the rolling pre-match history. Call repeatedly from disabled(); it
 *  returns at once, so the caller owns the loop. Costs one sensor read. */
void startAlignSample();
/** The settled pre-match average, mm, or a negative number if the history is empty, unsettled or
 *  stale. The brain's START line shows this live while disabled. */
double startAlignPreview();
/** One VALIDATED instantaneous back reading, mm, or negative. For the brain's live pre-match preview
 *  of the back-sensor Y calibration -- there is no rolling history for this sensor. Distinct from
 *  backDistanceMm() below, which is moveB's raw unvalidated read. */
double startBackPreview();
/** Average the BACK sensor for ms and return mm, or negative if nothing valid came back. The same
 *  validity rules as the left sensor (see startPose()). Blocks for ms. */
double readBackAveraged(int ms);

// ---------------------------------------------------------------------------
// moveB — drive until the REAR distance sensor reads a target distance
// ---------------------------------------------------------------------------
/**
 * Ported 1:1 from the team's old library (JokrPushBack, src/movement.cpp) and kept deliberately
 * as-is, signature included, so old calls paste straight in. It is NOT an odometry move: it is a
 * hand-rolled PD loop on config::BACK_DISTANCE_PORT's reading that drives BOTH sides at the same
 * power until the rear sensor reads `distance` mm from whatever is behind the robot, then settles.
 *
 *   distance   the target reading, in MILLIMETRES, from the rear sensor.
 *   forwards   true = positive power drives the robot forward; false = the whole output is negated,
 *              so the robot backs up. This is "which way does the robot physically move", and it is
 *              applied AFTER the PD output, so it does not change the sign of the error.
 *   decreasing true when the reading gets SMALLER as the move progresses (backing toward a wall:
 *              error = reading - distance); false when it gets BIGGER as the move progresses (driving
 *              away from the wall behind you: error = distance - reading). Getting this wrong makes
 *              the loop drive the wrong way, so it is the first thing to check.
 *   maxSpeed   power cap, -127..127 units (this is MotorGroup::move(), not voltage).
 *   minSpeed   the smallest power the loop is allowed to output once it wants any at all, so a tiny
 *              error still breaks friction. <= 0 means use config::MOVEB_DEFAULT_MIN_SPEED (15).
 *   timeOutMs  hard stop. ALWAYS give one.
 *
 * Example -- back up until the rear sensor is 150 mm from the wall, at up to 70 power, 2 s limit:
 *     auton::moveB(150, false, true, 70, 15, 2000);
 *
 * TWO THINGS TO KNOW. (1) It writes the drive motors DIRECTLY, so it cancels any LemLib motion at
 * entry and waits for the chassis to be idle before it starts -- never two writers on the drive. It
 * also refuses to run unless the auton tick is running (auton::begin()), so it can never be called
 * from driver control. (2) KEPT FROM THE OLD CODE: while the sensor reads out of range
 * (>= config::MOVEB_OUT_OF_RANGE_MM) the loop drives BLIND at maxSpeed until it comes back into
 * range or the timeout ends the move -- so an unplugged or missing rear sensor means full speed until
 * timeOutMs. Keep timeOutMs tight, and check the brain's BACK line before trusting a route to it.
 *
 * Gains and thresholds: config::MOVEB_* (they all need retuning -- the old robot had 3.25" traction
 * wheels, this one has 2.75" omnis).
 */
void moveB(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeOutMs);
/** The rear sensor's current reading, mm (what moveB is looking at). */
int backDistanceMm();

}  // namespace auton
