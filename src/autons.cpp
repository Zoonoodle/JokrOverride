// autons.cpp — ROUTES. Work here. (The API is include/auton.hpp: API — don't put routes there.)
//
// HOW TO USE THIS FILE
//   1. Pick the route to run: change SELECTED, one line, right below.
//   2. Write a route: copy matchRoute() below, give it a name, add it to the Route enum in
//      include/autons.hpp and to the switch in run().
//   3. Build and upload. Nothing else in the codebase has to be touched.
//
// WHAT HAS ALREADY HAPPENED BY THE TIME A ROUTE RUNS (autonomous() does all of it, so a route cannot
// forget any of it):
//   - the auton tick is running, so Scoring, the claw autoclamp and the intake are all being ticked
//     every 10 ms, exactly as they are in driver control (see include/periodic.hpp);
//   - the lift has been asked to home against its hard stop. It does that underneath the route --
//     nothing blocks on it -- and ends SEATED on the bottom. A lift command from a route queues
//     behind homing on its own;
//   - odometry/IMU calibration has finished (so chassis moves are safe);
//   - the autoclamp is ON (config::AUTOCLAMP_ON_IN_AUTON, 2026-10-03 -- it used to be off and every
//     route had to remember auton::autoclamp(true)). It is ASYNCHRONOUS: the auton tick watches the
//     claw's distance sensor, so a route never calls or waits for anything to get it. Whenever the jaws
//     are open and something comes within config::AUTOCLAMP_RANGE_AUTON_MM (65 mm in auton) they shut on
//     it, and the arm then
//     hops up config::CLAMP_RISE_DEG -- except at the R1 intake handoff, where it fires instantly and
//     forces the claw DOWN instead (config::AUTOCLAMP_INTAKE_PUSH_MS). auton::autoclamp(false) turns it
//     off for a route that does not want it; auton::waitUntilClamped() is OPTIONAL and exists only to
//     stop a drive at the instant of a clamp.
//
// THE TOOLS A ROUTE HAS. auton::* (include/auton.hpp) for every lift / claw / pivot / intake /
// macro operation, each one going through the identical code path the matching driver button uses;
// chassis.* for LemLib odometry moves; auton::moveB() for the rear-distance-sensor drive. Every
// auton::wait*() call takes a timeout and returns whether it actually happened, so a route can check
// instead of hoping -- and can never hang.

#include "autons.hpp"

#include "auton.hpp"
#include "config.hpp"
#include "robot.hpp"

using namespace robot;

namespace autons {

// ===========================================================================
// >>> THE ONE LINE TO CHANGE: which route autonomous() runs. <<<
// ===========================================================================
constexpr Route SELECTED = Route::MATCH;  // LIFT_TEST, ODOM_TEST, MATCH, SKILLS, NONE

namespace {

/** How long each hold in liftTest() lasts, so the brain's LIFT line can be read at every step. */
constexpr int kTestHoldMs = 2000;

// ===========================================================================
// WHERE EACH ROUTE STARTS, and what its left sensor should read there.
// ===========================================================================
// The robot is set against the front wall every time, so the distance from the front and the heading
// repeat; how far ALONG that wall it ends up does not. auton::startPose() measures that one unknown
// with the LEFT sensor and folds it into the starting pose -- further than expected = shifted +x,
// closer = -x. See README "Start alignment" for how to measure these numbers (set the robot in its
// spot, read `DIST L` on the brain a few times, type what you see here).
// These are FIELD positions, not robot constants, which is why they live next to the routes.
constexpr double MATCH_START_X = 0.0;        // the route starts at the origin (driver, 2026-10-03)
constexpr double MATCH_START_Y = 0.0;
constexpr double MATCH_START_THETA = 0.0;
constexpr double MATCH_START_LEFT_MM = 1570.0;  // measured on the robot 2026-10-03

constexpr double SKILLS_START_X = 0.0;       // TODO(team): map the real starting point
constexpr double SKILLS_START_Y = 0.0;       // TODO(team)
constexpr double SKILLS_START_THETA = 0.0;   // TODO(team)
constexpr double SKILLS_START_LEFT_MM = 0.0; // TODO(team): measure, 0 = no start alignment

// ---------------------------------------------------------------------------
// LIFT_TEST — lift only. No drivetrain motion at all.
// ---------------------------------------------------------------------------
// Watch the brain: ANGLE should read HOMED, then LIFT should read SEAT ~0 for two seconds, then
// HOLD ~31, then HOLD ~74.6, then SEAT ~0 again -- and stay there, pressed onto the stop, for the
// rest of the mode and on into driver control.
void liftTest() {
    // Homing was already requested by autonomous(); just wait for it. On success the lift adopts a
    // DOWN target already in SEAT, so the arm is actively pressed onto its hard stop from here --
    // not sitting on a passive brake. On HOME FAIL it stops pushing, keeps zero()'s seed, and the
    // brain shows HOME FAIL plus the angle it saw; the rest of this still runs off that seed.
    auton::waitHome();
    // Homing is off on this robot (config::LIFT_HOME_ON_ENABLE), so nothing has put the arm in SEAT:
    // command the trip to the bottom explicitly, which ends SEATED and then RESTs the same way.
    if (!auton::liftHomed()) auton::liftDownAndWait();
    auton::wait(kTestHoldMs);  // 1. stay seated

    auton::liftToAndWait(auton::Level::ONE);  // 2. stack level 1, then the damped HOLD
    auton::wait(kTestHoldMs);

    auton::liftToAndWait(auton::Level::THREE);  // 3. stack level 3
    auton::wait(kTestHoldMs);

    auton::liftDownAndWait();  // 4. the banded descent, ending SEATED -- and it stays seated
}

// ---------------------------------------------------------------------------
// ODOM_TEST — the team's two straight-line odometry moves.
// ---------------------------------------------------------------------------
// Forward 24", then back to where it started. Use it to check the tracking wheel's direction and
// scaling (ODOM_WHEEL_DIAMETER_IN) and the lateral PID: the robot should end up back on its starting
// tape, and the brain's pose line should read ~0,0 again.
void odomTest() {
    chassis.moveToPoint(0, 24, 2000);
    chassis.moveToPoint(0, 0, 2000, {.forwards = false});
}

// ---------------------------------------------------------------------------
// MATCH — the real route. TEMPLATE: copy this, keep the shape, fill in the numbers.
// ---------------------------------------------------------------------------
// Every call named in the comments below is real and compiles -- uncomment and adjust.
void matchRoute() {
    // WHERE THE ROBOT STARTS -- and the one thing about it that is not repeatable. startPose() sets
    // the pose AND corrects x for however far along the front wall the robot actually ended up, read
    // off the left distance sensor (further than expected = +x, closer = -x). It refuses to apply a
    // correction bigger than config::START_ALIGN_MAX_IN, so a sensor looking at a goal cannot teleport
    // the pose; the brain's START line says what it did. This replaces chassis.setPose() -- do not
    // call both.
    auton::startPose(MATCH_START_X, MATCH_START_Y, MATCH_START_THETA, MATCH_START_LEFT_MM);
    auton::clawGrip();  // grip the preloaded pin (ADI D HIGH = closed); same timing as a raw pin write

auton::liftToFrontToggle();
//   auton::liftPushUpFor(1000);
pros::delay(300);
    chassis.moveToPoint(0,3, 500);
    pros::delay(400);
       auton::liftDownFast();
   chassis.waitUntilDone();
   
   pros::delay(600);
      chassis.moveToPoint(0, -16, 1500, {.forwards = false, .maxSpeed = 70});
   

    
     chassis.turnToHeading(90, 550);
    auton::liftPushUpFor(350);  // was liftMotors.move(50)/delay(300)/move(0) -- 50 on the old -127..127
                                // scale is ~40% of volts, which is AUTON_LIFT_PUSH_UP_PCT. Swap this for
                                // auton::liftToFirstScoreAndWait() if you want a repeatable 9 deg instead
                                // of a timed push (see auton.hpp).
    chassis.moveToPoint(-14, -16, 1000, {.forwards = false, .maxSpeed = 70});
   chassis.moveToPoint(-12.5, -16, 300);
//    lift.move(-40);
 auton::liftPushFor(400);
pros::delay(200);
    auton::clawOpen();
   
    // chassis.moveToPoint(chassis.getPose().x-2, chassis.getPose().y, 500, {.forwards = false, .maxSpeed = 70});
 pros::delay(300);
 auton::liftPushUpFor(250);
    chassis.moveToPoint(-1, -14, 1000);
    // lift.move(0);
    chassis.turnToHeading(40, 700);
    auton::liftDownFast();
    auton::autoclamp(true);
    
    chassis.moveToPoint(-17, -35, 2000, {.forwards = false, .maxSpeed = 60});
    
    // chassis.waitUntil(21);
    // auton::clawGrip();
    chassis.waitUntilDone();
    auton::liftTo(auton::Level::ONE);
   pros::delay(500);
    chassis.turnToHeading(165, 700);
    chassis.moveToPoint(-18, -21, 1500, {.forwards = false, .maxSpeed = 80});
chassis.waitUntilDone();
                     // finish driving up to the stack
auton::scoreAndWait();                        // press down to contact, release, push 100 ms, rise and hold
chassis.moveToPoint(-15, -34, 1000);
pros::delay(500);                    // back clear of the pins, claw still up
auton::liftDownFast();

chassis.turnToHeading(135, 600);
auton::autoclamp(true);
chassis.moveToPoint(-40, -19, 2000, {.forwards = false, .maxSpeed = 55});


    chassis.waitUntilDone();
    auton::liftTo(auton::Level::TWO);
pros::delay(500);
    chassis.turnToHeading(265, 700);
    chassis.moveToPoint(-27, -15, 1500, {.forwards = false, .maxSpeed = 80});
    auton::scoreAndWait();
    auton::liftDownAndWait();    
    // The autoclamp needs NOTHING from a route: it is on by default and asynchronous, so simply driving
    // at a pin with the jaws open grips it (and hops the arm up CLAMP_RISE_DEG) on its own.
    // waitUntilClamped() is only for the extra step of STOPPING THE DRIVE at the instant of the clamp --
    // .async = true hands the move to LemLib's own task so this one can watch the claw, and the second
    // argument cancels that motion the moment the jaws shut, so the robot does not keep pushing.
    // chassis.moveToPoint(0, 24, 2000);   // async by default in LemLib 0.5.6
    // if (!auton::waitUntilClamped(2000, true)) return;   // nothing grabbed: bail out rather than
    //                                                     // carry on with an empty claw

    // Or grab a pin the claw is already lined up on, without driving into it: the same force-down
    // push L1 does at the handoff, holding wherever it ends.
    // auton::grabAndWait();

    // RUN THE LIFT AND THE DRIVE TOGETHER: command the lift, keep driving, wait only where the next
    // step needs the arm to be there. (A second chassis move would block this task until the first
    // finishes, so the lift command goes BETWEEN them -- see README "Blocking vs non-blocking".)
    // chassis.moveToPoint(-17.5, -24, 1500, {.forwards = false});
    // auton::liftTo(auton::Level::TWO);   // rises while the robot drives
    // chassis.waitUntilDone();
    // auton::waitLift();
    // auton::scoreAndWait();

    // Stack it. stepUp() is L1: it clamps (already clamped here, so no wait) and rises a level,
    // sweeping the floor with the intake on the way up, exactly like the driver's button. From the
    // handoff it does the force-down GRAB first, also like the button.
    // auton::stepUpAndWait();                  // level 1
    // auton::liftToAndWait(auton::Level::THREE);  // or straight to a level

    // Score it, in two presses like the driver's L2. Press 1 presses the pin onto the stack, releases
    // at the landing, and ends HOLDING 20 deg above it -- so the robot can drive away without
    // knocking the stack over. waitScore() is false on SCORE STUCK.
    // auton::scoreAndWait();
    // ... drive away here, with the claw still held above the pins ...
    // auton::scoreReturnAndWait();   // press 2: the run home, ending SEATED at 0

    // Back up to a known distance from the wall behind, using the rear sensor rather than odometry.
    // auton::moveB(150, false, true, 70, 15, 2000);

    // Finish with the arm down and SEATED, so the hand-off to driver control is in a known state.
    // auton::liftDownAndWait();
}

// ---------------------------------------------------------------------------
// SKILLS — the 60 s skills route. TEMPLATE.
// ---------------------------------------------------------------------------
void skillsRoute() {
         auton::startPose(MATCH_START_X, MATCH_START_Y, MATCH_START_THETA, MATCH_START_LEFT_MM);
    auton::clawGrip();  // grip the preloaded pin (ADI D HIGH = closed); same timing as a raw pin write

auton::liftToFrontToggle();
pros::delay(500);
    chassis.moveToPoint(0,3, 500);
    pros::delay(400);
       auton::liftDownFast();
   chassis.waitUntilDone();
   pros::delay(300);
      chassis.moveToPoint(0, -15, 1500, {.forwards = false, .maxSpeed = 70});
   

    
     chassis.turnToHeading(90, 700);
    auton::liftPushUpFor(250);  // was liftMotors.move(50)/delay(300)/move(0) -- 50 on the old -127..127
                                // scale is ~40% of volts, which is AUTON_LIFT_PUSH_UP_PCT. Swap this for
                                // auton::liftToFirstScoreAndWait() if you want a repeatable 9 deg instead
                                // of a timed push (see auton.hpp).
    chassis.moveToPoint(-14, -15, 1200, {.forwards = false, .maxSpeed = 70});
   chassis.waitUntilDone();
//    lift.move(-40);
 auton::liftPushFor(400);
 leftDrive.move(20);
 rightDrive.move(20);
 pros::delay(200);
  leftDrive.move(0);
 rightDrive.move(0);
    auton::clawOpen();
   
    // chassis.moveToPoint(chassis.getPose().x-2, chassis.getPose().y, 500, {.forwards = false, .maxSpeed = 70});
 pros::delay(300);
    chassis.moveToPoint(-1, -14, 1000);
    // lift.move(0);
    chassis.turnToHeading(35, 700);
    auton::liftDownFast();
    auton::autoclamp(true);
    
    chassis.moveToPoint(-16, -35, 2000, {.forwards = false, .maxSpeed = 60});
    
    // chassis.waitUntil(21);
    // auton::clawGrip();
    chassis.waitUntilDone();
    auton::liftTo(auton::Level::ONE);
    chassis.turnToHeading(165, 700);
    chassis.moveToPoint(-18, -21, 1500, {.forwards = false, .maxSpeed = 80});
chassis.waitUntilDone();
                     // finish driving up to the stack
auton::scoreAndWait();                        // press down to contact, release, push 100 ms, rise and hold
chassis.moveToPoint(-15, -34, 1000);
pros::delay(500);                    // back clear of the pins, claw still up
auton::liftDownFast();

chassis.turnToHeading(135, 600);
auton::autoclamp(true);
chassis.moveToPoint(-39, -16.5, 2000, {.forwards = false, .maxSpeed = 60});


    chassis.waitUntilDone();
    auton::liftTo(auton::Level::TWO);

    chassis.turnToHeading(265, 700);
    chassis.moveToPoint(-27, -15, 1500, {.forwards = false, .maxSpeed = 80});
    auton::scoreAndWait();  
    chassis.waitUntilDone();
    chassis.moveToPoint(-30, -20, 1000);
    chassis.moveToPoint(-10, -48, 2000,{.maxSpeed = 80});
    // auton::startPose(SKILLS_START_X, SKILLS_START_Y, SKILLS_START_THETA, SKILLS_START_LEFT_MM);
    // Same tools as matchRoute(); usually a loop of: drive to a pin, waitUntilClamped, stepUp,
    // score, moveB back to a wall to re-square, repeat.
}

}  // namespace

Route selected() { return SELECTED; }

double expectedLeftMm() {
    switch (SELECTED) {
        case Route::MATCH: return MATCH_START_LEFT_MM;
        case Route::SKILLS: return SKILLS_START_LEFT_MM;
        default: return 0.0;  // the test routes do not align
    }
}

const char* selectedName() {
    switch (SELECTED) {
        case Route::LIFT_TEST: return "LIFT_TEST";
        case Route::ODOM_TEST: return "ODOM_TEST";
        case Route::MATCH: return "MATCH";
        case Route::SKILLS: return "SKILLS";
        case Route::NONE: return "NONE";
    }
    return "?";
}

void run() {
    switch (SELECTED) {
        case Route::LIFT_TEST: liftTest(); break;
        case Route::ODOM_TEST: odomTest(); break;
        case Route::MATCH: matchRoute(); break;
        case Route::SKILLS: skillsRoute(); break;
        case Route::NONE: break;
    }
}

}  // namespace autons
