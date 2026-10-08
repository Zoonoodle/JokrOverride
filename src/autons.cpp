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
constexpr Route SELECTED = Route::MATCH;
// LIFT_TEST, ODOM_TEST, MATCH, SKILLS, MATCHLOAD_FINISH, MATCH_ALT, PIVOT_TEST, AUTOCLAMP_TEST, NONE
//
// PIVOT_TEST and AUTOCLAMP_TEST drive NO wheels -- safe to run on a stand, and they are what to run
// to check the two 2026-10-07 behaviour changes (pivot waits for the arm; an autoclamp at the handoff
// goes to level 1 on its own). MATCHLOAD_FINISH's field coordinates are PLACEHOLDERS -- see the ML_*
// constants below before running it.

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

// MATCHLOAD_FINISH's start. Same front wall as MATCH until measured otherwise.
constexpr double MATCHLOAD_START_X = 0.0;        // TODO(team): confirm for this route's start spot
constexpr double MATCHLOAD_START_Y = 0.0;        // TODO(team)
constexpr double MATCHLOAD_START_THETA = 0.0;    // TODO(team)
constexpr double MATCHLOAD_START_LEFT_MM = MATCH_START_LEFT_MM;  // TODO(team): re-measure if it differs

// ===========================================================================
// MATCHLOAD_FINISH's FIELD NUMBERS -- every one of these is a PLACEHOLDER.
// ===========================================================================
// These are the only things standing between this route and a real run. They are deliberately
// gathered here instead of being buried in the route so they can be measured and typed in one go.
// NOTHING BELOW WAS MEASURED ON THE FIELD -- drive the robot there by hand, read the pose off the
// brain, and type what you see. Until then the route will go to the wrong places.
constexpr double ML_STATION_X = 0.0;      // TODO(team): in front of the matchload station
constexpr double ML_STATION_Y = -12.0;    // TODO(team)
constexpr double ML_STATION_HEADING = 0.0;  // TODO(team): facing the station
constexpr double ML_STACK_X = -14.0;      // TODO(team): the stack this route scores on
constexpr double ML_STACK_Y = -18.0;      // TODO(team)
constexpr double ML_STACK_HEADING = 165.0;  // TODO(team)
constexpr double ML_CLEAR_X = -12.0;      // TODO(team): backed clear of the stack
constexpr double ML_CLEAR_Y = -32.0;      // TODO(team)
/** How many matchload -> score cycles to try. */
constexpr int ML_CYCLES = 2;              // TUNE

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

// ---------------------------------------------------------------------------
// PIVOT_TEST — lift / claw / pivot only. NO drivetrain motion: safe on a stand.
// ---------------------------------------------------------------------------
// WHAT TO WATCH (brain CLAW line shows OUT / BACK, LIFT line shows the height and phase):
// the pivot must stay BACK for the WHOLE rise and only snap OUT once the LIFT line settles on the
// level. If it swings while the arm is still moving, config::PIVOT_RISE_WAIT_MS fired early or
// liftArrived() is reporting too soon. See interlock 2 in subsystems/scoring.hpp.
void pivotTest() {
    auton::waitHome();
    if (!auton::liftHomed()) auton::liftDownAndWait();

    // Put the pivot BACK by hand first, otherwise it is already OUT (its power-on position) and
    // STACKING's swing would be a no-op with nothing to see.
    auton::pivotBack(true);
    auton::wait(500);

    // STACKING bails out and heads back to the handoff if the jaws are empty, so give it something
    // to "hold". Nothing is in the claw on a stand -- this just makes the state machine proceed.
    auton::clawGrip();

    auton::stepUpAndWait();   // rung 1: the pivot must stay BACK until the LIFT line settles
    auton::wait(kTestHoldMs);

    auton::pivotBack(true);   // put it BACK again and do it once more, one rung higher
    auton::wait(500);
    auton::stepUpAndWait();   // rung 2: same thing to watch
    auton::wait(kTestHoldMs);

    auton::clawOpen();
    auton::liftDownAndWait();
}

// ---------------------------------------------------------------------------
// AUTOCLAMP_TEST — lift / claw / pivot only. NO drivetrain motion: safe on a stand.
// ---------------------------------------------------------------------------
// HOW TO RUN IT: start the mode, and when the arm has settled at the handoff (brain STATE line reads
// INTAKE) put a pin into the jaws by hand, or let the intake feed one in.
// WHAT SHOULD THEN HAPPEN, with no button pressed at all:
//   1. the jaws shut on it the instant the sensor sees it (AUTOCLAMP_INTAKE_CONFIRM_MS);
//   2. the arm forces DOWN onto it for AUTOCLAMP_INTAKE_PUSH_MS (LIFT line: GRAB);
//   3. the arm then rises to level 1 BY ITSELF (config::AUTOCLAMP_INTAKE_AUTO_STACK);
//   4. the pivot swings OUT to the scoring position once the arm has arrived, not before.
// If step 3 never happens, AUTOCLAMP_INTAKE_AUTO_STACK is false or a button cancelled it.
void autoclampTest() {
    auton::waitHome();
    if (!auton::liftHomed()) auton::liftDownAndWait();

    auton::autoclamp(true);
    auton::goToIntakeAndWait();   // INTAKE_POS: pivot BACK, jaws open and armed

    // Wait for the pin to be put in. This only STOPS waiting early; the clamp itself is asynchronous
    // and needs nothing from here.
    auton::waitUntilClamped(15000);

    // Now watch the automatic push -> level 1 -> pivot OUT happen on its own.
    auton::waitLift();
    auton::wait(kTestHoldMs);
    auton::wait(kTestHoldMs);

    auton::clawOpen();
    auton::liftDownAndWait();
}

// ---------------------------------------------------------------------------
// MATCHLOAD_FINISH — matchload cycle, then finish. The route the driver asked for by name.
// ---------------------------------------------------------------------------
// !! THE FIELD COORDINATES IN THIS ROUTE ARE PLACEHOLDERS (the ML_* constants above). The SHAPE is
// real and every call compiles and is the same code path the matching driver button uses, but the
// positions were never measured. Measure them before running this anywhere near a real field.
//
// The shape: start -> grip the preload and score it -> then loop { go to the matchload station with
// the jaws open so the autoclamp takes a pin -> let the macro carry it to level 1 by itself -> drive
// to the stack -> score -> back clear } -> finish with the arm down and seated.
void matchloadFinishAuto() {
    auton::startPose(MATCHLOAD_START_X, MATCHLOAD_START_Y, MATCHLOAD_START_THETA,
                     MATCHLOAD_START_LEFT_MM);

    // --- The preload -------------------------------------------------------
    auton::clawGrip();               // grip the pin the robot started with
    auton::liftToFrontToggleAndWait();
    auton::liftDownFast();

    // --- The matchload cycles ---------------------------------------------
    // The autoclamp is already ON (config::AUTOCLAMP_ON_IN_AUTON) and asynchronous, so arriving at
    // the station with the jaws open is all it takes to pick a pin up.
    for (int cycle = 0; cycle < ML_CYCLES; ++cycle) {
        auton::autoclamp(true);
        auton::goToIntake();         // jaws open, pivot BACK, arm down at the handoff

        chassis.turnToHeading(ML_STATION_HEADING, 700);
        chassis.moveToPoint(ML_STATION_X, ML_STATION_Y, 1500, {.forwards = false, .maxSpeed = 60});
        chassis.waitUntilDone();

        // Take a pin. If nothing arrives in time, stop cycling rather than drive to the stack with
        // an empty claw and score nothing.
        if (!auton::waitUntilClamped(2000)) break;

        // NOTHING TO DO HERE. The autoclamp's own push runs and then the macro takes the arm to
        // level 1 and swings the pivot OUT by itself (AUTOCLAMP_INTAKE_AUTO_STACK). Just wait for
        // the arm to finish arriving before driving at the stack.
        auton::waitLift();

        // --- Score it ------------------------------------------------------
        chassis.turnToHeading(ML_STACK_HEADING, 700);
        chassis.moveToPoint(ML_STACK_X, ML_STACK_Y, 1500, {.forwards = false, .maxSpeed = 80});
        chassis.waitUntilDone();
        auton::scoreAndWait();       // press 1: press onto the stack, release at the landing, hold above

        // --- Back clear, arm down, go again --------------------------------
        chassis.moveToPoint(ML_CLEAR_X, ML_CLEAR_Y, 1000);
        chassis.waitUntilDone();
        auton::liftDownFast();
    }

    // Hand over to the driver in a known state: arm down and pressed onto its stop.
    auton::liftDownAndWait();
}

// ---------------------------------------------------------------------------
// MATCH_ALT — a second match route, so MATCH can stay the one that works.
// ---------------------------------------------------------------------------
// Copy of the MATCH opening only, deliberately left short: build the rest here and flip SELECTED
// between MATCH and MATCH_ALT to compare them without editing either one.
void matchAltRoute() {
    auton::startPose(MATCH_START_X, MATCH_START_Y, MATCH_START_THETA, MATCH_START_LEFT_MM);
    auton::clawGrip();

    auton::liftToFrontToggleAndWait();
    chassis.moveToPoint(0, 3, 500);
    chassis.waitUntilDone();
    auton::liftDownFast();

    // TODO(team): the rest of this route.
}

}  // namespace

Route selected() { return SELECTED; }

double expectedLeftMm() {
    switch (SELECTED) {
        case Route::MATCH: return MATCH_START_LEFT_MM;
        case Route::MATCH_ALT: return MATCH_START_LEFT_MM;
        case Route::SKILLS: return SKILLS_START_LEFT_MM;
        case Route::MATCHLOAD_FINISH: return MATCHLOAD_START_LEFT_MM;
        default: return 0.0;  // the lift/pivot/odom test routes do not align
    }
}

const char* selectedName() {
    switch (SELECTED) {
        case Route::LIFT_TEST: return "LIFT_TEST";
        case Route::ODOM_TEST: return "ODOM_TEST";
        case Route::MATCH: return "MATCH";
        case Route::SKILLS: return "SKILLS";
        case Route::MATCHLOAD_FINISH: return "MATCHLOAD_FIN";
        case Route::MATCH_ALT: return "MATCH_ALT";
        case Route::PIVOT_TEST: return "PIVOT_TEST";
        case Route::AUTOCLAMP_TEST: return "AUTOCLAMP_TEST";
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
        case Route::MATCHLOAD_FINISH: matchloadFinishAuto(); break;
        case Route::MATCH_ALT: matchAltRoute(); break;
        case Route::PIVOT_TEST: pivotTest(); break;
        case Route::AUTOCLAMP_TEST: autoclampTest(); break;
        case Route::NONE: break;
    }
}

}  // namespace autons
