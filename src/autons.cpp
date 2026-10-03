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
//   - the autoclamp is OFF. Turn it on in the route, where you want it: auton::autoclamp(true).
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
constexpr Route SELECTED = Route::ODOM_TEST;  // LIFT_TEST, ODOM_TEST, MATCH, SKILLS, NONE

namespace {

/** How long each hold in liftTest() lasts, so the brain's LIFT line can be read at every step. */
constexpr int kTestHoldMs = 2000;

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
    // Where the robot starts, in LemLib's frame (inches, degrees). Set it before any move.
    // chassis.setPose(0, 0, 0);

    // Drive into a pin and stop the moment the claw grabs it. .async = true hands the move to
    // LemLib's own task, so this one can watch the claw; waitUntilClamped's second argument cancels
    // that motion at the instant of the clamp, so the robot does not keep pushing.
    // auton::autoclamp(true);
    // chassis.moveToPoint(0, 24, 2000, {.async = true});
    // if (!auton::waitUntilClamped(2000, true)) return;   // nothing grabbed: bail out rather than
    //                                                     // carry on with an empty claw

    // Or grab a pin the claw is already lined up on, without driving into it: the same force-down
    // push L1 does at the handoff, holding wherever it ends.
    // auton::grabAndWait();

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
    // Same tools as matchRoute(); usually a loop of: drive to a pin, waitUntilClamped, stepUp,
    // score, moveB back to a wall to re-square, repeat.
}

}  // namespace

Route selected() { return SELECTED; }

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
