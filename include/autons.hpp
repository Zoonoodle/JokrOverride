#pragma once
/**
 * autons — ROUTES. Work here. (The API the routes are written with is include/auton.hpp, which is
 * the other half of this pair: API — don't put routes there.)
 *
 * Everything about WHICH route runs and WHAT it does lives in src/autons.cpp: the route functions
 * themselves, and the one line at the top of that file that picks the one autonomous() will run
 * (`constexpr Route SELECTED = ...`). main.cpp does the common setup every route needs and then makes
 * exactly one call -- autons::run() -- so a new route never means touching main.cpp, config.hpp or
 * anything else. See README "Writing an auton route".
 */

namespace autons {

/** Every route this robot has. Add one here, write the function in autons.cpp, add it to the switch
 *  in run(), and point SELECTED at it. */
enum class Route {
    LIFT_TEST,  // lift only, no drivetrain: home, seat, level 1, level 3, bottom
    ODOM_TEST,  // the two straight-line odometry test moves
    MATCH,      // the real match route (template)
    SKILLS,     // the skills route (template)
    NONE,       // do nothing at all
};

/** Run the selected route. Called once from autonomous(), which has already done the common setup
 *  (calibration wait, lift homing, the auton tick, autoclamp off). */
void run();

/** Which route run() will execute -- for the brain screen / a log line. */
Route selected();
const char* selectedName();
/** The SELECTED route's expected left-wall reading, mm (0 if that route does not align). The brain
 *  shows the live pre-match alignment against this while the robot sits disabled. */
double expectedLeftMm();

}  // namespace autons
