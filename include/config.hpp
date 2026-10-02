#pragma once
/**
 * config.hpp — every port, ratio and gain for the JOKR (57249C) Override robot.
 *
 * Rule: nothing robot-specific is typed anywhere else. If a value is not
 * confirmed from the robot or the notebook it is marked TODO(team): confirm.
 * PID gains are STARTING values only and are marked TUNE: they have not been
 * tuned on the robot.
 *
 * Sources (engineering notebook, 2026-27 Override):
 *   - Drivetrain: 55W, 6 wheels, 2.75" omnis, 450 RPM (Best Solution | Drivetrain RPM & Wheel Size)
 *   - Lift: DR6-4B, 2 motors on a high-strength axle, 12T -> 48T (confirmed on the robot, corrects
 *     an earlier 12T -> 36T note from Build | Lift Drive, 9/10/26)
 *   - Claw: one pneumatic cylinder (Build | Pneumatic Claw, Improve | Claw 9/16/26)
 *   - Intake: three-stage roller intake + bottom counter roller (Season Summary)
 *   - Odometry: LemLib, ONE odometry pod (Best Solution | Odometry, 9/18/26)
 */

#include <cstdint>
#include <vector>

namespace config {

// ---------------------------------------------------------------------------
// Drivetrain (LemLib)
// ---------------------------------------------------------------------------
// Negative port = reversed motor.
// TODO(team): confirm ports, direction, and whether the 55W is 5x 11W or 4x 11W + 2x 5.5W.
inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS = {-8, -2, -1};   // TODO(team): confirm
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = {3, 9, 10};     // TODO(team): confirm

inline constexpr float DRIVE_WHEEL_DIAMETER_IN = 2.75f;      // notebook: 2.75" omni wheels
inline constexpr float DRIVE_RPM = 450.0f;                   // notebook: 450 RPM
inline constexpr float TRACK_WIDTH_IN = 11.5f;               // TODO(team): measure wheel-center to wheel-center
inline constexpr float HORIZONTAL_DRIFT = 2.0f;              // LemLib: 2 for a drive with traction wheels

// Odometry: one vertical tracking wheel pod. No inertial sensor is on the robot yet.
inline constexpr int8_t ODOM_VERTICAL_ROTATION_PORT = 17;     // TODO(team): confirm (negative to reverse)
inline constexpr float ODOM_WHEEL_DIAMETER_IN = 2.0f;        // TODO(team): confirm pod wheel (2" / 2.75")
inline constexpr float ODOM_VERTICAL_OFFSET_IN = 0.0f;       // TODO(team): measure left/right offset from center
// Inertial sensor: the robot doesn't have one mounted yet (port 8, the old value here, is a right
// drive motor -- there is no IMU on this robot to conflict with it). 0 = none fitted: robot.cpp
// builds no pros::Imu at all, LemLib gets a null IMU pointer (its own documented way to run on the
// tracking wheel alone), nothing calibrates it, and the brain shows "IMU  none" instead of junk.
// To add one later: set this to the IMU's own port (not a drive port) -- nothing else changes.
inline constexpr int8_t IMU_PORT = 18;                        // TODO(team): set once an IMU is mounted

// Chassis PID — TUNE. Starting values from the LemLib tutorial, not tuned on JOKR.
inline constexpr float LATERAL_KP = 10.0f, LATERAL_KI = 0.0f, LATERAL_KD = 3.0f;   // TUNE
inline constexpr float ANGULAR_KP = 2.0f, ANGULAR_KI = 0.0f, ANGULAR_KD = 10.0f;   // TUNE

// ---------------------------------------------------------------------------
// Lift (DR6-4B)
// ---------------------------------------------------------------------------
// Two motors on the 12T motor axle, geared into the 48T lift-arm axle: 12T -> 48T = 4:1 torque.
// Everywhere else in the code (presets, PID target/error, lift.getPosition(), the brain screen)
// works in LIFT-ARM DEGREES -- this ratio is what converts the motor/sensor side of the 4:1 down
// to that one shared unit. See Lift::getMotorPosition()/getSensorPosition() in subsystems/lift.cpp.
inline const std::vector<std::int8_t> LIFT_PORTS = {20, -13};             // TODO(team): confirm ports + which side is reversed
inline constexpr double LIFT_GEAR_RATIO = 48.0 / 12.0;       // confirmed: 12T -> 48T = 4:1 torque
// Rotation sensor on the lift, mounted on the 12T MOTOR axle (confirmed) -- not the 48T arm axle.
// 0 = none fitted, use the motor encoders instead. When a port is set, the lift PID runs off THIS
// sensor, not the motor encoders.
inline constexpr int8_t LIFT_ROTATION_PORT = 16;
// DIRECTION CHECK (do this before anything else, see README "Lift direction check"):
// raise the lift by hand with the robot disabled and watch "ROT" on the brain screen.
// It must go UP (positive). If it goes negative, flip this to true.
inline constexpr bool LIFT_ROTATION_REVERSED = false;        // TODO(team): check on the robot
// Sensor degrees per lift-arm degree = the 4:1 above (sensor is on the fast/motor side, so it spins
// 4 turns for every 1 turn of the arm). At the full ~105 deg of arm travel the sensor itself turns
// ~420 deg -- more than one full revolution -- so the code reads it with Rotation::get_position()
// (multi-turn centidegrees, keeps counting past 360) and NEVER get_angle() (wraps at 360 and would
// make the lift think it jumped positions partway up). The brain's ROT line shows the raw sensor
// degrees in brackets next to the converted arm degrees so this can be checked by hand.
inline constexpr double LIFT_ROTATION_RATIO = 4.0;           // confirmed: sensor is on the 12T motor axle

// Preset heights, in DEGREES OF THE LIFT ARM (what lift.getPosition() returns -- rotation sensor
// centidegrees already divided by LIFT_ROTATION_RATIO, see the lift gearing note above). 0 = fully
// down (where the robot is turned on). Measured straight off the brain's LIFT line in 7 reference
// photos (docs/robot/photos/levels/, 2026-09-30) with the robot powered on and the arm physically
// moved to each position by hand; see docs/robot/ROBOT.md "Lift levels" for the photo-by-photo readout.
//
// CAVEAT: those photos were taken on a build where Lift::zero() still reset_position()'d the sensor
// at every boot (startup-relative), before it was changed to seed from the sensor's own persistent
// absolute zero (see the comment in Lift::zero(), lift.cpp). If the arm wasn't sitting exactly at
// the true physical bottom when the robot was powered on for that photo session, every one of these
// numbers carries that same constant offset. Re-measure against the brain's LIFT line (now reading
// off the absolute zero) if anything here looks off on the real robot.
namespace lift_preset {
inline constexpr double DOWN = 0.0;                          // resting / start position (IMG_6088 read 0.1, i.e. ~0)
// TODO(team): RE-MEASURE. 1.1 arm deg is only 4.4 sensor deg -- about a millimetre of end-effector
// travel on a six-bar, which is not a visible "lift to the handoff". It is almost certainly a
// mis-read of IMG_6089 (or the photo was taken with the arm already resting at the bottom). It was
// also smaller than the old LIFT_SETTLE_DEG (1.5), which is why R1 reported "arrived at INTAKE"
// without the arm moving at all. To fix: run the program, D-pad Up until the claw lines up with the
// intake handoff, read the brain's LIFT line, and put that number here.
inline constexpr double INTAKE = 1.1;                        // TODO(team): re-measure, see above (photo IMG_6089)
inline constexpr double MATCHLOAD = 15.0;                    // TUNE: not covered by the reference photos yet
// L1 taps: tap 1 -> level 1, tap 2 -> level 2 ... tap 5 -> level 5 (stays at 5, the top). One entry
// per pin-stack height, measured from photos IMG_6090..IMG_6094 (score levels 1-5), 2026-09-30.
inline constexpr int STACK_LEVEL_COUNT = 5;
inline constexpr double STACK_LEVELS[STACK_LEVEL_COUNT] = {
    26.6,   // measured from photo IMG_6090, 2026-09-30 (level 1)
    54.4,   // measured from photo IMG_6091, 2026-09-30 (level 2)
    72.8,   // measured from photo IMG_6092, 2026-09-30 (level 3)
    116.3,  // measured from photo IMG_6093, 2026-09-30 (level 4)
    128.3,  // measured from photo IMG_6094, 2026-09-30 (level 5, the top)
};
// Software hard stop: top level (128.3, IMG_6094) + 5 deg of margin. setTarget()/moveTo() clamp to
// this, so it also doubles as "never command past here" -- raise it only once a higher level is
// actually measured and added above, never just to let a bad preset through.
inline constexpr double MAX = 133.3;
}  // namespace lift_preset

// ===========================================================================
// LIFT TUNING — start here
// ===========================================================================
// Everything that decides how the lift feels is in this block. Each line says what it does and which
// way to move it for which symptom. Nothing else needs touching to tune the lift. The structure these
// numbers feed is described under "Lift motion" below, and the step-by-step order to tune them in is
// in README "Tuning the lift". Watch a move with the Brain Terminal trace open (LIFT_TRACE).
//
// HARDWARE FACTS (not knobs -- these are what the robot is):
// Both lift motors are RED (100 RPM) cartridges (MotorGears::red in Lift::Lift()). Through the
// 12T -> 48T (4:1) reduction the ARM's free speed is 100 / 4 = 25 arm RPM = 150 arm deg/s.
inline constexpr double LIFT_FREE_SPEED_DEG_PER_S = 150.0;   // red 100 RPM / 4:1 = 25 arm RPM = 150 deg/s
inline constexpr float LIFT_MAX_MV = 12000.0f;               // full battery volts; the D-pad's full-send power

// --- How fast a preset move cruises, as a % of the free speed above ---------
//   Feels sluggish on a long rise: raise LIFT_UP_SPEED_PCT (100 is the hardware limit).
//   Runs away / slams going down to a LOWER level: lower LIFT_DOWN_SPEED_PCT.
inline constexpr double LIFT_UP_SPEED_PCT = 100.0;           // TUNE: cruise speed going up
inline constexpr double LIFT_DOWN_SPEED_PCT = 60.0;          // TUNE: cruise speed going down (gravity helps, so less)

// --- When it starts slowing down (the braking curve) ------------------------
// The arm is never allowed to go faster than sqrt(2 * LIFT_DECEL_DEG_PER_S2 * d) with d = the degrees
// left minus LIFT_APPROACH_ZONE_DEG. This is the single number that decides whether it overshoots.
// Read it off the trace: in the B lines `vel` should fall by about a * 0.02 per 20 ms line.
//   Overshoots the level / arrives still moving: LOWER it (braking starts earlier).
//   Dawdles the last third of every move: RAISE it (braking starts later).
// 700 deg/s^2 means it starts braking 16 deg out from full speed -- deliberately well inside what two
// red motors through a 4:1 can really do. The old value, 2000, started braking only 7.6 deg out, which
// the arm physically cannot achieve: that was the overshoot.
inline constexpr double LIFT_DECEL_DEG_PER_S2 = 700.0;       // TUNE: how hard it brakes, arm deg/s^2

// --- The last few degrees (the final approach) ------------------------------
// Inside LIFT_APPROACH_ZONE_DEG the speed reference becomes LIFT_APPROACH_KP_PER_S * (degrees left),
// capped at LIFT_APPROACH_DEG_PER_S. So the arm walks the last few degrees in at a crawl and hands
// over to the hold already nearly stopped -- this is what stops the six-bar springing.
//   Still visibly moving when it grabs the hold: raise LIFT_APPROACH_ZONE_DEG, or lower
//     LIFT_APPROACH_DEG_PER_S / LIFT_APPROACH_KP_PER_S.
//   Takes too long to finish the last bit: raise LIFT_APPROACH_KP_PER_S first, then
//     LIFT_APPROACH_DEG_PER_S.
inline constexpr double LIFT_APPROACH_ZONE_DEG = 8.0;        // TUNE: where the crawl starts
inline constexpr double LIFT_APPROACH_DEG_PER_S = 18.0;      // TUNE: fastest the crawl may be
inline constexpr double LIFT_APPROACH_KP_PER_S = 3.0;        // TUNE: crawl speed per degree left (1/s)

// --- The velocity loop (phases BRAKE, APPROACH and DESCEND) -----------------
// Everything that is "go at this speed" runs through this one PI loop on the MEASURED arm speed:
//   mV = LIFT_VEL_KP * (reference - measured) + integral of LIFT_VEL_KI * (reference - measured)
// The integral is what learns gravity -- whatever voltage the arm needs at this angle to hold a speed,
// up or down -- so no gravity constant has to be right in advance.
//   `vel` swings above and below the reference in the trace (hunts): LOWER LIFT_VEL_KP.
//   Sags / is slow to reach the reference speed: RAISE LIFT_VEL_KI.
//   The braking feels violent: LOWER LIFT_VEL_MAX_MV (and lower LIFT_DECEL_DEG_PER_S2 to match).
inline constexpr float LIFT_VEL_KP = 30.0f;                  // TUNE: mV per (deg/s) of speed error
inline constexpr float LIFT_VEL_KI = 400.0f;                 // TUNE: mV per (deg/s) of speed error, per second
inline constexpr float LIFT_VEL_MAX_MV = 9000.0f;            // TUNE: most the velocity loop may push, either way

// --- Holding position (phase HOLD) -----------------------------------------
// A damped position hold, NOT the firmware's brake-hold. The firmware hold is a stiff position loop
// with gains we don't control: handing a still-moving, springy six-bar (chain stretch, 4:1 backlash,
// rubber bands) to it is what made the arm ring up and down after L1. This one has an explicit damping
// term, so the arriving arm's energy is absorbed instead of stored:
//   mV = LIFT_HOLD_KP * (error outside the deadband) + integral(LIFT_HOLD_KI * error) - LIFT_HOLD_KD * speed
//   Sags below the level over a few seconds: raise LIFT_HOLD_KI first, then LIFT_HOLD_KP.
//   Buzzes / twitches while holding: raise LIFT_HOLD_DEADBAND_DEG, then lower LIFT_HOLD_KP.
//   Still bounces once or twice on arrival: RAISE LIFT_HOLD_KD. This is the anti-bounce knob.
//   Audibly straining while parked at a high level: lower LIFT_HOLD_MAX_MV (expect some sag).
inline constexpr float LIFT_HOLD_KP = 180.0f;                // TUNE: mV per deg of sag
inline constexpr float LIFT_HOLD_KI = 400.0f;                // TUNE: mV per deg per second (learns gravity)
inline constexpr float LIFT_HOLD_KD = 60.0f;                 // TUNE: mV per deg/s -- the anti-bounce term
inline constexpr double LIFT_HOLD_DEADBAND_DEG = 0.8;        // TUNE: inside this, only the learned gravity term acts
inline constexpr float LIFT_HOLD_MAX_MV = 7000.0f;           // TUNE: most the hold may push

// --- The descent to the bottom (L2, B, auton DOWN, and R2 in clamped mode) --
// Three speed bands on the way down, as % of the free speed, so the arm walks itself onto its hard
// stop instead of falling onto it. Band boundaries are read off the arm's POSITION (degrees above 0):
//   top    : from wherever the descent started, down to the mid boundary -> LIFT_DESCENT_TOP_PCT
//   middle : from the mid boundary down to LIFT_DESCENT_SLOW_ZONE_DEG   -> LIFT_DESCENT_MID_PCT
//   slow   : LIFT_DESCENT_SLOW_ZONE_DEG down to the stop                -> LIFT_DESCENT_SLOW_PCT
// The mid boundary is a FRACTION of the travel between the start height and the slow zone, not a fixed
// height, so "the middle" really is the middle of this particular descent whether it starts at level 1
// or level 5: mid boundary = SLOW_ZONE + LIFT_DESCENT_MID_FRACTION * (start height - SLOW_ZONE).
// Band changes are short ramps at LIFT_DESCENT_RAMP_DEG_PER_S2, never stops.
//   Falls too fast off a high level: lower LIFT_DESCENT_TOP_PCT / LIFT_DESCENT_MID_PCT.
//   Still arrives at the bottom with a thump: raise LIFT_DESCENT_SLOW_ZONE_DEG, lower LIFT_DESCENT_SLOW_PCT.
//   The whole descent takes too long: raise LIFT_DESCENT_MID_PCT, lower LIFT_DESCENT_SLOW_ZONE_DEG.
inline constexpr double LIFT_DESCENT_TOP_PCT = 50.0;         // TUNE: ~75 deg/s
inline constexpr double LIFT_DESCENT_MID_PCT = 75.0;         // TUNE: ~112 deg/s
inline constexpr double LIFT_DESCENT_SLOW_PCT = 12.0;        // TUNE: ~18 deg/s, the constant slow final descent
inline constexpr double LIFT_DESCENT_SLOW_ZONE_DEG = 20.0;   // TUNE: slow zone starts this many deg above 0
inline constexpr double LIFT_DESCENT_MID_FRACTION = 0.5;     // TUNE: 0.5 = the mid band is the lower half of the fast travel
inline constexpr double LIFT_DESCENT_RAMP_DEG_PER_S2 = 300.0;// TUNE: how quickly the reference may change between bands

// --- Seating on the hard stop (phase SEAT) ---------------------------------
// The point of the slow final descent is that the arm does not STOP above the frame -- it keeps
// walking down until it physically cannot, and then stays pressed there so it cannot bounce back up.
// "Physically cannot" = the arm is below LIFT_SEAT_DETECT_DEG and has moved slower than
// LIFT_SEAT_STILL_DEG_PER_S for LIFT_SEAT_STILL_MS. After that the lift holds LIFT_SEAT_MV forever.
// HEAT: -1200 mV is 10% of full volts. A stalled V5 motor at 10% draws well under an amp (the winding
// is about 1.4 ohm), roughly a watt per motor, and the arm is resting on the frame so the seat voltage
// is only fighting bounce, not holding the arm up. That is safe for a whole match -- but watch the
// brain's TEMP line the first time, and LIFT_SEAT_TEMP_LIMIT_C drops the seat to a plain brake-hold if
// a lift motor ever does get that hot.
//   Bounces back up off the stop: raise the magnitude of LIFT_SEAT_MV (more negative, e.g. -1500).
//   Motors warm up while parked at the bottom: lower the magnitude (e.g. -800).
//   Declares itself seated while still a few degrees up: lower LIFT_SEAT_DETECT_DEG or
//     LIFT_SEAT_STILL_DEG_PER_S, or raise LIFT_SEAT_STILL_MS.
inline constexpr float LIFT_SEAT_MV = -1200.0f;              // TUNE: constant downward seating voltage
inline constexpr double LIFT_SEAT_DETECT_DEG = 6.0;          // TUNE: only this close to 0 may "stopped" mean "seated"
inline constexpr double LIFT_SEAT_STILL_DEG_PER_S = 4.0;     // TUNE
inline constexpr int LIFT_SEAT_STILL_MS = 150;               // TUNE
// If something knocks a seated arm this far back up above LIFT_SEAT_DETECT_DEG, the descent restarts
// and walks it down again rather than leaving it hanging.
inline constexpr double LIFT_SEAT_UNSEAT_DEG = 4.0;          // TUNE
inline constexpr double LIFT_SEAT_TEMP_LIMIT_C = 50.0;       // safety net, not a tuning knob

// --- Clamped-mode R1 / R2 lift (see src/main.cpp and README "Driver guide") --
// While the claw is CLAMPED on a pin, R1/R2 stop being intake/extake and become lift up/down.
// Up is plain voltage; down goes through the banded descent above (so it seats at the bottom exactly
// like L2 does) with this as its top-band speed.
inline constexpr double R_LIFT_UP_PCT = 90.0;                // TUNE: R1 up, % of full voltage
inline constexpr double R_LIFT_DOWN_PCT = 50.0;              // TUNE: R2 down, % of free speed

// --- Arrival bands ---------------------------------------------------------
// How close counts as arrived for the stack levels and MATCHLOAD. 5 deg of final error at a stack
// level is fine -- the pin only has to land in the cup. A goal already inside this band is "arrived"
// without moving at all. With the final approach above, moves normally land inside ~2.5 deg anyway.
inline constexpr double LIFT_TOLERANCE_DEG = 5.0;            // TUNE: stack levels, MATCHLOAD
// INTAKE is the exception: the claw has to line up with the handoff, so it is a PRECISE move and gets
// this tight band. Lift::setTarget(h, true) marks a move precise, and any goal within half a degree of
// lift_preset::INTAKE is promoted automatically so no call site can forget.
inline constexpr double LIFT_PRECISE_TOLERANCE_DEG = 1.0;    // TUNE: INTAKE only
// Any NON-PRECISE goal at or below this is "a trip to the bottom": it uses the banded descent + seating
// above instead of a position move, and Scoring's score() treats an arm already here as "nothing to
// score" (L2 is then a no-op). INTAKE (1.1) is inside it numerically but is always precise, so R1 keeps
// its own exact approach and never seats.
inline constexpr double LIFT_BOTTOM_GOAL_DEG = 5.0;          // TUNE
// Hands over to HOLD once the arm is inside its band AND slower than this. The hold has real damping
// now, so this does not have to be tiny -- but lowering it makes the handover gentler still.
inline constexpr double LIFT_STOPPED_DEG_PER_S = 8.0;        // TUNE
// The final approach can't take longer than this (arm fighting something, loop badly tuned): the lift
// hands over to HOLD, whose position term then pulls in whatever is left, gently.
inline constexpr int LIFT_APPROACH_TIMEOUT_MS = 1500;        // TUNE
// Velocity estimate: position differenced every tick, then this much of each new sample is blended in
// (1.0 = raw, no filtering). The rotation sensor's quantisation is 0.0025 arm deg, i.e. 0.25 deg/s per
// count, so there is nothing to filter out except task jitter -- and filtering costs LAG, which makes
// the brake react late and overshoot. 0.8 is a quarter of a tick of lag; 0.5 (the old value) was a
// whole tick, which at 150 deg/s is 1.5 deg of stale position.
inline constexpr double LIFT_VEL_FILTER = 0.8;

// ---------------------------------------------------------------------------
// Lift motion — how those knobs fit together. Read lift.hpp's class comment for the full picture.
// ---------------------------------------------------------------------------
// There are exactly two kinds of commanded move, and one manual input, all inside the single 10 ms
// Lift::update() that owns the motors:
//
// A POSITION MOVE (every goal except a trip to the bottom) is DRIVE -> BRAKE -> APPROACH -> HOLD:
//   DRIVE     full +-LIFT_MAX_MV, the exact thing the D-pad does, until the arm is up to the speed
//             the braking curve allows -- so a move is still off the line instantly.
//   BRAKE     the velocity loop tracks min(cruise cap, braking curve) all the way in.
//   APPROACH  the same loop, now tracking LIFT_APPROACH_KP_PER_S * (degrees left) capped at
//             LIFT_APPROACH_DEG_PER_S, so the arm is down to a crawl before it stops.
//   HOLD      the damped position hold (KP/KI/KD above). Its integral carries the learned gravity
//             voltage over to the next move, so no move ever starts by guessing it.
//
// A DESCENT (a non-precise goal at or below LIFT_BOTTOM_GOAL_DEG: L2, B, auton DOWN, R2 in clamped
// mode) is DESCEND -> SEAT: the three speed bands above, then a constant LIFT_SEAT_MV pressed into the
// hard stop. It does not stop above the frame and it does not bounce.
//
// THE MANUAL INPUT (D-pad Up/Down, and R1 in clamped mode) is plain voltage straight into the same
// loop, ahead of everything else; it cancels any target and any homing instantly, and letting go
// brake-holds where it is.
//
// Why this and not the designs before it: a jumped setpoint with kP = 600 oscillated (d2978c7); a
// trapezoidal profile + leash + feedforward + PID + gravity term + arrive/hold hysteresis (6b8165c)
// was slow off the line and still oscillated because its hold used a FIXED gravity feedforward that is
// wrong at most angles of a six-bar; and DRIVE -> BRAKE -> firmware-hold (3723ef2) overshot and rang,
// because LIFT_BRAKE_DECEL_DEG_PER_S2 = 2000 asked for more braking than the arm can do, the velocity
// estimate lagged a whole tick, enterBrake() seeded the loop's integral with an upward kick, and the
// handover at up to 20 deg/s dropped a still-moving springy arm into the firmware's stiff hold, which
// stored the energy and gave it back. All four of those are fixed above: realistic decel, less filter
// lag, the integral seeded from the learned gravity voltage, a crawl before the stop, and a hold with
// an explicit damping term. Nothing re-drives or re-chases any more -- the hold IS the correction, and
// it is gentle, so LIFT_REENGAGE_DEG / LIFT_MAX_REDRIVES are gone.

// ---------------------------------------------------------------------------
// Lift guards
// ---------------------------------------------------------------------------
// Stall guard: a phase that WANTS real motion (its speed reference is at least 5 deg/s) but the arm
// moved less than STALL_DEG in STALL_MS = it can't get there (preset above the real top, or jammed).
// The lift gives up on that target, holds where it is and the brain shows BLOCKED. It is NOT checked
// once the final approach has tapered the reference below 5 deg/s (where slow progress is the plan,
// not a jam), nor in HOLD, nor below LIFT_SEAT_DETECT_DEG during a descent -- an arm that has stopped
// down there is seated, which is the goal, not a fault. It cannot false-trip: 5 deg/s covers 0.5 deg
// in 100 ms, a fifth of the window.
//   Shows BLOCKED on a normal move: first check for a real jam or a preset above the top. Then raise
//   LIFT_STALL_MS.
inline constexpr double LIFT_STALL_DEG = 0.5;                // TUNE: arm deg of progress that counts as moving
inline constexpr int LIFT_STALL_MS = 500;                    // TUNE
// Runaway guard: during DRIVE the lift pushes full voltage toward the goal, so the distance to the goal
// can only shrink. If it instead GROWS by this much from the smallest it has been during this DRIVE,
// the sensor or the motors are reversed (rotation sensor direction, or a LIFT_PORTS sign): cut power,
// "LIFT FAULT" on the brain. A reversed drive runs away at 150 deg/s and trips this within ~60 ms. A
// B press while the arm is still rising fast coasts up by a degree or two before it reverses -- that is
// far below this threshold, and it is why the threshold is not smaller.
// A FAULT is cleared by the manual override OR by a new explicit target (see Lift::setTarget()): the
// guard's job is to stop the motion happening now, not to brick every button until someone remembers
// to wiggle the D-pad. A real wiring fault simply re-faults on the next DRIVE.
//   LIFT FAULT on a normal move: run the direction check in the README before touching this number.
inline constexpr double LIFT_FAULT_DEG = 8.0;                // TUNE

// ---------------------------------------------------------------------------
// Lift trace — one line per 20 ms to the PROS terminal while the arm moves
// ---------------------------------------------------------------------------
// VS Code -> PROS -> "Brain Terminal" (or `pros terminal`), press a lift button, paste the lines back.
// Columns: `LIFT <ms> <phase> <pos*10> <goal*10> <vel deg/s> <out mV>`; phase is D=DRIVE, B=BRAKE,
// A=APPROACH (the final crawl), H=HOLD, V=DESCEND (a banded trip to the bottom), S=SEAT (pressed onto
// the hard stop), M=manual (D-pad, or R1 in clamped mode), h=homing, F=fault, I=idle. Integers only (no float formatting on the
// control task), sampled every LIFT_TRACE_EVERY_TICKS, written from a separate low-priority task that
// drains a ring buffer -- the 10 ms control loop never calls printf. Keeps going LIFT_TRACE_TAIL_MS
// after the arm stops so the hold is visible too. Set false to silence it entirely (no task started).
inline constexpr bool LIFT_TRACE = true;
inline constexpr int LIFT_TRACE_EVERY_TICKS = 2;             // 2 x 10 ms = one line per 20 ms
inline constexpr int LIFT_TRACE_TAIL_MS = 500;

// ---------------------------------------------------------------------------
// Lift homing — find the hard stop and zero there, once per program run
// ---------------------------------------------------------------------------
// Why: the arm often powers on sitting well above the bottom. Lift::zero() can only seed the position
// counter from the rotation sensor's folded absolute angle, which is unambiguous only within +-45 arm
// deg (+-180 sensor deg) of the true bottom -- further up than that and every preset is off by a whole
// sensor revolution. Homing removes the guesswork: drive the arm down at a steady, moderate voltage
// until the sensor stops changing, confirm the sensor's absolute angle really is near its calibrated
// zero, and set_position(0) there.
//
// This is the ONE deliberate exception to "nothing moves at startup" (CLAUDE.md safety rule 1), and it
// is kept narrow: homing never runs in initialize(). It runs on the first ENABLED mode of a program
// run -- autonomous(), or opcontrol() if auton did not already do it -- so the field/competition
// switch, not power-on, is what makes the arm move. It runs inside Lift::update() as a mode, so the
// 10 ms task is still the only thing that writes the lift motors, nothing blocks on it, and holding
// D-pad Up/Down cancels it instantly (safety rule 3 is untouched). Lift targets asked for during
// homing are queued and run the moment it finishes.
inline constexpr float LIFT_HOME_MV = -4000.0f;              // TUNE: steady downward drive while homing (negative = down)
// Don't start looking for "stopped" until the arm has been driving this long, so a sticky start isn't
// mistaken for the bottom.
inline constexpr int LIFT_HOME_MIN_DRIVE_MS = 250;           // TUNE
// "Stopped" = the raw sensor moved less than LIFT_HOME_STILL_DEG (SENSOR degrees, not arm degrees)
// across a LIFT_HOME_STILL_MS window. Small wobble at the stop is expected and allowed.
inline constexpr double LIFT_HOME_STILL_DEG = 2.5;           // TUNE: sensor degrees
inline constexpr int LIFT_HOME_STILL_MS = 200;               // TUNE
// Plausibility window: at the stop, the sensor's own absolute angle (folded to +-180 sensor deg) must
// be within this much of its calibrated zero before we trust it and set_position(0). The user's own
// observation is that it settles around 358-359-0-1, i.e. within a degree or two. If the arm stopped
// somewhere else it is jammed on something, NOT at the bottom: homing then stops pushing, keeps the
// absolute-angle seed from zero(), and shows HOME FAIL instead of zeroing to a wrong place.
inline constexpr double LIFT_HOME_ZERO_WINDOW_DEG = 15.0;    // TUNE: sensor degrees either side of zero
// Hard stop on the whole attempt. Stops pushing and shows HOME FAIL.
inline constexpr int LIFT_HOME_TIMEOUT_MS = 3000;            // TUNE

// Default timeout for lift.moveTo() when the caller doesn't give one (autonomous safety net so a
// stuck lift can't hang a routine forever). Pass a different value per call, or 0 to wait forever.
// A blocking moveTo() called while homing waits for homing AND the move, so give it room for both.
inline constexpr int LIFT_DEFAULT_TIMEOUT_MS = 2500;         // TUNE

// ---------------------------------------------------------------------------
// Claw (pneumatic) + autoclamp sensor
// ---------------------------------------------------------------------------
// THE TWO CYLINDERS WERE CROSSED IN CODE UNTIL 2026-10-01. The port the code called the claw ('C')
// is physically the PIVOT cylinder, and the port it called the pivot ('D') is physically the CLAW.
// The user found this on the robot; the ports below are the corrected, physical assignment, and every
// polarity note here was re-derived from scratch afterwards (see the derivation under each constant).
inline constexpr char CLAW_PISTON_PORT = 'D';                // physically the claw (was 'C', which is the pivot)
// Which ADI level CLOSES the jaws.
//
// DERIVATION. Every build this robot has ever run wrote BOTH ADI pins LOW at construction (the claw
// object: Pneumatics(port, start_extended=false), then DigitalOut(port, LOW); the pivot object the
// same), so at power-on C and D are both LOW, which is also their unpowered level. On the 3723ef2
// build, R1/INTAKE wrote the "claw" object's open level to C (LOW) and the "pivot" object's BACK level
// to D (HIGH). The driver's report of that run was "clamp and pivot reversed at the intake": the claw
// was gripping when the code wanted it open. The pin that was HIGH at that moment was D -- the real
// claw. So HIGH closes the jaws, and the unpowered LOW level is OPEN.
// That also makes mechanical sense: a single-acting VEX cylinder needs air to grip, and this is the
// arrangement where losing air opens the jaws rather than locking them shut.
// CONSEQUENCE FOR STARTUP: the jaws are physically OPEN at power-on, so Claw starts in State::OPEN and
// writes the pin the level it already sits at -- nothing moves at startup (CLAUDE.md rule 1). It also
// means the claw cannot hold a preloaded pin with the program off; press A or L1 to grip one.
// This is a one-observation deduction, so confirm it in 10 seconds: at power-on the brain's CLAW line
// reads `CLAW OPEN D[0]`; press A and the jaws must CLOSE while the line reads `CLAMPED D[1]`. If A
// opens them instead, flip this one constant and nothing else.
inline constexpr bool CLAW_CLOSED_IS_ADI_HIGH = true;        // TODO(team): confirm with A on the brain's CLAW line
// Autoclamp: a distance sensor looking between the jaws. None is in the notebook yet.
inline constexpr int8_t CLAW_DISTANCE_PORT = 11;             // TODO(team): mount + confirm port
inline constexpr int CLAW_DETECT_MM = 40;                    // TODO(team): distance that means "pin is between the jaws"
inline constexpr int CLAW_DETECT_DEBOUNCE_MS = 60;           // pin must be seen this long before clamping
inline constexpr int CLAW_RELEASE_LOCKOUT_MS = 750;          // after a release, don't re-clamp for this long
inline constexpr bool CLAW_AUTOCLAMP_AT_START = false;       // Y toggles it in a match. Leave off until the sensor is mounted.

// ---------------------------------------------------------------------------
// Intake (ONE motor)
// ---------------------------------------------------------------------------
// Negative = reversed. If R1 spits cups out instead of pulling them in, flip the sign.
inline constexpr int8_t INTAKE_PORT = 12;
inline constexpr float INTAKE_MV = 12000.0f;

// ---------------------------------------------------------------------------
// Pivot + claw timing. See include/subsystems/scoring.hpp.
// ---------------------------------------------------------------------------
// If the claw/wrist flips with its own cylinder, set the port. ' ' = no pivot actuator.
inline constexpr char PIVOT_PISTON_PORT = 'C';               // physically the pivot (was 'D', which is the claw)
// THE PIVOT HAS TWO POSITIONS, named for where the end effector physically is (the driver's words):
//   OUT  = swung out. Where it sits at power-on, at every stack level while scoring, and in the
//          default down position after L2 / B. In the code, pivot OUT == true.
//   BACK = pulled in toward the intake handoff, for taking a pin from the intake (R1 / INTAKE).
//          In the code, pivot BACK == false.
// DERIVATION of which ADI level is OUT -- now from the correct pin, and confirmed twice:
//  1. At power-on every ADI output is LOW, the code writes LOW anyway, and the driver's own
//     observation is that the physical pivot sits OUT. C is the pivot. So C LOW = OUT.
//  2. Independently: on the 6b8165c build, Claw::clamp() at every stack level wrote HIGH to C (that
//     build's "claw" object lived on C), and the driver reported the pivot sat PULLED IN at every
//     stack level. So C HIGH = BACK. Same answer from a different run.
// So the unpowered/LOW level is OUT and HIGH is BACK -- which is also the only arrangement where
// Scoring can start with the pivot logically OUT without writing anything the ADI isn't already at, so
// nothing moves at startup (CLAUDE.md rule 1). Confirm with X: the line reads `PIV OUT C[0]` at
// power-on and must read `BACK C[1]` with the pivot pulled in after one press.
inline constexpr bool PIVOT_OUT_IS_ADI_HIGH = false;         // confirmed twice (see above); re-check with X
// The matchload spot (scoreOrMatchload(), not on a driver button): take the pin with the pivot BACK
// like at the intake, or OUT like everywhere else?
inline constexpr bool PIVOT_BACK_AT_MATCHLOAD = false;       // TODO(team): confirm
// The handoff zone: from lift_preset::INTAKE up to this many degrees above it. While the intake macro
// is heading to or sitting at the intake, the pivot is BACK inside this zone and OUT above it, so on
// R1 it swings BACK as the arm comes down into the zone (not only once the lift reports "arrived" --
// a slightly short stop still gets the pivot BACK), and on L1 it swings OUT as the arm rises out of
// the zone (or when it arrives at the level, whichever is first, so a level can never be left waiting
// for it). It only ever times the swing; it never gates the lift. This is the ONLY height the pivot's
// motion is tied to -- the old PIVOT_CLEAR interlock is gone (see scoring.hpp).
//   The pin swings into the intake as it leaves the handoff on L1: raise it.
//   The pivot is still swinging when the arm reaches level 1, or still OUT when the arm reaches the
//   handoff on R1: lower it.
inline constexpr double PIVOT_HANDOFF_ZONE_DEG = 10.0;       // TUNE
// How long the pivot takes to finish swinging. NOTE: until the port swap this timer was watching the
// CLAW cylinder, so it has never actually been measured against the pivot. 250 ms is a safe guess.
inline constexpr int PIVOT_TIME_MS = 250;                    // TUNE: never measured on the real pivot
// THIS IS THE L1 DELAY. The first L1 tap closes the jaws and waits here before the lift is given any
// target at all (Scoring::update(), case CLAMPING) -- interlock: the arm must not pull out of an open
// claw. So the gap between pressing L1 and the arm moving is at least this long, by design.
// NOTE: this was trimmed to 100 ms while the ports were crossed, i.e. it was timing the PIVOT
// cylinder, not the claw. Put back to 150 ms, which is a sane starting point for a pneumatic claw that
// has to actually grip a pin before the arm pulls away.
//   Pin slips / the arm starts up before the jaws are shut: raise it (175, 200).
//   The gap between the first L1 tap and the arm moving annoys you: lower it, carefully.
// (Every L1 tap AFTER the first one commands the next level immediately -- no wait.)
inline constexpr int CLAW_CLOSE_TIME_MS = 150;               // TUNE: never measured on the real claw
// Jaws open before the arm lowers -- only for scoreOrMatchload()'s path (not currently on a button).
// L2's score() uses L2_RELEASE_WAIT_MS below instead.
inline constexpr int CLAW_OPEN_TIME_MS = 200;                // TUNE: never measured on the real claw
// L2 (score). The jaws open, and the arm then stands completely still for this long before the descent
// starts, so the pin is free and settled in the cup before the end effector moves away from it.
//   The pin gets dragged / knocked as the arm leaves: raise it.
//   Scoring feels sluggish and the pin is clearly free: lower it (150).
inline constexpr int L2_RELEASE_WAIT_MS = 250;               // TUNE

}  // namespace config
