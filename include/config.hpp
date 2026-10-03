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
// Ports CONFIRMED on the robot 2026-10-02. TODO(team): confirm whether the 55W is 5x 11W or
// 4x 11W + 2x 5.5W (cartridge colour only, not the ports).
inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS = {-8, -2, -1};   // confirmed 2026-10-02
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = {3, 9, 10};     // confirmed 2026-10-02

inline constexpr float DRIVE_WHEEL_DIAMETER_IN = 2.75f;      // notebook: 2.75" omni wheels
inline constexpr float DRIVE_RPM = 450.0f;                   // notebook: 450 RPM
inline constexpr float TRACK_WIDTH_IN = 11.5f;               // TODO(team): measure wheel-center to wheel-center
inline constexpr float HORIZONTAL_DRIFT = 2.0f;              // LemLib: 2 for a drive with traction wheels

// Odometry: one vertical tracking wheel pod. No inertial sensor is on the robot yet.
inline constexpr int8_t ODOM_VERTICAL_ROTATION_PORT = 17;     // confirmed 2026-10-02 (negative to reverse)
inline constexpr float ODOM_WHEEL_DIAMETER_IN = 2.0f;        // TODO(team): confirm pod wheel (2" / 2.75")
inline constexpr float ODOM_VERTICAL_OFFSET_IN = 0.0f;       // TODO(team): measure left/right offset from center
// Inertial sensor: AN IMU IS NOW FITTED, on port 18 (confirmed 2026-10-02 -- this used to read 0,
// "none fitted", and before that 8, which is a right-drive motor). robot.cpp builds a pros::Imu on it
// and hands it to LemLib alongside the vertical tracking wheel, so heading comes from the IMU and
// distance from the pod. It is calibrated in a BACKGROUND task from initialize() and opcontrol()
// NEVER waits on it (CLAUDE.md safety rule 4: nothing may delay driver control, not even an unplugged
// or still-calibrating IMU). autonomous() does wait, because a drive move needs a heading.
// Set this back to 0 to run on the tracking wheel alone: robot.cpp then builds no pros::Imu at all,
// LemLib gets a null IMU pointer (its own documented way to do that), nothing calibrates it and the
// brain shows "IMU  none" instead of junk. Nothing else changes either way.
inline constexpr int8_t IMU_PORT = 18;                        // confirmed 2026-10-02: IMU fitted

// ---- Chassis PID: NOT HERE, ON PURPOSE ----
// The lateral/angular lemlib::ControllerSettings -- and the ExpoDriveCurve driver curves -- are typed
// as literal numbers directly in src/robot.cpp, where the driver tunes them. That is the user's own
// decision (2026-10-02): "remove these drive pid values from config, i want to be able to tune and
// enter them in here". Do NOT move them back into this file. Everything else about the robot still
// belongs here; the chassis PID and the drive curves are the one documented exception.

// ---------------------------------------------------------------------------
// Lift (DR6-4B)
// ---------------------------------------------------------------------------
// Two motors on the 12T motor axle, geared into the 48T lift-arm axle: 12T -> 48T = 4:1 torque.
// Everywhere else in the code (presets, PID target/error, lift.getPosition(), the brain screen)
// works in LIFT-ARM DEGREES -- this ratio is what converts the motor/sensor side of the 4:1 down
// to that one shared unit. See Lift::getMotorPosition()/getSensorPosition() in subsystems/lift.cpp.
inline const std::vector<std::int8_t> LIFT_PORTS = {20, -13};             // confirmed 2026-10-02
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
// FRONT TOGGLE: the height the arm goes to at the start of the match route to flip the front toggle,
// then comes straight back down from (fast -- see auton::liftDownFast()). Deliberately its OWN
// constant rather than STACK_LEVELS[0] (31.0), which it is near: the two are tuned for different jobs
// and must be able to move apart.
inline constexpr double FRONT_TOGGLE = 30.0;                 // TUNE: arm degrees
// L1 taps: tap 1 -> level 1, tap 2 -> level 2 ... tap 5 -> level 5 (stays at 5, the top). One entry
// per pin-stack height.
//
// MEASURED ON THE ROBOT, 2026-10-01, and these replace the photo-derived numbers below. The driver
// drove the arm to each scoring height with the D-pad and read the brain's LIFT / ROT line, so they
// are ARM DEGREES -- the UNBRACKETED number on both of those lines. (The bracketed figure on the ROT
// line is raw SENSOR degrees, 4x bigger and printed with no decimal: level 5 used to read
// "ROT 128.3 [513]". The values here have one decimal and are in the 31..107 range, so they are the
// arm-degree column; read as sensor degrees they would divide down to 8..27 arm deg, which is less
// than one old stack level of total travel.)
//
// The old photo-measured set was 26.6 / 54.4 / 72.8 / 116.3 / 128.3. Those were taken on a build whose
// Lift::zero() still reset the sensor at every boot, so they all carried whatever offset the arm was
// sitting at that session -- exactly the caveat written below. These are measured against the real
// absolute zero with homing working, which is why the top is 107 rather than 128.
inline constexpr int STACK_LEVEL_COUNT = 5;
inline constexpr double STACK_LEVELS[STACK_LEVEL_COUNT] = {
    31.0,   // measured on the robot 2026-10-01 (level 1)
    59.0,   // measured on the robot 2026-10-01 (level 2)
    74.6,   // measured on the robot 2026-10-01 (level 3)
    87.3,   // measured on the robot 2026-10-01 (level 4)
    106.9,  // measured on the robot 2026-10-01 (level 5, the top)
};
// Software hard stop: top level (106.9) + 5 deg of margin. setTarget()/moveTo() clamp to this, so it
// also doubles as "never command past here" -- raise it only once a higher level is actually measured
// and added above, never just to let a bad preset through. (The D-pad is raw voltage and is NOT
// clamped by this; it is only a limit on commanded heights.)
inline constexpr double MAX = 111.9;
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
// left minus LIFT_APPROACH_ZONE_DEG. This is the single number that decides whether it overshoots --
// and, just as much, how FAST a move feels, because it sets how much of the move is spent braking.
// Read it off the trace: in the B lines `vel` should fall by about a * 0.02 per 20 ms line.
//   Overshoots the level / arrives still moving: LOWER it (braking starts earlier): 1000, 850.
//   Feels slow / spends most of the move braking: RAISE it (braking starts later): 1400.
// SIZING IT. Braking distance from the 150 deg/s free speed is 150^2 / (2a), and braking begins that
// far out PLUS LIFT_APPROACH_ZONE_DEG. The stack levels are 12-44 deg apart, so for a move to have any
// full-power section at all, (150^2 / 2a) + zone has to be well under ~25 deg:
//   a =  700 -> 16.1 + 8 = 24.1 deg of braking. A 26.6 deg move is then ~2 deg of power and 24 of
//                braking -- that is exactly why the start and middle "felt slower".
//   a = 1200 -> 9.4 + 5 = 14.4 deg. Half the move is full power. This is the value here.
//   a = 2000 -> 5.6 + 5 = 10.6 deg, but 2000 deg/s^2 is more than the arm can actually do -- that was
//                the original overshoot (3723ef2). 1200 is 40% less demanding than that.
inline constexpr double LIFT_DECEL_DEG_PER_S2 = 1200.0;      // TUNE: how hard it brakes, arm deg/s^2

// --- A COARSE move: speed over precision, overshoot allowed ------------------
// For a height where only "get there quickly" matters and landing a few degrees past it costs
// nothing -- today that is the FRONT TOGGLE rise, which exists only to touch a height and come
// straight back down. The driver: "lift to front toggle quite fast too, like a little faster than how
// we rise to level 1, this movement doesnt have to be precise and requires speed over precision, can
// over shoot."
// A normal rise is already at full voltage off the line; what makes it feel measured is the BRAKING,
// which starts 150^2/(2a) + zone degrees out -- 14.4 deg of a 30 deg move at a = 1200, i.e. half the
// move. A coarse move brakes LATER, keeps a quick tail instead of the crawl, and counts a much wider
// band as arrived:
//   DECEL  2600 asks for more deceleration than this arm can actually deliver. On an ordinary move
//          that would be a bug (it is why 3723ef2's 2000 overshot and rang); here it is the POINT --
//          the arm arrives still moving and sails a little past, which is explicitly fine.
//   MIN    the approach tail does not taper to a crawl, it stays at this speed until the band.
//   TOL    a goal this wide counts as arrived, so it latches into the hold immediately instead of
//          walking the last few degrees in.
//   Overshoots too far / bangs into something above the goal: lower LIFT_COARSE_DECEL_DEG_PER_S2
//     (2600 -> 1800) first, then LIFT_COARSE_MIN_DEG_PER_S.
//   Still feels no faster than a normal rise: raise LIFT_COARSE_DECEL_DEG_PER_S2.
inline constexpr double LIFT_COARSE_DECEL_DEG_PER_S2 = 2600.0;  // TUNE: brake this late (normal 1200)
inline constexpr double LIFT_COARSE_MIN_DEG_PER_S = 45.0;       // TUNE: no crawl (normal 6)
inline constexpr double LIFT_COARSE_TOLERANCE_DEG = 10.0;       // TUNE: arrived band (normal 5)

// --- The last few degrees (the final approach) ------------------------------
// Inside LIFT_APPROACH_ZONE_DEG the speed reference becomes LIFT_APPROACH_KP_PER_S * (degrees left),
// clamped between LIFT_APPROACH_MIN_DEG_PER_S and LIFT_APPROACH_DEG_PER_S. So the arm walks the last
// few degrees in at a crawl and hands over to the hold already nearly stopped -- this is what stops
// the six-bar springing.
// THE MINIMUM MATTERS. kP * (degrees left) on its own decays exponentially, so the arm creeps toward
// the goal more and more slowly and never really gets there: with kP = 3 the last 6 deg took 0.8 s,
// which is most of why a short move felt slow. The floor turns the tail into a constant walk-in
// instead, and LIFT_STOPPED_DEG_PER_S is set above it so arrival still latches.
//   Still visibly moving when it grabs the hold: lower LIFT_APPROACH_DEG_PER_S, or
//     LIFT_APPROACH_MIN_DEG_PER_S, or raise LIFT_HOLD_KD (the anti-bounce term) first.
//   The last few degrees take too long: raise LIFT_APPROACH_KP_PER_S, then
//     LIFT_APPROACH_MIN_DEG_PER_S; shrink LIFT_APPROACH_ZONE_DEG to start the crawl later.
// At a = 1200 the braking curve is already down to 28 deg/s a third of a degree outside a 5 deg zone,
// so the handover from braking into the crawl is seamless rather than a step.
inline constexpr double LIFT_APPROACH_ZONE_DEG = 5.0;        // TUNE: where the crawl starts
inline constexpr double LIFT_APPROACH_DEG_PER_S = 28.0;      // TUNE: fastest the crawl may be
inline constexpr double LIFT_APPROACH_MIN_DEG_PER_S = 6.0;   // TUNE: slowest -- no exponential tail
inline constexpr double LIFT_APPROACH_KP_PER_S = 6.0;        // TUNE: crawl speed per degree left (1/s)

// --- The velocity loop (phases BRAKE, APPROACH and DESCEND) -----------------
// Everything that is "go at this speed" runs through this one PI loop on the MEASURED arm speed:
//   mV = LIFT_VEL_KP * (reference - measured) + integral of LIFT_VEL_KI * (reference - measured)
// The integral is what learns gravity -- whatever voltage the arm needs at this angle to hold a speed,
// up or down -- so no gravity constant has to be right in advance.
//   `vel` swings above and below the reference in the trace (hunts): LOWER LIFT_VEL_KP.
//   Sags / is slow to reach the reference speed: RAISE LIFT_VEL_KI.
//   The braking feels violent: LOWER LIFT_VEL_MAX_MV (and lower LIFT_DECEL_DEG_PER_S2 to match).
// LIFT_VEL_MAX_MV IS 12000 ON PURPOSE. It was 9000, and that was a second reason the middle of a long
// move felt slow: holding the arm at its 150 deg/s free speed needs essentially the full 12000 mV
// (back-EMF alone is 12000/150 = 80 mV per deg/s), so a 9000 mV clamp physically could not sustain the
// cruise -- the arm sagged to whatever 9000 mV holds, around 110 deg/s, for the whole middle of the
// rise. The clamp was never what limits the braking either; LIFT_DECEL_DEG_PER_S2 is.
inline constexpr float LIFT_VEL_KP = 30.0f;                  // TUNE: mV per (deg/s) of speed error
inline constexpr float LIFT_VEL_KI = 400.0f;                 // TUNE: mV per (deg/s) of speed error, per second
inline constexpr float LIFT_VEL_MAX_MV = 12000.0f;           // TUNE: most the velocity loop may push, either way
// Hysteresis between DRIVE (full voltage) and BRAKE (the velocity loop). DRIVE ends the moment the arm
// reaches the profile speed, and is only re-entered once the arm is this far BELOW it -- which is what
// stops the two flickering tick-to-tick around the boundary, and what lets a mid-move retarget drop
// straight back into full power.
//   The trace shows D and B alternating every line or two: RAISE it.
//   It takes too long to go back to full power after an L1 retarget: LOWER it.
inline constexpr double LIFT_DRIVE_RESUME_DEG_PER_S = 20.0;  // TUNE

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
inline constexpr float LIFT_HOLD_KD = 90.0f;                 // TUNE: mV per deg/s -- the anti-bounce term
inline constexpr float LIFT_HOLD_MAX_MV = 7000.0f;           // TUNE: most the hold may push
// THE SETTLE BAND (was LIFT_HOLD_DEADBAND_DEG = 0.8, which is most of why the arm hunted at a level --
// see "Settling instead of hunting" below). Inside it the hold applies NO position correction and
// FREEZES its integral: it just keeps pushing the gravity voltage it has already learned. 4 deg is
// well inside the 5 deg arrival tolerance the driver is happy with at a stack level ("5-10 deg is
// fine"), and the arm is simply left where it stopped instead of being argued with.
//   Still creeps down out of the band over a few seconds: raise LIFT_HOLD_KI a little, NOT this.
//   Visibly off the level: lower this (and LIFT_TOLERANCE_DEG with it).
inline constexpr double LIFT_HOLD_DEADBAND_DEG = 4.0;        // TUNE: stack levels, MATCHLOAD, any custom height
// INTAKE must still line up, so a PRECISE goal gets a tight band and keeps correcting right in.
inline constexpr double LIFT_HOLD_PRECISE_DEADBAND_DEG = 0.5;  // TUNE: INTAKE only

// --- Settling instead of hunting: the PARK phase ------------------------------
// WHY. A damped position hold on a springy six-bar with 4:1 backlash cannot sit perfectly still: the
// integral winds up against stiction until the arm breaks free, overshoots the 0.8 deg deadband, and
// the integral then has to unwind -- a slow limit cycle, which is the "still oscillating a good
// amount" the driver sees. Arrival also latches with up to LIFT_TOLERANCE_DEG of error and up to
// LIFT_STOPPED_DEG_PER_S of speed, so the hold starts with something real to chase.
// THE FIX, in the driver's own words ("id rather it just settle instead of keeping on trying to do
// it"): stop using feedback at all once the arm is near enough. PARK is a CONSTANT voltage -- the
// gravity feedforward the hold had learned, frozen at that instant -- with no P, no D and no integral
// update. A constant voltage has no loop, so it physically cannot oscillate. Entered from HOLD either
// as soon as the arm has been inside the settle band and slow for LIFT_PARK_CONFIRM_MS, or
// unconditionally after LIFT_SETTLE_TIMEOUT_MS of holding, whichever comes first.
//   It parks visibly off the level: lower LIFT_HOLD_DEADBAND_DEG, or raise LIFT_SETTLE_TIMEOUT_MS so
//     the active hold gets longer to pull in first.
//   It sags away after parking: raise LIFT_HOLD_KI (so the learned voltage it freezes is bigger), or
//     lower LIFT_REENGAGE_DEG so the hold wakes sooner.
//   It still oscillates: LOWER LIFT_SETTLE_TIMEOUT_MS and LIFT_PARK_CONFIRM_MS -- that parks sooner,
//     i.e. less feedback for less time. This is the knob for "it still hunts".
inline constexpr int LIFT_PARK_CONFIRM_MS = 150;             // TUNE: inside the band and slow for this long
inline constexpr int LIFT_SETTLE_TIMEOUT_MS = 700;           // TUNE: ...or just hold this long, then park anyway
// Once parked, the hold only wakes up again if the arm has really moved away from where it parked (a
// knock, or a slow sag the frozen voltage could not carry). Deliberately large: waking for anything
// smaller is exactly the hunting this exists to stop.
inline constexpr double LIFT_REENGAGE_DEG = 8.0;             // TUNE
// ...and never sooner than this after parking, so PARK and HOLD cannot ping-pong.
inline constexpr int LIFT_PARK_COOLDOWN_MS = 300;            // TUNE
// A position move that has not arrived by now has met something it cannot get past (a preset above the
// real top, a jam the stall guard did not catch, a badly tuned loop): stop chasing it and PARK where
// it is, shown as PARK(t/o). Sized from the longest legitimate move -- DOWN to level 5 is 107 deg,
// which at the braked profile is well under a second -- plus a wide margin, and it has to cover a
// CLAMPING/pivot wait in front of the move too.
//   A real long move gets cut short: raise it.
inline constexpr int LIFT_MOVE_TIMEOUT_MS = 2500;            // TUNE

// --- The descent to the bottom (L2, B, auton DOWN, and R2 in clamped mode) --
// Three speed bands on the way down, as % of the free speed, so the arm walks itself onto its hard
// stop instead of falling onto it. Band boundaries are read off the arm's POSITION (degrees above 0):
//   top      : from wherever the descent started, down to the mid boundary -> LIFT_DESCENT_TOP_PCT
//   middle   : from the mid boundary down to LIFT_DESCENT_SLOW_ZONE_DEG    -> LIFT_DESCENT_MID_PCT
//   approach : LIFT_DESCENT_SLOW_ZONE_DEG down to LIFT_FINAL_START_DEG     -> LIFT_DESCENT_SLOW_PCT
// then FINAL (see "Seating on the hard stop" below) takes over at LIFT_FINAL_START_DEG. The mid
// boundary is a FRACTION of the travel between the start height and the slow zone, not a fixed height,
// so "the middle" really is the middle of this particular descent whether it starts at level 1 or
// level 5: mid boundary = SLOW_ZONE + LIFT_DESCENT_MID_FRACTION * (start height - SLOW_ZONE). Band
// changes are short ramps at LIFT_DESCENT_RAMP_DEG_PER_S2, never stops.
//
// RETUNED 2026-10-02: the arm was crawling the whole bottom 0-20 deg (SLOW_PCT was 12%, ~18 deg/s, from
// 20 deg all the way to the old 6 deg FINAL handover) and the user asked for it to "stay fast much
// lower". The fix is to shrink the approach zone AND raise its speed, moving the real slowdown to the
// last ~14 deg instead of the last ~20, with FINAL now starting at 3 deg (LIFT_FINAL_START_DEG) instead
// of 6:
//   LIFT_DESCENT_SLOW_ZONE_DEG   20 -> 14   (approach band starts later -- fast bands run longer)
//   LIFT_DESCENT_SLOW_PCT        12 -> 30   (~45 deg/s instead of ~18 -- a real cruise, not a crawl)
//   LIFT_DESCENT_RAMP_DEG_PER_S2 800 -> 1200 (see why below)
// WHY THE RAMP HAD TO GET FIRMER. Distance to decelerate from v1 to v2 at rate a is (v1^2-v2^2)/(2a).
// Worst case entering the approach zone is the MID band's ~142 deg/s (95%). At the OLD ramp (800),
// decelerating 142 -> 45 takes (142^2-45^2)/1600 = 11.3 deg -- almost the entire new 14-to-3 deg zone
// (11 deg) on its own, leaving no room to then decelerate 45 -> FINAL's 12 deg/s before hitting
// LIFT_FINAL_START_DEG, i.e. the arm would still be doing ~70+ deg/s AT 3 deg and FINAL would have to
// brake it hard right at the floor. At the NEW ramp (1200): 142 -> 45 takes (20164-2025)/2400 = 7.56
// deg, landing around position 14-7.56 = 6.4 deg, which leaves a real ~3.4 deg CRUISE at 45 deg/s down
// to the 3 deg mark; then 45 -> 12 (FINAL's target) takes only (2025-144)/2400 = 0.78 deg, settling by
// position ~2.2 deg with ~2.2 deg left to cruise into the stop at a steady 12 deg/s -- comfortable
// margin, no slam. 1200 deg/s^2 is not an arbitrary new number either: it is the same braking rate
// already proven safe for position moves (LIFT_DECEL_DEG_PER_S2, used since the original lift tuning).
//   Falls too fast off a high level: lower LIFT_DESCENT_TOP_PCT / LIFT_DESCENT_MID_PCT.
//   Still arrives at the approach band going faster than the approach speed (overshoots into FINAL
//     hot): raise LIFT_DESCENT_RAMP_DEG_PER_S2 further, or raise LIFT_DESCENT_SLOW_ZONE_DEG so there is
//     more room to decelerate.
//   The whole descent takes too long: raise LIFT_DESCENT_MID_PCT, lower LIFT_DESCENT_SLOW_ZONE_DEG, or
//     raise LIFT_DESCENT_SLOW_PCT (the approach cruise itself).
inline constexpr double LIFT_DESCENT_TOP_PCT = 80.0;         // TUNE: ~120 deg/s
inline constexpr double LIFT_DESCENT_MID_PCT = 95.0;         // TUNE: ~142 deg/s
inline constexpr double LIFT_DESCENT_SLOW_PCT = 30.0;        // TUNE: ~45 deg/s, the approach cruise (was 12% / 18 deg/s)
inline constexpr double LIFT_DESCENT_SLOW_ZONE_DEG = 14.0;   // TUNE: approach zone starts this many deg above 0 (was 20)
inline constexpr double LIFT_DESCENT_MID_FRACTION = 0.5;     // TUNE: 0.5 = the mid band is the lower half of the fast travel
inline constexpr double LIFT_DESCENT_RAMP_DEG_PER_S2 = 1200.0;// TUNE: how quickly the reference may change between bands (was 800)

// --- The FAST descent (auton::liftDownFast(), the front toggle's trip back down) ---------------
// "needs to go acautally fast and forcefully down to, but still slow when it approvhes our 0 and
// settle, just a bit faster down than normal for toggle." Same banded DESCEND -> FINAL -> SEAT ->
// REST, with ONLY the two fast bands raised; the approach cruise below LIFT_DESCENT_SLOW_ZONE_DEG,
// FINAL's forced push and SEAT/REST are byte-for-byte the normal ones, so it still slows near 0 and
// settles exactly the same way.
// From the toggle height (30 deg) BOTH fast bands run: the mid boundary is
// SLOW_ZONE + 0.5 * (30 - SLOW_ZONE) = 22, so TOP covers 30 -> 22 and MID covers 22 -> 14.
// 120% of free speed is not a speed the arm can reach on its own -- it is how you say "do not hold
// back": the velocity loop saturates at its full 12000 mV of downward authority, which is the
// "forcefully" the driver asked for.
inline constexpr double LIFT_FAST_DESCENT_TOP_PCT = 105.0;   // TUNE: ~157 deg/s (normal: 80 -> 120)
inline constexpr double LIFT_FAST_DESCENT_MID_PCT = 120.0;   // TUNE: ~180 deg/s (normal: 95 -> 142)
// AND A FIRMER RAMP, which is not optional at those speeds. Decelerating v1 -> v2 at rate a covers
// (v1^2 - v2^2) / 2a degrees. Entering the approach zone at MID's ~180 deg/s and needing the 45 deg/s
// cruise: at the NORMAL 1200 that takes (32400 - 2025) / 2400 = 12.7 deg -- more than the whole
// 14 -> 3 deg zone, so the arm would still be doing 70+ deg/s when FINAL took over, right at the
// floor. At 2000 it takes 7.6 deg, landing at about position 6.4 with ~3.4 deg of real cruise at
// 45 deg/s before FINAL -- the same comfortable margin the normal descent was sized for.
//   It thumps / arrives at the slow zone hot: LOWER LIFT_FAST_DESCENT_MID_PCT first (120 -> 110),
//     then raise this ramp.
//   It is not noticeably faster than a normal descent: raise LIFT_FAST_DESCENT_MID_PCT (and the ramp
//     with it, by the sum above).
inline constexpr double LIFT_FAST_DESCENT_RAMP_DEG_PER_S2 = 2000.0;  // TUNE (normal: 1200)
// THE QUEUED TOGGLE. The route asks for the way down only 300 ms after the way up, which is less than
// a 0 -> 30 deg rise takes, so a fast descent requested while a FRONT_TOGGLE rise is still climbing is
// QUEUED rather than obeyed: the arm keeps rising until it is within LIFT_FRONT_TOGGLE_REACH_DEG of
// the toggle height and then turns straight around -- it touches the height, it does not settle on it.
// Shown as `TOGGLE ^` on the brain's LIFT line. If the rise never gets there (jammed, BLOCKED), it
// descends anyway after LIFT_FRONT_TOGGLE_MAX_WAIT_MS rather than hanging on the way up.
inline constexpr double LIFT_FRONT_TOGGLE_REACH_DEG = 2.0;   // TUNE: close enough to call it touched
inline constexpr int LIFT_FRONT_TOGGLE_MAX_WAIT_MS = 1500;   // TUNE: then go down regardless

// --- Seating on the hard stop (phases FINAL, then SEAT) ---------------------
// The point of the final descent is that the arm does not STOP above the frame -- it keeps walking
// down until it physically cannot, and then stays pressed there so it cannot bounce back up.
//
// TWO STEPS, not one -- the original version used a single check ("stopped under 6 deg" == "seated")
// and that was a bug: a cup caught in the intake, or plain six-bar/chain friction, can stall the arm a
// few degrees above the true hard stop, read as "stopped", and get declared SEATed right there -- after
// which only LIFT_SEAT_MV (a HOLD voltage, meant to fight bounce, not resistance) was ever applied
// again, so the arm simply parked short of 0.
// Below LIFT_FINAL_START_DEG the arm hands over from the ordinary DESCEND bands into FINAL: a
// dedicated closed-loop push at a constant LIFT_FINAL_DEG_PER_S, with up to LIFT_FINAL_MAX_MV of
// downward authority -- more force than the SEAT hold, less than DESCEND's full 12000 mV so a real
// hard-stop contact doesn't slam -- specifically to force through that last bit of resistance.
// TIME-BASED, ON PURPOSE (2026-10-02 retune): FINAL now always runs for a FIXED LIFT_FINAL_FORCE_MS
// window -- it does not early-exit the moment it looks settled, it just forces at
// LIFT_FINAL_DEG_PER_S/LIFT_FINAL_MAX_MV for that whole window ("force its way down for a second or
// two," the user's words), because a cup/friction stall can look momentarily still without actually
// being seated. Once the window ends, it hands over to SEAT: plain `SEAT` if the arm is within
// LIFT_SEAT_ZERO_DEG of 0 at that moment, `SEAT(short)` otherwise (a genuine jam, not just friction --
// LIFT_SEAT_MV still holds it down either way so it cannot bounce). This replaced an earlier
// conditional version (seat early once "both close and still", else give up after LIFT_SEAT_MAX_MS) --
// LIFT_SEAT_MAX_MS, and the still-detection constants LIFT_SEAT_STILL_DEG_PER_S/LIFT_SEAT_STILL_MS it
// needed, are gone with it; a straight timer needs neither.
// After a true SEAT the lift holds LIFT_SEAT_MV forever (or until something physically disturbs it,
// see LIFT_SEAT_UNSEAT_DEG below).
// HEAT: -1500 mV is 12.5% of full volts -- a stalled V5 motor there still draws well under an amp (the
// winding is about 1.4 ohm), a bit over a watt per motor, and the arm is resting on the frame so the
// seat voltage is only fighting bounce, not holding the arm up. Safe for a whole match -- but watch the
// brain's TEMP line the first time, and LIFT_SEAT_TEMP_LIMIT_C drops the seat to a plain brake-hold if
// a lift motor ever does get that hot.
//   Still stops short of 0 / shows SEAT(short) every time: raise LIFT_FINAL_MAX_MV first (5000 -> 6500),
//     then LIFT_FINAL_FORCE_MS. Check the brain's ANGLE line reads HOMED first -- "0" is only true
//     against a homed zero; an un-homed seed can read short even when the arm is physically all the way
//     down.
//   Thumps into the frame during FINAL: lower LIFT_FINAL_DEG_PER_S.
//   Bounces back up off the stop once truly seated: raise the magnitude of LIFT_SEAT_MV further (-1800).
//   Motors warm up while parked at the bottom: lower LIFT_SEAT_MV's magnitude (e.g. -1000), or
//     LIFT_FINAL_FORCE_MS if the motors are warming during FINAL itself (unlikely at 1.5 s).
//   Declares true SEAT while still visibly short: lower LIFT_SEAT_ZERO_DEG.
inline constexpr float LIFT_SEAT_MV = -1500.0f;              // TUNE: constant downward seating voltage
// Where DESCEND hands over to FINAL. Also doubles as the "close enough to re-check" threshold for
// un-seating (see LIFT_SEAT_UNSEAT_DEG below) -- was a separate constant (LIFT_SEAT_DETECT_DEG, 6 deg)
// before the 2026-10-02 retune moved the whole handover down to 3 so the fast bands run longer.
inline constexpr double LIFT_FINAL_START_DEG = 3.0;          // TUNE: hand DESCEND over to FINAL this close to 0
inline constexpr double LIFT_FINAL_DEG_PER_S = 12.0;         // TUNE: constant creep speed for the last stretch
inline constexpr float LIFT_FINAL_MAX_MV = 5000.0f;          // TUNE: magnitude, downward authority during FINAL
// FINAL forces at the above speed/authority for exactly this long, then hands to SEAT regardless of
// where it ended up (see LIFT_SEAT_ZERO_DEG for how that is judged). ~1.2 deg of this 1.5 s is normally
// spent just closing the remaining 3 deg at LIFT_FINAL_DEG_PER_S; the rest is pure forcing time against
// resistance.
inline constexpr int LIFT_FINAL_FORCE_MS = 1500;             // TUNE: "a second or two" of forcing, per the user
inline constexpr double LIFT_SEAT_ZERO_DEG = 1.0;            // TUNE: within this of 0 when FINAL ends = true SEAT
// If something knocks a seated arm this far back up above LIFT_FINAL_START_DEG, the descent restarts
// (DESCEND, then FINAL again) and walks it back down rather than leaving it hanging.
inline constexpr double LIFT_SEAT_UNSEAT_DEG = 4.0;          // TUNE
inline constexpr double LIFT_SEAT_TEMP_LIMIT_C = 50.0;       // safety net, not a tuning knob

// --- REST: motors loose once the arm is settled at the bottom ---------------
// THE DRIVER'S REQUEST, 2026-10-03: "turn off lift pid on default state once it settles and is in that
// down state, keep motors loose so they arent being usefd and burn out." So SEAT is no longer the
// final state at the bottom. Once the arm has been SEATed and genuinely still for LIFT_REST_AFTER_MS,
// the lift enters REST: brake mode COAST and 0 V. No PID, no LIFT_SEAT_MV push, and not the firmware
// brake-hold either (which is a position loop and does draw current) -- the motors are simply free.
// The frame is holding the arm, not the motors. Brain: `REST`.
// REST still counts as seated/settled everywhere (isSeated(), isSettled(), waitLiftSeated(), Scoring's
// at-the-bottom tests), and ANY new command, the D-pad, or clamped-mode R1/R2 leaves it instantly and
// puts the brake mode back.
// COASTING IS ONLY EVER DONE AT THE BOTTOM. The power-on / no-target state is still a BRAKE-hold, so
// an arm that powers on part-way up cannot fall (CLAUDE.md safety rule 1 is unchanged), and letting go
// of the manual override still brake-holds where it is.
// THE ONE RISK, and the guard. SEAT exists partly because the arm "does not park above the frame and
// cannot bounce": the lift's rubber bands pull UP, hardest at the folded position, so with the motors
// loose the arm may creep off its stop. If the arm rises past LIFT_REST_WAKE_DEG while resting, the
// lift re-seats it (FINAL's forced push, then SEAT) and rests again -- at most LIFT_REST_MAX_WAKES
// times, after which it stays in SEAT and keeps the gentle -1500 mV rather than pulsing forever.
//   The arm creeps up off the stop, or the brain cycles REST -> FINAL -> SEAT -> REST: set
//     LIFT_REST_AFTER_MS to 0, which TURNS REST OFF ENTIRELY and keeps the old always-seated
//     behaviour. (Then watch the TEMP line instead: -1500 mV is about a watt per motor.)
//   The motors still feel stiff at the bottom: check the brain says REST, not SEAT -- SEAT means the
//     stillness test never passed, so raise LIFT_REST_STILL_DEG_PER_S.
inline constexpr int LIFT_REST_AFTER_MS = 300;               // TUNE: still-at-the-stop time before coasting; 0 = never
inline constexpr double LIFT_REST_STILL_DEG_PER_S = 3.0;     // TUNE: what counts as "not moving" for the above
inline constexpr double LIFT_REST_WAKE_DEG = 4.0;            // TUNE: rising past this while resting = re-seat it
inline constexpr int LIFT_REST_MAX_WAKES = 3;                // TUNE: then stay in SEAT instead of pulsing

// --- The L1 GRAB at the intake handoff (see "SCORING WITH L2"'s sibling, "THE GRAB", in lift.hpp) --
// L1 pressed while the macro is parked at the INTAKE handoff no longer just closes the jaws and
// rises: the arm first FORCES ITSELF DOWN onto the pin for a moment, closes the jaws while it is
// pressed down there, and only then rises. The driver's words: "I want the claw to force its way
// down, stop spinning the intake when its doing that and grabbing (and force it down, like -40 speed
// or so) while grabbing, should be like a 300ms movement going down for 200ms then grab, then go up."
// The intake STOPS for the whole push (one decision per tick, in periodic::tick() -- never a second
// writer), so the rollers cannot drag the pin out from under the jaws while they close.
// SPEED. Open-loop, a plain percentage of LIFT_MAX_MV like every other *_PCT here. 40% is 4800 mV.
// (The driver said "-40 speed", which in their old library meant Motor::move(-40) on the -127..127
// scale, i.e. about 31%. 40% of volts is the same number in this codebase's units and a little
// firmer; drop it to 31 to match the old library exactly.)
//   The pin gets pushed out of the jaws / the intake is lifted off the floor: LOWER it.
//   The jaws close above the pin instead of around it: RAISE it, or raise L1_GRAB_CLOSE_AT_MS.
inline constexpr double L1_GRAB_PUSH_PCT = 40.0;             // TUNE: % of LIFT_MAX_MV, downward
// When the jaws close, measured from the start of the push.
inline constexpr int L1_GRAB_CLOSE_AT_MS = 200;              // TUNE
// How long the push lasts. The arm keeps pressing down until BOTH this has elapsed AND the jaws have
// had CLAW_CLOSE_TIME_MS to shut from the moment they were commanded -- it must never pull up out of
// a claw that is still closing (the same interlock CLAMPING enforces). With the numbers here that is
// max(300, 200 + 150) = 350 ms of push.
inline constexpr int L1_GRAB_PUSH_MS = 300;                  // TUNE
// Safety net: if nobody ever acknowledges the close request (no Scoring::update() running at all),
// stop pushing after this long anyway rather than leaning on the pin for the rest of the match.
inline constexpr int L1_GRAB_MAX_MS = 1000;                  // TUNE

// --- Clamped-mode R1 / R2 lift (see src/main.cpp and README "Driver guide") --
// While the claw is CLAMPED on a pin, R1/R2 stop being intake/extake and become lift up/down.
// Up is plain voltage; down goes through the banded descent above (so it seats at the bottom exactly
// like L2 does) with this as its top-band speed.
inline constexpr double R_LIFT_UP_PCT = 90.0;                // TUNE: R1 up, % of full voltage
inline constexpr double R_LIFT_DOWN_PCT = 65.0;              // TUNE: R2 down, % of free speed (~98 deg/s)

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
// Hands over to HOLD once the arm is inside its band AND slower than this. It has to sit ABOVE
// LIFT_APPROACH_MIN_DEG_PER_S or the crawl's floor speed would mean arrival never latches and every
// move ended on the LIFT_APPROACH_TIMEOUT_MS instead. With kP = 6 /s, the reference is down to 12
// deg/s about 2 deg out, so that is where a move actually latches -- comfortably inside the 5 deg
// tolerance, and the hold's LIFT_HOLD_KD (90) then absorbs 12 deg/s as ~1080 mV of opposing voltage.
//   Any bounce as it grabs the hold: raise LIFT_HOLD_KD first; lowering this to 8 is the second lever
//   (gentler handover, slightly slower finish).
inline constexpr double LIFT_STOPPED_DEG_PER_S = 12.0;       // TUNE
// The final approach can't take longer than this (arm fighting something, loop badly tuned): the lift
// hands over to HOLD, whose position term then pulls in whatever is left, gently.
inline constexpr int LIFT_APPROACH_TIMEOUT_MS = 1500;        // TUNE
// Velocity estimate: position differenced every tick, then this much of each new sample is blended in
// (1.0 = raw, no filtering). The rotation sensor's quantisation is 0.0025 arm deg, i.e. 0.25 deg/s per
// count, so there is nothing to filter out except task jitter -- and filtering costs LAG, which makes
// the brake react late and overshoot. 0.8 is a quarter of a tick of lag; 0.5 (the old value) was a
// whole tick, which at 150 deg/s is 1.5 deg of stale position.
// (Checked while chasing the "slow start/middle": at 0.8 the EMA's effective lag is (1-a)/a * 10 ms =
// 2.5 ms, i.e. 0.4 deg at full speed. It is NOT what delays DRIVE ending or BRAKE responding -- that
// was the decel value and the 9000 mV velocity clamp above. Leave it at 0.8.)
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
//   HOLD      the damped position hold (KP/KI/KD above), with no P correction and a FROZEN integral
//             inside the settle band. Its integral carries the learned gravity voltage over to the
//             next move, so no move ever starts by guessing it.
//   PARK      settled: that learned voltage, frozen, pushed as a constant with no feedback at all, so
//             it cannot hunt. Entered after LIFT_PARK_CONFIRM_MS inside the band or
//             LIFT_SETTLE_TIMEOUT_MS of holding; a move that never arrives parks as PARK(t/o) after
//             LIFT_MOVE_TIMEOUT_MS. Left only on a LIFT_REENGAGE_DEG drift or a new command.
//
// A DESCENT (a non-precise goal at or below LIFT_BOTTOM_GOAL_DEG: L2, B, auton DOWN, R2 in clamped
// mode) is DESCEND -> FINAL -> SEAT: the three speed bands above, then (below LIFT_FINAL_START_DEG) a
// dedicated closed-loop push at LIFT_FINAL_DEG_PER_S with up to LIFT_FINAL_MAX_MV of authority -- more
// force than the seat hold, enough to push through a cup or friction a few degrees short of 0 -- for a
// fixed LIFT_FINAL_FORCE_MS, and only then a constant LIFT_SEAT_MV pressed into the hard stop. It does
// not stop above the frame and it does not bounce; see "Seating on the hard stop" below for why the
// original single-step version could park short. Finally, once that seated arm has been still for
// LIFT_REST_AFTER_MS, REST coasts the motors at 0 V so they draw nothing (see "REST" below) -- the
// only place in the program that ever coasts.
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
// not a jam), nor in HOLD, nor below LIFT_FINAL_START_DEG during a descent -- an arm that has stopped
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
// A=APPROACH (the final crawl), H=HOLD, V=DESCEND (a banded trip to the bottom), R=RETURN (L2's fast
// run home, same bands as V), Z=FINAL (the forced push-to-0 step below LIFT_FINAL_START_DEG, always
// LIFT_FINAL_FORCE_MS long), S=SEAT (pressed onto the hard stop, or holding short -- see
// LIFT_SEAT_ZERO_DEG), M=manual (D-pad, or R1 in clamped
// mode), h=homing, F=fault, I=idle. Integers only (no float formatting on the
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
// ===========================================================================
// AUTOCLAMP — the distance sensor in the jaws, and when it is allowed to clamp
// ===========================================================================
// The sensor looks out between the jaws and sees both pins and cups. The rule (the driver's own
// words): while the claw is open and the autoclamp is enabled, anything at or inside
// AUTOCLAMP_RANGE_MM for AUTOCLAMP_CONFIRM_MS straight closes the jaws on it. The debounce is the
// whole point -- "if there's a random spike and then not something 50/100 ms later and it spikes back
// down, it's not real" -- and it is implemented by subsystems::AutoclampDetector, which is PURE (no
// sensor, no clock, no cylinder: see its header for the full rule and for why fresh samples are
// counted separately from elapsed time). Every threshold it uses is here.
inline constexpr int8_t CLAW_DISTANCE_PORT = 11;             // confirmed 2026-10-02: mounted in the claw
// ON BY DEFAULT IN DRIVER (opcontrol() enables it every time it starts), OFF in auton (the routine
// turns it on with auton::autoclamp(true)), and OFF at power-on -- CLAW_AUTOCLAMP_AT_START stays
// false so a preloaded pin cannot fire a cylinder in initialize() (CLAUDE.md safety rule 1). Y still
// toggles it in a match.
inline constexpr bool AUTOCLAMP_ON_IN_DRIVER = true;
inline constexpr bool CLAW_AUTOCLAMP_AT_START = false;       // power-on / initialize(): never armed
// THE TRIGGER. 15 mm is the user's starting point. NOTE: the V5 Distance Sensor's specified MINIMUM
// range is about 20 mm, so 15 sits below the distance it is specified to measure reliably -- it
// usually still reads sensibly closer in, but this is the first number to question if the clamp never
// fires. Watch the brain's DIST line with a pin held in the open jaws:
//   it reads 20-25 and never reaches 15      -> raise AUTOCLAMP_RANGE_MM to just above what you see.
//   it reads 0 with the pin in the jaws      -> the pin is INSIDE the sensor's floor: raise the range
//                                               and/or move the sensor back a few mm. (A 0 is treated
//                                               as "cannot measure", never as "very close" -- see
//                                               AUTOCLAMP_MIN_VALID_MM.)
//   it clamps on a cup/pin going past        -> raise AUTOCLAMP_CONFIRM_MS (50 -> 80) first.
inline constexpr int AUTOCLAMP_RANGE_MM = 15;                // TUNE: at or inside this = something in the jaws
inline constexpr int AUTOCLAMP_CONFIRM_MS = 50;              // TUNE: ...continuously for this long
// ...over at least this many FRESH readings. The sensor refreshes on the order of 30 ms, i.e. slower
// than the 10 ms tick, so the same measurement comes back two or three times in a row; without this,
// ONE lucky in-range measurement repeated for 50 ms would satisfy the time window on its own, which is
// exactly the spike this is meant to reject. 2 fresh samples at ~30 ms apart is ~60 ms of real
// evidence.
//   Clamps feel laggy: lower AUTOCLAMP_CONFIRM_MS first, this second (never below 2).
inline constexpr int AUTOCLAMP_MIN_SAMPLES = 2;              // TUNE
// How long an UNCHANGED reading has to persist before it is credited as another sample. About two
// sensor refreshes: long enough that the sensor must really have re-measured, short enough that a pin
// sitting dead still at exactly one millimetre value still gets its second sample and gets gripped.
//   Clamps on a steady reading feel a touch slow: lower it (but not below ~35 ms, one refresh).
inline constexpr int AUTOCLAMP_STALE_GRACE_MS = 60;          // TUNE
// RE-ARM. The instant the jaws open, the pin they just let go of is still millimetres away, so a
// timer-based re-arm would re-grab it every single time and no score could ever finish. After ANY
// open, the autoclamp waits until the sensor reads CLEAR -- further than AUTOCLAMP_REARM_MM, or
// nothing in range at all -- for AUTOCLAMP_REARM_CLEAR_MS. The gap between 15 and 30 is hysteresis:
// the two thresholds cannot chatter at one distance.
//   It re-grabs what it just put down: raise AUTOCLAMP_REARM_MM / AUTOCLAMP_REARM_CLEAR_MS.
//   It takes too long to be ready for the next pin: lower them.
inline constexpr int AUTOCLAMP_REARM_MM = 30;                // TUNE: further than this = clear
inline constexpr int AUTOCLAMP_REARM_CLEAR_MS = 150;         // TUNE
// VALIDITY of a raw reading. At or above MAX is the sensor's own "nothing in range" (9999) and counts
// as CLEAR. Below MIN (a 0: the sensor cannot measure, e.g. an object inside its floor) and any
// negative value (PROS_ERR / unplugged) count as NOT DETECTED and are not clear either -- so a dead or
// unplugged sensor makes the autoclamp do nothing at all, which is the correct failure.
inline constexpr int AUTOCLAMP_MAX_VALID_MM = 9999;
inline constexpr int AUTOCLAMP_MIN_VALID_MM = 1;
// The sensor's own 0..63 confidence. 0 = ignore it (the default: confidence drops at close range on
// this sensor, and requiring it there is a good way to never clamp). Raise it only if the DIST line
// shows plausible mm values that turn out to be junk.
inline constexpr int AUTOCLAMP_MIN_CONFIDENCE = 0;           // TUNE
// Minimum time after a release before the autoclamp may arm again, on top of the clear-reading rule
// above (belt and braces: it also covers the jaws physically swinging open past the sensor).
inline constexpr int CLAW_RELEASE_LOCKOUT_MS = 750;          // TUNE

// ---------------------------------------------------------------------------
// Field-facing distance sensors (brain screen, and moveB's feedback)
// ---------------------------------------------------------------------------
// Three of them, all confirmed on the robot 2026-10-03: one each side and one looking backward. The
// REAR one is the entire feedback loop for auton::moveB() (its own constants are in the moveB block
// further down). The left and right ones are read by the brain's DIST line only -- nothing in the
// control code uses them yet; they are there to be aimed at a wall or a stack while tuning a route.
// The fourth distance sensor on this robot is the one inside the claw (CLAW_DISTANCE_PORT, 11) and it
// belongs to the autoclamp, not here.
inline constexpr int8_t LEFT_DISTANCE_PORT = 15;             // confirmed 2026-10-03
inline constexpr int8_t RIGHT_DISTANCE_PORT = 19;            // confirmed 2026-10-03

// --- START-OF-AUTON ALIGNMENT (auton::startPose(), see README "Start alignment") ---
// The robot is set against the front wall every time, so the distance from the front and the heading
// are repeatable -- but how far along that wall it ends up is not. The LEFT sensor measures that:
// reading FURTHER than expected means the robot is shifted +x (toward the robot's right), CLOSER
// means -x, and because the heading is fixed the difference in the reading IS the lateral shift --
// no sensor-offset geometry is involved. Each route carries its own expected reading, next to the
// route in src/autons.cpp (it is a field position, not a robot constant). These are the shared rules.
//
// VALIDITY, mirroring the team's old accumulateReading(): a sample counts only if it is under
// START_ALIGN_MAX_MM and at least START_ALIGN_MIN_CONFIDENCE. 0, 9999 and PROS_ERR are rejected by
// the same test. If NOTHING valid comes back, the correction is skipped entirely.
inline constexpr int START_ALIGN_MAX_MM = 2000;              // TUNE: further than this is not our wall
inline constexpr int START_ALIGN_MIN_CONFIDENCE = 38;        // TUNE: 60% of the sensor's 0..63 scale
// SANITY GUARD. A shift bigger than this is not a hand-placement error, it is the sensor seeing a
// goal, a robot, or the wrong wall -- so the correction is dropped and the nominal pose is used. It
// must never be able to teleport the pose.
//   Legitimate setups keep getting skipped: raise it, but only after checking the DIST line.
inline constexpr double START_ALIGN_MAX_IN = 4.0;            // TUNE: inches
// IN-AUTON sampling window, used only when there is no usable disabled-mode history (e.g. autonomous
// launched straight from the brain menu). This time is spent at the start of the route.
inline constexpr int START_ALIGN_SAMPLE_MS = 300;            // TUNE
// DISABLED-MODE history. While the robot sits disabled before the match, the left sensor is sampled
// continuously and the last START_ALIGN_HISTORY_MS are kept. startPose() uses that average -- zero
// auton time spent, and a far longer average than a route could afford -- but only if the window is
// SETTLED: max minus min under START_ALIGN_HISTORY_SPREAD_MM, so a hand still placing the robot (or a
// person walking past the beam) does not count as a measurement.
inline constexpr int START_ALIGN_HISTORY_MS = 500;           // TUNE
inline constexpr int START_ALIGN_HISTORY_SPREAD_MM = 40;     // TUNE: a settled window is this tight
inline constexpr int START_ALIGN_HISTORY_STALE_MS = 2000;    // older than this = not this match's setup
// ...plus a short fresh sample at the start of auton to confirm the robot has not been moved since.
// If the confirm disagrees with the history by more than START_ALIGN_CONFIRM_TOL_MM, the CONFIRM wins
// (it is the newer truth) and that is what gets used.
inline constexpr int START_ALIGN_CONFIRM_MS = 80;            // TUNE
inline constexpr int START_ALIGN_CONFIRM_TOL_MM = 150;       // TUNE
// chassis.calibrate() RESETS THE POSE, so a calibration that finishes after startPose() would wipe
// the correction. startPose() waits this long for calibration before applying anything, and if it is
// still not done it applies the pose and re-applies it once calibration really finishes.
inline constexpr int START_ALIGN_CALIBRATE_WAIT_MS = 2500;   // TUNE
inline constexpr int START_ALIGN_REAPPLY_WINDOW_MS = 4000;   // how long that one-shot re-apply waits

// ---------------------------------------------------------------------------
// Intake (ONE motor)
// ---------------------------------------------------------------------------
// Negative = reversed. If R1 spits cups out instead of pulling them in, flip the sign.
inline constexpr int8_t INTAKE_PORT = 12;
inline constexpr float INTAKE_MV = 12000.0f;

// AUTO-INTAKE ON L1. An L1 press that starts a rise FROM THE FLOOR -- the arm at or below
// LIFT_BOTTOM_GOAL_DEG, seated, or sitting at the INTAKE handoff -- also runs the rollers IN, from
// the press itself (so they are already going during the CLAW_CLOSE_TIME_MS clamp wait) until the arm
// climbs past this height. The point is to sweep up whatever is at floor level while the arm is on
// its way up anyway, without the driver having to hold R1 at the same time as tapping L1.
//   Cups get left behind / it stops sweeping too early: raise it.
//   The intake is still running when the arm is clear of the floor: lower it.
inline constexpr double L1_AUTO_INTAKE_UNTIL_DEG = 20.0;     // TUNE: arm degrees
// Safety net: the rollers stop after this long no matter what, so a rise that never gets to
// L1_AUTO_INTAKE_UNTIL_DEG (jam, BLOCKED lift, a preset below it) can't leave them spinning.
inline constexpr int L1_AUTO_INTAKE_MAX_MS = 1500;           // TUNE

// AUTO-OUTTAKE ON L2 SCORE. While an L2 score descent is running (Scoring::isScoreDescent(): RELEASING
// with scoreToHome_ set -- the sensing scorer, not L2's at-the-bottom clamp toggle) and the arm is
// below this height, run the rollers OUT at L2_OUTTAKE_PCT so anything still in the intake ejects
// instead of riding the lift down. Covers the whole trip through PRESS / REL / UNSTK / RETURN,
// whichever phase the arm happens to be under this height in.
//   NOTE: stack level 1 (31.0 deg) is already below this, so scoring at level 1 outtakes from the
//   start of the press, not partway down. That is this constant as specified, not a bug.
//   Pin/cup gets knocked loose too early: lower it. Rollers start too late to clear the handoff
//   before the arm seats: raise it.
inline constexpr double L2_OUTTAKE_BELOW_DEG = 40.0;         // TUNE: arm degrees
// % of full intake power (config::INTAKE_MV) the rollers run OUT at during the above. Lower than
// R2's full-power outtake on purpose -- this runs unattended during a score, not under driver thumb.
inline constexpr double L2_OUTTAKE_PCT = 60.0;               // TUNE: % of INTAKE_MV
// Stop condition (2026-10-02 retune): as soon as the lift reaches LIFT_FINAL_START_DEG -- tied
// directly to it, not a separate guess, because that is also where the lift's own FINAL phase takes
// over forcing the arm the rest of the way down at real authority (see "Seating on the hard stop" in
// the LIFT TUNING block above). The roller does not need to keep running while FINAL does that forcing;
// if a cup really was the obstruction it has already been cleared by then. main.cpp uses this constant
// directly (see L2_OUTTAKE_STOP_DEG below) rather than position-matching LIFT_FINAL_START_DEG by eye.
inline constexpr double L2_OUTTAKE_STOP_DEG = LIFT_FINAL_START_DEG;
// Safety net: the rollers stop after this long no matter what, so a score that stalls above
// L2_OUTTAKE_STOP_DEG (BLOCKED, FAULT, stuck claw) can't leave them spinning for the rest of the match.
inline constexpr int L2_OUTTAKE_MAX_MS = 3000;               // TUNE

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
// How long the jaws take to actually open. Two users: scoreOrMatchload() waits it out before the arm
// moves away, and L2's scorer starts its "did the claw come free" check this long after it asked for
// the release (see L2_FREE_MS). Too short and L2 judges freedom while the jaws are still closing.
inline constexpr int CLAW_OPEN_TIME_MS = 200;                // TUNE: never measured on the real claw
// ---------------------------------------------------------------------------
// L2 — the sensing scorer. See "Scoring with L2" in lift.hpp and README.
// ---------------------------------------------------------------------------
// L2 above the bottom with a pin held no longer opens the jaws and lowers. It presses the pin DOWN
// onto the stack with the claw still SHUT, watches the rotation sensor for the moment the pin lands
// (the arm stops descending as fast as it is being told to), releases the claw at that instant, keeps
// pushing down to slide the jaws off the pin, checks the arm really is free, and only then runs home
// fast. Every phase of that lives in Lift::update() (phases PRESS / REL / UNSTK, then the ordinary
// DESCEND -> SEAT for the run home); Scoring still owns the claw and opens it when the lift asks.
//
// PRESS-DOWN SPEED, % of the free speed. The user's starting point. It is closed-loop on the measured
// arm speed, which is what makes contact detectable at all: an open-loop descent would just slow down
// against the stack and nothing would know.
//   Slams the pin into the stack / knocks the tower: LOWER it (45, 35).
//   Scoring feels slow: raise it, but contact detection gets less sensitive as the gap between
//   "moving" and "stopped" shrinks relative to the noise.
inline constexpr double L2_SCORE_SPEED_PCT = 60.0;           // TUNE: ~90 deg/s
// Most the loop may push DOWNWARD (magnitude, mV) while pressing. This is the crush limit: the
// velocity loop has 12000 mV of authority and an integral that winds up, so without a cap it would
// keep increasing the push until it moved -- i.e. flatten the stack. 7000 mV through the 4:1 is about
// 56% of stall torque: firm enough to seat a pin, not enough to crumple a tower.
// RAISED 5000 -> 7000 (2026-10-02): holding a pin+cup, this six-bar can need real authority just to
// keep descending at the commanded speed over the part of the arc where gravity resists most -- the
// extra weight adds directly to that resistance. At 5000 mV a loaded arm could saturate the velocity
// loop and settle into a steady creep under STALL's absolute floor (config::L2_CONTACT_STALL_DEG_PER_S)
// with NOTHING under the arm, which is most of why a loaded claw auto-released almost immediately every
// time (see "ARM ON MOTION, AND SPLIT STALL" in contact_detector.hpp for the other half of that fix).
// 7000 is still well short of the full 12000 the loop could use outside a score, so a real hard-stack
// contact still can't get the crush authority of an ordinary move.
//   The stack gets pushed over / the cup crumples: LOWER it (5500).
//   The pin does not seat fully before the release: raise it (8000).
//   A loaded arm still creeps under L2_CONTACT_STALL_DEG_PER_S with nothing under it: raise this
//   further, or raise L2_CONTACT_NOSTART_MS so it has longer to prove it is still accelerating.
inline constexpr float L2_PRESS_MAX_MV = 7000.0f;            // TUNE: magnitude, downward (was 5000)
// CONTACT DETECTION. Implemented by subsystems::ContactDetector (include/subsystems/contact_detector.hpp),
// which is a PURE class with host-side tests in test/ -- run test/run.sh on the Mac after changing any
// of these, it proves the thresholds against synthetic landings before anything is uploaded.
//
// WHY THE FIRST VERSION NEVER FIRED, and what changed. It compared the measured speed against the
// NOMINAL commanded speed (~90 deg/s): a relative trigger armed only once the arm reached 85% of it,
// and an absolute trigger at 8 deg/s. Both are dead if the arm simply cannot make 90 deg/s -- and on
// this six-bar it often cannot, because the arm swings up and BACK, so over much of the arc gravity
// RESISTS the descent and the loop can only push down with L2_PRESS_MAX_MV of authority. A free
// descent settling at 50 deg/s never armed the relative trigger (50 < 76.5) and never tripped the
// absolute one (50 > 8); the landing then slowed it to 15, still over 8. Nothing could ever fire.
// Every signal below is now measured against WHAT THE ARM WAS ACTUALLY DOING a moment ago -- a learned
// baseline -- not against what it was told to do. Three signals:
//   LAG     a leaky one-sided integral of the speed shortfall against the baseline, in arm degrees:
//           "how far behind where it would have been the arm now is". The POSITION-vs-EXPECTED signal,
//           and the sensitive one -- it catches a landing that only slows the arm by a quarter.
//   EFFORT  the loop pushing DOWN harder than the free descent needed. The MOTOR-WORK signal: it
//           catches a pin being forced into the tower even while it still creeps.
//   SPEED   measured below a fraction of the baseline. The absolute near-stop (STALL) is a separate,
//           unconditional backstop that needs no baseline at all, so it still catches a pin already
//           resting on the stack by itself.
// AGREEMENT, added 2026-10-02: a lone LAG/EFFORT/SPEED signal is no longer enough -- at least TWO of
// the three must be over threshold on the SAME tick (STALL is exempt, see contact_detector.hpp for
// why). This is the fix for the robot false-firing in plain free descent: this six-bar's gravity torque
// swings up and back past vertical, which can move a single signal fast enough to look like a landing
// with nothing under the arm at all. On top of AGREEMENT, PRESS also has a GUARANTEED release height
// (L2_AUTO_RELEASE_DEG, below) -- so a real but gentle landing the detector is slow to confirm is no
// longer a lost score, just a later one.
// Which one fired is on the brain (REL(lag) / REL(eff) / REL(spd) / REL(stall), or REL(32) if nothing
// was ever detected and L2_AUTO_RELEASE_DEG forced it) and in the trace's `trg` column, so the next
// trace says exactly which signal moved -- or that none did.
//
//   Releases early, with the pin still in the air: raise L2_CONTACT_MS (30 -> 50), lower
//     L2_CONTACT_SPEED_FRAC (0.55 -> 0.45), raise L2_CONTACT_LAG_DEG (2.2 -> 3.0). Read the trace's
//     `trg` column first -- fix the signal that actually fired, not all three. Remember AGREEMENT (see
//     contact_detector.hpp) means this is now rare: a lone signal can no longer fire by itself.
//   Presses on after the pin has landed / never releases until L2_AUTO_RELEASE_DEG forces it anyway:
//     lower L2_CONTACT_LAG_DEG and L2_CONTACT_EFFORT_MV together (raising just one won't help once
//     agreement needs two), and lower L2_CONTACT_MS. Check the trace: if only ONE of `lag`/`eff`/`v-b`
//     ever crosses its threshold, that is AGREEMENT doing its job on a real landing too, which means
//     the thresholds are too tight for this stack's actual landing signature -- loosen the one that
//     stays furthest from firing, not all three.
//   False-fires in plain free descent with nothing under the arm: this is the failure mode AGREEMENT
//     and the 2026-10-02 retune exist to fix (six-bar gravity torque swings up and back past vertical,
//     moving speed/effort faster than the 200 ms baselines adapt). If it still false-fires, raise
//     L2_CONTACT_MS further, or lower L2_CONTACT_FREEZE_FRAC (0.82 -> 0.75) so the baseline keeps
//     tracking a genuine swing longer before it locks and starts measuring a shortfall against it.
//     AGREEMENT itself (contact_detector.hpp) is doing most of the real work here, not these three
//     thresholds alone -- they were only moved moderately (not all the way to the first-draft numbers
//     this comment used to suggest) because the host test's own creep-landing case stopped firing
//     altogether past a certain point: AGREEMENT plus thresholds that are TOO strict together can
//     reject a genuinely gentle landing as readily as a false one. L2_AUTO_RELEASE_DEG is the backstop
//     for whatever this still misses.
inline constexpr double L2_CONTACT_LAG_DEG = 2.2;            // TUNE: arm degrees of accumulated shortfall (was 1.5)
inline constexpr double L2_CONTACT_LAG_LEAK_MS = 200.0;      // TUNE: makes LAG measure a change, not a drift
inline constexpr double L2_CONTACT_EFFORT_MV = 1800.0;       // TUNE: extra downward push over the baseline (was 2000 --
                                                              // LOWERED, not raised: see the AGREEMENT note above)
inline constexpr double L2_CONTACT_SPEED_FRAC = 0.55;        // TUNE: measured / baseline that counts as contact (was 0.70)
inline constexpr double L2_CONTACT_STALL_DEG_PER_S = 6.0;    // TUNE: absolute near-stop, needs no baseline, no AGREEMENT
inline constexpr int L2_CONTACT_MS = 30;                     // TUNE: confirm window (was 20 -- see AGREEMENT below)
// ARM ON MOTION, added 2026-10-02 (see contact_detector.hpp): arming now also needs this much
// cumulative travel since the press started, not just a lucky instantaneous sample over
// L2_CONTACT_STALL_DEG_PER_S once L2_CONTACT_SETTLE_MS has elapsed. This is what stops a loaded arm's
// slow, still-accelerating start from snapshotting a baseline too early.
//   Baseline looks wrong / fires early on a loaded press: raise this (3.5 -> 5.0).
//   Takes too long to arm on a normal unloaded press: lower it, but not below a degree or two -- it
//   needs to be more than sensor noise could ever integrate into over L2_CONTACT_SETTLE_MS.
inline constexpr double L2_CONTACT_ARM_TRAVEL_DEG = 3.5;     // TUNE: arm degrees, cumulative since PRESS started
// How long the press gets to reach a steady descent speed before the baseline is snapshotted and the
// baseline-relative signals come alive. The absolute near-stop is live before this (after the dead
// time below), so a pin that is already resting is still caught immediately.
inline constexpr int L2_CONTACT_SETTLE_MS = 200;             // TUNE
// SPLIT STALL's "never really started" window, added 2026-10-02: a pin+cup's weight can keep a
// perfectly free arm under L2_CONTACT_STALL_DEG_PER_S for longer than the ordinary ignoreMs+confirmMs
// (L2_CONTACT_IGNORE_MS + L2_CONTACT_MS) while it fights gravity on the way to actually arming -- that
// was most of why a loaded claw auto-released almost immediately. Once the arm HAS armed_ at any point
// this press, a later stop is unambiguous and still confirms at the fast ignoreMs+confirmMs; only a
// STALL that has never seen the arm arm_ waits this long. See contact_detector.hpp's "ARM ON MOTION,
// AND SPLIT STALL".
//   A genuinely empty floor still auto-releases almost instantly: that is now L2_CONTACT_MS/IGNORE_MS
//     (fast path) if it never looked like it might be loaded, which is correct -- this window only
//     matters once the arm starts out moving slowly.
//   A loaded claw over bare floor still reads as resting before L2_AUTO_RELEASE_DEG: raise this
//     further (350 -> 500), and/or raise L2_PRESS_MAX_MV so the loaded arm can actually clear
//     L2_CONTACT_STALL_DEG_PER_S sooner.
//   A real resting pin now takes noticeably longer to confirm: that trade is intentional -- see the
//     class comment in contact_detector.hpp. L2_AUTO_RELEASE_DEG's travel rule is the backstop either way.
inline constexpr int L2_CONTACT_NOSTART_MS = 350;            // TUNE: ms, STALL's window while never armed_
// Baseline EMA time constants. The baselines must track the slow, genuine sag in a free descent's
// speed and cost as the gravity torque changes with angle -- otherwise LAG drifts into a false
// trigger -- but they FREEZE as soon as the arm is below L2_CONTACT_FREEZE_FRAC of the baseline, so
// they can never chase a developing landing and mask it. That two-level gate is the whole trick; the
// host test has a scenario for each half of it.
inline constexpr double L2_CONTACT_BASELINE_TAU_MS = 200.0;  // TUNE
inline constexpr double L2_CONTACT_OUTPUT_TAU_MS = 50.0;     // TUNE: low-pass on the mV, for EFFORT
// Lowered 0.90 -> 0.82 (2026-10-02): freeze LESS aggressively, so the baseline keeps tracking a real
// but transient gravity-torque swing a little longer before locking onto a stale value and reading the
// swing's continuation as an ever-growing shortfall. See AGREEMENT in contact_detector.hpp -- this and
// the threshold changes above are parts of the same false-fire fix, and AGREEMENT is the one doing
// most of the work; this just gives the baseline a little more rope first.
inline constexpr double L2_CONTACT_FREEZE_FRAC = 0.82;       // TUNE: loose gate -- stops LEARNING, never fires (was 0.90)
// Dead time at the very start of the press before any trigger may fire. Longer than before (60 -> 80)
// now that AGREEMENT does most of the false-fire rejection -- this just gives the baseline's own
// settle time (above) a little more cushion against a rough handover from HOLD.
inline constexpr int L2_CONTACT_IGNORE_MS = 80;              // TUNE (was 60)
// After the release, the arm has to be clearly moving again (faster than L2_CONTACT_STALL_DEG_PER_S)
// for this long to count as "the claw came off the pin". Then it goes home. The RELEASE phase uses
// this plain speed test rather than the full detector: with the jaws open the arm either moves or it
// does not, and there is nothing subtle to catch.
inline constexpr int L2_FREE_MS = 100;                       // TUNE
// ...and conversely, this long below that speed with the jaws open means the claw is hooked on the pin.
inline constexpr int L2_STUCK_MS = 250;                      // TUNE
// If the release is never acknowledged (Scoring not running, e.g. an auton that never calls
// Scoring::update()), carry on anyway after this long rather than pressing down forever. Worst case
// the arm goes home with the pin still clamped either way.
inline constexpr int L2_RELEASE_ACK_MS = 400;
// STUCK RECOVERY. Still stalled after the jaws opened = the claw is hooked on the pin. Lift UP this
// far (claw open) and try pressing down again, up to L2_MAX_UNSTICK_TRIES times. After that it stops,
// holds where it is and shows SCORE STUCK -- it never keeps grinding.
inline constexpr double L2_UNSTICK_UP_DEG = 20.0;            // TUNE: arm degrees
inline constexpr double L2_UNSTICK_SPEED_PCT = 40.0;         // TUNE: ~60 deg/s, it is only a short hop
inline constexpr int L2_UNSTICK_TIMEOUT_MS = 1200;           // safety net: don't grind upward either
inline constexpr int L2_MAX_UNSTICK_TRIES = 2;               // TUNE
// GUARANTEED RELEASE, BY RELATIVE TRAVEL (2026-10-02, replaces L2_NO_CONTACT_DEG and the old
// "NO CONTACT: keep the pin, seat with it still clamped" behaviour; corrected 2026-10-02 from an
// earlier, WRONG absolute-height version -- the user's own words: "i dnt mean the actual 32 degree
// reading, but 32 degrees relative down from the starting position it was at when we first clicked
// L2"). The detector is a real sensor reading a real stack, but it is not the only thing allowed to
// decide the pin has to come off any more: once PRESS has travelled this many degrees DOWN FROM
// Lift::scoreStartPos_ -- the position captured the instant startScore() ran, fixed for the whole
// sequence -- without a detection, it releases anyway, exactly as if the detector had fired (claw
// opens, keep pushing so the jaws slide off, same freed/UNSTK handling, same fast run home). Shown as
// REL(32) on the brain/trace. If the detector fires first, that still wins and releases immediately --
// this is strictly a floor under it, never a delay.
// RELATIVE, NOT ABSOLUTE, because the whole point is "give the detector this much of THIS press to
// prove itself", the same travel budget whether scoring starts at level 1 (31.0) or level 5 (106.9).
// An absolute reading has no such thing to offer a press that starts below 32 in the first place --
// see L2_RELEASE_FLOOR_DEG below for that case.
//   Pin dropped from a bit too high / too early on a real stack: raise it only a little -- raising it a
//     lot just turns more of the press into a guaranteed-release, no-sensing region.
//   Height rule fires before a real landing that the detector should have caught: that is a tuning
//     problem in the CONTACT_* block above, not this constant -- the detector is supposed to win first.
inline constexpr double L2_AUTO_RELEASE_DEG = 32.0;          // TUNE: arm degrees, RELATIVE travel from the L2 start
// TWO JOBS, both "we're close enough to the bottom, stop being delicate about it", at the same height
// on purpose:
//   1. PRESS fallback for a start too low for L2_AUTO_RELEASE_DEG's travel to ever complete before the
//      floor (level 1, 31.0, can only travel ~21-31 deg before 0 depending on exactly where it seats --
//      never the full 32). If nothing is detected and the travel rule can't apply, release here instead,
//      shown as REL(floor) -- so level 1 still gets roughly 21 deg of real sensing before it gives up,
//      rather than the old (wrong) behaviour of releasing on the very first PRESS tick.
//   2. RELEASE-phase safety net (the original role): if the jaws still haven't resolved free-or-stuck
//      by the time the arm has pressed down to this height -- a slow claw-open, a borderline UNSTICK
//      retry -- stop treating this as a press and hand over to the fast run-home descent right there,
//      instead of grinding further down with L2_PRESS_MAX_MV's authority. This check is gated on
//      clawReleased_ (the claw-open request has been acknowledged, or L2_RELEASE_ACK_MS has elapsed) so
//      a PRESS-fallback release landing exactly ON this floor can never race Scoring out of ever seeing
//      needsClawRelease() -- see Lift::updateScore().
// Below any plausible landing height and below L2_AUTO_RELEASE_DEG either way, so it can only ever
// matter once a release (by any path) has already happened or the travel rule structurally cannot.
inline constexpr double L2_RELEASE_FLOOR_DEG = 10.0;         // TUNE: arm degrees
// TWO L2 PRESSES PER SCORE (2026-10-02, the driver's own design):
//   PRESS 1  press the pin down, detect the landing, open the jaws, keep pushing for
//            L2_POST_RELEASE_PUSH_MS so they slide off, then RISE L2_CLEAR_RISE_DEG and HOLD there,
//            indefinitely. "this is so i am able to drive away with claw up above the pins without
//            any chance of knocking over pins."
//   PRESS 2  the ordinary fast run home: descend at L2_RETURN_SPEED_PCT, FINAL, SEAT at 0.
// The held position after press 1 is a DELIBERATE, driver-requested hold that press 2 resolves into a
// proper seat. It is NOT the old "parks a few degrees above the bottom instead of seating" bug that
// CLAUDE.md bans -- that one was a trip to the BOTTOM that failed to seat, with no second step.
// How long the arm keeps pressing DOWN after the jaws have been told to open, so they slide off the
// pin instead of lifting it back out of the cup.
//   The pin comes back up with the claw: raise it (100 -> 200).
//   The jaws grind on the pin / the stack gets pushed: lower it.
inline constexpr int L2_POST_RELEASE_PUSH_MS = 100;          // TUNE
// How far the arm rises after that, relative to wherever it actually stopped, before it holds. It has
// to clear the top of the stack by enough that driving away cannot knock a pin over.
//   Still catches a pin on the way out: raise it.
//   The arm is in the way of the next pickup: lower it.
inline constexpr double L2_CLEAR_RISE_DEG = 20.0;            // TUNE: arm degrees, RELATIVE
// A score or a grab asked for while a lift MOVE IS STILL IN FLIGHT waits for the arm to arrive first,
// then starts (Scoring::State::LIFT_SETTLE). That is what lets an auton route fire the lift and the
// drivetrain off together -- `auton::liftTo(level); ...drive...; auton::scoreAndWait();` -- without the
// press starting from a moving arm, and it fixes the same hazard for a driver who taps L2 mid-rise.
// Bounded, so a lift that never arrives cannot hang the macro: the lift's own LIFT_MOVE_TIMEOUT_MS
// parks it (which counts as settled) well before this, so this is only the backstop behind that.
inline constexpr int SCORING_SETTLE_WAIT_MS = 3000;          // TUNE
// The run home on PRESS 2, once the claw is free: % of free speed for the top and middle bands of the
// ordinary descent (the slow final zone and the SEAT are unchanged, so it still cannot slam or bounce).
//   "Don't hesitate to be going down faster than we currently are going when we are done scoring."
inline constexpr double L2_RETURN_SPEED_PCT = 95.0;          // TUNE

// ---------------------------------------------------------------------------
// Autonomous — the auton API and its tick. See include/auton.hpp, include/periodic.hpp.
// ---------------------------------------------------------------------------
// WHICH ROUTE autonomous() RUNS IS NOT HERE. It is a strategy choice, not a robot constant, so it
// lives at the top of src/autons.cpp (`constexpr Route SELECTED = ...`) with the routes themselves --
// one line, in the one file the driver works in. See README "Writing an auton route".
// The auton tick's period. The same 10 ms the driver loop runs at, because it runs THE SAME per-loop
// logic (periodic::tick()) -- Scoring's state machine, the claw autoclamp, and the one intake
// decision. The lift PID is NOT in here: it has its own 10 ms task from initialize() and is the only
// thing that ever writes the lift motors (CLAUDE.md safety rule 2).
inline constexpr int AUTON_TICK_MS = 10;
// Default timeouts for the blocking waits in auton.hpp. EVERY wait takes one, so a jammed mechanism
// can slow a routine down but can never hang it; each wait also returns whether it succeeded.
// Homing has to cover LIFT_HOME_TIMEOUT_MS plus a little slack for the task to notice.
inline constexpr int AUTON_HOME_TIMEOUT_MS = LIFT_HOME_TIMEOUT_MS + 500;
// A lift move: the longest travel is DOWN -> level 5 (107 deg), which at the braked profile is well
// under a second and a half; this leaves room for a macro's CLAMPING / pivot wait in front of it.
inline constexpr int AUTON_LIFT_TIMEOUT_MS = 2500;
// A whole L2 score: PRESS (up to L2_AUTO_RELEASE_DEG of travel) + the release checks + up to
// L2_MAX_UNSTICK_TRIES hops + the fast run home + FINAL's LIFT_FINAL_FORCE_MS.
inline constexpr int AUTON_SCORE_TIMEOUT_MS = 6000;
// A cylinder: the jaws' own CLAW_CLOSE_TIME_MS / CLAW_OPEN_TIME_MS, plus slack for waitClamped() to
// sit on the autoclamp for a moment.
inline constexpr int AUTON_CLAW_TIMEOUT_MS = 500;
// auton::waitUntilClamped(): long enough to drive into a pin and have the autoclamp confirm it.
inline constexpr int AUTON_CLAMP_TIMEOUT_MS = 2000;
// auton::grabAndWait(): the push (L1_GRAB_PUSH_MS, plus the jaws' own close time) and a little slack.
inline constexpr int AUTON_GRAB_TIMEOUT_MS = 1500;
// How long autonomous() will wait for odometry/IMU calibration before starting the route anyway. It
// is a BOUND, not a requirement: an unplugged or non-answering IMU must not be able to eat the whole
// autonomous period, and a route that never drives (the lift test) does not need odometry at all.
inline constexpr int AUTON_CALIBRATE_TIMEOUT_MS = 3000;

// ---------------------------------------------------------------------------
// moveB — the rear-distance-sensor drive, ported 1:1 from the team's old library
// ---------------------------------------------------------------------------
// NOT an odometry move: a hand-rolled PD loop on the REAR distance sensor that drives both sides at
// equal power until that sensor reads a target distance from whatever is behind the robot, then
// settles. See auton::moveB() for the full description. Every number the old library had baked in is
// here, and EVERY ONE NEEDS RETUNING: the old robot had 3.25" traction wheels, this one has 2.75"
// omnis, so the same power produces a different speed and the same error produces a different
// overshoot.
inline constexpr int8_t BACK_DISTANCE_PORT = 14;             // confirmed 2026-10-03
inline constexpr double MOVEB_KP = 0.28;                     // TUNE: power per mm of error
inline constexpr double MOVEB_KD = 0.68;                     // TUNE: power per mm of error change per tick
inline constexpr double MOVEB_TOLERANCE_MM = 5.0;            // TUNE: inside this counts as there
inline constexpr int MOVEB_SETTLE_MS = 55;                   // TUNE: ...held this long = done
inline constexpr int MOVEB_DEFAULT_MIN_SPEED = 15;           // TUNE: used when the caller passes minSpeed <= 0
inline constexpr int MOVEB_OUT_OF_RANGE_MM = 9910;           // at/above this the sensor sees nothing
inline constexpr int MOVEB_LOOP_MS = 20;                     // TUNE: the old library's loop period

}  // namespace config
