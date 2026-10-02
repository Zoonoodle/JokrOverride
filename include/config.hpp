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
// SPEEDS WENT UP (50/75 -> 80/95) once L2 stopped being "lower away from a pin we just dropped". The
// pin is already seated in the stack by the time the arm comes home now, so the trip down is purely
// "get out of the way" -- for L2's return, for B's stow and for an auton move to DOWN alike. The ramp
// went 300 -> 800 deg/s^2 to match: at 95% the reference has to fall from ~142 deg/s to the slow
// band's ~18 inside LIFT_DESCENT_SLOW_ZONE_DEG, which needs about (142-18)/0.8s^-1 ... concretely
// 800 deg/s^2 gets there in 0.155 s and about 12 deg of travel, comfortably inside the 20 deg zone.
// At 300 it took 0.41 s and ~33 deg, i.e. it would still have been doing 100 deg/s at the frame.
//   Falls too fast off a high level: lower LIFT_DESCENT_TOP_PCT / LIFT_DESCENT_MID_PCT.
//   Still arrives at the bottom with a thump: raise LIFT_DESCENT_SLOW_ZONE_DEG, lower
//     LIFT_DESCENT_SLOW_PCT, or raise LIFT_DESCENT_RAMP_DEG_PER_S2 so the slowdown happens sooner.
//   The whole descent takes too long: raise LIFT_DESCENT_MID_PCT, lower LIFT_DESCENT_SLOW_ZONE_DEG.
inline constexpr double LIFT_DESCENT_TOP_PCT = 80.0;         // TUNE: ~120 deg/s
inline constexpr double LIFT_DESCENT_MID_PCT = 95.0;         // TUNE: ~142 deg/s
inline constexpr double LIFT_DESCENT_SLOW_PCT = 12.0;        // TUNE: ~18 deg/s, the constant slow final descent
inline constexpr double LIFT_DESCENT_SLOW_ZONE_DEG = 20.0;   // TUNE: slow zone starts this many deg above 0
inline constexpr double LIFT_DESCENT_MID_FRACTION = 0.5;     // TUNE: 0.5 = the mid band is the lower half of the fast travel
inline constexpr double LIFT_DESCENT_RAMP_DEG_PER_S2 = 800.0;// TUNE: how quickly the reference may change between bands

// --- Seating on the hard stop (phases FINAL, then SEAT) ---------------------
// The point of the slow final descent is that the arm does not STOP above the frame -- it keeps
// walking down until it physically cannot, and then stays pressed there so it cannot bounce back up.
//
// TWO STEPS, not one -- this used to be a single check ("stopped under LIFT_SEAT_DETECT_DEG" ==
// "seated") and that was the bug: a cup caught in the intake, or plain six-bar/chain friction, can
// stall the arm a few degrees above the true hard stop, read as "stopped", and get declared SEATed
// right there -- after which only LIFT_SEAT_MV (a HOLD voltage, meant to fight bounce, not resistance)
// was ever applied again, so the arm simply parked short of 0 forever.
// Below LIFT_SEAT_DETECT_DEG the arm now hands over from the ordinary DESCEND bands into FINAL: a
// dedicated closed-loop push at a constant LIFT_FINAL_DEG_PER_S, with up to LIFT_FINAL_MAX_MV of
// downward authority -- more force than the SEAT hold, less than DESCEND's full 12000 mV so a real
// hard-stop contact doesn't slam -- specifically to push through that last bit of resistance. Only
// once the arm is BOTH within LIFT_SEAT_ZERO_DEG of 0 AND still (under LIFT_SEAT_STILL_DEG_PER_S for
// LIFT_SEAT_STILL_MS) does it hand over to true SEAT.
// TIME BUDGET: FINAL gets up to LIFT_SEAT_MAX_MS of real pushing. If it still has not reached
// LIFT_SEAT_ZERO_DEG by then (a genuine jam, not just friction), it gives up and seats wherever it
// stopped -- LIFT_SEAT_MV still holds it down so it cannot bounce -- and the brain shows `SEAT(short)`
// instead of `SEAT`, so the driver can tell it did not actually reach 0.
// After a true SEAT the lift holds LIFT_SEAT_MV forever (or until something physically disturbs it,
// see LIFT_SEAT_UNSEAT_DEG below).
// HEAT: -1500 mV is 12.5% of full volts -- a stalled V5 motor there still draws well under an amp (the
// winding is about 1.4 ohm), a bit over a watt per motor, and the arm is resting on the frame so the
// seat voltage is only fighting bounce, not holding the arm up. Safe for a whole match -- but watch the
// brain's TEMP line the first time, and LIFT_SEAT_TEMP_LIMIT_C drops the seat to a plain brake-hold if
// a lift motor ever does get that hot.
//   Still stops short of 0 / shows SEAT(short) every time: raise LIFT_FINAL_MAX_MV first (5000 -> 6500),
//     then LIFT_SEAT_MAX_MS. Check the brain's ANGLE line reads HOMED first -- "0" is only true against
//     a homed zero; an un-homed seed can read short even when the arm is physically all the way down.
//   Thumps into the frame during FINAL: lower LIFT_FINAL_DEG_PER_S.
//   Bounces back up off the stop once truly seated: raise the magnitude of LIFT_SEAT_MV further (-1800).
//   Motors warm up while parked at the bottom: lower LIFT_SEAT_MV's magnitude (e.g. -1000).
//   Declares true SEAT while still visibly short: lower LIFT_SEAT_ZERO_DEG, lower
//     LIFT_SEAT_STILL_DEG_PER_S, or raise LIFT_SEAT_STILL_MS.
inline constexpr float LIFT_SEAT_MV = -1500.0f;              // TUNE: constant downward seating voltage
inline constexpr double LIFT_SEAT_DETECT_DEG = 6.0;          // TUNE: this close to 0, hand DESCEND over to FINAL
inline constexpr double LIFT_FINAL_DEG_PER_S = 12.0;         // TUNE: constant creep speed for the last stretch
inline constexpr float LIFT_FINAL_MAX_MV = 5000.0f;          // TUNE: magnitude, downward authority during FINAL
inline constexpr double LIFT_SEAT_ZERO_DEG = 1.0;            // TUNE: only this close to 0 may FINAL call it seated
inline constexpr int LIFT_SEAT_MAX_MS = 2000;                // TUNE: FINAL's time budget before giving up in place
inline constexpr double LIFT_SEAT_STILL_DEG_PER_S = 4.0;     // TUNE
inline constexpr int LIFT_SEAT_STILL_MS = 150;               // TUNE
// If something knocks a seated arm this far back up above LIFT_SEAT_DETECT_DEG, the descent restarts
// (DESCEND, then FINAL again) and walks it back down rather than leaving it hanging.
inline constexpr double LIFT_SEAT_UNSEAT_DEG = 4.0;          // TUNE
inline constexpr double LIFT_SEAT_TEMP_LIMIT_C = 50.0;       // safety net, not a tuning knob

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
//   HOLD      the damped position hold (KP/KI/KD above). Its integral carries the learned gravity
//             voltage over to the next move, so no move ever starts by guessing it.
//
// A DESCENT (a non-precise goal at or below LIFT_BOTTOM_GOAL_DEG: L2, B, auton DOWN, R2 in clamped
// mode) is DESCEND -> FINAL -> SEAT: the three speed bands above, then (below LIFT_SEAT_DETECT_DEG) a
// dedicated closed-loop push at LIFT_FINAL_DEG_PER_S with up to LIFT_FINAL_MAX_MV of authority -- more
// force than the seat hold, enough to push through a cup or friction a few degrees short of 0 -- for up
// to LIFT_SEAT_MAX_MS, and only then a constant LIFT_SEAT_MV pressed into the hard stop. It does not
// stop above the frame and it does not bounce; see "Seating on the hard stop" below for why the old
// single-step version could park short.
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
// A=APPROACH (the final crawl), H=HOLD, V=DESCEND (a banded trip to the bottom), R=RETURN (L2's fast
// run home, same bands as V), Z=FINAL (the push-to-0 step below LIFT_SEAT_DETECT_DEG), S=SEAT (pressed
// onto the hard stop, or holding short -- see LIFT_SEAT_MAX_MS), M=manual (D-pad, or R1 in clamped
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
// instead of riding the lift down. Covers the whole trip through PRESS / REL / UNSTK / RETURN / FINAL,
// not just the final run home -- whichever phase the arm happens to be under this height in -- and
// keeps running through the WHOLE slow final descent (DESCEND's slow band, then FINAL pushing for true
// 0, see the lift's "Seating on the hard stop" above) rather than cutting off at LIFT_BOTTOM_GOAL_DEG:
// a cup sitting in the intake can be exactly what is blocking the arm from reaching the bottom, so the
// rollers need to still be clearing it while FINAL is forcing the last few degrees.
//   NOTE: stack level 1 (31.0 deg) is already below this, so scoring at level 1 outtakes from the
//   start of the press, not partway down. That is this constant as specified, not a bug.
//   Pin/cup gets knocked loose too early: lower it. Rollers start too late to clear the handoff
//   before the arm seats: raise it.
inline constexpr double L2_OUTTAKE_BELOW_DEG = 40.0;         // TUNE: arm degrees
// % of full intake power (config::INTAKE_MV) the rollers run OUT at during the above. Lower than
// R2's full-power outtake on purpose -- this runs unattended during a score, not under driver thumb.
inline constexpr double L2_OUTTAKE_PCT = 60.0;               // TUNE: % of INTAKE_MV
// Stop condition is now "actually SEATed" (lift.isSeated(), true SEAT or SEAT(short) alike), not a
// position threshold -- see main.cpp -- plus this much extra afterward, so a cup that only clears once
// the arm stops pressing on it still gets ejected rather than left riding along at the very end.
inline constexpr int L2_OUTTAKE_AFTER_SEAT_MS = 500;         // TUNE
// Safety net: the rollers stop after this long no matter what, so a score that stalls while under
// L2_OUTTAKE_BELOW_DEG (BLOCKED, FAULT, stuck claw) can't leave them spinning for the rest of the match.
// Sized for the longer worst case now: DESCEND's slow band + FINAL's full LIFT_SEAT_MAX_MS budget +
// L2_OUTTAKE_AFTER_SEAT_MS, with margin.
inline constexpr int L2_OUTTAKE_MAX_MS = 4000;               // TUNE

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
// keep increasing the push until it moved -- i.e. flatten the stack. 5000 mV through the 4:1 is about
// 40% of stall torque: firm enough to seat a pin, not enough to crumple a tower.
//   The stack gets pushed over / the cup crumples: LOWER it (3500).
//   The pin does not seat fully before the release: raise it (6500).
inline constexpr float L2_PRESS_MAX_MV = 5000.0f;            // TUNE: magnitude, downward
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
// baseline -- not against what it was told to do. Three signals, any one of which fires:
//   LAG     a leaky one-sided integral of the speed shortfall against the baseline, in arm degrees:
//           "how far behind where it would have been the arm now is". The POSITION-vs-EXPECTED signal,
//           and the sensitive one -- it catches a landing that only slows the arm by a quarter.
//   EFFORT  the loop pushing DOWN harder than the free descent needed. The MOTOR-WORK signal: it
//           catches a pin being forced into the tower even while it still creeps.
//   SPEED   measured below a fraction of the baseline, or an absolute near-stop. The backstop; the
//           absolute one needs no baseline, so it still catches a pin already resting on the stack.
// Which one fired is on the brain (REL(lag) / REL(eff) / REL(spd) / REL(stall)) and in the trace's
// `trg` column, so the next trace says exactly which signal moved.
//
//   Releases early, with the pin still in the air: raise L2_CONTACT_MS (20 -> 40), lower
//     L2_CONTACT_SPEED_FRAC (0.70 -> 0.60), raise L2_CONTACT_LAG_DEG (1.5 -> 2.5). Read the trace's
//     `trg` column first -- fix the signal that actually fired, not all three.
//   Presses on after the pin has landed: lower L2_CONTACT_SPEED_FRAC is NOT the fix (that makes it
//     later); lower L2_CONTACT_LAG_DEG (-> 1.0) and L2_CONTACT_EFFORT_MV (-> 1400), which are the
//     early-warning signals, and lower L2_CONTACT_MS.
//   Never detects contact: lower L2_CONTACT_LAG_DEG and L2_CONTACT_EFFORT_MV, and raise
//     L2_CONTACT_SPEED_FRAC toward 0.8. Check the trace: if `lag` and `eff` both stay near 0 through
//     the landing, the arm is not actually slowing OR working harder -- the claw is sliding along the
//     pin, which is a mechanical problem, not a threshold one.
inline constexpr double L2_CONTACT_LAG_DEG = 1.5;            // TUNE: arm degrees of accumulated shortfall
inline constexpr double L2_CONTACT_LAG_LEAK_MS = 200.0;      // TUNE: makes LAG measure a change, not a drift
inline constexpr double L2_CONTACT_EFFORT_MV = 2000.0;       // TUNE: extra downward push over the baseline
inline constexpr double L2_CONTACT_SPEED_FRAC = 0.70;        // TUNE: measured / baseline that counts as contact
inline constexpr double L2_CONTACT_STALL_DEG_PER_S = 6.0;    // TUNE: absolute near-stop, needs no baseline
inline constexpr int L2_CONTACT_MS = 20;                     // TUNE: confirm window -- short, so the release is fast
// How long the press gets to reach a steady descent speed before the baseline is snapshotted and the
// baseline-relative signals come alive. The absolute near-stop is live before this (after the dead
// time below), so a pin that is already resting is still caught immediately.
inline constexpr int L2_CONTACT_SETTLE_MS = 200;             // TUNE
// Baseline EMA time constants. The baselines must track the slow, genuine sag in a free descent's
// speed and cost as the gravity torque changes with angle -- otherwise LAG drifts into a false
// trigger -- but they FREEZE as soon as the arm is below L2_CONTACT_FREEZE_FRAC of the baseline, so
// they can never chase a developing landing and mask it. That two-level gate is the whole trick; the
// host test has a scenario for each half of it.
inline constexpr double L2_CONTACT_BASELINE_TAU_MS = 200.0;  // TUNE
inline constexpr double L2_CONTACT_OUTPUT_TAU_MS = 50.0;     // TUNE: low-pass on the mV, for EFFORT
inline constexpr double L2_CONTACT_FREEZE_FRAC = 0.90;       // TUNE: loose gate -- stops LEARNING, never fires
// Dead time at the very start of the press before any trigger may fire. Short on purpose: the
// baseline's settle time is what really protects the spin-up, and a long dead time would miss a pin
// that lands immediately.
inline constexpr int L2_CONTACT_IGNORE_MS = 60;              // TUNE
// After the release, the arm has to be clearly moving again (faster than L2_CONTACT_STALL_DEG_PER_S)
// for this long to count as "the claw came off the pin". Then it goes home. The RELEASE phase uses
// this plain speed test rather than the full detector: with the jaws open the arm either moves or it
// does not, and there is nothing subtle to catch.
inline constexpr int L2_FREE_MS = 100;                       // TUNE
// ...and conversely, this long below that speed with the jaws open means the claw is hooked on the pin.
inline constexpr int L2_STUCK_MS = 250;                      // TUNE
// If the release is never acknowledged (Scoring not running, e.g. an auton that never calls
// Scoring::update()), carry on anyway after this long rather than pressing down forever. Worst case
// the arm goes home with the pin still clamped, which is the same safe outcome as NO CONTACT.
inline constexpr int L2_RELEASE_ACK_MS = 400;
// STUCK RECOVERY. Still stalled after the jaws opened = the claw is hooked on the pin. Lift UP this
// far (claw open) and try pressing down again, up to L2_MAX_UNSTICK_TRIES times. After that it stops,
// holds where it is and shows SCORE STUCK -- it never keeps grinding.
inline constexpr double L2_UNSTICK_UP_DEG = 20.0;            // TUNE: arm degrees
inline constexpr double L2_UNSTICK_SPEED_PCT = 40.0;         // TUNE: ~60 deg/s, it is only a short hop
inline constexpr int L2_UNSTICK_TIMEOUT_MS = 1200;           // safety net: don't grind upward either
inline constexpr int L2_MAX_UNSTICK_TRIES = 2;               // TUNE
// If the press reaches this height without ever detecting contact there was no stack under the arm.
// Nothing is released -- the arm slows into the bottom and SEATs with the pin still clamped, and the
// brain shows NO CONTACT. (L2 again at the bottom then releases it; that is the clamp toggle.)
// Lower than any plausible landing height for stack level 1 (31.0) and above the seat zone.
inline constexpr double L2_NO_CONTACT_DEG = 12.0;            // TUNE
// The run home once the claw is free: % of free speed for the top and middle bands of the ordinary
// descent (the slow final zone and the SEAT are unchanged, so it still cannot slam or bounce).
//   "Don't hesitate to be going down faster than we currently are going when we are done scoring."
inline constexpr double L2_RETURN_SPEED_PCT = 95.0;          // TUNE

}  // namespace config
