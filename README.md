# JokrOverride: 57249C JOKR robot code (VEX V5RC 2026-27 Override)

PROS 4.2.2 · LemLib 0.5.6 · liblvgl 9.2.0

## Setup
1. Install the PROS CLI and toolchain. The easiest route is the VS Code PROS extension. From a terminal: `pip install pros-cli` (with `setuptools<81`) plus the `arm-none-eabi-gcc` toolchain.
2. `git clone https://github.com/Zoonoodle/JokrOverride && cd JokrOverride`
3. `pros make`, then `pros upload` with the brain plugged in.
4. **Every robot-specific value is in `include/config.hpp`.** Anything marked `TODO(team): confirm` hasn't been checked on the robot yet. Anything marked `TUNE` is a starting value, not a tuned one.

## Layout
```
include/config.hpp            every port, ratio, preset and gain
include/robot.hpp, src/robot.cpp   chassis (LemLib) + subsystem objects
include/subsystems/ src/subsystems/
  lift.*      DR6-4B position PID (gravity feedforward, windup clamp, settle, stall/runaway guards,
              plain manual override for D-pad Up/Down, moveTo()/setTarget() shared by driver + auton)
  claw.*      pneumatic claw + autoclamp state machine
  intake.*    one-motor roller intake (driver-controlled only)
  scoring.*   intake macro: intake / matchload / 5-level stacking state machine with interlocks,
              drives the lift only through Lift::setTarget() (the PID), never raw power
src/screen.cpp                brain screen sensor readout
src/main.cpp                  initialize / autonomous stub / opcontrol + button map
docs/notebook/                notebook-ready write-ups, flowcharts and blank test tables
```

## Port map
All ports live in `include/config.hpp`. Current values:

| Device | Port |
|---|---|
| Left drive | 2, 3, 1 (reversed) |
| Right drive | 8, 9, 10 |
| Odometry pod (rotation) | 7 |
| Inertial sensor | **none installed yet** (`IMU_PORT = 0`; odometry runs on the tracking wheel alone until one is added -- set a real port to enable it) |
| Lift motors | 20, 13 (reversed), on the 12T axle, geared 4:1 into the 48T lift-arm axle |
| Lift rotation sensor | 16, on the 12T motor axle (same 4:1 as the motors) -- the lift PID runs off this sensor |
| Intake (one motor) | 12 |
| Claw distance sensor (autoclamp) | 11 |
| Claw cylinder | ADI C |
| Pivot cylinder | ADI D |

## Driver guide
The lift PID (`subsystems/lift.*`) runs the whole program, starting idle in `initialize()` -- nothing
moves on its own. **D-pad Up/Down (hold) are always plain manual power**, same feel as full-send motor
control; every other lift button below goes through the PID (`lift.setTarget()`/`moveTo()`), never raw
voltage. Holding Up or Down immediately cancels whatever height was active (a preset button or the
intake macro) and takes over; letting go brakes and holds right there -- it does **not** snap back to
the old target. See the class comment in `include/subsystems/lift.hpp` for why only one thing is ever
allowed to own the lift motors.

| Button | What it does |
|---|---|
| Left stick Y / Right stick X | arcade drive |
| **D-pad Up** (hold) | lift **up**, full power (manual, cancels any preset/macro target) |
| **D-pad Down** (hold) | lift **down**, full power (same) |
| let go of Up/Down | lift motors HOLD where they are (no target resumes) |
| **L1** | intake macro: step up a stack level (1st tap clamps + goes to level 1; each tap after goes up one level, staying at level 5 once reached) |
| **L2** | intake macro: holding a pin → **SCORE** (open the jaws, pivot in, lower all the way to `DOWN`) and reset the level count so the next L1 starts at level 1 again |
| **R1** (hold) | intake roller in; (tap) also sends the lift to the **INTAKE** handoff position |
| **R2** (hold) | intake roller out |
| **B** | **stow**: lift straight down to `DOWN` (0°) and cancel the intake macro |
| **A** | claw clamp / open |
| **X** | pivot in / out |
| **Y** | autoclamp on / off |

`L1`/`L2`/R1's tap-to-INTAKE/`B` are locked out while Up/Down are held, so a tap can never
re-command the lift out from under a manual nudge (see interlock 5 in `subsystems/scoring.hpp`).
They don't touch the claw or pivot, either -- those stay exactly where they physically are.

`scoreOrMatchload()` (the old "score, or go to matchload if empty" behaviour) and `resetLevel()`
(back to level 1 without scoring) are still in `subsystems/scoring.*` but aren't wired to a driver
button right now -- available for autonomous, or a future button.

**Note:** `X` (manual pivot toggle) is independent of the intake macro above. Avoid tapping it while
the macro is mid-move (`STACKING`/`MOVING`) -- the macro also drives the pivot as part of its
interlocks, and the two aren't coordinated with each other.

**Brain `DRV` line:** `not started` means driver control never ran (not enabled, or the program crashed; look for a red error on the brain). Otherwise it shows a loop counter going up plus the stick values and the manual-override Up/Down it reads. If the counter climbs and `Up 1` shows but the lift doesn't move, the problem is wiring or motor direction, not code.

## Auton API
Autonomous and the driver's preset buttons share the same named heights (`config::lift_preset`, see
"Tuning the claw positions" below) and the same `Lift` class:

```cpp
lift.moveTo(config::lift_preset::INTAKE, 1500, true);            // jump to INTAKE, wait up to 1.5s or until settled
lift.moveTo(config::lift_preset::STACK_LEVELS[2], 2000, true);   // 3rd of 5 stack levels
if (!lift.isSettled()) { /* ... */ }
lift.cancel();                                                    // stop chasing a target, brake-hold in place
```

`moveTo(height, timeoutMs, waitUntilSettled)` blocks the calling task only (autonomous or driver
code) -- the lift's own background task keeps running the PID at a fixed 10 ms regardless.
`setTarget(height, speedDegPerSec)` is the lower-level, non-blocking version (used internally, and
by `scoring.cpp` for the post-score lower). Speed 0 means "jump the setpoint, let the PID go as fast
as it can" -- that is what every upward move and every mid-height move uses, and it runs the full
12000 mV until it is close in. The one exception is a goal at or below `LIFT_BOTTOM_GOAL_DEG`
(`DOWN` and `INTAKE`): those are always profiled, whatever speed the caller asked for, so nothing
arrives at the frame at full speed. See "Lift speed" below.

## Lift direction check (do this first)
The lift runs off the **rotation sensor** on port 16, mounted on the 12T motor axle (confirmed, geared 4:1 into the 48T lift-arm axle -- `LIFT_GEAR_RATIO` / `LIFT_ROTATION_RATIO` are both `4.0`). Everything in the code -- presets, PID target/error, the brain's `LIFT` lines -- works in **lift-arm degrees**; that 4:1 is what converts the sensor/motor side down to that one shared unit. If the sensor and the motors disagree on which way is up, the PID pushes the wrong way, the error grows, and the lift drives itself down. That's the "randomly spinning down" problem. To check:
1. Start the program with the lift fully down, and **don't enable**. The brain's `ROT` line shows the rotation sensor and motor encoders side by side, already converted to lift-arm degrees, with the raw sensor/motor degrees in brackets.
2. Lift the arm up by hand. **Both (unbracketed) readings must go positive**, and should read about the same as each other. The bracketed raw sensor reading should be about 4x the bracketed raw motor reading (the 4:1) -- if it isn't, `LIFT_GEAR_RATIO`/`LIFT_ROTATION_RATIO` don't match the real gearing, fix those first.
   - the motor reading goes negative → the lift motors are reversed. Flip the signs in `LIFT_PORTS`.
   - the rotation reading goes negative → set `LIFT_ROTATION_REVERSED = true`.
3. Enable and hold **D-pad Up**. The arm must go up. If it goes down, the motor signs are wrong (go back to step 2).

This has already been checked once, from 7 reference photos with the robot powered on and the arm moved by hand through every named height (`docs/robot/photos/levels/`, see `docs/robot/ROBOT.md` "Lift levels") -- the readings rose the whole way from `DOWN` to the top score level, confirming `LIFT_ROTATION_REVERSED = false` is correct. Still worth re-running this check after any work on the lift's mechanical build.

Because of the 4:1, the sensor turns more than a full revolution (~420° at the sensor for ~105°+ of arm travel) -- the code reads it with `Rotation::get_position()` (multi-turn, keeps counting past 360°), never `get_angle()` (wraps at 360° and would make the lift think it jumped positions partway up). This is already handled in `subsystems/lift.cpp`; it's noted here so nobody "fixes" it back to `get_angle()` later.

A runaway guard catches this if it happens anyway: when the lift pushes hard one way but moves the other, it cuts power and shows `LIFT FAULT` on the brain's `LIFT` line. The manual override (D-pad Up/Down) still works and clears the fault.

## Lift speed
**The arm's free speed is 150 deg/s.** Both lift motors are red (100 RPM) cartridges; through the
12T -> 48T (4:1) reduction that is 100 / 4 = 25 arm RPM = 150 arm deg/s. `LIFT_FREE_SPEED_DEG_PER_S`
in `config.hpp` records it, and nothing should be tuned above it -- the motors cannot turn faster, so
a setpoint asking for more just runs away from the arm. Everything below lives in `include/config.hpp`.

| Move | How it is driven | Speed |
|---|---|---|
| Up to any preset (L1 stack levels, `MATCHLOAD`) | setpoint jumps, PID drives it | full 12000 mV until within `LIFT_MAX_MV / LIFT_KP` (20 deg) of the target, then closes in |
| Down to a mid-height preset (e.g. `resetLevel()` back to level 1) | same -- setpoint jumps | full voltage, PID brakes it; nothing to hit at that height |
| Down to `DOWN` (L2 score, `B` stow, auton) or `INTAKE` (R1) | **profiled**: the setpoint walks down, with a velocity feedforward | `LIFT_LOWER_START_SPEED_DEG_PER_S` (80) ramping over `LIFT_LOWER_RAMP_DEG` (5 deg) to `LIFT_LOWER_SPEED_DEG_PER_S` (130, ~87% of free speed), then `LIFT_SLOW_SPEED_DPS` (35) for the last `LIFT_SLOW_ZONE_DEG` (25 deg), then brake-hold inside `LIFT_REST_BAND_DEG` (3) |
| D-pad Up/Down manual | `lift.driverPower(+-1.0)` -> `+-LIFT_MAX_MV` | full 12000 mV, both directions |

A full descent from the top stack level (128.3 deg) takes about **1.5 s**.

`LIFT_KV` (80 mV per arm deg/s = 12000 / 150) is what makes the fast descent a *tracked* profile
rather than a lag. A 130 deg/s descent needs roughly -9000 mV; with kP alone the PID could only
produce that by letting the arm sit 15-20 deg below the setpoint the whole way down, which makes the
slow zone near the bottom useless (the arm gets there late and still moving) and looks like a runaway
to the fault guard. The feedforward supplies that voltage from the setpoint's own speed, so the arm
tracks it within a couple of degrees at any speed.

The slow zone is sized from the cruise speed: stopping from 130 deg/s with the full 12000 mV of
upward authority plus the rubber bands takes ~0.2 s, so v^2/(2a) with a ~ 650 deg/s^2 is ~13 deg;
plus ~10 deg of residual tracking lag, rounded up to 25.

**If it...**

| Symptom | Change |
|---|---|
| Slams/bangs the frame at the bottom | lower `LIFT_SLOW_SPEED_DPS` (toward 20), then raise `LIFT_SLOW_ZONE_DEG`, then raise `LIFT_REST_BAND_DEG` |
| Overshoots or bounces at a preset | raise `LIFT_KD` first (it damps without slowing the approach much), then back `LIFT_KP` off toward 450 |
| Still sluggish arriving at a preset | raise `LIFT_KP` (700-800), keeping `LIFT_KD` about 2x it |
| Still sluggish coming down | raise `LIFT_LOWER_SPEED_DEG_PER_S` toward 150, raise `LIFT_SLOW_SPEED_DPS`, lower `LIFT_SLOW_ZONE_DEG` |
| Jerks at the start of a descent | raise `LIFT_LOWER_RAMP_DEG` or lower `LIFT_LOWER_START_SPEED_DEG_PER_S` |
| `BLOCKED` on a normal move (stall guard) | the arm really isn't moving -- check for a jam or a preset above the real top before touching `LIFT_STALL_MV`/`_DEG`/`_MS` |
| `LIFT FAULT` on a normal fast move (runaway guard) | raise `LIFT_FAULT_DEG`; also check `LIFT_KV` isn't too low (watch `err` on the brain during a descent, it should stay within ~2 deg) |
| Arm lags the setpoint going down (`err` several deg negative at cruise) | raise `LIFT_KV` |
| Arm dives ahead of the setpoint (`err` positive going down) | lower `LIFT_KV` |

## Tuning the claw positions
All heights are **lift-arm degrees** in `config::lift_preset` (`include/config.hpp`). 0 is where the lift sits when the program starts. `DOWN`, `INTAKE` and the 5 `STACK_LEVELS` are already measured from reference photos (see `docs/robot/ROBOT.md` "Lift levels") -- `MATCHLOAD` and `PIVOT_CLEAR` are still placeholders. To (re)measure any of them:
1. Start the program with the lift fully down.
2. Use **D-pad Up/Down** to put the claw exactly where you want it, then read the brain's `LIFT` line for the current position.
3. Type that number into the preset:
   - `INTAKE`: claw lined up with the intake handoff
   - `MATCHLOAD`: claw at matchload height
   - `PIVOT_CLEAR`: lowest height where the pivot can flip out without hitting anything
   - `STACK_LEVELS[0..4]`: the height to drop the 1st through 5th pin on a stack
   - `MAX`: a few degrees above the top level, as a software hard stop -- never raise it just to let a bad preset through
   - **Measure the top first.** Hold Up until the arm is at the top and read the position. Every preset has to be below that number. If the lift shows `BLOCKED` (brain + controller), it was asked to go somewhere it can't physically reach, so it stopped pushing and is holding where it is. Lower that preset.
4. Timing: `CLAW_CLOSE_TIME_MS` (grip before lifting), `CLAW_OPEN_TIME_MS` (release before lowering -- `scoreOrMatchload()`'s path only; L2's `score()` opens the claw and starts lowering in the same tick), `PIVOT_TIME_MS`, and the descent speed curve the lift itself profiles for every trip to the bottom -- see "Lift speed" above.

**Lift PID** (`LIFT_KG`, `LIFT_KP`, `LIFT_KD`, `LIFT_KI`), in this order, watching the brain's `LIFT P/I/D` line:
1. **kG:** set kP = kD = 0. Raise kG until the arm stays put at mid-height without drifting down, and stop before it creeps up.
2. **kP:** raise it until presets are reached quickly with a small overshoot (watch the `P` term and `err`).
3. **kD:** raise it until the overshoot and bounce go away (watch the `D` term).
4. **kI:** only if the arm settles consistently short of a preset while holding a pin (watch the `I` term -- it should only move near the target, inside `LIFT_WINDUP_RANGE`).

## Brain screen
Shows live readings the whole time the program runs (disabled, auton and driver), drawn with
`pros::screen` (not the 8-line `pros::lcd` console) so there's room for the full lift tuning section
below alongside the existing sensor readouts. A sensor that isn't plugged in shows `not connected
(port N)`, and the wrong device on a port shows `WRONG DEVICE on port`, instead of junk numbers.

| Line | Shows |
|---|---|
| POSE | odometry X / Y (in) and theta (deg), from the tracking wheel + IMU |
| IMU | `none (not installed)` (no IMU on the robot yet), or heading and total rotation, or `calibrating... keep still` |
| ODOM | tracking wheel rotation sensor, in degrees |
| ROT | lift rotation sensor vs motor encoders. Both must go + when the lift goes up |
| DIST | claw distance sensor mm, confidence, and `PIN` when inside `CLAW_DETECT_MM` |
| LIFT (position line) | position → target (or `none`), the active named preset (or `custom`/blank), and `SETTLED`/`BLOCKED`/`FAULT` |
| LIFT (err/out line) | error in lift degrees and total PID output in mV |
| LIFT (P/I/D line) | the P, I and D terms separately, in mV, so you can watch each one while tuning |
| GAIN | `kP`/`kI`/`kD`/`kG` read straight from `config.hpp` -- tuning happens in code, not on the controller |
| LIFT (temp/current line) | lift motor temp (°C) and current draw (mA) |
| TEMP | hottest left drive / right drive / lift motor and the intake motor (C). Motors lose power around 55 C. |
| DRV | driver-loop heartbeat: `not started`, or a loop counter plus stick values and the manual-override Up/Down |
