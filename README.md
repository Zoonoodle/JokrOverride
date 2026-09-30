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
  lift.*      DR6-4B position PID (gravity feedforward, windup clamp, settle, manual override)
  claw.*      pneumatic claw + autoclamp state machine
  intake.*    one-motor roller intake (driver-controlled only)
  scoring.*   intake / matchload / 7-level stacking state machine with interlocks
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
| Inertial sensor | 8 (**conflicts with right drive port 8, fix one of them**) |
| Lift motors | 20, 13 (reversed) |
| Lift rotation sensor | 16 (the lift PID runs off this sensor) |
| Intake (one motor) | 12 |
| Claw distance sensor (autoclamp) | 11 |
| Claw cylinder | ADI C |
| Pivot cylinder | ADI D |

## Driver guide
During driver control the lift is **plain motor power**. The PID, presets, stall/runaway guards and scoring macros are all off, so nothing else touches the lift.

| Button | What it does |
|---|---|
| Left stick Y / Right stick X | arcade drive |
| **L1** (hold) | lift **up**, full power (both motors) |
| **L2** (hold) | lift **down**, full power |
| let go of L1/L2 | lift motors HOLD where they are |
| **R1** (hold) | intake in |
| **R2** (hold) | intake out |
| **A** | claw clamp / open |
| **X** | pivot in / out |

The stack-level / matchload macros (`subsystems/scoring.*`) and the lift PID are still in the code for autonomous. `autonomous()` starts the PID; `opcontrol()` turns it off again.

**Brain `DRV` line:** `not started` means driver control never ran (not enabled, or the program crashed; look for a red error on the brain). Otherwise it shows a loop counter going up plus the stick values and L1/L2 it reads. If the counter climbs and `L1 1` shows but the lift doesn't move, the problem is wiring or motor direction, not code.

## Lift direction check (do this first)
The lift runs off the **rotation sensor** on port 16, not the motor encoders. If the sensor and the motors disagree on which way is up, the PID pushes the wrong way, the error grows, and the lift drives itself down. That's the "randomly spinning down" problem. To check:
1. Start the program with the lift fully down, and **don't enable**. Brain line 3 shows `rot` (rotation sensor) and `mot` (motor encoders), both in lift degrees.
2. Lift the arm up by hand. **Both must go positive.**
   - `mot` goes negative → the lift motors are reversed. Flip the signs in `LIFT_PORTS`.
   - `rot` goes negative → set `LIFT_ROTATION_REVERSED = true`.
   - `rot` is about 3× `mot` → the sensor is on the motor axle. Set `LIFT_ROTATION_RATIO = 3.0`.
3. Enable and hold **Up**. The arm must go up. If it goes down, the motor signs are wrong (go back to step 2).

A runaway guard catches this if it happens anyway: when the lift pushes hard one way but moves the other, it cuts power and shows `LIFT FAULT`. Up/Down still work, and letting go gives the PID another try.

## Tuning the claw positions
All heights are **lift-arm degrees** in `config::lift_preset` (`include/config.hpp`). 0 is where the lift sits when the program starts.
1. Start the program with the lift fully down.
2. Use **Up/Down** to put the claw exactly where you want it, then read `Lift x.x` on the brain screen.
3. Type that number into the preset and repeat for each one:
   - `INTAKE`: claw lined up with the intake handoff
   - `MATCHLOAD`: claw at matchload height
   - `PIVOT_CLEAR`: lowest height where the pivot can flip out without hitting anything
   - `STACK_LEVELS[0..6]`: the height to drop the 1st through 7th cup on a stack
   - `MAX`: just under the hard stop
   - **Measure the top first.** Hold Up until the arm is at the top and read `Lift`. Every preset has to be below that number. If the lift shows `BLOCKED` (brain + controller), it was asked to go somewhere it can't physically reach, so it stopped pushing and is holding where it is. Lower that preset.
4. Timing: `CLAW_CLOSE_TIME_MS` (grip before lifting), `CLAW_OPEN_TIME_MS` (release before lowering), `PIVOT_TIME_MS`, and `LIFT_LOWER_SPEED_DEG_PER_S` (how slowly the arm comes down after a score).

**Lift PID** (`LIFT_KG`, `LIFT_KP`, `LIFT_KD`), in this order:
1. **kG:** set kP = kD = 0. Raise kG until the arm stays put at mid-height without drifting down, and stop before it creeps up.
2. **kP:** raise it until presets are reached quickly with a small overshoot.
3. **kD:** raise it until the overshoot and bounce go away.
4. **kI:** only if the arm settles consistently short of a preset while holding a cup.

## Brain screen
Shows live readings the whole time the program runs (disabled, auton and driver). A sensor that isn't plugged in shows `not connected (port N)`, and the wrong device on a port shows `WRONG DEVICE on port`, instead of junk numbers.

| Line | Shows |
|---|---|
| POSE | odometry X / Y (in) and theta (deg), from the tracking wheel + IMU |
| IMU | heading and total rotation, or `calibrating... keep still` |
| ODOM | tracking wheel rotation sensor, in degrees |
| LIFT | lift position → target (lift deg), `SETTLED` or `LIFT FAULT` |
| ROT | lift rotation sensor vs motor encoders. Both must go + when the lift goes up |
| DIST | claw distance sensor mm, confidence, and `PIN` when inside `CLAW_DETECT_MM` |
| CLAW | claw state, autoclamp on/off, scoring state and stack level |
| TEMP | hottest left drive / right drive / lift motor and the intake motor (C). Motors lose power around 55 C. |
