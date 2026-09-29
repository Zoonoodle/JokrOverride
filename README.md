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
Nothing moves when the match starts. The intake only spins while you're holding a button, and the lift stays down until you press something.

| Button | What it does |
|---|---|
| Left stick Y / Right stick X | arcade drive |
| **R1** (hold) | intake **in**. The first press also sends the claw to the **INTAKE** position with the jaws open. If you're holding a cup, R1 only runs the intake and the arm doesn't move. |
| **L1** (tap) | **Tap 1**: clamp, then lift to **stack level 1**. **Each extra tap** goes up one level, to a max of **level 7**. |
| **L2** (tap) | **Holding a cup: SCORE.** The claw opens, then the arm lowers **slowly** back to INTAKE. **No cup:** the claw goes to the **MATCHLOAD** position with the jaws open. |
| **R2** | **Holding a cup:** tap to drop **back to level 1** (for when you overshoot). **No cup:** hold for intake **out** (extake). |
| B (hold) | intake out, always (unjam) |
| Up / Down (hold) | lift fine-tune; the lift holds where you let go |
| A | manual claw clamp / open |
| Y | autoclamp on / off (off at start) |

The controller screen shows `LVL n` while you're holding a cup, so you always know which tap you're on. It shows `LIFT FAULT` and rumbles if the lift guard trips.

**A normal cycle:**
1. Hold **R1**. The claw drops to INTAKE and opens, and the intake pulls a cup into the claw.
2. Tap **L1** once per stack height: 1 tap for an empty goal, 3 taps for the 3rd cup, and so on. The claw clamps, the lift rises, and the pivot flips out once it's above `PIVOT_CLEAR`.
3. Overshot? Tap **R2** to go back to level 1, then L1 up again.
4. Tap **L2** to score. The claw opens, waits `CLAW_OPEN_TIME_MS`, the pivot comes back in, and the arm lowers slowly to INTAKE, ready for step 1.

**Matchload cycle:** with an empty claw, tap **L2**. The claw moves to MATCHLOAD and opens. Load the cup, then tap **L1** to clamp and go to level 1, same as above.

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
