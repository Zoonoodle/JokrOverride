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
  intake.*    three-stage roller intake
  scoring.*   intake → scoring pivot state machine with interlocks
src/main.cpp                  initialize / autonomous stub / opcontrol + button map
docs/notebook/                notebook-ready write-ups, flowcharts and blank test tables
```

## Port map
| Device | Port | Status |
|---|---|---|
| Left drive motors | 1, 2, 3 (reversed) | TODO(team): confirm ports + 55W motor split |
| Right drive motors | 4, 5, 6 | TODO(team): confirm |
| Odometry pod (rotation sensor) | 7 | TODO(team): confirm. **One pod is confirmed** (Best Solution \| Odometry, 9/18/26) |
| Inertial sensor | 8 | TODO(team): confirm |
| Lift motors | 9, 10 (reversed) | TODO(team): confirm. **2 motors, 12T → 36T is confirmed** (Build \| Lift Drive, 9/10/26) |
| Lift rotation sensor | none | none in the notebook; the code falls back to motor encoders |
| Claw distance sensor | 11 | TODO(team): not on the robot yet; needed for autoclamp |
| Intake motors | 12, 13 (reversed) | TODO(team): confirm count/ports |
| Claw cylinder | ADI A | TODO(team): confirm. **One cylinder is confirmed** (Improve \| Claw, 9/16/26) |
| Pivot cylinder | none | TODO(team): is there a separate pivot? |

**Confirmed from the notebook:** 2.75" omni wheels, 450 RPM, a 55W drivetrain, a DR6-4B with 2 motors geared 12T → 36T, a single-cylinder pneumatic claw, one odometry pod, and LemLib as the library.

## Button map
| Button | Action |
|---|---|
| Left stick Y / Right stick X | arcade drive |
| R1 | score HIGH (only while holding a pin) |
| R2 | score LOW |
| L1 | drop the pin (only once the lift has settled at the goal) |
| L2 | claw manual clamp / release |
| A | cancel → back to intaking |
| Up / Down (hold) | lift manual override; the lift holds where you let go |
| B (hold) | intake reverse (unjam) |
| Y | autoclamp on / off |

## Brain screen
The brain screen shows the robot's X/Y/heading, the lift angle vs its target and whether it's SETTLED, the claw state, and the scoring state. Take photos of it for the notebook and use it while tuning.
