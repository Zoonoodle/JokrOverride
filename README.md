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
  lift.*      DR6-4B lift: DRIVE -> BRAKE -> HOLD moves (full voltage, braking curve, firmware hold),
              homing, stall/runaway guards, plain manual override for D-pad Up/Down, a terminal trace,
              moveTo()/setTarget() shared by driver + auton
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
| **L2** | intake macro: holding a pin → **SCORE** (open the jaws and lower all the way to `DOWN` in one motion; the pivot stays **OUT**) and reset the level count so the next L1 starts at level 1 again |
| **R1** (hold) | intake roller in; (tap) also sends the lift to the **INTAKE** handoff position, pivot **BACK** once it is there |
| **R2** (hold) | intake roller out |
| **B** | **stow**: pivot **OUT**, lift straight down to `DOWN` (0°), cancel the intake macro |
| **A** | claw clamp / open |
| **X** | pivot **OUT** / **BACK** (the brain's `CLAW` line shows `PIV OUT[0]` / `PIV BACK[1]`, word + raw ADI level) |
| **Y** | autoclamp on / off |

**Pivot positions, in the driver's words:** **OUT** = swung out -- where it sits at power-on, at every
stack level while scoring, and in the default down position after L2 / B. **BACK** = pulled in toward
the intake handoff, only for taking a pin from the intake (R1). In code `true` = OUT. Which ADI level
is OUT is `PIVOT_OUT_IS_ADI_HIGH` (`false`: the unpowered LOW level is OUT, because that is where the
pivot sits at power-on). If X makes the pivot swing BACK while the `CLAW` line says `OUT`, flip that one
constant. The pivot swings BACK → OUT during the L1 rise once the arm is `PIVOT_OUT_CLEARANCE_DEG` above
`INTAKE` (or has arrived at the level); nothing else in the lift's motion waits on it.

`L1`/`L2`/R1's tap-to-INTAKE are locked out while Up/Down are held, so a tap can never re-command the
lift out from under a manual nudge (see interlock 5 in `subsystems/scoring.hpp`). The manual override
doesn't touch the claw or pivot, either -- those stay exactly where they physically are.

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
`setTarget(height, precise)` is the lower-level, non-blocking version (used internally, and by
`scoring.cpp`). Every move is DRIVE → BRAKE → HOLD (see "Lift speed and motion" below). `precise = true`
gets the tight tolerance and slow creep that `INTAKE` needs; any goal within half a degree of `INTAKE`
is promoted automatically.

A blocking `moveTo(..., true)` at the top of a routine also waits out lift homing (see "Lift homing"),
so give it time for homing plus the move.

## Lift direction check (do this first)
The lift runs off the **rotation sensor** on port 16, mounted on the 12T motor axle (confirmed, geared 4:1 into the 48T lift-arm axle -- `LIFT_GEAR_RATIO` / `LIFT_ROTATION_RATIO` are both `4.0`). Everything in the code -- presets, PID target/error, the brain's `LIFT` lines -- works in **lift-arm degrees**; that 4:1 is what converts the sensor/motor side down to that one shared unit. If the sensor and the motors disagree on which way is up, the PID pushes the wrong way, the error grows, and the lift drives itself down. That's the "randomly spinning down" problem. To check:
1. Start the program with the lift fully down, and **don't enable**. The brain's `ROT` line shows the rotation sensor and motor encoders side by side, already converted to lift-arm degrees, with the raw sensor/motor degrees in brackets.
2. Lift the arm up by hand. **Both (unbracketed) readings must go positive**, and should read about the same as each other. The bracketed raw sensor reading should be about 4x the bracketed raw motor reading (the 4:1) -- if it isn't, `LIFT_GEAR_RATIO`/`LIFT_ROTATION_RATIO` don't match the real gearing, fix those first.
   - the motor reading goes negative → the lift motors are reversed. Flip the signs in `LIFT_PORTS`.
   - the rotation reading goes negative → set `LIFT_ROTATION_REVERSED = true`.
3. Enable and hold **D-pad Up**. The arm must go up. If it goes down, the motor signs are wrong (go back to step 2).

This has already been checked once, from 7 reference photos with the robot powered on and the arm moved by hand through every named height (`docs/robot/photos/levels/`, see `docs/robot/ROBOT.md` "Lift levels") -- the readings rose the whole way from `DOWN` to the top score level, confirming `LIFT_ROTATION_REVERSED = false` is correct. Still worth re-running this check after any work on the lift's mechanical build.

Because of the 4:1, the sensor turns more than a full revolution (~420° at the sensor for ~105°+ of arm travel) -- the code reads it with `Rotation::get_position()` (multi-turn, keeps counting past 360°), never `get_angle()` (wraps at 360° and would make the lift think it jumped positions partway up). This is already handled in `subsystems/lift.cpp`; it's noted here so nobody "fixes" it back to `get_angle()` later.

A runaway guard catches this if it happens anyway: during DRIVE the lift pushes full voltage toward the goal, so if the distance to the goal *grows* by `LIFT_FAULT_DEG` it cuts power and shows `LIFT FAULT` on the brain's `LIFT` line. The manual override (D-pad Up/Down) or any new preset button clears the fault.

## Lift speed and motion
**The arm's free speed is 150 deg/s.** Both lift motors are red (100 RPM) cartridges; through the
12T -> 48T (4:1) reduction that is 100 / 4 = 25 arm RPM = 150 arm deg/s. `LIFT_FREE_SPEED_DEG_PER_S`
in `config.hpp` records it. Everything below lives in `include/config.hpp`.

**Every preset move is DRIVE → BRAKE → HOLD.** There is no position PID, no setpoint profile and no
gravity feedforward. Each 10 ms tick the lift looks at the arm's measured position and velocity and
does one of three things:

- **DRIVE.** Full ±12000 mV toward the goal, from the very first tick -- exactly what the D-pad does --
  until the arm is going as fast as the braking curve allows for the distance left:
  `v = sqrt(2 · LIFT_BRAKE_DECEL_DEG_PER_S2 · (distance − LIFT_BRAKE_MARGIN_DEG))`.
- **BRAKE.** A velocity loop (`LIFT_VEL_KP` / `LIFT_VEL_KI`, clamped to `LIFT_BRAKE_MAX_MV`) follows
  that curve down to a stop. Its integral learns whatever voltage gravity needs at that angle, so the
  six-bar's torque curve never has to be known in advance. A normal goal is never chased back the way
  the arm came: a small overshoot is simply stopped. Once the arm is slower than
  `LIFT_STOPPED_DEG_PER_S` and inside `LIFT_TOLERANCE_DEG` (5 deg -- a deliberate trade of accuracy for
  a stop that cannot hunt) → HOLD.
  `INTAKE` (precise) and `DOWN` use the same loop with a floor: the curve is offset by
  `LIFT_CREEP_ZONE_DEG` so the arm is down to `LIFT_CREEP_DEG_PER_S` that far out, then **creeps** until
  it is inside `LIFT_PRECISE_TOLERANCE_DEG` (INTAKE, from either side) or `LIFT_REST_BAND_DEG` above the
  hard stop (DOWN -- nothing ever drives the arm into the frame).
- **HOLD.** `brake()` with the motors in hold mode, every tick -- the same firmware hold that catches
  the arm when you let go of the D-pad. Nothing in this phase pushes toward the goal, so nothing can
  hunt. **Arrival latches**: HOLD only goes back to DRIVE if the arm is more than `LIFT_REENGAGE_DEG`
  off, at most `LIFT_MAX_REDRIVES` times per button press. The number of pushes per command is finite.

**The one number that sets where it stops is `LIFT_BRAKE_DECEL_DEG_PER_S2`** -- how hard the arm can
really brake. Under the real capability = stops short (safe), over = overshoots. Read it off the trace.

> Two earlier designs are documented in `git log` so nobody brings them back: a jumped setpoint with
> `kP = 600` (oscillated, `d2978c7`), and a trapezoidal profile + leash + velocity feedforward + PID +
> gravity term + arrive/hold hysteresis (`6b8165c`: slow off the line because the leash capped the net
> drive at ~3000 mV, and still oscillated because its hold relied on a fixed gravity feedforward that is
> wrong at most angles of a six-bar -- the arm drifted out of the band, got re-chased, drifted again).

### Capturing a trace
`LIFT_TRACE = true` prints one integer line per 20 ms to the PROS terminal whenever the arm moves, and
for half a second after. In VS Code: PROS sidebar → **Brain Terminal** (or `pros terminal` from a
shell), press a lift button, copy the lines out. Columns:

```
LIFT <ms> <phase> <pos*10> <goal*10> <vel deg/s> <out mV>
```

`phase` is `D` drive, `B` brake, `C` creep, `H` hold, `M` manual D-pad, `h` homing, `F` fault, `I` idle.
`goal*10` is `-1` when nothing is commanded. The lines are written by a separate low-priority task from a
ring buffer; the control loop itself never prints. In the `B` lines, `vel` should fall by about
`LIFT_BRAKE_DECEL_DEG_PER_S2 × 0.02` per line -- if it falls much faster the arm can brake harder than
the constant says (raise it); if `pos` runs past `goal` before `vel` reaches 0, lower it.

### If it...

| Symptom | Change |
|---|---|
| Stops short of every preset by the same few degrees | raise `LIFT_BRAKE_DECEL_DEG_PER_S2` (braking starts later). Up to 5 deg short is `LIFT_TOLERANCE_DEG` working as designed |
| Overshoots a level / arrives still moving / taps the frame at `DOWN` | lower `LIFT_BRAKE_DECEL_DEG_PER_S2`; for `DOWN` also raise `LIFT_CREEP_ZONE_DEG` |
| `vel` swings above and below the curve in the trace during `B`/`C` (hunts) | lower `LIFT_VEL_KP` |
| Creep is slow to get going / sags before it starts moving | raise `LIFT_VEL_KI` |
| Brake feels violent | lower `LIFT_BRAKE_MAX_MV` (expect it to stop a little later: lower `LIFT_BRAKE_DECEL_DEG_PER_S2` to match) |
| Small spring-back as it grabs the hold | lower `LIFT_STOPPED_DEG_PER_S` (hands over at a slower crawl) |
| Re-drives once or twice after a stop (`redrive 1` on the brain, `H` then `D` in the trace) | the brake missed by more than `LIFT_REENGAGE_DEG`: fix `LIFT_BRAKE_DECEL_DEG_PER_S2` first; raise `LIFT_REENGAGE_DEG` only if you would rather it just stayed put |
| Parks too high above the frame after `DOWN` | lower `LIFT_REST_BAND_DEG` (1.5) |
| Lag between L1's *first* tap and the arm moving | that is `CLAW_CLOSE_TIME_MS` -- the jaws must shut before the arm pulls away |
| `BLOCKED` on a normal move | DRIVE or CREEP ran `LIFT_STALL_MS` without `LIFT_STALL_DEG` of progress -- look for a real jam or a preset above the real top before raising `LIFT_STALL_MS` |
| `LIFT FAULT` on a normal move | the distance to the goal grew during DRIVE: run the direction check above before touching `LIFT_FAULT_DEG` |
| A button seems to do nothing | read the brain's `CLAW` line first. `CLAMPED` makes R1 a no-op (interlock 4), and `MACRO STACKING` means the macro thinks it is holding a pin. `B` resets the macro; `A` opens the jaws |

## Lift homing (automatic, every program run)
The arm often powers on sitting well above the bottom. `Lift::zero()` can only seed the position
counter from the rotation sensor's folded absolute angle, which is unambiguous **only within ±45 arm
deg (±180 sensor deg)** of the true bottom — past that every preset is off by a whole sensor
revolution. So the lift finds its own hard stop instead.

**When:** at the start of the first *enabled* mode — `autonomous()`, or `opcontrol()` if auton didn't
run. Never in `initialize()`, so power-on alone never moves the arm; it takes the field/competition
switch. A match therefore homes exactly once, during auton.

**What happens:** the arm drives down at `LIFT_HOME_MV` (a moderate steady voltage, not full speed —
it ends against the frame) until the raw sensor **stops changing**: less than `LIFT_HOME_STILL_DEG` of
*sensor* movement across a `LIFT_HOME_STILL_MS` window, only checked after `LIFT_HOME_MIN_DRIVE_MS` of
driving so a sticky start isn't mistaken for the bottom. Small wobble at the stop is allowed. At the
stop it reads the sensor's folded absolute angle, and **only if** that is within
`LIFT_HOME_ZERO_WINDOW_DEG` of the calibrated zero does it `set_position(0)` and brake-hold at the
bottom. `Lift::updateHoming()` is the only place in the codebase allowed to zero that sensor.

**It never blocks.** Homing is a mode inside `Lift::update()`, so the 10 ms task is still the only
thing writing the lift motors, driving starts immediately, and a lift target asked for during homing
(L1/L2/R1/B, or an auton `moveTo`) is *queued* and runs the moment homing ends. A blocking
`moveTo(..., true)` waits for homing and then the move.

**Safety nets.** Holding D-pad Up/Down cancels homing instantly and it is not retried that run
(`HOME CANCELLED`); the position stays as `zero()` seeded it. If the arm stops somewhere implausible
it is jammed, so homing does **not** zero there — it stops pushing, leaves no target active, keeps the
seed and shows `HOME FAIL`. A `LIFT_HOME_TIMEOUT_MS` timeout does the same. No rotation sensor
answering gives `HOME SKIPPED` and no motion at all. The stall and runaway guards are not reached
while homing, because driving into a hard stop is homing's whole job.

**Watch the brain's `ANGLE` line:** `HOMING` -> `HOMED @ 0.4`, or `HOME FAIL @ 37.2`. The number is
the absolute angle it saw at the stop — the one the plausibility window was checked against.

| Symptom | Change |
|---|---|
| Stops partway down and says `HOME FAIL` with a large angle | it is catching on something — check the mechanism. If the stop is real but the sensor's calibrated zero is off, re-calibrate the zero on the brain's device screen |
| `HOME FAIL` with a small angle (just outside the window) | raise `LIFT_HOME_ZERO_WINDOW_DEG` |
| Stops early, before the bottom | raise `LIFT_HOME_MIN_DRIVE_MS`, raise `LIFT_HOME_STILL_MS`, or lower `LIFT_HOME_STILL_DEG` |
| Grinds at the bottom before noticing | lower `LIFT_HOME_STILL_MS`, raise `LIFT_HOME_STILL_DEG`, or lower the drive with `LIFT_HOME_MV` |
| Too slow / times out | raise `LIFT_HOME_MV` magnitude (more negative) or `LIFT_HOME_TIMEOUT_MS` |

## Tuning the claw positions
All heights are **lift-arm degrees** in `config::lift_preset` (`include/config.hpp`). 0 is where the lift sits when the program starts. `DOWN`, `INTAKE` and the 5 `STACK_LEVELS` are already measured from reference photos (see `docs/robot/ROBOT.md` "Lift levels") -- `MATCHLOAD` is still a placeholder, and `INTAKE` (1.1) almost certainly needs re-measuring. To (re)measure any of them:
1. Start the program with the lift fully down.
2. Use **D-pad Up/Down** to put the claw exactly where you want it, then read the brain's `LIFT` line for the current position.
3. Type that number into the preset:
   - `INTAKE`: claw lined up with the intake handoff
   - `MATCHLOAD`: claw at matchload height
   - `STACK_LEVELS[0..4]`: the height to drop the 1st through 5th pin on a stack
   - `MAX`: a few degrees above the top level, as a software hard stop -- never raise it just to let a bad preset through
   - **Measure the top first.** Hold Up until the arm is at the top and read the position. Every preset has to be below that number. If the lift shows `BLOCKED` (brain + controller), it was asked to go somewhere it can't physically reach, so it stopped pushing and is holding where it is. Lower that preset.
4. Timing: `CLAW_CLOSE_TIME_MS` (grip before lifting), `CLAW_OPEN_TIME_MS` (release before lowering -- `scoreOrMatchload()`'s path only; L2's `score()` opens the claw and starts lowering in the same tick), `PIVOT_TIME_MS`, `PIVOT_OUT_CLEARANCE_DEG` (how far above `INTAKE` the arm is when the pivot swings OUT during the L1 rise), and the braking constants -- see "Lift speed and motion" above. `CLAW_CLOSE_TIME_MS` is specifically the gap between the first L1 tap and the arm moving: the jaws must shut before the arm pulls away.

**Lift braking** (`LIFT_BRAKE_DECEL_DEG_PER_S2` first, then `LIFT_VEL_KP` / `LIFT_VEL_KI`), with a trace open (see "Capturing a trace"):
1. **Decel:** tap L1 from the bottom (level 1) and B back down. In the `B` lines, note how fast `vel` really falls, and where `pos` ends up relative to `goal`. Stops short every time → raise `LIFT_BRAKE_DECEL_DEG_PER_S2`; runs past → lower it. Repeat until stack levels land within `LIFT_TOLERANCE_DEG` and `DOWN` creeps the last `LIFT_CREEP_ZONE_DEG`.
2. **Velocity loop:** if `vel` oscillates around the curve during `B`/`C`, lower `LIFT_VEL_KP`; if the creep toward `INTAKE` is slow to reach `LIFT_CREEP_DEG_PER_S`, raise `LIFT_VEL_KI`.
3. **INTAKE:** tap R1 and confirm the `C` lines end with `pos` within `LIFT_PRECISE_TOLERANCE_DEG × 10` of `goal`.

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
| ANGLE | the rotation sensor's live raw absolute angle, the startup seed, and the homing state (`HOMING` / `HOMED @ n` / `HOME FAIL @ n`) |
| CLAW | claw state (`OPEN`/`ARMED`/`CLAMPED`/`RELEASED`), pivot `PIV OUT[0]` / `PIV BACK[1]` (word + raw ADI level), intake-macro state and stack level |
| LIFT (position line) | position → target (or `none`), the active named preset (or `custom`), the phase (`DRIVE`/`BRAKE`/`CREEP`/`HOLD`), and `SETTLED`/`BLOCKED`/`FAULT` |
| LIFT (err/vel/out line) | error in lift degrees, measured arm speed in deg/s, and what was sent to the motors in mV (0 = brake-hold) |
| LIFT (curve line) | the braking curve's allowed speed at this distance, how many times this command re-drove, then `DIST`: claw distance sensor mm and `PIN` when inside `CLAW_DETECT_MM` |
| BRAKE | `LIFT_BRAKE_DECEL_DEG_PER_S2`, `LIFT_BRAKE_MARGIN_DEG`, `LIFT_CREEP_DEG_PER_S`, `LIFT_TOLERANCE_DEG` read straight from `config.hpp` -- tuning happens in code, not on the controller |
| LIFT (temp/current line) | lift motor temp (°C) and current draw (mA) |
| TEMP | hottest left drive / right drive / lift motor and the intake motor (C). Motors lose power around 55 C. |
| DRV | driver-loop heartbeat: `not started`, or a loop counter plus stick values and the manual-override Up/Down |
