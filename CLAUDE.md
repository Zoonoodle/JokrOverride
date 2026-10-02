# JokrOverride — instructions for Claude

VEX V5RC 2026-27 Override competition code for team 57249C ("JOKR").
PROS 4.2.2 · LemLib 0.5.6 · liblvgl 9.2.0 · C++20. The physical robot is with the user
every session, so changes get built, uploaded and tested for real.

## Code map

| File | Owns |
|---|---|
| `include/config.hpp` | **Every** port, ratio, preset and PID gain. Nothing robot-specific goes anywhere else. |
| `include/robot.hpp`, `src/robot.cpp` | The LemLib chassis + the single instance of each subsystem. |
| `include/subsystems/lift.*`, `src/subsystems/lift.cpp` | DR6-4B position PID, gravity feedforward, stall + runaway guards, manual override. **Only** place that writes the lift motors. |
| `include/subsystems/claw.*` | Pneumatic claw + autoclamp state machine. |
| `include/subsystems/intake.*` | One-motor roller intake, with an optional power fraction (`set(mode, power)`, default full). Driver-controlled (R1/R2, full power) **plus** L1's auto-intake sweep (full power) **plus** L2's auto-outtake near the bottom (`L2_OUTTAKE_PCT`, 60%) — `main.cpp` makes one decision per loop; never add a second writer. |
| `include/subsystems/scoring.*` | Intake / matchload / 5-level stacking state machine and its interlocks. |
| `src/screen.cpp`, `include/screen.hpp` | Brain screen sensor readout. |
| `src/main.cpp` | `initialize()` / `autonomous()` / `opcontrol()` and the button map. |
| `README.md` | Port map, button map, tuning procedures. |
| `docs/robot/ROBOT.md` | Hardware description + photos. |

## Hardware

Full description: **`docs/robot/ROBOT.md`**, photos in `docs/robot/photos/`.

- **Drivetrain**: 6 wheels, 2.75" omnis, 450 RPM, gear-driven. Left `-2,-3,-1`, right `8,9,10`.
- **Intake**: front-mounted paddle/roller intake, ONE motor on port `12`, chain-driven. Sweeps cups in at floor level to a handoff in the middle of the robot.
- **Lift**: DR6-4B six-bar, 2 motors (`20, -13`), **12T→48T (4:1)** — motors on the 12T axle, arm on the 48T. Chain-levelled end effector. Down = folded over the chassis = 0 deg; up = swung up and back.
- **Lift sensing**: rotation sensor port `16` is what the PID reads, and it sits on the **motored 12T axle**, so **arm degrees = sensor degrees ÷ 4**. Its absolute zero is calibrated (on the brain's device screen) to the true bottom of the lift and survives power cycles on its own — `Lift::zero()` seeds the position counter from that absolute angle instead of resetting it, so never add back a `reset_position()`/`set_position(0)` **there**. That seed is only unambiguous within ±45 arm deg (±180 sensor deg) of the true bottom; further up and it is wrong by a whole sensor revolution, which is why the arm used to "start up too much".
- **Lift homing**: `Lift::requestHome()` fixes that properly. At the start of the first **enabled** mode (`autonomous()`, or `opcontrol()` if auton didn't) the arm drives down at `LIFT_HOME_MV` until the raw sensor stops changing, and **only if** the sensor's folded absolute angle is then within `LIFT_HOME_ZERO_WINDOW_DEG` of its calibrated zero does it `set_position(0)` there. `Lift::updateHoming()` is the **one** place in the codebase allowed to zero that sensor — never add one anywhere else. If the arm stops somewhere implausible (jammed) or it times out, homing stops pushing, keeps `zero()`'s seed, leaves no target active, and shows `HOME FAIL` with the angle it saw. Holding D-pad Up/Down cancels it for the rest of the run. Watch the brain's `ANGLE` line.
- **Claw + pivot**: two pneumatic cylinders, ADI `D` (**claw**) and ADI `C` (**pivot**). **These were crossed in code until 2026-10-01** — the port the code called the claw was the pivot and vice versa — so every polarity note written before that was derived from the wrong pin. The ports above are the corrected, physical assignment and both polarities were re-derived from scratch; the full evidence chain is in `config.hpp` on each constant. The brain's `CLAW` line shows word + port letter + raw level (`CLAW OPEN D[0]  PIV OUT C[0]` at power-on) so A / X each confirm one constant in seconds. Claw: `CLAW_CLOSED_IS_ADI_HIGH = true` — **the jaws need air to grip, so unpowered/LOW is OPEN** and power-on is `Claw::State::OPEN` (there is no "closed at power-on" state any more; a preloaded pin has to be gripped with A or L1). Pivot: two positions named for where it physically is — **OUT** (`true`, ADI LOW) = swung out, at power-on, at every stack level while scoring, and the default down position after L2 / B; **BACK** (`false`, ADI HIGH) = pulled in toward the intake handoff, only at INTAKE (R1). `PIVOT_OUT_IS_ADI_HIGH = false`, confirmed by two independent observations. There is no height at which the pivot is forbidden to be OUT or BACK; the only height tied to it is the handoff zone `INTAKE .. INTAKE + PIVOT_HANDOFF_ZONE_DEG` (BACK inside it while heading to / at the intake, OUT above it). The old `PIVOT_CLEAR` interlock and L2 descent floor were written for the opposite meaning and are gone — don't bring them back. Neither cylinder is fired at startup: each pin is written to the level it already sits at unpowered.
- **Game elements — two separate things**: a **pin** (solid yellow: hex head, fluted tapered body, flared base skirt) and a **cup** (ribbed tapered vessel, clear or red). The pin drops into the cup. **The claw grips the pin; the intake handles cups.** Stacks go 5 levels (`config::lift_preset::STACK_LEVEL_COUNT`), measured from photos — see `docs/robot/ROBOT.md` "Lift levels".
- **Fixed in `config.hpp`**: there is no IMU on the robot, so `IMU_PORT = 0` (none fitted; it used to be `8`, which is wrong and collides with right-drive port 8) — `robot.cpp` builds no `pros::Imu` and gives LemLib a null IMU pointer. `LIFT_GEAR_RATIO` is `48.0/12.0` and `LIFT_ROTATION_RATIO` is `4.0`, matching the confirmed 4:1 lift gearing above.

## Workflow — every change

1. **Build**: `"/Users/brennenprice/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros" make`
   (`pros` is not on PATH — it is the VS Code extension's copy, v3.5.6. Quote the path, it has spaces.)
2. **Fix every error and warning** before going further. A clean ARM build + link is the bar.
3. **Upload**: `"/Users/brennenprice/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros" upload --slot 1 --after none`
   Slot 1 is what the brain is running; `project.pros` sets no slot, so pass
   it explicitly. `--after none` so the program does not auto-run the moment it lands.
   If no brain is connected, say so once and stop — **do not retry in a loop**.
4. **Commit** only the files you touched, staged by explicit path. Never `git add -A` or
   `git add .`. Message style: imperative subject line, then `- ` bullets explaining the
   *why*, matching `git log`.
5. **Tell the user what to test**: the exact buttons to press, what should move, and which
   brain screen line to watch (`LIFT`, `ROT`, `DRV`, `DIST`, `CLAW`, `TEMP`).

Standing permission: build, upload and commit every time without asking. Do not push.

## Robot safety rules — these are not style preferences

1. **Nothing moves at startup.** `initialize()` must not command any mechanism — including
   `Lift::zero()`, which only seeds a counter. The lift task brakes until a target is explicitly
   requested. (`17ab865`, `d3efc00`)
   **The one deliberate exception is lift homing**, and it is narrow on purpose: it is requested from
   the first *enabled* mode, never from `initialize()`, so it is the field/competition switch and not
   power-on that makes the arm move. It runs as a mode *inside* `Lift::update()` (rule 2 holds), it
   blocks nothing, the manual override cancels it instantly (rule 3 holds), it drives at a moderate
   `LIFT_HOME_MV` rather than full speed, and it refuses to zero the sensor unless the stop is
   plausibly the real bottom. Do not widen it, and do not move it into `initialize()`.
2. **Exactly one code path owns the lift motors.** `Lift::update()`, called only by the 10 ms
   background task, is the sole caller of `move_voltage()`/`brake()` on the lift. Never create
   a second `MotorGroup` or `Motor` on the lift ports — that duplicate was the real bug behind
   both the startup bounce and the failing builds. (`399c6d0`, `1de3225`, `d3efc00`)
3. **Manual lift always wins.** Holding D-pad Up/Down feeds voltage into that same loop ahead of
   everything else and cancels any active target immediately. Letting go brakes and holds; it must
   never snap back to an old preset. (L1/L2 are the intake-macro buttons, not manual lift.)
   **R1/R2 while a pin is CLAMPED** are a second manual lift input (R1 up at `R_LIFT_UP_PCT` through
   the same `driverPower()` path; R2 down through `Lift::descendManual()`, the banded seating descent).
   They obey the same rules — cancel the macro and any target, brake-hold on release — and the D-pad
   still wins over them. Which mode R1/R2 are in is *latched at the press*, so a clamp changing
   mid-press can never swap a lift move for a spinning intake; the brain's `CLAW` line shows
   `R:INTAKE` / `R:LIFT` / `R:wait`.
4. Nothing may block driver control from starting — there is no IMU fitted, so calibration
   runs in a background task and is skipped when no IMU answers. Never make startup wait on
   a sensor that may not be there. (`495bb66`, `1de3225`)

## Tunables

All of them live in `include/config.hpp`. **Everything that decides how the lift feels is in one
block at the top of the lift section, `// ===== LIFT TUNING — start here =====`**: speed caps, the
braking decel, the final-approach knobs, the velocity loop's KP/KI, the hold's KP/KI/KD, the descent
bands, the seating voltage, and R1/R2's clamped-mode speeds. Each line says which way to move it for
which symptom; the order to turn them in is README "Tuning the lift". Also in `config.hpp`: ports,
`lift_preset::*` heights, chassis PID gains, claw/pivot timings and polarity, homing constants, stall
and runaway thresholds, and the `LIFT_TRACE` switch. `TODO(team): confirm` = never checked on the
robot. `TUNE` = a starting value, not a tuned one. Procedures are in `README.md` ("Lift direction
check", "Lift speed and motion", "Tuning the lift", "Capturing a trace", "Tuning the claw positions").

**Lift motion, in one line:** there are two kinds of commanded move and one manual input, all inside
the single 10 ms `Lift::update()`. A **position move** (everything except a trip to the bottom) is
DRIVE (full ±12000 mV off the line, like the D-pad) → BRAKE (a velocity loop tracks
`min(cruise cap, sqrt(2·LIFT_DECEL_DEG_PER_S2·d))`) → APPROACH (inside `LIFT_APPROACH_ZONE_DEG` the
reference becomes `LIFT_APPROACH_KP_PER_S × degrees left`, clamped between
`LIFT_APPROACH_MIN_DEG_PER_S` and `LIFT_APPROACH_DEG_PER_S` — the floor is load-bearing, without it
the tail decays exponentially and dominates the move) → HOLD (a **damped** position hold:
`LIFT_HOLD_KP` + an integral that learns gravity + `LIFT_HOLD_KD` × speed, seeded from the velocity
loop's own integral). **Those first three are chosen fresh from (distance, speed) every tick, not
latched** (hysteresis: `LIFT_DRIVE_RESUME_DEG_PER_S`); only arrival into HOLD latches. That is what
makes rapid L1 taps merge: `setTarget()` on a move already in flight *only moves `goal_`* — it must
never restart the move, because that wipes the velocity integral and resets the approach clock. A **descent**
(a non-precise goal at or below `LIFT_BOTTOM_GOAL_DEG`: L2's run home, B, auton DOWN, R2 in clamped
mode) is DESCEND (three position-banded speeds — `LIFT_DESCENT_TOP_PCT`, `_MID_PCT` through the middle,
an **approach cruise** `_SLOW_PCT` (30%, ≈45 deg/s — retuned 2026-10-02, was 12%/18 deg/s) below
`LIFT_DESCENT_SLOW_ZONE_DEG` (14°, was 20°), ramped at `LIFT_DESCENT_RAMP_DEG_PER_S2` (1200, was 800 —
had to get firmer so the arm actually finishes decelerating before the floor instead of still carrying
~70 deg/s into FINAL) → FINAL (below `LIFT_FINAL_START_DEG`, 3° — was `LIFT_SEAT_DETECT_DEG`, 6°; a
dedicated closed-loop push at a constant `LIFT_FINAL_DEG_PER_S` with up to `LIFT_FINAL_MAX_MV` of
downward authority — more force than SEAT's hold, so it can push through a cup caught in the intake or
plain six-bar/chain friction a few degrees short of 0, which is what "stopped under 6°" used to get
mistaken for seated). **FINAL is TIME-BASED (2026-10-02 retune):** it forces at that speed/authority for
a fixed `LIFT_FINAL_FORCE_MS` (1.5 s, "a second or two") regardless of how settled it looks partway
through, then judges once by position — within `LIFT_SEAT_ZERO_DEG` of 0 is plain SEAT, anything more is
`SEAT(short)` — rather than the earlier conditional version (seat early once "close and still", else
give up after a budget), which `LIFT_SEAT_MAX_MS`/`LIFT_SEAT_STILL_*` belonged to and are now gone. →
SEAT (a constant `LIFT_SEAT_MV`, −1500 pressed into the hard stop, so it does not park above the frame
and cannot bounce). An **L2 score** is
a third kind: `Lift::startScore()` → PRESS (push the pin onto the stack with the jaws **shut**, capped
at `L2_PRESS_MAX_MV`) → contact, decided by `subsystems::ContactDetector` **OR** forced by
`L2_AUTO_RELEASE_DEG` (32°, 2026-10-02) the moment PRESS crosses that height with nothing detected —
there is no more "pressed to the bottom with nothing released"; level 1 (31°) is already below it, so
L2 at level 1 releases on the first tick → REL (the lift raises `needsClawRelease()`, shown as `REL(32)`
if the height rule fired it; **Scoring** opens the claw, the lift never touches a cylinder) → the freed
check → UNSTK up `L2_UNSTICK_UP_DEG` and retry if the claw is caught → the descent above at
`L2_RETURN_SPEED_PCT`. **The detector's rule, learned the hard way:** never threshold against the
*nominal command* — the first version did, and could never fire, because this six-bar often cannot make
the commanded descent speed at all (the arm swings up and *back*, so gravity resists the descent over
much of the arc and the press cap limits the push). Every signal is measured against a **learned
baseline of what the arm was actually doing a moment ago**: a leaky integral of the position shortfall
(`LAG`), the loop's effort over the free-descent baseline (`EFFORT`), and a speed-ratio / near-stop
backstop. The baselines track slow gravity drift but **freeze** below `L2_CONTACT_FREEZE_FRAC` so they
cannot follow a gentle landing down and mask it. **AGREEMENT (2026-10-02):** a lone `LAG`/`EFFORT`/
`SPEED` signal is no longer enough — at least two of the three must be over threshold on the *same*
tick (`STALL` is exempt). Fix for the robot false-firing in plain free descent: this six-bar's gravity
torque swings up and back past vertical fast enough to move one signal alone, with nothing under the
arm at all. Since `L2_AUTO_RELEASE_DEG` now guarantees a release regardless, the detector is allowed to
be the more conservative side of that trade. Out of unstick tries → `SCORE STUCK`, holds where it is,
never grinds. Nothing re-drives
or re-chases: the hold *is* the correction. Four earlier designs are in `git log` and must not come
back: `d2978c7` (jumped setpoint, high kP), `6b8165c` (trapezoid + leash + fixed `LIFT_KG`), `3723ef2`
(`LIFT_BRAKE_DECEL_DEG_PER_S2 = 2000` asked for more braking than the arm can do, and handing a
still-moving springy six-bar to the firmware's stiff brake-hold made it ring), and anything that
parks the arm a few degrees above the bottom instead of seating it.

**Keep the README port map and button map in sync** whenever a port or a button changes.

## Troubleshooting

This repo lives under iCloud-synced `~/Documents`, so macOS can evict files from disk. If
`git status`, a read or a build hangs with no output, the cause is almost certainly eviction,
not the command. Check and re-download:

```sh
find . -flags +dataless 2>/dev/null | head
find . -flags +dataless -print0 | xargs -0 -n 20 brctl download
```

Wait until `find . -flags +dataless | wc -l` reads 0, then retry. If any command sits with no
output for more than ~60 s, stop it and report rather than waiting.
