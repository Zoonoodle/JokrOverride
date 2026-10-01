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
| `include/subsystems/intake.*` | One-motor roller intake. Driver-controlled only. |
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
- **Lift sensing**: rotation sensor port `16` is what the PID reads, and it sits on the **motored 12T axle**, so **arm degrees = sensor degrees ÷ 4**. Its absolute zero is calibrated (on the brain's device screen) to the true bottom of the lift and survives power cycles on its own — `Lift::zero()` seeds the position counter from that absolute angle instead of resetting it, so never add back a `reset_position()`/`set_position(0)` there.
- **Claw + pivot**: two pneumatic cylinders, ADI `C` (claw) and ADI `D` (pivot). Pivot only flips out above `PIVOT_CLEAR` and always comes back in before the lift lowers.
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

1. **Nothing moves at startup.** `initialize()` must not command any mechanism. The lift task
   brakes until a target is explicitly requested. (`17ab865`, `d3efc00`)
2. **Exactly one code path owns the lift motors.** `Lift::update()`, called only by the 10 ms
   background task, is the sole caller of `move_voltage()`/`brake()` on the lift. Never create
   a second `MotorGroup` or `Motor` on the lift ports — that duplicate was the real bug behind
   both the startup bounce and the failing builds. (`399c6d0`, `1de3225`, `d3efc00`)
3. **Manual lift always wins.** Holding D-pad Up/Down feeds voltage into that same loop ahead of
   the PID and cancels any active target immediately. Letting go brakes and holds; it must never
   snap back to an old preset. (L1/L2 are the intake-macro buttons now, not manual lift.)
4. Nothing may block driver control from starting — there is no IMU fitted, so calibration
   runs in a background task and is skipped when no IMU answers. Never make startup wait on
   a sensor that may not be there. (`495bb66`, `1de3225`)

## Tunables

All of them live in `include/config.hpp`: ports, `lift_preset::*` heights, lift and chassis
PID gains, claw/pivot timings, stall and runaway thresholds. `TODO(team): confirm` = never
checked on the robot. `TUNE` = a starting value, not a tuned one. Procedures are in `README.md`
("Lift direction check", "Tuning the claw positions").

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
