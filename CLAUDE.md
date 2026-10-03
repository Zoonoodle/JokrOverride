# JokrOverride — instructions for Claude

VEX V5RC 2026-27 Override competition code for team 57249C ("JOKR").
PROS 4.2.2 · LemLib 0.5.6 · liblvgl 9.2.0 · C++20. The physical robot is with the user
every session, so changes get built, uploaded and tested for real.

## Code map

| File | Owns |
|---|---|
| `include/config.hpp` | **Every** port, ratio, preset and PID gain — with ONE documented exception: the chassis PID (`lateral`/`angular` `ControllerSettings`) and the drive curves are typed directly in `src/robot.cpp` at the user's request (2026-10-02) and must not be moved here. Nothing else robot-specific goes anywhere else. |
| `include/robot.hpp`, `src/robot.cpp` | The LemLib chassis + the single instance of each subsystem. |
| `include/subsystems/lift.*`, `src/subsystems/lift.cpp` | DR6-4B position PID, gravity feedforward, stall + runaway guards, manual override. **Only** place that writes the lift motors. |
| `include/subsystems/claw.*` | Pneumatic claw + the distance-sensor autoclamp. **Only** place that writes the claw cylinder. |
| `include/subsystems/autoclamp_detector.hpp` | PURE "is something really in the jaws?" debounce (no PROS, no clock, no cylinder), host-tested in `test/`. Range + confirm window + the clear-to-re-arm rule. |
| `include/subsystems/intake.*` | One-motor roller intake, with an optional power fraction (`set(mode, power)`, default full). Driver-controlled (R1/R2, full power) **plus** L1's auto-intake sweep (full power) **plus** L2's auto-outtake near the bottom (`L2_OUTTAKE_PCT`, 60%) — `periodic::tick()` makes one decision per loop, for driver AND auton; never add a second writer. |
| `include/subsystems/scoring.*` | Intake / matchload / 5-level stacking state machine and its interlocks. |
| `src/screen.cpp`, `include/screen.hpp` | Brain screen: **four lines only** (POSE / LIFT / CLAW / DIST), a START line for the start alignment, and `!!` warning lines when a device is missing or wrong. Every live tuning number lives in the `LIFT_TRACE` Brain Terminal output instead — don't add numbers back to the screen. |
| `include/auton.hpp`, `src/auton.cpp` | **AUTONOMOUS API — don't put routes here.** One readable call per lift / claw / pivot / intake / macro operation, each going through the identical code path the matching driver button uses; the blocking waits (every one takes a timeout and returns success); `begin()`/`end()` for the auton tick; `moveB()`. |
| `include/autons.hpp`, `src/autons.cpp` | **ROUTES — work here.** Every auton route, plus the single `constexpr Route SELECTED` line that picks the one `autonomous()` runs. The only file to open when coding routes. |
| `include/periodic.hpp`, `src/periodic.cpp` | THE per-loop subsystem logic, in **one** copy: L1's auto-intake window, L2's auto-outtake window, the single intake decision, `Claw::update()`, `Scoring::update()`. Run by opcontrol's loop in driver and by the auton tick in auton — never both at once. |
| `src/main.cpp` | `initialize()` / `autonomous()` (common setup + one `autons::run()` call) / `opcontrol()` and the button map. |
| `README.md` | Port map, button map, tuning procedures. |
| `docs/robot/ROBOT.md` | Hardware description + photos. |

## Hardware

Full description: **`docs/robot/ROBOT.md`**, photos in `docs/robot/photos/`.

- **Drivetrain**: 6 wheels, 2.75" omnis, 450 RPM, gear-driven. Left `-8,-2,-1`, right `3,9,10`
  (confirmed 2026-10-02). **The chassis PID (`lateral` / `angular` `lemlib::ControllerSettings`) and
  the `ExpoDriveCurve` driver curves are typed as literal numbers directly in `src/robot.cpp`**, where
  the driver tunes them — the user's own decision, 2026-10-02 ("i want to be able to tune and enter
  them in here"). **Do not move them into `config.hpp`.** They are the one documented exception;
  everything else robot-specific still lives in `config.hpp`.
- **Odometry**: ONE vertical tracking-wheel pod on rotation port `17`, **plus an IMU on port `18`**
  (fitted and confirmed 2026-10-02). Heading comes from the IMU, distance from the pod. Calibration
  runs in a background task from `initialize()`; `opcontrol()` never waits on it, `autonomous()` waits
  but only up to `AUTON_CALIBRATE_TIMEOUT_MS`.
- **Distance sensors**: four, all confirmed 2026-10-03 — **left `15`**, **right `19`**, **rear `14`**
  (`LEFT_/RIGHT_/BACK_DISTANCE_PORT`) and the one inside the claw on `11`. All four are on the brain's
  `DIST` line. The REAR one is the whole feedback loop for `auton::moveB()`, and while it reads out of
  range `moveB` drives blind at full commanded speed until the timeout. The LEFT one is the
  start-of-auton x correction (`auton::startPose()`, see README "Start alignment"): the robot's
  distance along the front wall is the one thing that is not repeatable at setup, so a route's first
  line measures it — further than expected = shifted +x, closer = −x — with a
  `START_ALIGN_MAX_IN` guard so a sensor seeing a goal can never teleport the pose. The right one is
  read by the screen only, so far.
- **Intake**: front-mounted paddle/roller intake, ONE motor on port `12`, chain-driven. Sweeps cups in at floor level to a handoff in the middle of the robot.
- **Lift**: DR6-4B six-bar, 2 motors (`20, -13`), **12T→48T (4:1)** — motors on the 12T axle, arm on the 48T. Chain-levelled end effector. Down = folded over the chassis = 0 deg; up = swung up and back.
- **Lift sensing**: rotation sensor port `16` is what the PID reads, and it sits on the **motored 12T axle**, so **arm degrees = sensor degrees ÷ 4**. Its absolute zero is calibrated (on the brain's device screen) to the true bottom of the lift and survives power cycles on its own — `Lift::zero()` seeds the position counter from that absolute angle instead of resetting it, so never add back a `reset_position()`/`set_position(0)` **there**. That seed is only unambiguous within ±45 arm deg (±180 sensor deg) of the true bottom; further up and it is wrong by a whole sensor revolution, which is why the arm used to "start up too much".
- **Lift homing is OFF** (`config::LIFT_HOME_ON_ENABLE = false`, the driver's call 2026-10-03: the arm
  is set at its hard stop every time, so `zero()`'s absolute-angle seed is already right and a route
  should start lifting immediately). `Lift::requestHome()` records `HOME SKIPPED` and moves nothing —
  gated in that one function, so **autonomous and opcontrol are both covered**; gating it in auton
  alone would have made opcontrol's call live and driven the arm down at the start of driver control.
  Set it back to `true` if the arm is ever powered on more than ~45 arm deg above the bottom. The rest
  of this bullet describes what homing does when it IS enabled, and is unchanged:
- **Lift homing**: `Lift::requestHome()` fixes that properly. At the start of the first **enabled** mode (`autonomous()`, or `opcontrol()` if auton didn't) the arm drives down at `LIFT_HOME_MV` until the raw sensor stops changing, and **only if** the sensor's folded absolute angle is then within `LIFT_HOME_ZERO_WINDOW_DEG` of its calibrated zero does it `set_position(0)` there. `Lift::updateHoming()` is the **one** place in the codebase allowed to zero that sensor — never add one anywhere else. If the arm stops somewhere implausible (jammed) or it times out, homing stops pushing, keeps `zero()`'s seed, leaves no target active, and shows `HOME FAIL` with the angle it saw. Holding D-pad Up/Down cancels it for the rest of the run. Watch the `LIFT` line on the brain (`HOMING` / `HOMED` / `HOME FAIL @ <angle>`).
- **Claw + pivot**: two pneumatic cylinders, ADI `D` (**claw**) and ADI `C` (**pivot**). **These were crossed in code until 2026-10-01** — the port the code called the claw was the pivot and vice versa — so every polarity note written before that was derived from the wrong pin. The ports above are the corrected, physical assignment and both polarities were re-derived from scratch; the full evidence chain is in `config.hpp` on each constant. The brain's `CLAW` line shows the words (`CLAW  OPEN  AC:ARMED  PIV OUT`); the port letters and raw levels it used to carry were removed on 2026-10-03 once both polarities were confirmed. Claw: `CLAW_CLOSED_IS_ADI_HIGH = true` — **the jaws need air to grip, so unpowered/LOW is OPEN** and power-on is `Claw::State::OPEN` (there is no "closed at power-on" state any more; a preloaded pin has to be gripped with A or L1). Pivot: two positions named for where it physically is — **OUT** (`true`, ADI LOW) = swung out, at power-on, at every stack level while scoring, and the default down position after L2 / B; **BACK** (`false`, ADI HIGH) = pulled in toward the intake handoff, only at INTAKE (R1). `PIVOT_OUT_IS_ADI_HIGH = false`, confirmed by two independent observations. There is no height at which the pivot is forbidden to be OUT or BACK; the only height tied to it is the handoff zone `INTAKE .. INTAKE + PIVOT_HANDOFF_ZONE_DEG` (BACK inside it while heading to / at the intake, OUT above it). The old `PIVOT_CLEAR` interlock and L2 descent floor were written for the opposite meaning and are gone — don't bring them back. Neither cylinder is fired at startup: each pin is written to the level it already sits at unpowered.
- **Autoclamp**: the distance sensor is **mounted in the claw, on port `21`** (`CLAW_DISTANCE_PORT`,
  confirmed 2026-10-03 by the user — it was `11` before that) and sees both pins and cups. While the jaws are open and the autoclamp is
  enabled, anything at or inside the trigger range for `AUTOCLAMP_CONFIRM_MS` (50) over
  at least `AUTOCLAMP_MIN_SAMPLES` (2) **fresh** readings closes the jaws. **The range differs by mode**,
  all three measured 2026-10-03: `AUTOCLAMP_RANGE_DRIVER_MM` **55**, `AUTOCLAMP_RANGE_AUTON_MM` **65**,
  `AUTOCLAMP_INTAKE_RANGE_MM` **75** at the R1 handoff in either mode. Auton reaches further because
  nothing is aiming the claw there; the handoff further still because the intake pushes the pin *across*
  the jaws. Which is in force is PASSED every tick (`periodic::Inputs::auton` →
  `Claw::setAutonMode()`, `Scoring::isAtIntakeHandoff()` → `setIntakeMode()`), never latched, so a field
  switch cannot leave it wrong — see `Claw::activeRangeMm()`. The rule is a pure class
  (`subsystems::AutoclampDetector`, host-tested); `Claw::update()` owns the cylinder. Fresh samples are
  counted as well as milliseconds because the V5 sensor refreshes ~30 ms, slower than the tick, so one
  lucky reading can otherwise look like 50 ms of evidence. **Re-arm is clear-based, not a timer**:
  after ANY open the autoclamp is disarmed until the sensor reads clear — beyond **(the range in force +
  `AUTOCLAMP_REARM_GAP_MM`, 30)** — for `AUTOCLAMP_REARM_CLEAR_MS`, because the pin just released is still
  millimetres away and a timer re-grabs it every time. **The clear threshold is DERIVED, never a separate
  constant**, and that is deliberate: a clear threshold at or below the range in force makes anything
  between the two read as both clear AND in range, so the claw re-grabs what it just released forever and
  no score can finish. With three ranges the only way to keep that ordering true is to compute it. (The
  old absolute `AUTOCLAMP_REARM_MM` nearly caused exactly this when the range went 15 → 50; it is gone.)
  The gap is `static_assert`ed positive in `Claw::detectorConfig()` and the broken ordering has a host
  test. `CLAW_RELEASE_LOCKOUT_MS` is kept on top as a floor. It is inhibited for the
  whole L2 score. **ON by default in BOTH modes** — driver (`AUTOCLAMP_ON_IN_DRIVER`, re-enabled each
  `opcontrol()`, Y toggles) and **auton** (`AUTOCLAMP_ON_IN_AUTON`, applied by `autonomous()` since
  2026-10-03; `auton::autoclamp(false)` is the per-route switch). OFF at power-on. It is **entirely
  asynchronous** — `periodic::tick()` does the watching in both modes — so nothing calls it and nothing
  blocks on it; `auton::waitUntilClamped()` still exists but is **optional**, only for "drive into a pin
  and stop at the clamp". The brain's `CLAW` line shows
  `AC:OFF/ARMED/SEEING/CLEAR?/HELD`, and the raw mm are the `CLAW` value on the `DIST` line. Note the
  sensor's specified minimum range is ~20 mm, so 15 mm is the first number to question if it never fires.
- **Every clamp moves the lift**, in exactly one place: `Scoring::handleClampEvent()`, run at the top of
  `Scoring::update()`. `Claw` raises a one-shot `takeClampEvent()` (with a `ClampSource`: MANUAL or
  AUTOCLAMP) on any transition into CLAMPED; the handler **always consumes it** so nothing can queue, and
  `periodic::reset()` drops it at the start of each enabled mode so `competition_initialize()`'s pre-match
  grip can never make the arm move when auton starts. Two outcomes:
  - **`CLAMP_RISE_DEG` (5, TUNE) up**, via `Lift::startClampRise()` — brain `CLAMP+5`. RELATIVE and
    upward only (`position + 5`, clamped to `MAX`). It is a **precise** move and that is load-bearing: a
    5° goal is inside `LIFT_TOLERANCE_DEG` (5) and inside `setTargetFast()`'s `LIFT_COARSE_TOLERANCE_DEG`
    (10), so a non-precise move would latch "arrived" on the first tick and never move. Fires for A, L2's
    at-the-bottom clamp toggle, `auton::clawGrip()`, and the autoclamp anywhere but the handoff.
    **Skipped** at the R1 handoff, during an L1 GRAB, anywhere in an L2 score cycle, and whenever a lift
    move is in flight or a Scoring macro owns the lift — **a commanded move always wins**, which is what
    keeps a route's `clawGrip(); liftToFrontToggle();` from being hijacked.
  - **At the R1 INTAKE handoff** (`Scoring::isAtIntakeHandoff()`, fed to `Claw::setIntakeMode()` every
    tick) the autoclamp reaches **further** (`AUTOCLAMP_INTAKE_RANGE_MM` 75, against 55/65 — the intake
    pushes the pin *across* the jaws there, so the claw has to commit while it is still arriving) and
    fires on the **first** valid in-range reading
    (`AUTOCLAMP_INTAKE_CONFIRM_MS` 0 / `AUTOCLAMP_INTAKE_MIN_SAMPLES` 1 — no debounce; the pure detector is
    unchanged, only its Config numbers) and `Lift::startClampPush(AUTOCLAMP_INTAKE_PUSH_MS)` forces the
    claw **down** at `L1_GRAB_PUSH_PCT` for 300 ms, then holds there. It is the same GRAB mechanism, so
    the intake stops for the push and the D-pad cancels it; there is no grip request inside it because the
    jaws are already shut. A following L1 from the handoff still grabs and rises to level 1 normally.
- **Game elements — two separate things**: a **pin** (solid yellow: hex head, fluted tapered body, flared base skirt) and a **cup** (ribbed tapered vessel, clear or red). The pin drops into the cup. **The claw grips the pin; the intake handles cups.** Stacks go 5 levels (`config::lift_preset::STACK_LEVEL_COUNT`), measured from photos — see `docs/robot/ROBOT.md` "Lift levels".
- **Fixed in `config.hpp`**: `LIFT_GEAR_RATIO` is `48.0/12.0` and `LIFT_ROTATION_RATIO` is `4.0`,
  matching the confirmed 4:1 lift gearing above. (The old "there is no IMU, `IMU_PORT = 0`" note is
  history: an IMU is fitted on `18` — see the Odometry bullet. `IMU_PORT = 0` is still the supported
  way to run on the pod alone.)

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
   brain screen line to watch — there are only four: `POSE`, `LIFT`, `CLAW`, `DIST` (plus `START` for
   the start alignment, and `!!` warning lines). Anything finer-grained is in the Brain Terminal's
   `LIFT_TRACE` output, not on the screen.

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
   everything else and cancels any active target immediately. The magnitudes are
   `LIFT_MANUAL_UP_PCT` / `LIFT_MANUAL_DOWN_PCT` (100% / **50%** — down is half speed, the driver's call
   2026-10-03, because gravity already helps downward and the travel ends on the hard stop). They affect
   the D-pad ONLY: clamped-mode R1/R2 keep their own `R_LIFT_*_PCT`, and a commanded move's DRIVE phase
   is still full `LIFT_MAX_MV`. Letting go brakes and holds; it must
   never snap back to an old preset. (L1/L2 are the intake-macro buttons, not manual lift.)
   **R1/R2 while a pin is CLAMPED** are a second manual lift input (R1 up at `R_LIFT_UP_PCT` through
   the same `driverPower()` path; R2 down through `Lift::descendManual()`, the banded seating descent).
   They obey the same rules — cancel the macro and any target, brake-hold on release — and the D-pad
   still wins over them. Which mode R1/R2 are in is *latched at the press*, so a clamp changing
   mid-press can never swap a lift move for a spinning intake; the brain's `CLAW` line shows
   `R:INTAKE` / `R:LIFT` / `R:wait`.
4. Nothing may block driver control from starting. **An IMU IS now fitted (port 18)**, and
   calibration still runs in a background task from `initialize()` — `opcontrol()` never waits on it,
   and it is skipped entirely when no IMU answers (`IMU_PORT = 0`). Never make startup wait on a
   sensor that may not be there. `autonomous()` may wait for calibration, but only with a bound
   (`AUTON_CALIBRATE_TIMEOUT_MS`), so an unplugged IMU can never eat the autonomous period.
   (`495bb66`, `1de3225`)

5. **One tick, one owner.** The per-loop subsystem logic (both intake macro windows, the single
   intake decision, `Claw::update()`, `Scoring::update()`) exists in exactly one copy, in
   `periodic::tick()`. In driver control opcontrol's loop calls it; in autonomous the auton tick task
   (`auton::begin()`, 10 ms) calls it. **They must never both run**: `opcontrol()` and `disabled()`
   both call `auton::end()` first, and `end()` BLOCKS until the tick task has really exited. The lift
   PID is not in there at all — it has its own 10 ms task and remains the only writer of the lift
   motors (rule 2). Do not add a second caller of `periodic::tick()`, and do not write the intake or
   a cylinder from a route: use the `auton::*` API, which routes everything through the same paths the
   driver buttons use.

6. **Routes go in `src/autons.cpp`, nowhere else.** `autonomous()` does the setup a route must not be
   able to forget (auton tick, autoclamp off, lift homing, the bounded calibration wait) and then makes
   exactly one call, `autons::run()`. Lift homing is requested for **every** route.

## Tunables

All of them live in `include/config.hpp`. **Everything that decides how the lift feels is in one
block at the top of the lift section, `// ===== LIFT TUNING — start here =====`**: speed caps, the
braking decel, the final-approach knobs, the velocity loop's KP/KI, the hold's KP/KI/KD, the descent
bands, the seating voltage, and R1/R2's clamped-mode speeds. Each line says which way to move it for
which symptom; the order to turn them in is README "Tuning the lift". Also in `config.hpp`: ports,
`lift_preset::*` heights, claw/pivot timings and polarity, homing constants, stall
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
loop's own integral — with **no P and a FROZEN integral inside `LIFT_HOLD_DEADBAND_DEG`**, 4° at a level
and `LIFT_HOLD_PRECISE_DEADBAND_DEG` 0.5° at INTAKE) → **PARK** (the user's 2026-10-03 request, "id
rather it just settle instead of keeping on trying to do it. neds a timeout": after
`LIFT_PARK_CONFIRM_MS` inside the band or `LIFT_SETTLE_TIMEOUT_MS` of holding, the learned gravity
voltage is **frozen and pushed as a constant** with no P, no D and no integral update, so there is no
loop left to oscillate. The hold only wakes on a `LIFT_REENGAGE_DEG` drift, never within
`LIFT_PARK_COOLDOWN_MS`. A position move that has not arrived within `LIFT_MOVE_TIMEOUT_MS` also stops
chasing and parks where it is, shown as `PARK(t/o)`. This is the fix for the hold's
integral-plus-backlash limit cycle; do not replace it with more gain tuning). **Those first three are chosen fresh from (distance, speed) every tick, not
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
and cannot bounce) → **REST** (`LIFT_REST_AFTER_MS`, 300 ms, after the seated arm goes still: brake mode
**COAST** and 0 V — no PID, no seat push, not even the firmware hold, so the motors draw nothing. The
user asked for this on 2026-10-03, "keep motors loose so they arent being usefd and burn out", and it
reverses "pressed into the hard stop" as the *final* state: the press still happens, it just stops once
the arm is settled. REST counts as seated/settled everywhere. The idle path does the same thing: **no target + at or below
`LIFT_BOTTOM_GOAL_DEG` + still = coast**, not a brake-hold (the driver, 2026-10-03: "during down state
like scored state, it needs to not use arm pid and let the motors just coast and chill"). **Coasting
happens ONLY at the bottom** — the
no-target state and every manual release are still brake-holds, so an arm that is up cannot fall, and
safety rule 1 is unchanged. Because the bands pull UP hardest at the folded position, a resting arm that
creeps past `LIFT_REST_WAKE_DEG` is re-seated and rests again, at most `LIFT_REST_MAX_WAKES` times;
`LIFT_REST_AFTER_MS = 0` disables REST entirely). A **grab** (L1 from the INTAKE handoff, and `auton::grab()`) is a third kind: `Lift::startGrab()` presses the
arm DOWN open-loop at `L1_GRAB_PUSH_PCT` (40%), raises `needsClawGrip()` at `L1_GRAB_CLOSE_AT_MS` (200 ms)
for **Scoring** to shut the jaws, keeps pressing until both `L1_GRAB_PUSH_MS` (300) and the jaws'
`CLAW_CLOSE_TIME_MS` are satisfied (~350 ms total), then HOLDS where it ended — L1 commands the rise
afterwards, `auton::grab()` simply does not. The intake is forced STOPped for the whole push in
`periodic::tick()`, and L1's floor sweep is deferred to the rise. It is refused while homing (the caller
falls back to the old close-then-rise). An **L2 score** is
a fourth kind, and it is now TWO PRESSES: `Lift::startScore()` (records `scoreStartPos_`, the arm's position right then, fixed for
the whole score) → PRESS (push the pin onto the stack with the jaws **shut**, capped at
`L2_PRESS_MAX_MV`, 7000 mV — raised from 5000, see loaded-claw note below) → contact, decided by
`subsystems::ContactDetector` **OR** forced by `L2_AUTO_RELEASE_DEG` (32°, corrected 2026-10-02 to be
**RELATIVE travel from `scoreStartPos_`, NOT an absolute reading** — the first version wrongly compared
the live position against 32 directly) the moment PRESS has travelled that far down with nothing
detected — there is no more "pressed to the bottom with nothing released". A start too low for 32° of
travel to complete before the floor (level 1, 31°) falls back to `L2_RELEASE_FLOOR_DEG` (10°) instead,
so level 1 still gets ~21° of real sensing rather than releasing on the first tick → REL (the lift
raises `needsClawRelease()`, shown as `REL(32)` for the travel rule or `REL(floor)` for the low-start
fallback; **Scoring** opens the claw, the lift never touches a cylinder) → the freed check (gated on
`clawReleased_` before honoring `L2_RELEASE_FLOOR_DEG`'s own safety-net role here too, so a PRESS-side
floor release landing exactly on that height can never race Scoring out of ever seeing
`needsClawRelease()`) → UNSTK up `L2_UNSTICK_UP_DEG` and retry if the claw is caught → a further `L2_POST_RELEASE_PUSH_MS` (100 ms) of pressing
so the jaws slide clear → **a RISE of `L2_CLEAR_RISE_DEG` (20°) above wherever it stopped, held there
indefinitely** (`SCORED UP`). **That is the end of press 1: there is no automatic descent any more**, because
the driver asked to be able to drive away with the claw above the pins. **Press 2** (L2 again, or B, or
`auton::scoreReturn()`) is `Lift::startScoreReturn()`: the descent above at `L2_RETURN_SPEED_PCT`, FINAL, SEAT.
**The held +20° is a deliberate, user-requested hold that press 2 resolves into a real seat — it is NOT the
banned "parks a few degrees above the bottom instead of seating" design**, which was a trip to the BOTTOM that
failed to seat and had no second step. Do not "fix" it. The clear rise is only ever reached through the FREED
check, so it can never drag a pin the claw is still hooked on; UNSTK and `SCORE STUCK` are unchanged. The
claw's autoclamp is inhibited for the whole cycle (`Lift::isScoreCycle()`, press 1 → the hold → press 2 →
seated) so the pin just released cannot be re-grabbed on the way back down. **The detector's rule, learned the hard way:** never threshold against
the *nominal command* — the first version did, and could never fire, because this six-bar often cannot
make the commanded descent speed at all (the arm swings up and *back*, so gravity resists the descent
over much of the arc and the press cap limits the push). Every signal is measured against a **learned
baseline of what the arm was actually doing a moment ago**: a leaky integral of the position shortfall
(`LAG`), the loop's effort over the free-descent baseline (`EFFORT`), and a speed-ratio / near-stop
backstop. The baselines track slow gravity drift but **freeze** below `L2_CONTACT_FREEZE_FRAC` so they
cannot follow a gentle landing down and mask it. **AGREEMENT (2026-10-02):** a lone `LAG`/`EFFORT`/
`SPEED` signal is no longer enough — at least two of the three must be over threshold on the *same*
tick (`STALL` is exempt). Fix for the robot false-firing in plain free descent with an **empty** claw:
this six-bar's gravity torque swings up and back past vertical fast enough to move one signal alone,
with nothing under the arm at all. **ARM ON MOTION + SPLIT STALL (2026-10-02):** a second, different
false-fire — a **loaded** claw (pin + cup) auto-released almost immediately, every time, because the
load adds to that same gravity resistance and the one signal exempt from AGREEMENT (`STALL`, so a
resting pin is still caught) read a loaded-but-still-free arm's slow start as resting on something.
`armed_` now also needs `L2_CONTACT_ARM_TRAVEL_DEG` of real cumulative travel (self-integrated inside
the pure detector), and STALL splits: unambiguous once the arm has armed_ at all (fast confirm), but a
STALL before it has EVER armed_ waits the much longer `L2_CONTACT_NOSTART_MS`. Since
`L2_AUTO_RELEASE_DEG` now guarantees a release regardless, the detector is allowed to be the more
conservative side of every one of these trades. Out of unstick tries → `SCORE STUCK`, holds where it
is, never grinds. Nothing re-drives
or re-chases: the hold *is* the correction. Four earlier designs are in `git log` and must not come
back: `d2978c7` (jumped setpoint, high kP), `6b8165c` (trapezoid + leash + fixed `LIFT_KG`), `3723ef2`
(`LIFT_BRAKE_DECEL_DEG_PER_S2 = 2000` asked for more braking than the arm can do, and handing a
still-moving springy six-bar to the firmware's stiff brake-hold made it ring), and anything that
parks the arm a few degrees above the bottom instead of seating it.

Also in `config.hpp`: the `AUTOCLAMP_*` block, the `AUTON_*` timeouts and `MOVEB_*`. **The chassis
PID and the drive curves are the exception — they are literals in `src/robot.cpp`, see Hardware.**
**Which
auton route runs is NOT in `config.hpp`** — it is `constexpr Route SELECTED` at the top of
`src/autons.cpp`, because it is a strategy choice, not a robot constant.

**The auton lift push.** `auton::liftPush(pct)` / `liftPushStop()` / `liftPushFor(ms, pct)` press the
arm DOWN at a constant open-loop `AUTON_LIFT_PUSH_PCT` (40, TUNE) until stopped — the driver's 2026-10-03
request, for scoring a pin at the very bottom where there is no height left to command. It is **not** a
new lift mode: it goes through `Lift::driverPower()`, the same manual path the D-pad feeds, so safety
rules 2 and 3 are untouched (one owner, it cancels any target and the macro instantly, release
brake-holds where the arm is and never resumes an old target). It is LATCHED like holding a button, and
`auton::end()` releases it, so a route that forgets — or one the field switch cuts off mid-push — cannot
hand driver control an arm still leaning on the floor. `liftPush()`'s `pct` is a magnitude; down is the
only direction.

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
