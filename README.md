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
  lift.*      DR6-4B lift: DRIVE -> BRAKE -> APPROACH -> HOLD position moves, DESCEND -> SEAT trips to
              the bottom, homing, stall/runaway guards, manual voltage for D-pad Up/Down and R1,
              a terminal trace, moveTo()/setTarget() shared by driver + auton
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
| Claw cylinder | ADI **D** (was coded as C until 2026-10-01 -- the two cylinders were crossed) |
| Pivot cylinder | ADI **C** (was coded as D) |

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
| **L2** | intake macro: holding a pin → **SCORE**. The jaws open, the arm stands **still** for `L2_RELEASE_WAIT_MS` (250 ms) so the pin is free, then the arm runs the banded descent all the way down and **seats** on its hard stop. Pivot stays **OUT** throughout. Resets the level count so the next L1 starts at level 1 |
| **B** | **stow**: pivot **OUT**, lift straight down to `DOWN` and seated (the same descent as L2, without the claw), cancel the intake macro |
| **A** | claw clamp / open. The claw is physically **OPEN at power-on** -- it needs air to grip: the `CLAW` line shows `OPEN D[0]` and A closes it. `CLAW_CLOSED_IS_ADI_HIGH` (`true`) says which ADI level is closed; if A *opens* the jaws while the line says `CLAMPED`, flip it |
| **X** | pivot **OUT** / **BACK** (the brain's `CLAW` line shows `PIV OUT C[0]` / `PIV BACK C[1]`: word, port letter, raw ADI level) |
| **Y** | autoclamp on / off |

### R1 / R2 have two modes
Which one is in force is **latched the moment the button goes down**, from whether the claw is
`CLAMPED` (the brain's `CLAW` line shows `R:INTAKE`, `R:LIFT` or `R:wait`):

| | **R1** (hold) | **R2** (hold) |
|---|---|---|
| **Nothing clamped** (`R:INTAKE`) | intake roller **in**; a tap also sends the lift to the **INTAKE** handoff with the jaws open and the pivot **BACK** | intake roller **out** |
| **A pin `CLAMPED`** (`R:LIFT`) | lift **up** at `R_LIFT_UP_PCT` (90%). Plain voltage, same feel as the D-pad | lift **down**, controlled: closed-loop on the measured arm speed at `R_LIFT_DOWN_PCT` (65%) of the free speed, slowing to the crawl near the bottom and **seating** on the hard stop. Hold it all the way down and it ends exactly like an L2 |

Letting go of either brakes and holds where it is -- no old target resumes. **D-pad Up/Down still win
over both.** Neither runs the intake in `R:LIFT` mode. If the claw clamps or unclamps *while* R1/R2 is
held, the mode goes `R:wait`: the lift move and the intake both stop and nothing happens until you let
go and press again, so a clamp change can never swap a lift move for a spinning intake mid-press.

**The claw and the pivot were on each other's ports in code until 2026-10-01** -- the port called the
claw was the pivot and vice versa. Both ports and both polarities have been corrected and re-derived
(the evidence chain is in `config.hpp` on each constant), and the `CLAW` line now prints the ADI port
letter next to each so this can never go unnoticed again. **The first thing to do on a new build is the
30-second A/X check** at the bottom of this section.

**Pivot positions, in the driver's words:** **OUT** = swung out -- where it sits at power-on, at every
stack level while scoring, and in the default down position after L2 / B. **BACK** = pulled in toward
the intake handoff, only for taking a pin from the intake (R1). In code `true` = OUT. Which ADI level
is OUT is `PIVOT_OUT_IS_ADI_HIGH` (`false`: the unpowered LOW level is OUT, confirmed twice -- the
pivot sits OUT at power-on when the pin is LOW, and it sat pulled in on the build that drove that pin
HIGH at every stack level). If X makes the pivot swing BACK while the `CLAW` line says `OUT`, flip that
one constant. The handoff zone, `INTAKE` up to `INTAKE + PIVOT_HANDOFF_ZONE_DEG`, times the swing: heading
to or at the intake the pivot is BACK inside the zone and OUT above it, so it swings BACK as R1 brings
the arm down into the zone and OUT as L1 lifts it out (or when the arm arrives at the level). Nothing in
the lift's motion waits on it.

`L1`/`L2`/R1's tap-to-INTAKE are locked out while Up/Down are held, so a tap can never re-command the
lift out from under a manual nudge (see interlock 5 in `subsystems/scoring.hpp`). The manual override
doesn't touch the claw or pivot, either -- those stay exactly where they physically are.

`scoreOrMatchload()` (the old "score, or go to matchload if empty" behaviour) and `resetLevel()`
(back to level 1 without scoring) are still in `subsystems/scoring.*` but aren't wired to a driver
button right now -- available for autonomous, or a future button.

**The 30-second A/X polarity check -- do this first on any new build.** Power on, don't enable, and
read the brain's `CLAW` line. It must say `CLAW OPEN D[0]  PIV OUT C[0]`, and the mechanisms must
physically match: jaws open, pivot swung out. Then:
- press **A**: the **jaws** must close. The line must read `CLAMPED D[1]`. If the *pivot* moved
  instead, the two ports are crossed again -- swap `CLAW_PISTON_PORT` / `PIVOT_PISTON_PORT`. If the
  jaws *opened* (they were already open, so nothing visible happens, then A again shuts nothing), flip
  `CLAW_CLOSED_IS_ADI_HIGH`.
- press **A** again: the jaws open, line back to `RELEASED D[0]` or `OPEN D[0]`.
- press **X**: the **pivot** must pull BACK (in toward the intake). The line must read `BACK C[1]`. If
  it swung further out, flip `PIVOT_OUT_IS_ADI_HIGH`.
- press **X** again: the pivot swings OUT, line back to `OUT C[0]`.

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
`scoring.cpp`). See "Lift speed and motion" below for what a move does. `precise = true` gets the tight
tolerance `INTAKE` needs; any goal within half a degree of `INTAKE` is promoted automatically. A
**non-precise** goal at or below `LIFT_BOTTOM_GOAL_DEG` is not a position move at all -- it runs the
banded descent and ends **seated** on the hard stop, so `lift.moveTo(lift_preset::DOWN, …, true)` in
auton gets exactly the same bounce-free bottom that L2 does. `isSettled()` is true in `SEAT`.

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
in `config.hpp` records it.

**Everything that decides how the lift feels is in one block** at the top of the lift section of
`include/config.hpp`, marked `// ===== LIFT TUNING — start here =====`. Each constant there has a
one-line note saying which way to move it for which symptom. Nothing else needs touching.

There are **two kinds of commanded move** and **one manual input**, all inside the single 10 ms
`Lift::update()` that owns the motors.

### A position move — every goal except a trip to the bottom
`DRIVE → BRAKE → APPROACH → HOLD`:

- **DRIVE.** Full ±12000 mV toward the goal, from the very first tick -- exactly what the D-pad does --
  until the arm is up to the speed the profile allows. Moves are still instant off the line.
- **BRAKE.** The velocity loop (`LIFT_VEL_KP` / `LIFT_VEL_KI`, clamped to `LIFT_VEL_MAX_MV` = 12000)
  tracks `min(cruise cap, sqrt(2 · LIFT_DECEL_DEG_PER_S2 · (distance − LIFT_APPROACH_ZONE_DEG)))`,
  where the cruise cap is `LIFT_UP_SPEED_PCT` / `LIFT_DOWN_SPEED_PCT` of the free speed. Computed from
  *distance*, not time, so an arm ahead of or behind the profile corrects itself.
- **APPROACH.** Inside `LIFT_APPROACH_ZONE_DEG` (5°) the reference becomes
  `LIFT_APPROACH_KP_PER_S × (degrees left)`, **clamped between `LIFT_APPROACH_MIN_DEG_PER_S` and
  `LIFT_APPROACH_DEG_PER_S`** -- a controlled walk-in, not an exponential decay. **This is what stopped
  the spring/bounce:** the arm is nearly stopped before the hold takes it, so there is no energy for
  the six-bar to store and give back. The floor matters: without it, `kP × distance` decays forever and
  the last couple of degrees dominate the whole move's duration.

**Those three are not a latching state machine.** Every tick picks one from (distance, speed), with
`LIFT_DRIVE_RESUME_DEG_PER_S` (20 deg/s) of hysteresis between DRIVE and BRAKE so they can't flicker.
Only arrival into HOLD latches. **That is what makes rapid L1 taps merge into one fluid move:**
`setTarget()` on a move that is already flying just moves the goal (see "Retargeting" below) and the
next tick sees a longer distance, a profile that allows more speed, and therefore full DRIVE again --
with the velocity loop's integral, the stall window and the approach clock all untouched. It still
slows into the *final* level with the normal braking curve and approach.
- **HOLD.** A **damped position hold**, not the firmware's brake-hold:
  `LIFT_HOLD_KP × (error outside the deadband) + integral(LIFT_HOLD_KI × error) − LIFT_HOLD_KD × speed`,
  clamped to `LIFT_HOLD_MAX_MV`. The integral is the learned gravity voltage and is seeded from the
  velocity loop's own integral the moment the move arrives, so the hold never has to sag first and then
  catch up. `LIFT_HOLD_KD` is the anti-bounce term. **Nothing re-drives and nothing re-chases** -- the
  hold *is* the correction, and it is gentle, so there is no `LIFT_REENGAGE_DEG` any more.

### A descent — a non-precise goal at or below `LIFT_BOTTOM_GOAL_DEG`
L2's score, B's stow, an auton move to `DOWN`, and R2 in clamped mode. `DESCEND → SEAT`:

- **DESCEND.** The velocity loop tracks a speed reference read off the arm's **position**, in three
  bands: `LIFT_DESCENT_TOP_PCT` (50% ≈ 75 deg/s) from the start height down to the mid boundary,
  `LIFT_DESCENT_MID_PCT` (75% ≈ 112 deg/s) through the middle, then a constant
  `LIFT_DESCENT_SLOW_PCT` (12% ≈ 18 deg/s) from `LIFT_DESCENT_SLOW_ZONE_DEG` (20°) to the stop. The
  mid boundary is a **fraction** of this descent's fast travel
  (`SLOW_ZONE + LIFT_DESCENT_MID_FRACTION × (start height − SLOW_ZONE)`), not a fixed height, so "the
  middle" is the middle whether the descent started at level 1 or level 5. Band changes are ramps at
  `LIFT_DESCENT_RAMP_DEG_PER_S2`, never stops. It is **closed-loop on the measured arm speed**, so
  gravity plus the pin's weight cannot make it run away.
- **SEAT.** When the arm has gone still (slower than `LIFT_SEAT_STILL_DEG_PER_S` for
  `LIFT_SEAT_STILL_MS`) below `LIFT_SEAT_DETECT_DEG`, it is physically on its hard stop: the lift then
  holds a constant `LIFT_SEAT_MV` (−1200 mV) pressed into it, forever, so it **cannot bounce back up**.
  It no longer parks a few degrees above the frame. −1200 mV is 10% of full volts, well under an amp
  per motor, and the frame is carrying the arm -- safe for a whole match. If a lift motor ever reaches
  `LIFT_SEAT_TEMP_LIMIT_C` the seat drops to a plain brake-hold and the phase reads `SEAT(hot)`.
  An arm knocked `LIFT_SEAT_UNSEAT_DEG` back up restarts the descent and walks it down again.

`R1`/`INTAKE` is not a descent: it is a precise position move, so it keeps its own exact approach and
never seats.

### The manual input
`driverPower()` -- D-pad Up/Down at full send, and R1 at `R_LIFT_UP_PCT` while a pin is clamped -- is
plain voltage fed into the same loop ahead of everything else. It cancels any target and any homing
instantly, and letting go brake-holds where it is. R2 in clamped mode is *not* manual: it runs the
descent above with `R_LIFT_DOWN_PCT` as its top band.

> Four earlier designs are in `git log` so nobody brings them back: a jumped setpoint with `kP = 600`
> (`d2978c7`, oscillated); a trapezoid + leash + feedforward + PID + fixed gravity term + arrive/hold
> hysteresis (`6b8165c`, slow off the line and still oscillated, because a fixed `LIFT_KG` is wrong at
> most angles of a six-bar); DRIVE → BRAKE → **firmware** hold (`3723ef2`, overshot and rang -- see
> below); and anything that parks the arm above the bottom instead of seating it.

### Why L1 felt slow in the start and middle (fixed, don't reintroduce)
Three separate causes, all from the first pass at this design (`56fe5f8`):
1. `LIFT_DECEL_DEG_PER_S2 = 700` puts braking distance at `150²/1400 = 16.1°`, plus an 8° approach
   zone, so braking began **24° before the goal**. Stack levels are 12–44° apart, so a level-to-level
   move was ~2° of full power and ~24° of braking and crawling. Now 1200 → 9.4° + a 5° zone = 14.4°,
   so half a typical move is full power.
2. `LIFT_VEL_MAX_MV = 9000` physically cannot sustain the cruise: back-EMF alone is `12000/150 = 80` mV
   per deg/s, so holding 150 deg/s needs ~12000 mV and a 9000 mV clamp sagged the arm to ~110 deg/s
   for the whole middle of a long rise. Now 12000. The braking force was never set by this clamp --
   `LIFT_DECEL_DEG_PER_S2` sets that.
3. The approach reference `3.0 × degrees left` with no floor decays exponentially (τ ≈ 0.33 s): the
   last 6° took ~0.8 s. Now `6.0 × degrees left` clamped to 6…28 deg/s, so the tail is a constant
   walk-in. `LIFT_STOPPED_DEG_PER_S` went 8 → 12 so arrival still latches above that floor (it latches
   around 2° out), and `LIFT_HOLD_KD` went 60 → 90 so the slightly faster handover doesn't bring the
   bounce back.

`LIFT_VEL_FILTER = 0.8` was checked and cleared: its effective lag is `(1−α)/α × 10 ms = 2.5 ms`,
0.4° at full speed. It is not what delayed DRIVE ending. DRIVE *is* ±12000 mV from tick 1 of every up
move (`velocity ≈ 0 < profile` ⇒ DRIVE ⇒ `move_voltage(+LIFT_MAX_MV)`).

### Why L1 used to overshoot and bounce (fixed, don't reintroduce)
Four separate causes, all in the `3723ef2` design:
1. `LIFT_BRAKE_DECEL_DEG_PER_S2 = 2000` meant braking only started 7.6° out from full speed, which
   asks the arm to stop in 75 ms. It cannot, so it ran past. (Now `LIFT_DECEL_DEG_PER_S2 = 700`.)
2. `LIFT_VEL_FILTER = 0.5` is a whole tick of lag on the velocity estimate -- 1.5° of stale position at
   150 deg/s -- so the brake both started late and under-pushed. (Now `0.8`; there is nothing to filter
   out, the sensor quantises at 0.25 deg/s.)
3. `enterBrake()` seeded the velocity loop's integral with `dir × 12000 − kv × velocity`. With a
   velocity estimate 30 deg/s low that is **+2400 mV of upward bias injected at the start of braking**.
   (Now seeded from the learned gravity voltage.)
4. The handover to the firmware's stiff brake-hold happened at up to 20 deg/s. A still-moving springy
   six-bar handed to an un-dampable position loop stores that energy and gives it back: that was the
   visible spring up and down. (Now the crawl arrives nearly stopped, and the hold has `LIFT_HOLD_KD`.)

## Tuning the lift
Do this with a Brain Terminal trace open (see "Capturing a trace"). Work top to bottom; each step
assumes the one above is settled. **Start from the values in `config.hpp` -- they are not zeros.**

1. **Check direction first.** "Lift direction check" above. Nothing below means anything if the sensor
   and the motors disagree.
2. **Speed.** Tap `L1` from the bottom and watch the `D`/`B` lines. `vel` should climb to about 150 and
   stay there while cruising. If the rise feels slow, `LIFT_UP_SPEED_PCT` is the cap. Going *down* to a
   lower level, `LIFT_DOWN_SPEED_PCT` (60) is the cap -- raise it only if the descent to a lower level
   feels sluggish, and lower it if the arm ever feels like it is falling.
3. **How much of the move is at full power: `LIFT_DECEL_DEG_PER_S2`.** This is the main knob, and it
   sets both where it stops *and* how fast it feels: braking starts `150²/(2a) + LIFT_APPROACH_ZONE_DEG`
   degrees before the goal, so at `a = 1200` that is 14.4° of a 12–44° move. Tap `L1` to level 1 and
   read the last `A` line's `pos` against `goal`, and count how many `D` lines there are.
   - runs past the goal before `vel` reaches the crawl → **lower** it (1000, 850, …)
   - barely any `D` lines / feels like it brakes the whole way → **raise** it (1400, …)
   Repeat until stack levels land inside `LIFT_TOLERANCE_DEG`. Don't touch anything else yet.
4. **The crawl: `LIFT_APPROACH_*`.** Watch the `A` lines.
   - still visibly moving when it grabs the hold (`H` appears while `vel` is above ~15) → lower
     `LIFT_APPROACH_DEG_PER_S` / `LIFT_APPROACH_MIN_DEG_PER_S`, or raise `LIFT_HOLD_KD` (step 6 --
     try that first, it costs no speed)
   - the last couple of degrees take visibly long → raise `LIFT_APPROACH_KP_PER_S` (6 → 8), then
     `LIFT_APPROACH_MIN_DEG_PER_S`; shrink `LIFT_APPROACH_ZONE_DEG` to start the crawl later
   - **keep `LIFT_STOPPED_DEG_PER_S` above `LIFT_APPROACH_MIN_DEG_PER_S`**, or arrival never latches
     and every move ends on the `LIFT_APPROACH_TIMEOUT_MS` instead
5. **The velocity loop: `LIFT_VEL_KP`, then `LIFT_VEL_KI`.** Only if the `B`/`A`/`V` lines show `vel`
   swinging above and below the reference (the brain's `LIFT ref` field).
   - `vel` oscillates around the reference → **lower** `LIFT_VEL_KP` (30 → 22 → 16)
   - `vel` sits persistently below the reference, or sags before it gets going → **raise**
     `LIFT_VEL_KI` (400 → 600 → 800)
   Leave `LIFT_VEL_MAX_MV` at 12000 unless the braking feels mechanically violent: it is what lets the
   loop sustain the cruise, and lowering it both slows the middle of long moves and means lowering
   `LIFT_DECEL_DEG_PER_S2` to match.
   If the trace shows `D` and `B` alternating line-to-line, raise `LIFT_DRIVE_RESUME_DEG_PER_S`.
6. **The hold: `LIFT_HOLD_KD`, then `KP`, then `KI`.** Park at level 3 and watch the `H` lines.
   - **any bounce at all on arrival → raise `LIFT_HOLD_KD`** (90 → 120 → 150). This is the anti-bounce
     knob, it costs no speed, and it is the first thing to try. The second lever is lowering
     `LIFT_STOPPED_DEG_PER_S` (12 → 8) for a gentler handover, at the cost of a slightly slower finish.
   - the arm sags below the level over a few seconds → raise `LIFT_HOLD_KI` (400 → 600), then
     `LIFT_HOLD_KP`
   - it buzzes or twitches while parked → raise `LIFT_HOLD_DEADBAND_DEG` (0.8 → 1.2), then lower
     `LIFT_HOLD_KP` (180 → 140)
   - it audibly strains at a high level → lower `LIFT_HOLD_MAX_MV`, and accept some sag
7. **The descent: the `LIFT_DESCENT_*` bands.** Clamp nothing, go to level 5 with `L1`, press `L2` and
   watch the `V` lines, then the `S` lines.
   - too fast off the top → lower `LIFT_DESCENT_TOP_PCT`
   - the middle is slower than the top (shouldn't be) → check `LIFT_DESCENT_MID_PCT` > `_TOP_PCT`
   - any thump at the bottom → raise `LIFT_DESCENT_SLOW_ZONE_DEG` (20 → 28) and/or lower
     `LIFT_DESCENT_SLOW_PCT` (12 → 9)
   - the whole descent is too slow → raise `LIFT_DESCENT_MID_PCT`, lower `LIFT_DESCENT_SLOW_ZONE_DEG`
8. **Seating.** The last `V` lines must show `pos` reaching ~0 and `vel` going to 0, then `S` lines at
   `-1200` mV. Push the arm up by hand: it should press straight back down.
   - still bounces up off the stop → more negative `LIFT_SEAT_MV` (−1500)
   - `SEAT` appears while the arm is still a few degrees up → lower `LIFT_SEAT_DETECT_DEG` or
     `LIFT_SEAT_STILL_DEG_PER_S`, or raise `LIFT_SEAT_STILL_MS`
   - lift motors warming on the `TEMP` line while parked at the bottom → less negative (−800)
9. **Rapid taps.** From the bottom, clamp with `L1`, then double-tap and triple-tap `L1`. The `LIFT`
   line's phase must go `DRIVE` → (maybe `BRAKE`) → `DRIVE` again as each tap lands, and never
   `APPROACH` or `HOLD` at an intermediate level. In the trace there must be no `H` line between taps.
   If a tap only takes effect once the arm has arrived, the retarget path in `Lift::setTarget()` is
   being bypassed -- check `blocked_` / `faulted_` aren't set (the brain shows them).
10. **`INTAKE`.** Tap `R1` and confirm the last `A` line's `pos` is within
   `LIFT_PRECISE_TOLERANCE_DEG × 10` of `goal`. If `INTAKE` itself is in the wrong place, that is a
   preset, not a gain -- see "Tuning the claw positions".

### Capturing a trace
`LIFT_TRACE = true` prints one integer line per 20 ms to the PROS terminal whenever the arm moves, and
for half a second after. In VS Code: PROS sidebar → **Brain Terminal** (or `pros terminal` from a
shell), press a lift button, copy the lines out. Columns:

```
LIFT <ms> <phase> <pos*10> <goal*10> <vel deg/s> <out mV>
```

`phase` is `D` drive, `B` brake, `A` approach (the final crawl), `H` hold, `V` descend, `S` seat,
`M` manual (D-pad or R1 in clamped mode), `h` homing, `F` fault, `I` idle. `goal*10` is `-1` when
nothing is commanded. The lines are written by a separate low-priority task from a ring buffer; the
control loop itself never prints.

**To capture an L1 move specifically:** open the Brain Terminal, enable, put the arm at the bottom,
clamp with `L1`, let it finish, then tap `L1` once more and let the trace go quiet (half a second).
Copy everything from the first `D` line to the last `H` line. In those lines, `vel` should rise to
~150 in the `D` lines, fall smoothly through the `B` lines at about
`LIFT_DECEL_DEG_PER_S2 × 0.02` per line, be under ~20 by the first `A` line, and the `H` lines should
show `pos` holding within a degree with no sign change in `vel`. Any oscillation in the `H` lines'
`vel` is a bounce -- go to step 6 above.

### If it...

| Symptom | Change |
|---|---|
| Overshoots a level / arrives still moving | lower `LIFT_DECEL_DEG_PER_S2` |
| Stops short of every preset by the same few degrees | raise `LIFT_DECEL_DEG_PER_S2`. Up to 5 deg short is `LIFT_TOLERANCE_DEG` working as designed, and the hold pulls the rest in |
| Springs / bounces up and down as it arrives | raise `LIFT_HOLD_KD` first (costs no speed), then lower `LIFT_STOPPED_DEG_PER_S` |
| Feels slow at the start / through the middle | raise `LIFT_DECEL_DEG_PER_S2` (less of the move is braking), and check `LIFT_VEL_MAX_MV` is 12000 -- a lower clamp cannot sustain the cruise |
| The last couple of degrees crawl forever | raise `LIFT_APPROACH_KP_PER_S` and `LIFT_APPROACH_MIN_DEG_PER_S`, or shrink `LIFT_APPROACH_ZONE_DEG` |
| Rapid L1 taps stop at each level instead of merging | the arm is reaching `APPROACH`/`HOLD` before the next tap lands -- raise `LIFT_DECEL_DEG_PER_S2` so less of the move is slow. A tap is only *dropped* if D-pad Up/Down is held, or mid-L2 (`RELEASING`) |
| Trace shows `D` and `B` alternating every line | raise `LIFT_DRIVE_RESUME_DEG_PER_S` |
| Sags below a stack level while parked | raise `LIFT_HOLD_KI`, then `LIFT_HOLD_KP` |
| Buzzes or twitches while parked | raise `LIFT_HOLD_DEADBAND_DEG`, then lower `LIFT_HOLD_KP` |
| `vel` swings above and below the reference in the trace (hunts) | lower `LIFT_VEL_KP` |
| Slow to reach a commanded speed / sags before it gets going | raise `LIFT_VEL_KI` |
| The braking feels mechanically violent | lower `LIFT_VEL_MAX_MV`, and lower `LIFT_DECEL_DEG_PER_S2` to match |
| The last couple of degrees take too long | raise `LIFT_APPROACH_KP_PER_S`, then `LIFT_APPROACH_DEG_PER_S` |
| L2's descent thumps into the bottom | raise `LIFT_DESCENT_SLOW_ZONE_DEG`, lower `LIFT_DESCENT_SLOW_PCT` |
| L2's descent is too slow overall | raise `LIFT_DESCENT_MID_PCT`, lower `LIFT_DESCENT_SLOW_ZONE_DEG` |
| The arm bounces back up off the bottom after L2 | more negative `LIFT_SEAT_MV` |
| Lift motors warm while parked at the bottom | less negative `LIFT_SEAT_MV`; `SEAT(hot)` on the brain means `LIFT_SEAT_TEMP_LIMIT_C` already cut it |
| `SEAT` appears with the arm still a few degrees up | lower `LIFT_SEAT_DETECT_DEG` / `LIFT_SEAT_STILL_DEG_PER_S`, or raise `LIFT_SEAT_STILL_MS` |
| Lag between L1's *first* tap and the arm moving | that is `CLAW_CLOSE_TIME_MS` -- the jaws must shut before the arm pulls away |
| Lag between L2 and the arm starting down | that is `L2_RELEASE_WAIT_MS` -- deliberate, so the pin is free first |
| `BLOCKED` on a normal move | the arm made less than `LIFT_STALL_DEG` of progress in `LIFT_STALL_MS` while a real speed was commanded -- look for a jam or a preset above the real top before raising `LIFT_STALL_MS`. The lift holds **where it stopped**, not at the goal, so it never pushes into the jam |
| `LIFT FAULT` on a normal move | the distance to the goal grew during DRIVE, or the arm climbed during a DESCEND: run the direction check above before touching `LIFT_FAULT_DEG` |
| R1/R2 run the intake when you wanted lift (or vice versa) | read `R:` on the brain's `CLAW` line. `R:LIFT` needs the claw `CLAMPED`; `R:wait` means the clamp changed while the button was held -- let go and press again |
| A button seems to do nothing | read the brain's `CLAW` line first. `CLAMPED` makes R1's go-to-intake a no-op (interlock 4) and turns R1/R2 into lift up/down, and `MACRO STACKING` means the macro thinks it is holding a pin. `B` resets the macro; `A` opens the jaws |

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
4. Timing: `CLAW_CLOSE_TIME_MS` (grip before lifting -- specifically the gap between the first L1 tap and the arm moving), `L2_RELEASE_WAIT_MS` (L2: the arm stands still this long after the jaws open, so the pin is free before the end effector leaves), `CLAW_OPEN_TIME_MS` (`scoreOrMatchload()`'s path only), `PIVOT_TIME_MS`, and `PIVOT_HANDOFF_ZONE_DEG` (how far above `INTAKE` the pivot swaps between BACK and OUT on the way to / from the intake). **`CLAW_CLOSE_TIME_MS`, `CLAW_OPEN_TIME_MS` and `PIVOT_TIME_MS` have never been measured against the cylinder they now control** -- they were tuned while the two ADI ports were crossed, so treat all three as fresh `TUNE` values.

**Lift gains** are a separate procedure -- see "Tuning the lift" above.

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
| CLAW | claw state (`OPEN`/`ARMED`/`CLAMPED`/`RELEASED`) and pivot `PIV OUT` / `PIV BACK`, **each with its ADI port letter and raw level** (`CLAW OPEN D[0]  PIV OUT C[0]` at power-on) so A / X each confirm one polarity constant; then intake-macro state, stack level, and `R:INTAKE`/`R:LIFT`/`R:wait` -- which mode R1/R2 are in |
| LIFT (position line) | position → target (or `none`), the active named preset (or `custom`), the phase (`DRIVE`/`BRAKE`/`APPROACH`/`HOLD`/`DESCEND`/`SEAT`), and `SETTLED`/`BLOCKED`/`FAULT` |
| LIFT (err/vel/out line) | error in lift degrees, measured arm speed in deg/s, and what was sent to the motors in mV |
| LIFT (ref line) | the speed reference the velocity loop is tracking this tick (0 in `HOLD`/`SEAT`), the hold loop's learned gravity voltage in mV, then `DIST`: claw distance sensor mm and `PIN` when inside `CLAW_DETECT_MM` |
| TUNE | `LIFT_DECEL_DEG_PER_S2`, the approach zone / speed, `LIFT_HOLD_KD` and `LIFT_TOLERANCE_DEG` read straight from `config.hpp`, so you can see which build is on the brain while iterating |
| LIFT (temp/current line) | lift motor temp (°C) and current draw (mA) |
| TEMP | hottest left drive / right drive / lift motor and the intake motor (C). Motors lose power around 55 C. |
| DRV | driver-loop heartbeat: `not started`, or a loop counter plus stick values and the manual-override Up/Down |
