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
  lift.*      DR6-4B lift: DRIVE -> BRAKE -> APPROACH -> HOLD position moves, DESCEND -> FINAL -> SEAT
              trips to the bottom (FINAL forces the last few degrees onto the true 0), homing,
              stall/runaway guards, manual voltage for D-pad Up/Down and R1, a terminal trace,
              moveTo()/setTarget() shared by driver + auton
  contact_detector.hpp  PURE landing detector for the L2 scorer (no PROS) -- host-tested in test/
  autoclamp_detector.hpp PURE "is something really in the jaws?" debounce (no PROS) -- host-tested
  claw.*      pneumatic claw + the distance-sensor autoclamp
  intake.*    one-motor roller intake (driver-controlled only)
  scoring.*   intake macro: intake / matchload / 5-level stacking state machine with interlocks,
              drives the lift only through Lift::setTarget() (the PID), never raw power
include/auton.hpp, src/auton.cpp   AUTONOMOUS API -- one call per lift/claw/pivot/intake/macro
                              operation, the blocking waits, and moveB(). Don't put routes here.
include/autons.hpp, src/autons.cpp ROUTES -- every auton route, and the one SELECTED line that
                              picks which one runs. This is the file to work in when coding routes.
include/periodic.hpp, src/periodic.cpp  THE per-loop subsystem logic, in one copy: both macro
                              windows, the single intake decision, Claw::update(), Scoring::update().
                              Run by opcontrol's loop in driver and by the auton tick in auton.
src/screen.cpp                brain screen sensor readout
src/main.cpp                  initialize / autonomous (setup + one autons::run() call) / opcontrol
                              + button map
test/                         host-side tests for the pure parts (test/run.sh, Mac clang++ only --
                              pros make never sees this directory, it only globs src/)
docs/notebook/                notebook-ready write-ups, flowcharts and blank test tables
```

## Port map
All ports live in `include/config.hpp`. Current values:

| Device | Port |
|---|---|
| Left drive | 8, 2, 1 (all reversed) |
| Right drive | 3, 9, 10 |
| Odometry pod (rotation) | 17 |
| Inertial sensor | **18** (fitted and confirmed 2026-10-02 -- heading comes from the IMU, distance from the pod. Calibrated in a background task; driver control never waits on it. Set `IMU_PORT = 0` to run on the pod alone) |
| Lift motors | 20, 13 (reversed), on the 12T axle, geared 4:1 into the 48T lift-arm axle |
| Lift rotation sensor | 16, on the 12T motor axle (same 4:1 as the motors) -- the lift PID runs off this sensor |
| Intake (one motor) | 12 |
| Claw distance sensor (autoclamp) | 11 (mounted in the claw, confirmed 2026-10-02) |
| Rear distance sensor (`auton::moveB`, brain `DIST BACK`) | 14 (confirmed 2026-10-03) |
| Left distance sensor (start alignment, brain `DIST L`) | 15 (confirmed 2026-10-03) |
| Right distance sensor (brain `DIST R`) | 19 (confirmed 2026-10-03) |
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
| **L1** | intake macro: step up a stack level. **From the `INTAKE` handoff (the R1 position) it does THE GRAB first:** the arm forces itself **down** onto the pin at `L1_GRAB_PUSH_PCT` (40% of volts, open-loop), the jaws close `L1_GRAB_CLOSE_AT_MS` (200 ms) into the push, and the push keeps pressing until **both** `L1_GRAB_PUSH_MS` (300 ms) and the jaws' own `CLAW_CLOSE_TIME_MS` (150 ms from the moment they were commanded) are satisfied -- about **350 ms** of pressing down -- so the arm can never pull up out of a claw that is still closing. Only then does it rise to level 1. The **intake is stopped for the whole push** (it would drag the pin out from under the jaws), and L1's floor sweep below starts from the **rise** instead. Brain `LIFT` line: `GRAB`; macro state `GRABBING`. If the claw is already clamped the push still happens (it seats the grip), the close is skipped, and it rises. `L1_GRAB_MAX_MS` is the safety cutoff. From anywhere else L1 is unchanged: (1st tap clamps + goes to level 1; each tap after goes up one level, staying at level 5 once reached). If the claw is already `CLAMPED` (L2 just clamped it at the bottom), the first tap skips the clamp wait and rises immediately. **Auto-intake:** an L1 that starts a rise **from the floor** (at the bottom / seated, e.g. after an L2 score or a B stow, or parked at the `INTAKE` handoff after R1) also runs the intake rollers **in** from the press itself -- including during the claw-close wait -- until the arm climbs past `L1_AUTO_INTAKE_UNTIL_DEG` (20°), sweeping the floor while the arm is going up anyway.  Extra L1 taps during the same rise neither restart nor extend it, and an L1 from a stack level never starts it. It stops early on D-pad, B, L2, clamped-mode R1/R2, or a `BLOCKED`/`FAULT` lift, and after `L1_AUTO_INTAKE_MAX_MS` regardless |
| **L2** | intake macro, **above the bottom**, holding a pin → **THE SENSING SCORER**. The jaws stay **shut**; the arm presses the pin **down onto the stack** at `L2_SCORE_SPEED_PCT` (60%), watches the rotation sensor for the landing -- at least **two** of the three `LAG`/`EFFORT`/`SPEED` signals must agree on the same tick (`AGREEMENT`, 2026-10-02; a lone signal can no longer fire by itself, see "Tuning the L2 scorer"), and now also won't mistake a **loaded** arm (pin + cup) slowly out-running gravity for something resting on it (`ARM ON MOTION` + `SPLIT STALL`, 2026-10-02). **Guaranteed release:** if nothing is detected once the arm has travelled `L2_AUTO_RELEASE_DEG` (32°) **down from where L2 was first pressed** (a *relative* travel budget, not an absolute reading) it releases anyway, right there (`REL(32)` on the brain) -- there is no more "pressed all the way down with nothing released". A start too low for that much travel before the floor (level 1, 31°) instead falls back to releasing at `L2_RELEASE_FLOOR_DEG` (`REL(floor)`), so level 1 still gets real sensing time rather than releasing on the first tick. Either way it then **releases the claw and keeps pushing down** so the jaws slide off → confirm the arm runs free again → **keep pressing a further `L2_POST_RELEASE_PUSH_MS`** (100 ms) so they slide fully clear → **rise `L2_CLEAR_RISE_DEG`** (20°) above wherever it stopped → **HOLD there, latched, indefinitely** (brain: `SCORED UP`). **PRESS 1 ENDS THERE -- there is no automatic descent to the bottom any more**, so the robot can drive away with the claw above the pins without any chance of knocking one over. **PRESS 2** (L2 again from that held position, or **B**) is the ending that used to be automatic: the fast run home at `L2_RETURN_SPEED_PCT`, `FINAL` forcing the last few degrees onto the hard stop, and `SEAT` at 0. If the claw stays caught on the pin it lifts `L2_UNSTICK_UP_DEG` (20°) and retries, up to `L2_MAX_UNSTICK_TRIES`, then stops and shows `SCORE STUCK` -- the 20° clear rise is only ever reached through the **freed** check, so it can never drag a pin the claw is still hooked on. Pivot stays **OUT** throughout; the D-pad cancels at any stage; L2 pressed mid-sequence is ignored. The level count resets at press 1. **While held above:** R1 still runs the rollers and sends the arm to the handoff, L1 still starts a new stack from level 1, and the autoclamp stays **inhibited until press 2 has seated** so the pin just released cannot be re-grabbed on the way down past it. **Auto-outtake:** whenever either press has the arm below `L2_OUTTAKE_BELOW_DEG` (40°) the intake rollers run **out** at `L2_OUTTAKE_PCT` (60%) on their own -- so on **press 1** through PRESS/REL/UNSTK/PUSH (it ends when press 1 reaches its hold), and on **press 2** through the run home, stopping as soon as the arm reaches `L2_OUTTAKE_STOP_DEG` (tied to `LIFT_FINAL_START_DEG`, 3°) where `FINAL`'s own authority takes over. `L2_OUTTAKE_MAX_MS` is the safety cutoff. **At the bottom** (seated, or within `LIFT_BOTTOM_GOAL_DEG` of it) and not mid-move: L2 is instead the **clamp trigger** -- not clamped → clamp, clamped → release -- the same toggle A uses, moving neither the lift, the pivot, nor the intake |
| **B** | **stow**: pivot **OUT**, lift straight down to `DOWN` and seated, cancel the intake macro. Uses the same fast banded descent as L2's run home (`LIFT_DESCENT_TOP_PCT` / `_MID_PCT`, now 80% / 95%), minus any claw work |
| **A** | claw clamp / open. The claw is physically **OPEN at power-on** -- it needs air to grip: the `CLAW` line shows `OPEN` and A closes it. `CLAW_CLOSED_IS_ADI_HIGH` (`true`) says which ADI level is closed; if A *opens* the jaws while the line says `CLAMPED`, flip it |
| **X** | pivot **OUT** / **BACK** (the brain's `CLAW` line shows `PIV OUT C[0]` / `PIV BACK C[1]`: word, port letter, raw ADI level) |
| **Y** | autoclamp on / off (it is **on** from the moment driver control starts -- see "Autoclamp") |

### R1 / R2 have two modes
Which one is in force is **latched the moment the button goes down**, from whether the claw is
`CLAMPED` (the brain's `CLAW` line shows `R:INTAKE`, `R:LIFT` or `R:wait`):

| | **R1** (hold) | **R2** (hold) |
|---|---|---|
| **Nothing clamped** (`R:INTAKE`) | intake roller **in**; a tap also sends the lift to the **INTAKE** handoff with the jaws open and the pivot **BACK** | intake roller **out** |

**The driver always wins over L1's auto-intake:** R1/R2 in `R:INTAKE` mode are the roller buttons, and
the auto-intake only gets the rollers when neither is being asked for.
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
read the brain's `CLAW` line. It must say `CLAW  OPEN  AC:... PIV OUT`, and the mechanisms must
physically match: jaws open, pivot swung out. Then:
- press **A**: the **jaws** must close. The line must read `CLAMPED D[1]`. If the *pivot* moved
  instead, the two ports are crossed again -- swap `CLAW_PISTON_PORT` / `PIVOT_PISTON_PORT`. If the
  jaws *opened* (they were already open, so nothing visible happens, then A again shuts nothing), flip
  `CLAW_CLOSED_IS_ADI_HIGH`.
- press **A** again: the jaws open, line back to `RELEASED` or `OPEN`.
- press **X**: the **pivot** must pull BACK (in toward the intake). The line must read `BACK C[1]`. If
  it swung further out, flip `PIVOT_OUT_IS_ADI_HIGH`.
- press **X** again: the pivot swings OUT, line back to `OUT C[0]`.

**Note:** `X` (manual pivot toggle) is independent of the intake macro above. Avoid tapping it while
the macro is mid-move (`STACKING`/`MOVING`) -- the macro also drives the pivot as part of its
interlocks, and the two aren't coordinated with each other.

**Is the driver loop alive?** The `LIFT` line's `R:` field changes the moment R1 or R2 is held, and the `CLAW` line reacts to A and X. If none of them move, driver control never started (not enabled, or the program crashed -- look for a red error on the brain). If they do move but the lift does not, the problem is wiring or motor direction, not code.

## Autonomous API
Everything autonomous can do lives in **`include/auton.hpp`** (the API) and is written into routes in
**`src/autons.cpp`** (the routes). Every operation goes through the *identical* code path the matching
driver button uses -- `stepUp()` is L1, `score()` is L2, `goToIntake()` is R1's new-press, `stow()` is
B, `clawGrip()`/`clawOpen()` are A -- so auton and driver can never drift apart, and tuning one tunes
both. The named heights are the same `config::lift_preset` constants the buttons use.

### Blocking vs non-blocking
**Commands return immediately; `*AndWait` blocks.** That is how the lift and the drivetrain move at
the same time:

```cpp
chassis.moveToPoint(-17.5, -24, 1500, {.forwards = false});
auton::liftTo(auton::Level::TWO);   // rises WHILE the robot drives
chassis.waitUntilDone();
auton::waitLift();                  // ...and only now insist it has arrived
auton::scoreAndWait();
```

* To wait **later**, put `auton::waitLift(ms)` right before the step that needs the arm there --
  command early, wait late.
* **The LemLib gotcha:** `chassis.move…` is async, but asking for a *second* move while the first is
  still running **blocks the route thread** until the first finishes. So a lift command written after
  two chassis moves does not start until the first one ends. Put the lift command *between* them, or
  use `chassis.waitUntil(inches)` to let the drive get partway first.
* You do **not** have to wait before scoring or grabbing. `score()`, `grab()` and L1's handoff grab
  wait for an in-flight lift move to *arrive* before they start pressing (Scoring's `LIFT_SETTLE`
  state, bounded by `SCORING_SETTLE_WAIT_MS`, shown as `WAIT LIFT` on the brain). A press started from
  a moving arm would feed the contact detector the move's own deceleration and measure
  `L2_AUTO_RELEASE_DEG`'s travel from the wrong place, so the macro refuses to start early.
* `startPose()`, `moveB()`, `intakeInFor/OutFor()` and `wait()` block by their nature. Everything else
  not named `*AndWait` returns at once.

Two halves: **commands never block** and return immediately; **waits always take a timeout and always
return whether it actually happened** (`false` = timed out, or the lift reported BLOCKED / FAULT /
SCORE STUCK). So a routine can check instead of hoping, can drive while the arm moves, and can never
hang on a jammed mechanism.

```cpp
auton::liftTo(auton::Level::THREE);
if (!auton::waitLift(1500)) { /* didn't get there -- decide what to do */ }
auton::scoreAndWait();
```

### Lifecycle (autonomous() does all of this; a route never has to)
| Call | What it does |
|---|---|
| `auton::begin()` | starts the 10 ms **auton tick** -- the same per-loop logic the driver loop runs (`periodic::tick()`): Scoring's state machine, the claw autoclamp, the one intake decision. Commands no motion. Idempotent. |
| `auton::end()` | stops it and **blocks until the tick task has really exited**, then stops the intake. Called by `opcontrol()` and `disabled()`, so the auton tick and the driver loop can never both run. Never touches the lift. |
| `auton::running()` | true while the auton tick is up |
| `auton::wait(ms)` | `pros::delay`, with a name that reads in a routine |

### Lift
| Call | What it does |
|---|---|
| `auton::homeLift()` | pivot OUT, then `Lift::requestHome()`. Returns at once; homing runs in the lift's task |
| `auton::waitHome(ms)` | blocks until homing ends. `true` only if it really HOMED (so the arm is seated on a trustworthy zero) |
| `auton::liftTo(deg, precise=false)` | an arbitrary height, clamped to `0..lift_preset::MAX` |
| `auton::liftTo(Level::ONE..FIVE)` | a stack level (`config::lift_preset::STACK_LEVELS`) |
| `auton::liftToIntake()` | the precise INTAKE handoff height (tight tolerance, slow final approach, never seats) |
| `auton::liftToFrontToggle()` / `liftToFrontToggleAndWait(ms)` | the front-toggle height, `lift_preset::FRONT_TOGGLE` (30°, its own constant -- deliberately *not* stack level 1 at 31°, so the two can be tuned apart). A **coarse** move: speed over precision, **overshoot allowed**, because the height only has to be *touched*. It brakes late (`LIFT_COARSE_DECEL_DEG_PER_S2` 2600 vs 1200), keeps a quick tail instead of the final crawl (`LIFT_COARSE_MIN_DEG_PER_S` 45 vs 6 °/s) and counts `LIFT_COARSE_TOLERANCE_DEG` (10° vs 5) as arrived -- so it gets there noticeably faster than a rise to a stack level |
| `auton::liftDownFast()` / `liftDownFastAndWait(ms)` | the **fast** trip to the bottom -- the toggle's way back down. Same bands → `FINAL` → `SEAT` → `REST` as `liftDown()`, with only the two *fast* bands raised (`LIFT_FAST_DESCENT_TOP_PCT` 105 % vs 80, `_MID_PCT` 120 % vs 95) and a firmer ramp (2000 vs 1200 deg/s²). The approach cruise, `FINAL` and the seat are unchanged, so it still slows near 0 and settles. **Queued behind a toggle rise:** called while a `FRONT_TOGGLE` move is still climbing, it waits (inside the lift task, never blocking the route) until the arm *touches* the toggle height and turns straight around -- brain `TOGGLE ^`, then `DESC FAST` |
| `auton::liftToMatchload()` | the MATCHLOAD height |
| `auton::liftDown()` | the banded descent that ends **SEATED** on the hard stop (same as L2's run home, B, R2) |
| `auton::liftStop()` | abandon the move and brake-hold here, **no target active**. The one call that leaves a passive hold -- for stopping a move, not for ending a routine |
| `auton::waitLift(ms)` | blocks until arrival (HOLD at a height, SEAT at the bottom) |
| `auton::waitLiftSeated(ms)` | blocks until the arm is pressed onto the hard stop |
| `auton::liftToAndWait(deg/Level, ms)` | the two in one line |
| `auton::liftToIntakeAndWait(ms)`, `auton::liftDownAndWait(ms)` | ditto |
| `auton::liftPosition()`, `liftTarget()`, `liftSettled()`, `liftSeated()`, `liftSeatedShort()`, `liftBlocked()`, `liftFaulted()`, `liftHoming()`, `liftHomed()`, `liftPhase()` | non-blocking status, for driving while the arm moves. `liftPhase()` is the brain's own phase word |
| `auton::heightOf(Level)` | the arm degrees a level means |

### Claw, pivot, autoclamp
| Call | What it does |
|---|---|
| `auton::clawGrip()` / `clawOpen()` / `clawToggle()` | exactly what A does (through `Scoring::toggleClaw()`); grip/open are idempotent |
| `auton::gripAndWait(ms)` | grip, then wait out `CLAW_CLOSE_TIME_MS` so the arm never pulls out of closing jaws |
| `auton::openAndWait(ms)` | open, then wait out `CLAW_OPEN_TIME_MS` |
| `auton::clawClamped()` | is a pin held? |
| `auton::waitUntilClamped(ms, stopChassis=false)` | **the autoclamp wait.** Blocks until the claw is actually clamped, then waits out the close time. `stopChassis = true` cancels the running LemLib motion at the instant of the clamp -- see "Autoclamp" |
| `auton::autoclamp(bool)` / `autoclampOn()` | the autoclamp, which auton starts with **off** |
| `auton::pivotOut(wait=false)` / `pivotBack(wait=false)` / `pivotIsOut()` | the pivot's two positions (X); `wait = true` adds `PIVOT_TIME_MS` |

### Intake
| Call | What it does |
|---|---|
| `auton::intakeIn(power=1.0)` / `intakeOut(power)` | latch a roller request (like holding R1/R2). Applied by the tick as the **one** intake write, and it beats both macro windows |
| `auton::intakeStop()` | drop the request |
| `auton::intakeInFor(ms, power)` / `intakeOutFor(ms, power)` | run it for a time, then stop. Blocks for `ms` |

### The driver's macros
| Call | What it does |
|---|---|
| `auton::goToIntake()` / `goToIntakeAndWait(ms)` | R1: jaws open, pivot BACK, precise move to the handoff |
| `auton::stepUp()` / `stepUpAndWait(ms)` | L1: clamp + rise a level (and L1's auto-intake floor sweep, and the force-down GRAB when it starts from the handoff -- exactly like the button) |
| `auton::grab()` / `grabAndWait(ms)` | "grab it right here": the same force-down push + close, then **hold where the push ended** instead of rising. `clawGrip()` is the cylinder-only version |
| `auton::score()` / `waitScore(ms)` / `scoreAndWait(ms)` | L2 **press 1**: press -> release -> post-push -> rise `L2_CLEAR_RISE_DEG` -> hold there. `false` on SCORE STUCK |
| `auton::scoreReturn()` / `scoreReturnAndWait(ms)` | L2 **press 2**: the fast run home from that hold, ending SEATED at 0 |
| `auton::scoredUp()` | is the arm holding above a finished score, waiting for press 2? |
| `auton::scoreStuck()` | did the last score give up with the claw caught? |
| `auton::matchload()` | `Scoring::scoreOrMatchload()` (no driver button) |
| `auton::stow()` / `stowAndWait(ms)` | B: pivot OUT, straight down, seated |
| `auton::resetMacro()` / `resetLevel()` / `level()` / `macroState()` | the macro's own state |

### moveB -- drive until the REAR sensor reads a distance
Ported 1:1 from the team's old library, signature included, so old calls paste straight in. **Not an
odometry move:** a hand-rolled PD loop on the rear distance sensor (port `BACK_DISTANCE_PORT`) that
drives both sides at equal power until that sensor reads `distance` mm, then settles.

```cpp
auton::moveB(150, /*forwards=*/false, /*decreasing=*/true, /*maxSpeed=*/70, /*minSpeed=*/15, /*timeOutMs=*/2000);
// back up until the rear sensor is 150 mm from the wall behind, at up to 70 power, give up after 2 s
```

* `forwards` -- which way the robot physically moves. It negates the whole output, after the PD maths.
* `decreasing` -- `true` when the reading gets **smaller** as the move progresses (backing toward a
  wall: `error = reading - distance`); `false` when it gets **bigger** (driving away from whatever is
  behind: `error = distance - reading`). Get this wrong and the loop drives the wrong way -- check it first.
* Gains and thresholds are `config::MOVEB_*`, all marked `TUNE`: **they need retuning**, because the old
  robot had 3.25" traction wheels and this one has 2.75" omnis.
* It writes the drive motors directly, so it cancels any LemLib motion at entry and waits for the
  chassis to be idle -- never two writers on the drive -- and it refuses to run unless the auton tick is
  up, so it can never be called from driver control.
* **Kept from the old code:** while the rear sensor reads out of range (>= `MOVEB_OUT_OF_RANGE_MM`,
  9910) it drives **blind at `maxSpeed`** until something comes into range or the timeout ends the move.
  A missing or unplugged rear sensor therefore means full speed until `timeOutMs`. Watch the brain's
  `BACK` line before trusting a route to it, and keep `timeOutMs` tight.
* `auton::backDistanceMm()` is the raw reading. The old library's `moveF` and friends (front sensor)
  were **not** ported -- this robot has no front distance sensor.

### The lower-level `Lift` API (still there)
`auton::*` is a wrapper, not a replacement. `lift.moveTo(height, timeoutMs, waitUntilSettled)` blocks
the calling task only -- the lift's own task keeps running the PID at 10 ms regardless -- and
`setTarget(height, precise)` is the non-blocking version `scoring.cpp` uses. `precise = true` gets the
tight tolerance `INTAKE` needs; any goal within half a degree of `INTAKE` is promoted automatically. A
**non-precise** goal at or below `LIFT_BOTTOM_GOAL_DEG` is not a position move at all -- it runs the
banded descent and ends **seated** on the hard stop. `isSettled()` is true in `SEAT`. A blocking
`moveTo(..., true)` at the top of a routine also waits out lift homing.

### Every state holds
Nothing in the API leaves the lift limp. A level ends in the damped HOLD; the bottom ends SEATED
(`LIFT_SEAT_MV` pressed into the hard stop, not parked above it); a score ends seated; and **successful
homing ends seated too** -- `Lift::updateHoming()` adopts a DOWN target already in `SEAT`, so the arm is
actively held on its stop from the moment auton starts. `HOME FAIL` is the one exception, on purpose:
the arm stopped somewhere implausible, so it stops pushing and brake-holds rather than driving into
whatever it is caught on. Nothing in `opcontrol()` cancels a target it did not start, so whatever auton
left the arm holding survives the hand-off to driver control.

## Writing an auton route
Everything route-specific is in **`src/autons.cpp`**, and that is the only file to open:

1. **Pick the route:** change the one line at the top, `constexpr Route SELECTED = Route::LIFT_TEST;`
2. **Write a route:** copy `matchRoute()` (it is a commented template whose every line is a real,
   compiling call), rename it, add it to the `Route` enum in `include/autons.hpp` and to the `switch`
   in `run()`.
3. Build and upload. Nothing else in the codebase needs touching.

Routes that ship today: `LIFT_TEST` (lift only, no drivetrain), `ODOM_TEST` (two straight-line moves),
`MATCH` and `SKILLS` (templates), `NONE`.

`autonomous()` in `main.cpp` does the setup no route may forget, then makes exactly one call,
`autons::run()`. The setup is: start the auton tick, autoclamp **off**, **request lift homing** (every
route, every time -- it blocks nothing and runs underneath the route), then wait for odometry/IMU
calibration, bounded by `config::AUTON_CALIBRATE_TIMEOUT_MS` so an unplugged IMU cannot eat the
autonomous period.

**`LIFT_TEST` on the robot** (lift, claw and intake only -- nothing drives):
1. Put the arm anywhere reasonable, power on, enable autonomous (competition switch, or the brain's
   program-run menu).
2. The arm drives **down** to its hard stop. Brain `LIFT` line: `HOMING`, then `HOMED`.
3. For 2 s: `LIFT 0.0 -> 0.0  DOWN  SEAT  SETTLED`. It must not creep up.
4. Up to stack level 1: `LIFT ... -> 31.0  STACK 1  DRIVE` then `HOLD` for 2 s, within ~5 deg of 31.
5. Up to stack level 3: ends `HOLD` near 74.6 for 2 s.
6. Back down: `DESCEND` -> `FINAL` -> `SEAT` near 0 -> `REST` about 300 ms later, and it **stays**
   `REST` -- including after the switch to driver control. At `REST` the motors are coasting: the arm
   should feel completely free if you lift it by hand.

**`ODOM_TEST` on the robot:** clear 3 ft in front, mark the starting position, enable autonomous. The
robot drives forward 24", then reverses back; it should finish on its mark and the brain's `POSE` line
should read ~`X 0 Y 0`. Homing runs at the same time, so expect the arm to go down as it drives.

## Autoclamp
A distance sensor in the claw (port `CLAW_DISTANCE_PORT`, 11) looks out between the jaws and sees both
pins and cups. **While the jaws are open and the autoclamp is enabled, anything at or inside
`AUTOCLAMP_RANGE_MM` for `AUTOCLAMP_CONFIRM_MS` straight gets gripped.** The debounce is the whole
point: a reading that appears, vanishes and comes back is not a pin.

The rule lives in `subsystems::AutoclampDetector` -- a **pure** class (no sensor, no clock, no cylinder,
like `ContactDetector`) with host-side tests in `test/autoclamp_detector_test.cpp`, so spike rejection
is proven on the Mac (`test/run.sh`) before anything is uploaded. `Claw::update()` owns the cylinder and
calls it once per 10 ms tick.

**When it is on.** Driver: **on by default**, re-enabled every time `opcontrol()` starts
(`AUTOCLAMP_ON_IN_DRIVER`), and **Y** still toggles it mid-match. Auton: **off by default** --
`auton::autoclamp(true)` in the route turns it on where it is wanted. Power-on/`initialize()`: off
(`CLAW_AUTOCLAMP_AT_START = false`), so a preloaded pin cannot fire a cylinder before anyone enables
anything. It also cannot fire during an L2 score: the claw is inhibited for the whole sequence.

**Re-arm, and why it is not a timer.** The instant the jaws open, the pin they let go of is still
millimetres away, so a timer-based re-arm re-grabs it every time and no score can finish. After **any**
open the autoclamp is disarmed until the sensor reads **clear** -- further than `AUTOCLAMP_REARM_MM`, or
nothing in range at all -- for `AUTOCLAMP_REARM_CLEAR_MS`. The gap between 15 mm and 30 mm is
hysteresis, so the two thresholds cannot chatter at one distance. `CLAW_RELEASE_LOCKOUT_MS` (750 ms) is
kept **on top** of that as a floor, covering the jaws physically swinging open past the sensor.

**Watching it.** Brain `CLAW` line, the `AC:` field: `OFF` (disabled, or the jaws are shut by hand),
`ARMED` (watching, nothing seen), `SEEING` (something in range, the confirm window is running), `CLEAR?`
(disarmed after an open, waiting for a clear reading) or `HELD`. The raw millimetres are on the `DIST`
field of line 8, with ` PIN` appended while the reading is inside the range.

| Knob | What it does |
|---|---|
| `AUTOCLAMP_RANGE_MM` (15) | at or inside this = something in the jaws |
| `AUTOCLAMP_CONFIRM_MS` (50) | ...continuously for this long |
| `AUTOCLAMP_MIN_SAMPLES` (2) | ...over at least this many fresh readings |
| `AUTOCLAMP_STALE_GRACE_MS` (60) | an unchanged reading counts as a new sample again after this long (~2 sensor refreshes) |
| `AUTOCLAMP_REARM_MM` (30) / `AUTOCLAMP_REARM_CLEAR_MS` (150) | what counts as clear, and for how long, before it re-arms |
| `AUTOCLAMP_MAX_VALID_MM` (9999) / `AUTOCLAMP_MIN_VALID_MM` (1) | the sensor's "nothing there" (counts as clear) and "cannot measure" (counts as neither) |
| `AUTOCLAMP_MIN_CONFIDENCE` (0 = ignore) | the sensor's own 0..63 confidence |
| `AUTOCLAMP_ON_IN_DRIVER` (true) | the driver default |

**Why fresh samples are counted as well as milliseconds:** the V5 Distance Sensor refreshes on the
order of 30 ms, which is slower than the 10 ms tick, so the same number comes back two or three times
running. A pure time window could therefore be satisfied by **one** lucky measurement repeated for
50 ms -- exactly the spike this rejects. A reading counts as a new sample when the value changes, or
when `AUTOCLAMP_STALE_GRACE_MS` has passed since the last counted one (so a pin sitting dead still at
one millimetre value still gets its second sample).

**Heads-up on 15 mm:** the V5 Distance Sensor's *specified* minimum range is about **20 mm**, so 15 sits
below where it is specified to read reliably. It usually still reports something sensible closer in, but
this is the first number to question. With a pin held in the open jaws, watch `DIST`:

* it reads **20-25 and never reaches 15** -> raise `AUTOCLAMP_RANGE_MM` to just above what you see;
* it reads **0** with the pin there -> the pin is inside the sensor's floor: raise the range and/or move
  the sensor back a few mm (a 0 is treated as "cannot measure", never as "very close");
* it **clamps on a pin or cup going past** -> raise `AUTOCLAMP_CONFIRM_MS` (50 -> 80) first;
* it **re-grabs what it just put down** -> raise `AUTOCLAMP_REARM_MM` / `AUTOCLAMP_REARM_CLEAR_MS`;
* **nothing ever happens** -> check the `AC:` field. `OFF` means it is disabled (press Y). `CLEAR?`
  means it is waiting for a clear reading -- take everything away from the jaws. `ARMED` with a pin
  present means the range is too tight. `SEEING` that never becomes a clamp means the reading is not
  staying inside the range long enough.

**On-robot test, driver:** power on, enable driver control. `AC:ARMED` on the `CLAW` line, jaws open.
Move a pin slowly into the jaws: `DIST` falls, `AC:` goes `SEEING`, and about 50 ms later the jaws shut
and the line reads `CLAMPED D[1]  AC:HELD`. Press **A** to open: `AC:CLEAR?`. With the pin still sitting
there it must **not** re-grab. Take the pin away: `AC:ARMED` again. Wave the pin past the jaws quickly:
nothing should happen. Press **Y**: `AC:OFF`, and now nothing clamps at all.

**On-robot test, auton:** in `autons.cpp` use the `MATCH` template with `auton::autoclamp(true)`, an
`{.async = true}` drive at a pin, and `auton::waitUntilClamped(2000, true)`. The robot should drive, stop
**at** the clamp rather than pushing through it, and move on to the next call. With no pin in front of it
the wait returns `false` after 2 s and the route can bail out.

## Lift direction check (do this first)
The lift runs off the **rotation sensor** on port 16, mounted on the 12T motor axle (confirmed, geared 4:1 into the 48T lift-arm axle -- `LIFT_GEAR_RATIO` / `LIFT_ROTATION_RATIO` are both `4.0`). Everything in the code -- presets, PID target/error, the brain's `LIFT` lines -- works in **lift-arm degrees**; that 4:1 is what converts the sensor/motor side down to that one shared unit. If the sensor and the motors disagree on which way is up, the PID pushes the wrong way, the error grows, and the lift drives itself down. That's the "randomly spinning down" problem. To check:
1. Start the program with the lift fully down, and **don't enable**. The brain's `LIFT` line shows the arm position in lift-arm degrees; open the Brain Terminal too (`LIFT_TRACE`) if you want the raw numbers.
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
L2's score, B's stow, an auton move to `DOWN`, and R2 in clamped mode. `DESCEND → FINAL → SEAT`.

**2026-10-02 retune: stay fast much lower.** The arm used to crawl the whole bottom 0–20° (the old
approach band was 12% ≈ 18 deg/s all the way from 20° down to a 6° seat-detect line). Top to bottom now:

| Zone | Speed | Why |
|---|---|---|
| top / mid | `LIFT_DESCENT_TOP_PCT` 80% (≈120°/s) → `LIFT_DESCENT_MID_PCT` 95% (≈142°/s) | unchanged -- the bulk of a long drop |
| approach, `LIFT_DESCENT_SLOW_ZONE_DEG` (14°) → `LIFT_FINAL_START_DEG` (3°) | `LIFT_DESCENT_SLOW_PCT` 30% (≈45°/s) | a real cruise, not a crawl (was 12% / 18°/s from 20°) |
| FINAL, below 3° | `LIFT_FINAL_DEG_PER_S` 12°/s, forced | the last stretch, see below |
| SEAT | `LIFT_SEAT_MV` constant | holding, not moving |

- **DESCEND.** The velocity loop tracks a speed reference read off the arm's **position**, in three
  bands: `LIFT_DESCENT_TOP_PCT` from the start height down to the mid boundary, `LIFT_DESCENT_MID_PCT`
  through the middle, then a constant `LIFT_DESCENT_SLOW_PCT` **approach cruise** from
  `LIFT_DESCENT_SLOW_ZONE_DEG` down to `LIFT_FINAL_START_DEG`. The mid boundary is a **fraction** of
  this descent's fast travel (`SLOW_ZONE + LIFT_DESCENT_MID_FRACTION × (start height − SLOW_ZONE)`), not
  a fixed height, so "the middle" is the middle whether the descent started at level 1 or level 5. Band
  changes are ramps at `LIFT_DESCENT_RAMP_DEG_PER_S2` (**1200 deg/s², was 800**). It is **closed-loop on
  the measured arm speed**, so gravity plus the pin's weight cannot make it run away. L2's post-score run
  home uses `L2_RETURN_SPEED_PCT` (95%) for both fast bands (`RETURN` on the brain / `R` in the trace);
  B's stow and an auton `DOWN` use the 80/95 bands above.
  **Why the ramp had to get firmer.** Decelerating from v₁ to v₂ at rate *a* takes (v₁²−v₂²)/(2a) of
  travel. Worst case entering the approach zone is ≈142°/s. At the old 800 deg/s², braking to 45°/s
  alone takes 11.3° -- almost the whole new 14°→3° zone, leaving no room to then brake 45→12°/s before
  the floor. At 1200, that first brake only takes 7.56°, landing around 6.4° with a real ~3.4° cruise at
  45°/s, then 45→12°/s only needs 0.78° -- settled by ~2.2° with margin to spare before 0. 1200 is not
  arbitrary: it's the same rate already proven safe for position moves (`LIFT_DECEL_DEG_PER_S2`).
- **FINAL** (below `LIFT_FINAL_START_DEG`, 3°). The arm might be on its hard stop, or it might be
  stalled a few degrees short -- a cup caught in the intake, six-bar/chain friction. FINAL is a
  dedicated closed-loop push at a constant `LIFT_FINAL_DEG_PER_S` (12 deg/s), with up to
  `LIFT_FINAL_MAX_MV` (5000 mV) of downward authority -- real force, more than SEAT's hold, but capped
  well under DESCEND's full 12000 mV so a genuine hard-stop contact doesn't slam.
  **Time-based, on purpose (2026-10-02 retune).** FINAL no longer early-exits the moment it looks
  settled -- it forces at that speed/authority for a fixed `LIFT_FINAL_FORCE_MS` (1500 ms, "a second or
  two") regardless, because a stall can look momentarily still without actually being seated. Once the
  window ends it hands straight to SEAT: plain `SEAT` if the arm is within `LIFT_SEAT_ZERO_DEG` (1°) of
  0 at that moment, `SEAT(short)` otherwise (a genuine jam, not just friction -- `LIFT_SEAT_MV` still
  holds it down either way).
- **SEAT.** Hold a constant `LIFT_SEAT_MV` (−1500 mV) pressed into the stop, forever, so it **cannot
  bounce back up**. It no longer parks a few degrees above the frame (that was FINAL's whole point).
  −1500 mV is 12.5% of full volts, still well under an amp per motor, and the frame is carrying the arm
  -- safe for a whole match. If a lift motor ever reaches `LIFT_SEAT_TEMP_LIMIT_C` the seat drops to a
  plain brake-hold and the phase reads `SEAT(hot)`. An arm knocked `LIFT_SEAT_UNSEAT_DEG` back up above
  `LIFT_FINAL_START_DEG` restarts the descent (DESCEND, then FINAL again) and walks it back down.

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

### Why the first contact detector could never fire (fixed, don't reintroduce)
It compared the measured speed against the **nominal** commanded speed (~90 deg/s): a relative trigger
armed only once the arm reached 85% of it, and an absolute trigger at 8 deg/s. **Both are dead if the
arm simply cannot make 90 deg/s** -- and on this six-bar it often cannot, because the arm swings up and
*back*, so over much of the arc gravity **resists** the descent and the loop can only push down with
`L2_PRESS_MAX_MV` of authority. A free descent settling at 50 deg/s never armed the relative trigger
(50 < 76.5) and never tripped the absolute one (50 > 8); the landing then slowed it to 15, still over 8.
Nothing could fire, so every score ran to the bottom and reported `NO CONTACT` with the pin still held.
The lesson, and the rule now: **never threshold against the nominal command -- threshold against what
the arm was actually doing a moment ago.**

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
6. **The hold, and how fast it gives up correcting: `LIFT_HOLD_*`, then `LIFT_PARK_*`.** Go to level 3
   and watch the `H` lines, then the `K` lines. **The hold is deliberately short-lived now:** inside
   `LIFT_HOLD_DEADBAND_DEG` (4°) it applies no position correction and freezes its integral, and after
   `LIFT_PARK_CONFIRM_MS` inside that band (or `LIFT_SETTLE_TIMEOUT_MS` of holding, whichever comes
   first) it **PARK**s -- a frozen constant voltage, no feedback at all, which is why it cannot hunt.
   The brain shows `PARK`; the trace shows `K` lines with a constant `mv`.
   - **it still oscillates at a level → lower `LIFT_SETTLE_TIMEOUT_MS`** (700 → 400) and
     `LIFT_PARK_CONFIRM_MS` (150 → 80). That is "park sooner", i.e. less feedback for less time, and it
     is the first knob for this symptom now -- not the gains. If the `H` lines show the arm crossing the
     goal back and forth before `K` appears, that is the limit cycle; parking sooner ends it.
   - any bounce in the last moment of the move itself (the `A` lines, before `H`) → raise
     `LIFT_HOLD_KD` (90 → 120 → 150), or lower `LIFT_STOPPED_DEG_PER_S` (12 → 8) for a gentler handover
   - it parks visibly off the level → lower `LIFT_HOLD_DEADBAND_DEG` (4 → 2.5), or raise
     `LIFT_SETTLE_TIMEOUT_MS` so the active hold gets longer to pull in first
   - it sags away *after* parking → raise `LIFT_HOLD_KI` (400 → 600) so the voltage it freezes is
     bigger, or lower `LIFT_REENGAGE_DEG` (8 → 5) so the hold wakes sooner
   - it wakes and re-parks repeatedly (`K` → `H` → `K` on the brain) → raise `LIFT_REENGAGE_DEG` and
     `LIFT_PARK_COOLDOWN_MS`
   - it audibly strains at a high level → lower `LIFT_HOLD_MAX_MV`, and accept some sag
   - **INTAKE is exempt from the wide band:** a precise goal uses
     `LIFT_HOLD_PRECISE_DEADBAND_DEG` (0.5°) and keeps correcting right in, then parks like everything
     else. Only the band differs.
   - a move that never arrives (preset above the real top, a jam the stall guard missed) stops chasing
     after `LIFT_MOVE_TIMEOUT_MS` and parks where it is -- brain `PARK(t/o)`. If a legitimate long move
     ever gets cut short, raise it.
7. **The descent: the `LIFT_DESCENT_*` bands.** Clamp nothing, go to level 5 with `L1`, press `L2` and
   watch the `V` lines, then the `S` lines.
   - too fast off the top → lower `LIFT_DESCENT_TOP_PCT`
   - the middle is slower than the top (shouldn't be) → check `LIFT_DESCENT_MID_PCT` > `_TOP_PCT`
   - any thump entering the approach band → raise `LIFT_DESCENT_SLOW_ZONE_DEG` (14 → 18) and/or raise
     `LIFT_DESCENT_RAMP_DEG_PER_S2` further so it finishes braking with more margin before the zone ends
   - the whole descent is too slow → raise `LIFT_DESCENT_MID_PCT`, lower `LIFT_DESCENT_SLOW_ZONE_DEG`,
     or raise `LIFT_DESCENT_SLOW_PCT` (the approach cruise itself, 30%)
7a. **The coarse rise (`auton::liftToFrontToggle()`).** An ordinary rise is already at full voltage
   off the line; what makes it feel measured is the braking, which starts `150²/(2a) + zone` degrees
   out -- 14.4° of a 30° move at `a = 1200`, i.e. half of it. A coarse move brakes later, skips the
   crawl and latches a wide band, and is allowed to sail past.
   - overshoots too far → lower `LIFT_COARSE_DECEL_DEG_PER_S2` (2600 → 1800), then
     `LIFT_COARSE_MIN_DEG_PER_S`
   - still no faster than a normal rise → raise `LIFT_COARSE_DECEL_DEG_PER_S2`
   - note 2600 deliberately asks for more deceleration than the arm can deliver. On a normal move that
     is a bug (it is what `3723ef2` did wrong); here it is the point
7b. **The fast descent (`auton::liftDownFast()`, the front toggle's way down).** Only the two fast
   bands and the ramp differ: `LIFT_FAST_DESCENT_TOP_PCT` (105 %, ~157 °/s, vs 80 %/120) and
   `LIFT_FAST_DESCENT_MID_PCT` (120 %, ~180 °/s, vs 95 %/142), with
   `LIFT_FAST_DESCENT_RAMP_DEG_PER_S2` 2000 vs 1200. 120 % of free speed is not a speed the arm can
   reach -- it is how you tell the velocity loop not to hold back, so it pushes with its full
   12 000 mV. From 30° both bands run (the mid boundary is 22°). Watch the `V` lines.
   - it thumps into the approach zone or the floor → **lower `LIFT_FAST_DESCENT_MID_PCT` first**
     (120 → 110), then raise the fast ramp. The two go together: decelerating v1 → 45 °/s needs
     (v1² − 45²)/2a degrees and only ~11° of zone (14 → 3) is available
   - it is not noticeably faster than a normal descent → raise `_MID_PCT`, and the ramp with it
   - the approach cruise, `FINAL` and `SEAT`/`REST` are deliberately identical to the normal descent;
     tune those in steps 8 / 8b, not here
8. **Seating.** Below `LIFT_FINAL_START_DEG` (3°) the phase goes `V` → `Z` (`FINAL`, pushing with up to
   `LIFT_FINAL_MAX_MV` for a fixed `LIFT_FINAL_FORCE_MS`, 1.5 s) → `S` (`SEAT`, holding `LIFT_SEAT_MV`).
   `FINAL` no longer early-exits the moment it looks settled -- it forces the whole window on purpose,
   then judges once by position: within `LIFT_SEAT_ZERO_DEG` (1°) of 0 is plain `SEAT`, anything more is
   `SEAT(short)`. Push the arm up by hand during `SEAT`: it should press straight back down.
   - `LIFT` reads `SEAT(short)` / stops a few degrees short every time → first confirm the `LIFT` line
     says `HOMED` (an un-homed seed makes "0" a lie), then raise `LIFT_FINAL_MAX_MV` (5000 → 6500), then
     `LIFT_FINAL_FORCE_MS` (1500 → 2000). If a cup in the intake is the actual obstruction, check the
     rollers are still ejecting up to `L2_OUTTAKE_STOP_DEG` -- see `L2_OUTTAKE_*` below.
   - `FINAL` thumps into the frame → lower `LIFT_FINAL_DEG_PER_S` (12 → 8)
   - still bounces up off the stop once truly `SEAT`ed → more negative `LIFT_SEAT_MV` (−1500 → −1800)
   - `SEAT` appears while the arm is still a few degrees up (not `SEAT(short)` -- a real false arrival)
     → lower `LIFT_SEAT_ZERO_DEG`
   - lift motors warming while parked at the bottom (check them by hand) → see `REST` below; the seat
     only lasts `LIFT_REST_AFTER_MS` now, so this should no longer happen
8b. **`REST`: loose motors at the bottom.** `LIFT_REST_AFTER_MS` (300 ms) after the seated arm has gone
   still, the phase becomes `REST`: brake mode **coast**, 0 V, no PID and no seat push, so the motors
   draw nothing at all for the rest of the match. The frame holds the arm. Trace: `E` lines with
   `mv 0`. With the robot disabled-but-powered after an auton, the arm should feel **completely free**
   by hand at the bottom, and stiff everywhere else.
   - the arm **creeps up** off the stop, or the brain cycles `REST` → `FINAL` → `SEAT` → `REST`: this
     lift's bands pull up hardest at the folded position, and coasting lets them. `LIFT_REST_WAKE_DEG`
     (4°) re-seats it at most `LIFT_REST_MAX_WAKES` (3) times before it just stays `SEAT`ed. If it
     happens at all, **set `LIFT_REST_AFTER_MS = 0`, which turns `REST` off entirely** and restores the
     old always-pressed behaviour.
   - it says `SEAT` and never `REST`: the stillness test never passed -- raise
     `LIFT_REST_STILL_DEG_PER_S` (3 → 6).
9. **Rapid taps.** From the bottom, clamp with `L1`, then double-tap and triple-tap `L1`. The `LIFT`
   line's phase must go `DRIVE` → (maybe `BRAKE`) → `DRIVE` again as each tap lands, and never
   `APPROACH` or `HOLD` at an intermediate level. In the trace there must be no `H` line between taps.
   If a tap only takes effect once the arm has arrived, the retarget path in `Lift::setTarget()` is
   being bypassed -- check `blocked_` / `faulted_` aren't set (the brain shows them).
10. **`INTAKE`.** Tap `R1` and confirm the last `A` line's `pos` is within
   `LIFT_PRECISE_TOLERANCE_DEG × 10` of `goal`. If `INTAKE` itself is in the wrong place, that is a
   preset, not a gain -- see "Tuning the claw positions".

### Tuning the L2 scorer
Everything here is in the `L2_*` block of `config.hpp`. Capture an L2 score with the trace open (next
section) and read the **`aux`** column -- it is the measured/baseline speed ratio in percent, so you
can see the exact tick contact fired.

**Two independent layers, not one.** The detector (below) is the fast path -- a real landing, caught
early. `L2_AUTO_RELEASE_DEG` (32°, 2026-10-02) is the floor under it: if PRESS has not been detected
once the arm has travelled 32° **down from `scoreStartPos_`** -- the position captured the instant L2
was first pressed, fixed for the whole score -- it releases anyway, right there (shown as `REL(32)`),
exactly as if contact had fired. **This is RELATIVE travel, not an absolute reading** (corrected
2026-10-02 -- the first version wrongly compared against the absolute 32° mark): every score gets the
same 32° budget to be detected, whatever height it starts from. A start too low for 32° of travel to
ever happen before the floor (level 1, 31.0°) instead falls back to `L2_RELEASE_FLOOR_DEG` (10°, shown
as `REL(floor)`), so level 1 still gets real sensing time -- roughly 21°, since it can't travel the
full 32 before hitting 10 -- rather than releasing on the very first tick like the old (wrong) version
did. There is no more "pressed all the way down with nothing released" outcome either way.

**AGREEMENT** (2026-10-02, `contact_detector.hpp`): a lone `LAG`/`EFFORT`/`SPEED` signal is no longer
enough to fire on its own -- at least two of the three must be over threshold on the *same* tick
(`STALL`, the absolute near-stop, is exempt). Fix for the robot false-firing in plain free descent: the
six-bar's gravity torque swings up and back past vertical, which can move a single signal fast enough
to look like a landing with nothing under the arm at all.

**ARM ON MOTION + SPLIT STALL** (2026-10-02, `contact_detector.hpp`): a second, unrelated false-fire --
*holding a pin + cup* auto-released almost immediately, every time, even though an *empty* claw was
correct in both directions (fired on real resistance, didn't fire in the air). The load adds directly
to the gravity resistance over the part of the arc this six-bar already fights descending, and
`L2_PRESS_MAX_MV` caps how hard the loop may push back -- so a loaded arm can sit under
`L2_CONTACT_STALL_DEG_PER_S` for longer than an empty one while still perfectly free, and the one
signal exempt from AGREEMENT (`STALL`, by design, so a pin already resting is still caught) read that
as resting on something. Two changes: `armed_` now also requires `L2_CONTACT_ARM_TRAVEL_DEG` of real
cumulative travel, not just one lucky instantaneous sample over the stall floor; and STALL itself
splits in two -- once the arm has armed_ at all, a later stop is unambiguous and still confirms fast,
but a STALL that has *never* seen the arm get moving waits the much longer `L2_CONTACT_NOSTART_MS`
before it counts, giving a loaded arm time to prove it is still (slowly) accelerating. `L2_PRESS_MAX_MV`
was also raised 5000 → 7000 mV so a loaded arm has a real chance to clear the stall floor at all.

Because `L2_AUTO_RELEASE_DEG` now guarantees a release regardless, the detector is allowed to be the
more conservative side of every one of these trades -- a real but gentle (or loaded, or slow-starting)
landing it is slow to confirm is a later release, not a lost one.

| What you see | Change |
|---|---|
| **Releases early, pin still in the air** | read the `trg` column first and fix only the signal that fired (remember AGREEMENT means this is rarer now -- a lone signal can't do it alone). `spd` → lower `L2_CONTACT_SPEED_FRAC` (0.55 → 0.45). `lag` → raise `L2_CONTACT_LAG_DEG` (2.2 → 3.0). `eff` → raise `L2_CONTACT_EFFORT_MV` (1800 → 2400). Any of them → raise `L2_CONTACT_MS` (30 → 50) |
| **Presses on until `REL(32)`/`REL(floor)` forces it / never detects a real landing** | lower `L2_CONTACT_LAG_DEG` and `L2_CONTACT_EFFORT_MV` **together** -- raising just one doesn't help once AGREEMENT needs two. Check the trace: if only ONE of `lag`/`eff`/`v-b` ever crosses its threshold on a real landing, that is AGREEMENT rejecting it for lack of a second signal, not a threshold that's merely too tight |
| **False-fires in plain free descent, nothing under the arm (empty claw)** | the failure AGREEMENT and the 2026-10-02 retune exist to fix. Read the trace `trg` column: if it took two signals agreeing, raise whichever was closer to its threshold; if it still happens, lower `L2_CONTACT_FREEZE_FRAC` (0.82 → 0.75) so the baseline tracks a genuine gravity swing longer before locking, or raise `L2_CONTACT_MS` further |
| **Fires almost immediately while HOLDING a pin+cup, over bare floor** | the loaded-claw failure `L2_CONTACT_ARM_TRAVEL_DEG`/`L2_CONTACT_NOSTART_MS` exist to fix -- check the trace `trg` column reads `stall`. Raise `L2_CONTACT_NOSTART_MS` (350 → 500) first; if the loaded arm genuinely can't clear `L2_CONTACT_STALL_DEG_PER_S` within that, raise `L2_PRESS_MAX_MV` (7000 → 8500) so it has real authority to out-run gravity under load |
| **Fires during the spin-up** | raise `L2_CONTACT_SETTLE_MS`, `L2_CONTACT_IGNORE_MS`, or `L2_CONTACT_ARM_TRAVEL_DEG` |
| **Presses too hard / knocks the tower** | lower `L2_PRESS_MAX_MV` first, then `L2_SCORE_SPEED_PCT` (60 → 45) |
| **Pin doesn't seat fully before the release** | raise `L2_PRESS_MAX_MV`; a higher `L2_CONTACT_MS` also presses a little longer before releasing |
| **`L2_AUTO_RELEASE_DEG`/`L2_RELEASE_FLOOR_DEG` fires before a real landing the detector should have caught** | that is a `L2_CONTACT_*` tuning problem (above), not these constants -- the detector is supposed to win first. Raising them only widens the no-sensing region, it doesn't fix detection |
| **Keeps getting stuck** (`UNSTK` lines, or `SCORE STUCK`) | raise `L2_UNSTICK_UP_DEG` (20 → 30) so the hop clears the pin, raise `L2_MAX_UNSTICK_TRIES`, and check `CLAW_OPEN_TIME_MS` is long enough for the jaws to actually open (the freed check starts counting from it) |
| **`SCORE STUCK` with the arm obviously free** | lower `L2_FREE_MS`, or raise `L2_STUCK_MS` so it waits longer before declaring a jam |
| **Goes home too slowly after scoring** | raise `L2_RETURN_SPEED_PCT` (95 → 100); if it then thumps, raise `LIFT_DESCENT_SLOW_ZONE_DEG` |

**After changing any `L2_CONTACT_*` value, run `test/run.sh`.** It compiles the detector with the Mac's
own `clang++` (no PROS toolchain needed) and drives it through synthetic 10 ms traces: free descent at
55 and 90 deg/s with noise, spin-up only (from a clean standstill and from HOLD's residual creep), a
hard/soft/creep landing, a pin already resting (now confirming at the longer `L2_CONTACT_NOSTART_MS`
window, not the fast one -- see "ARM ON MOTION + SPLIT STALL" above), a slow gravity drift, a
gravity-torque *speed* swing (90→50→90 deg/s over 1 s -- the real empty-claw false-fire shape), a
gravity-torque *effort* swing (several 1000 mV with flat speed), heavier (±8 deg/s) noise, and four
**loaded-claw** scenarios: a slow loaded spin-up (only ~45 deg/s by ~200 ms), a loaded free descent with
a much higher effort baseline, a loaded landing (must still fire, and fast), and a loaded arm that never
starts at all (must still fire, just later). It prints the trigger and the detection latency for each
and fails if any of them fires when it shouldn't, fires before the landing, or fires too late. The
thresholds in the test are mirrored from `config.hpp` **by hand on purpose** -- if you change one there
and the test still
passes with the old value, that mismatch is the thing worth noticing.

### Capturing a trace
`LIFT_TRACE = true` prints one integer line per 20 ms to the PROS terminal whenever the arm moves, and
for half a second after. In VS Code: PROS sidebar → **Brain Terminal** (or `pros terminal` from a
shell), press a lift button, copy the lines out. Columns:

```
LIFT <ms> <phase> <pos*10> <goal*10> <vel deg/s> <out mV> <v/base %> <lag*10> <effort mV> <trg>
```

`phase` is `D` drive, `B` brake, `A` approach (the final crawl), `H` hold, `V` descend, `R` descend as
L2's post-score run home, `Z` final (the forced push-to-0 step below `LIFT_FINAL_START_DEG`, always
`LIFT_FINAL_FORCE_MS` long), `S` seat, `P` press (L2 pushing the pin onto the stack), `r` release (L2
after the jaws were asked to open), `U` unstick, `M` manual (D-pad or R1 in clamped mode), `h` homing,
`F` fault, `I` idle. `goal*10` is `-1` when nothing is commanded. **The last four columns are the L2
scorer's view** and are 0 on every other phase: `v/base %` the measured/baseline speed ratio, `lag*10`
the accumulated position shortfall in arm degrees x10, `effort mV` the extra downward push over the
free-descent baseline, and `trg` which signal fired -- `0` none, `1` stall, `2` lag, `3` effort,
`4` speed, `5` the relative-travel rule (`L2_AUTO_RELEASE_DEG`, `REL(32)`), `6` the low-start floor
rule (`L2_RELEASE_FLOOR_DEG`, `REL(floor)`). The first four are exactly what the detector thresholds,
so a trace says precisely which signal moved when the pin landed; `5`/`6` mean the detector never
fired at all and one of the guaranteed-release rules forced it. The lines are written by a separate
low-priority task from a ring buffer; the control loop itself never prints.

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
| The approach band (entering below `LIFT_DESCENT_SLOW_ZONE_DEG`) thumps or overshoots into `FINAL` hot | raise `LIFT_DESCENT_RAMP_DEG_PER_S2` further, or raise `LIFT_DESCENT_SLOW_ZONE_DEG` for more room to brake |
| L2's descent is too slow overall | raise `LIFT_DESCENT_MID_PCT`, lower `LIFT_DESCENT_SLOW_ZONE_DEG`, or raise `LIFT_DESCENT_SLOW_PCT` |
| `LIFT` reads `SEAT(short)` / stops a few degrees short of 0 | check the `LIFT` line says `HOMED` first, then raise `LIFT_FINAL_MAX_MV`, then `LIFT_FINAL_FORCE_MS` -- see "Seating" in step 8 above |
| `FINAL` (the `Z` phase) thumps into the frame | lower `LIFT_FINAL_DEG_PER_S` |
| The arm bounces back up off the bottom after a true `SEAT` | more negative `LIFT_SEAT_MV` |
| Lift motors warm while parked at the bottom | less negative `LIFT_SEAT_MV`; `SEAT(hot)` on the brain means `LIFT_SEAT_TEMP_LIMIT_C` already cut it |
| `SEAT` (not `SEAT(short)`) appears with the arm still a few degrees up -- a real false arrival | lower `LIFT_SEAT_ZERO_DEG` |
| L2's auto-outtake (`INT:OUT`) stops before the arm is near the bottom | it stops at `L2_OUTTAKE_STOP_DEG` (tied to `LIFT_FINAL_START_DEG`), not a seated check any more -- if it still cuts off too early, raise `L2_OUTTAKE_BELOW_DEG`; if it never starts, check `Scoring::isScoreDescent()` is true (not `scoreOrMatchload()` or the at-bottom clamp toggle) |
| Lag between L1's *first* tap and the arm moving | that is `CLAW_CLOSE_TIME_MS` -- the jaws must shut before the arm pulls away |
| L2 releases the pin in mid-air / crushes the stack / never detects contact | see "Tuning the L2 scorer" above -- the `aux` column of the trace is the number to read |
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

**Watch the brain's `LIFT` line:** `HOMING` -> `HOMED`, or `HOME FAIL @ 37.2`. The number is
the absolute angle it saw at the stop — the one the plausibility window was checked against.

| Symptom | Change |
|---|---|
| Stops partway down and says `HOME FAIL` with a large angle | it is catching on something — check the mechanism. If the stop is real but the sensor's calibrated zero is off, re-calibrate the zero on the brain's device screen |
| `HOME FAIL` with a small angle (just outside the window) | raise `LIFT_HOME_ZERO_WINDOW_DEG` |
| Stops early, before the bottom | raise `LIFT_HOME_MIN_DRIVE_MS`, raise `LIFT_HOME_STILL_MS`, or lower `LIFT_HOME_STILL_DEG` |
| Grinds at the bottom before noticing | lower `LIFT_HOME_STILL_MS`, raise `LIFT_HOME_STILL_DEG`, or lower the drive with `LIFT_HOME_MV` |
| Too slow / times out | raise `LIFT_HOME_MV` magnitude (more negative) or `LIFT_HOME_TIMEOUT_MS` |

## Tuning the claw positions
All heights are **lift-arm degrees** in `config::lift_preset` (`include/config.hpp`) -- the number on the brain's `LIFT` line. 0 is the hard stop at the bottom. The 5 `STACK_LEVELS` were **measured on the robot 2026-10-01** by driving to each scoring height and reading the `LIFT` line: **31.0 / 59.0 / 74.6 / 87.3 / 106.9**, with `MAX` = 111.9 as the software stop. They replace the older photo-derived set (26.6 / 54.4 / 72.8 / 116.3 / 128.3), which was taken before homing existed and carried that session's zero offset. `MATCHLOAD` is still a placeholder, and `INTAKE` (1.1) almost certainly needs re-measuring. To (re)measure any of them:
1. Start the program with the lift fully down.
2. Use **D-pad Up/Down** to put the claw exactly where you want it, then read the brain's `LIFT` line for the current position.
3. Type that number into the preset:
   - `INTAKE`: claw lined up with the intake handoff
   - `MATCHLOAD`: claw at matchload height
   - `STACK_LEVELS[0..4]`: the height to drop the 1st through 5th pin on a stack
   - `MAX`: a few degrees above the top level, as a software hard stop -- never raise it just to let a bad preset through
   - **Measure the top first.** Hold Up until the arm is at the top and read the position. Every preset has to be below that number. If the lift shows `BLOCKED` (brain + controller), it was asked to go somewhere it can't physically reach, so it stopped pushing and is holding where it is. Lower that preset.
4. Timing: `CLAW_CLOSE_TIME_MS` (grip before lifting -- specifically the gap between the first L1 tap and the arm moving), `CLAW_OPEN_TIME_MS` (how long the jaws take to open: `scoreOrMatchload()` waits it out, and L2's freed check starts counting from it), `PIVOT_TIME_MS`, and `PIVOT_HANDOFF_ZONE_DEG` (how far above `INTAKE` the pivot swaps between BACK and OUT on the way to / from the intake). **`CLAW_CLOSE_TIME_MS`, `CLAW_OPEN_TIME_MS` and `PIVOT_TIME_MS` have never been measured against the cylinder they now control** -- they were tuned while the two ADI ports were crossed, so treat all three as fresh `TUNE` values.

**Lift gains** are a separate procedure -- see "Tuning the lift" above.

## Brain screen
Four lines, and nothing else unless something is broken (the driver's own layout, 2026-10-03):

```
POSE  X 0.0  Y 0.0  T 0.0
LIFT  31.0 -> 31.0  STACK 1  PARK  HOMED  R:INTAKE
CLAW  CLAMPED  AC:HELD  PIV OUT
DIST  L 152  R 148  BACK 300  CLAW 12 mm
START dx +0.4 in (L 1574/1570)
```

| Line | Shows |
|---|---|
| POSE | odometry X / Y (in) and heading (deg), or `calibrating...` until the IMU is done |
| LIFT | position → target (or `none`), the named preset (or `custom`), the phase -- `DRIVE`/`BRAKE`/`APPROACH`/`HOLD`/`PARK`/`PARK(t/o)`/`GRAB`/`PRESS`/`REL(...)`/`PUSH`/`SCORED UP`/`DESCEND`/`RETURN`/`FINAL`/`SEAT`/`SEAT(short)`/`REST` -- then `FAULT`/`SCORE STUCK`/`BLOCKED` if any, the homing state (`HOMING` / `HOMED` / `HOME FAIL @ n`), and `R:INTAKE`/`R:LIFT`/`R:wait` (which mode R1/R2 are in). If a lift button seems to do nothing, read this line first |
| CLAW | the jaws (`OPEN`/`ARMED`/`CLAMPED`/`RELEASED`), the autoclamp (`AC:OFF`/`ARMED`/`SEEING`/`CLEAR?`/`HELD` -- see "Autoclamp"), and the pivot (`PIV OUT`/`PIV BACK`) |
| DIST | the three field-facing distance sensors and the one in the claw, in mm. `--` = nothing in range or an unusable reading. `BACK` is `moveB`'s entire feedback loop -- `--` there means it would drive blind |
| START | before the match: the **live** left-wall alignment against the selected route's expected reading. After auton has run: the shift `auton::startPose()` applied, or `!! START align skipped`. Blank otherwise -- see "Start alignment" |
| `!! ...` | **only when something is wrong**: a missing or wrong-type device on any port (drive, lift, intake, both rotation sensors, the IMU, all four distance sensors), one line each, at most 3. A healthy robot shows none |

**Everything else moved to the Brain Terminal.** The old ROT / ANGLE / err-vel-out / velocity-reference /
learned-gravity / L2-contact-signals / TUNE / motor-temperature / DRV lines are gone. Every one of
those numbers is in the `LIFT_TRACE` output at 20 ms resolution (VS Code → PROS → **Brain Terminal**,
see "Capturing a trace"), which is a better place to read a moving number than a screen refreshed
every 50 ms. Watch the terminal, not the screen, when tuning.

## Start alignment
The robot is set against the front wall every match, so the distance from the front and the heading
repeat -- but **how far along that wall it ends up does not**. One call at the top of a route measures
that single unknown with the **left** distance sensor and folds it into the starting pose:

```cpp
auton::startPose(MATCH_START_X, MATCH_START_Y, MATCH_START_THETA, MATCH_START_LEFT_MM);
```

* `dx_in = (measured left mm − expected left mm) / 25.4`.
* **Further than expected → the robot is shifted +x; closer → −x.** Because the heading is fixed and
  known, the change in the reading *is* the lateral shift -- no sensor-mounting geometry is involved.
* The shift is applied along the **robot's right axis**, `(cos θ, −sin θ) × dx`, so it stays physically
  correct if a route ever starts facing another way. At θ = 0 that is exactly `x + dx`, which is how
  the driver describes it, and it matches LemLib's frame (at heading 0 the robot's right is +x).
* It **replaces** `chassis.setPose()` in a route -- don't call both.

**Where the reading comes from.** While the robot sits disabled before the match, `disabled()` averages
the left sensor continuously and keeps the last `START_ALIGN_HISTORY_MS` (500 ms). `startPose()` uses
that -- a long average, costing no auton time -- but only if the window is **settled**
(`START_ALIGN_HISTORY_SPREAD_MM`, 40 mm), so a hand still placing the robot or someone walking through
the beam does not count as a measurement. It then takes one short fresh sample
(`START_ALIGN_CONFIRM_MS`, 80 ms) to confirm the robot has not moved since; if the two disagree by more
than `START_ALIGN_CONFIRM_TOL_MM` the fresh one wins. With no disabled history at all (auton launched
straight from the brain menu) it falls back to sampling for `START_ALIGN_SAMPLE_MS` (300 ms).

**It can refuse.** A sample only counts if it is under `START_ALIGN_MAX_MM` (2000) and at least
`START_ALIGN_MIN_CONFIDENCE` (38/63 ≈ 60%) -- the same rule the team's old `accumulateReading()` used --
which also throws out 0, 9999 and `PROS_ERR`. If nothing valid comes back, **or the correction would be
bigger than `START_ALIGN_MAX_IN` (4 in)**, the nominal pose is used unchanged and the brain says
`!! START align skipped`. A sensor looking at a goal instead of the wall must never teleport the pose.

**Calibration ordering.** `chassis.calibrate()` resets the pose, so `startPose()` waits for it
(`START_ALIGN_CALIBRATE_WAIT_MS`), and if calibration is *still* unfinished it applies the pose anyway
and re-applies it once, from a short-lived task, the moment calibration lands
(`START_ALIGN_REAPPLY_WINDOW_MS`).

**Measuring the expected value for a route:**
1. Set the robot in its real starting spot, square against the front wall, and power it on.
2. Read `DIST  L ...` on the brain. Leave it there a few seconds; the `START` line also shows the
   settled average live.
3. Put that number into the route's own constant at the top of `src/autons.cpp` --
   `MATCH_START_LEFT_MM` is `1570` (measured 2026-10-03). They live with the routes, not in
   `config.hpp`, because they are field positions, not robot constants.
4. Test it: shift the robot 1 inch to its **right** and run auton. `POSE` should come up at
   `X ≈ +1` (and the `START` line should read `dx +1.0 in`). Shift it 1 inch left and it should read
   `−1.0`.

**Range note.** At ~1.5 m the V5 Distance Sensor's absolute accuracy is only about ±5 % (±75 mm, ~3 in)
and its beam cone is wide, but *repeatability against the same flat wall* is far better -- and
repeatability is all this needs, because the expected value is measured with the same sensor against
the same wall. Expect roughly ±10–20 mm of per-reading noise at that range, which the 500 ms average
reduces to a couple of millimetres. The 4 in guard is sized for that, not for the absolute spec.

`auton::leftDistanceMm()`, `auton::rightDistanceMm()`, `auton::backDistanceMm()` and
`auton::readLeftAveraged(ms)` are available to routes for anything else.

