# JOKR (57249C) — hardware description

What the real robot looks like, written for someone about to change the code. Every
mechanism is tied to its name and port in `include/config.hpp`. Anything that could not be
read off the photos with confidence is marked **(unclear from photos)** — do not fill those
in by guessing, check the robot.

Photos: [`docs/robot/photos/`](photos/). Five states, listed in "Photo index" at the bottom.

---

## Chassis and drivetrain

A tall, open steel C-channel frame. Roughly square in footprint, with the frame extending
upward into a tower that carries the lift. Four low, angled outrigger plates stick out from
the four bottom corners — two of them carry rows of small free-spinning rollers that act as
a funnel/guide into the intake, not as drive wheels (see
[`02-lift-raised-intake-front.jpg`](photos/02-lift-raised-intake-front.jpg)).

The drive wheels are dark, inboard, and driven through large black spur gears on the axles —
clearly visible on the near side in
[`03-lift-down-stowed-rear.jpg`](photos/03-lift-down-stowed-rear.jpg), three gear/wheel
stations per side. That matches the notebook figure quoted in `config.hpp`: six wheels,
2.75" omnis, 450 RPM, 55 W per side.

Code names (`config.hpp` / `robot.hpp`):

| Thing | Name | Port |
|---|---|---|
| Left drive | `config::LEFT_DRIVE_PORTS`, `robot::leftDrive` | `-8, -2, -1` (all reversed) |
| Right drive | `config::RIGHT_DRIVE_PORTS`, `robot::rightDrive` | `3, 9, 10` |
| Chassis | `robot::chassis` (LemLib) | — |

Geometry constants: `DRIVE_WHEEL_DIAMETER_IN = 2.75`, `DRIVE_RPM = 450`,
`TRACK_WIDTH_IN = 11.5` (marked TODO — measure it), `HORIZONTAL_DRIFT = 2.0`.

Three grey omni wheels are visible along the left side at floor level in
[`02-lift-raised-intake-front.jpg`](photos/02-lift-raised-intake-front.jpg), consistent with
three per side. Whether any of the six are traction rather than omni is **(unclear from
photos)**.

## Odometry and sensors

- **Tracking wheel pod** — `ODOM_VERTICAL_ROTATION_PORT = 17` (confirmed 2026-10-02), exposed as
  `robot::verticalEncoder`. One vertical pod only. I could not positively identify the pod
  in any photo. **(unclear from photos)**
- **Inertial sensor — an IMU IS now fitted, on port 18.** `IMU_PORT = 18`, confirmed
  2026-10-02 (it was `0`, "none fitted", and before that `8`, which was simply wrong -- port 8
  is a drive port). `robot.cpp` constructs a `pros::Imu` whenever `IMU_PORT != 0` and hands it
  to LemLib's `OdomSensors` alongside the vertical pod, so **heading comes from the IMU and
  distance from the pod**. It is calibrated in a background task started from `initialize()`,
  and driver control NEVER waits on it (safety rule 4) -- `autonomous()` does wait, bounded by
  `config::AUTON_CALIBRATE_TIMEOUT_MS`, because a drive move needs a heading. Setting
  `IMU_PORT` back to `0` is still the supported way to run on the tracking wheel alone:
  `robot::imu` becomes a null `std::unique_ptr`, LemLib gets a null IMU pointer, nothing
  calibrates and the brain shows `IMU  none (not installed)`.
- **Field-facing distance sensors** — three, all confirmed 2026-10-03: **left
  `LEFT_DISTANCE_PORT = 15`**, **right `RIGHT_DISTANCE_PORT = 19`** and **rear
  `BACK_DISTANCE_PORT = 14`**, exposed as `robot::leftDistance` / `rightDistance` /
  `backDistance` and all shown on the brain's `DIST` line. The REAR one is the entire
  feedback loop for `auton::moveB()`, which drives until it reads a target distance from
  whatever is behind the robot. The LEFT one is the start-of-auton x correction
  (`auton::startPose()`): the robot is set against the front wall each match, so only its
  position ALONG that wall varies, and the left reading measures exactly that.
- **Lift rotation sensor** — `LIFT_ROTATION_PORT = 16`. The lift PID runs off this sensor,
  not the motor encoders. **It is mounted on the motored 12T axle**, not on the 48T arm axle,
  so it turns 4× for every degree the arm moves:
  **arm degrees = sensor degrees ÷ 4**, i.e. `LIFT_ROTATION_RATIO = 4.0` (`config.hpp` is
  fixed to this value now -- see the note under "Lift"). The Brain Terminal's `LIFT_TRACE` output shows the
  converted arm degrees with the raw sensor degrees in brackets so the 4:1 can be checked by
  hand, and the lift reads the sensor with `Rotation::get_position()` (multi-turn), not
  `get_angle()` (wraps at 360°) -- at 4:1 the sensor turns more than a full revolution over
  the arm's ~105° of travel, so a wrapping read would make the lift think it jumped position.
- **Claw distance sensor (autoclamp)** — `CLAW_DISTANCE_PORT = 21`, **mounted in the claw and
  confirmed 2026-10-03** (it was on port 11 until then; the photos further down predate both,
  which is why their `DIST` line reads `WRONG DEVICE`). It looks out between the jaws and sees
  both pins and cups. The rule:
  anything at or inside the trigger range -- **55 mm in driver control, 65 in autonomous, 75 at the R1
  handoff**, all measured on the robot 2026-10-03 -- for `AUTOCLAMP_CONFIRM_MS` (50) over at
  least `AUTOCLAMP_MIN_SAMPLES` (2) fresh readings closes the jaws. Off at power-on
  (`CLAW_AUTOCLAMP_AT_START = false`), **on** throughout driver control
  (`AUTOCLAMP_ON_IN_DRIVER`, Y toggles it) **and on by default in auton too**
  (`AUTOCLAMP_ON_IN_AUTON`, 2026-10-03). Two things follow a clamp: the arm hops up
  `CLAMP_RISE_DEG` (5°) everywhere, and **at the R1 intake handoff** it fires instantly instead
  (`AUTOCLAMP_INTAKE_CONFIRM_MS` 0) and forces the claw *down* for `AUTOCLAMP_INTAKE_PUSH_MS`
  (300 ms) rather than lifting. See README
  "Autoclamp" -- and note that the clear-to-rearm threshold is DERIVED as (range +
  `AUTOCLAMP_REARM_GAP_MM`, 30), so it is always further out than a pin whichever range is in force. A
  clear threshold at or inside the range makes the claw re-grab whatever it just released, forever.

Visible electronics: the V5 brain sits low and flat in the middle of the chassis, screen
facing up ([`05-cup-in-claw-brain-screen.jpg`](photos/05-cup-in-claw-brain-screen.jpg)).
Several black V5 smart motors are bolted around the lower frame. A grey/white cylindrical
air reservoir sits in the lower frame near the rear, and at least two silver pneumatic
cylinders are visible in the frame. One black module on the rear right shows a red LED —
most likely the pneumatic solenoid or the radio. **(unclear from photos)**

## Intake

A front-mounted, full-width roller intake set into the front face of the chassis, behind the
two angled corner guide bars. Looking down into it
([`02-lift-raised-intake-front.jpg`](photos/02-lift-raised-intake-front.jpg)) you can see a
horizontal shaft carrying several flat black paddle blades with light flexible flaps between
them, and a second roller at the back wrapped with an orange flex band. The whole assembly is
chain-driven from the right-hand side — a chain run is clearly visible along the top of the
intake box. `config.hpp` and `intake.hpp` describe it as "three-stage roller intake with a
bottom counter roller, driven by ONE motor", which is consistent with what the photos show.

Cups are swept in at floor level at the front, carried up and rearward over the paddles, and
land in the middle of the robot, at the handoff spot. The handoff height is
`config::lift_preset::INTAKE` (currently `5.0` lift degrees, TUNE). The claw itself grips
**pins**, not cups — see "Game elements" below.

| Thing | Name | Port |
|---|---|---|
| Intake motor | `config::INTAKE_PORT`, `robot::intake` (`subsystems::Intake`) | `12` |
| Intake power | `config::INTAKE_MV` | `12000` mV |

Driver-controlled only: **R1** in, **R2** out. Nothing automatic ever turns the intake on.

## Lift

A **DR6-4B** (six-bar) built out of long steel C-channel, rising from the tower at the rear
of the chassis. Two long parallel bars form the main pair; a second set of shorter linkage
bars at the top holds the end effector level as the arm sweeps up
([`01-lift-full-up-rear.jpg`](photos/01-lift-full-up-rear.jpg) shows the upper linkage and
the sprocket at the joint). A chain runs along the arm to the end effector — this is what
keeps the claw pointing the same way through the whole travel
([`04-claw-arm-extended-cup-on-floor.jpg`](photos/04-claw-arm-extended-cup-on-floor.jpg)).

Driven by two motors on a high-strength axle, geared **12T → 48T (4:1 torque)**. The motors
drive the 12T axle; **the arm is on the 48T axle**. Red cartridges (commit `505ef6c`).

| Thing | Name | Port |
|---|---|---|
| Lift motors | `config::LIFT_PORTS`, `robot::lift` (`subsystems::Lift`) | `20, -13` (13 reversed) |
| Lift rotation sensor | `config::LIFT_ROTATION_PORT` (on the 12T motor axle) | `16` |

> **`config.hpp` is fixed to match this 4:1 build:** `LIFT_GEAR_RATIO = 48.0 / 12.0` and
> `LIFT_ROTATION_RATIO = 4.0` (the sensor is on the motored 12T axle, so it reads 4 sensor
> degrees per arm degree). Everything downstream -- presets, PID gains, the stall and
> runaway thresholds -- is in lift-arm degrees via those two numbers, so re-check this note
> first if either ratio is ever touched again.

The rotation sensor's absolute zero has been calibrated (on the brain's device screen, not in
code) to the true physical bottom of the lift and persists across power cycles on its own —
`Lift::zero()` only seeds the software position counter from that absolute angle at startup, it
never resets or re-references the sensor's own zero. See the brain's new `ANGLE` line and the
comment in `subsystems/lift.cpp`'s `zero()`.

**Down** = the arm folded forward and low over the chassis, end effector near the intake
handoff in the middle of the robot
([`03-lift-down-stowed-rear.jpg`](photos/03-lift-down-stowed-rear.jpg)). This is where the
robot is switched on, and it defines `0.0` lift degrees.

**Up** = the arm swung up and back, reaching well above the robot — in
[`01-lift-full-up-rear.jpg`](photos/01-lift-full-up-rear.jpg) it is roughly head height on a
standing person. All preset heights are in **lift-arm degrees from that resting position**:

```
DOWN           0.0    resting / start
INTAKE         6.1    claw at the intake handoff (R1 tap) -- driver-adjusted 2026-10-03, was 1.1
MATCHLOAD     15.0    claw at matchload height -- TUNE, not yet measured
STACK_LEVELS  31.0 / 59.0 / 74.6 / 87.3 / 106.9   levels 1..5 (L1 steps through these)
                       MEASURED ON THE ROBOT 2026-10-01 (arm degrees, the unbracketed LIFT/ROT
                       number), replacing the photo-derived set below
MAX          111.9    software hard stop: top level (106.9) + 5 deg margin
```

`DOWN` and the 5 `STACK_LEVELS` are measured, not guessed -- see "Lift levels" below
for the 7 reference photos they were read off. `MATCHLOAD` is still marked TUNE in `config.hpp`;
it wasn't covered by that photo set, and `INTAKE` is now **6.1** (the photo's 1.1 plus the driver's
"go up 5 degrees more", 2026-10-03) -- by eye, not surveyed, so it still deserves
re-measuring off the brain's `LIFT` line.

Driving it: **D-pad Up/Down (hold)** are plain manual power and always win -- holding either cancels
whatever the lift was doing. Up is `LIFT_MANUAL_UP_PCT` (100%), down `LIFT_MANUAL_DOWN_PCT` (**50%**,
the driver's call 2026-10-03: gravity already helps downward); neither = brake/hold. Everything else (**L1**
step up a stack level, **L2** score and return home, **R1** tap to go to INTAKE, **B** stow to
DOWN) goes through the PID. See "Safety rules" in `CLAUDE.md` and the README "Driver guide" --
this single-owner split is deliberate and was hard-won.

## Lift levels

Measured by powering the robot on, moving the arm to each named height by hand, and reading
the brain's `LIFT` line (already converted to lift-arm degrees). Photos:
[`photos/levels/`](photos/levels/).

| Photo | Named height | Brain `LIFT` line read | `config::lift_preset` |
|---|---|---|---|
| [`levels/0-start.jpg`](photos/levels/0-start.jpg) | start / rest | `LIFT 0.1 -> none` | `DOWN = 0.0` |
| [`levels/1-intake.jpg`](photos/levels/1-intake.jpg) | intake handoff | `LIFT 1.1 -> none` | `INTAKE` was 1.1 here; now **6.1** (driver, 2026-10-03) |
| [`levels/2-score-L1.jpg`](photos/levels/2-score-L1.jpg) | score level 1 | `LIFT 26.6 -> none` | `STACK_LEVELS[0] = 26.6` |
| [`levels/3-score-L2.jpg`](photos/levels/3-score-L2.jpg) | score level 2 | `LIFT 54.4 -> none` | `STACK_LEVELS[1] = 54.4` |
| [`levels/4-score-L3.jpg`](photos/levels/4-score-L3.jpg) | score level 3 | `LIFT 72.8 -> none` | `STACK_LEVELS[2] = 72.8` |
| [`levels/5-score-L4.jpg`](photos/levels/5-score-L4.jpg) | score level 4 | `LIFT 116.3 -> none` | `STACK_LEVELS[3] = 116.3` |
| [`levels/6-score-L5.jpg`](photos/levels/6-score-L5.jpg) | score level 5 (top) | `LIFT 128.3 -> none` | superseded: `STACK_LEVELS[4] = 106.9` |

> **The photo-derived numbers in this table are superseded.** They were read on a build whose
> `Lift::zero()` still reset the rotation sensor at every boot, so every one of them carries whatever
> offset the arm happened to be sitting at during that session -- the caveat written below. On
> 2026-10-01, with homing working and the sensor's own absolute zero in use, the driver drove to each
> scoring height and read the brain's `LIFT` line: **31.0 / 59.0 / 74.6 / 87.3 / 106.9** arm degrees.
> Those are what `config::lift_preset::STACK_LEVELS` holds. The photos are still the right reference
> for *what each level looks like*, just not for the numbers.

The readings rise the whole way from `DOWN` to the top, which is the direction check too --
confirms `LIFT_ROTATION_REVERSED = false` is correct as committed.

**Caveat:** these photos were taken on a build where `Lift::zero()` still reset the rotation
sensor's position to 0 at every boot (startup-relative), before it was changed to seed from the
sensor's own persistent absolute zero instead (see "Lift" above and the comment in
`subsystems/lift.cpp`'s `zero()`). If the arm wasn't sitting exactly at the true bottom when the
robot was powered on for this photo session, every reading in the table carries that same
constant offset. Re-measure against the brain's `LIFT` line (now reading off the real absolute
zero) if any of these look off on the robot.

Two things worth a physical look, found while reading these photos, neither of which affects
the heights above (the brain's `LIFT`/`ROT` lines are the rotation sensor, which is what every
preset and the PID actually use):

- Every photo's `DIST` line reads `WRONG DEVICE on port (port 11)` and `ODOM` reads
  `not connected (port 7)` -- the claw distance sensor and the odometry tracking wheel were
  both disconnected (or something else was on port 11) at the time of these photos. **Both are
  fitted now** (claw sensor on 11, pod on 17, plus an IMU on 18), so those readings are history,
  not the current state of the robot.
  **(unclear from photos -- check the robot)**
- The brain's `ROT`/`MOT` line shows the rotation-sensor-based reading running almost exactly
  **2x** the motor-encoder-based reading at every level (e.g. level 5: `ROT 128.3 [513]` vs
  `MOT 63.9 [256]`), not the 1:1 you'd expect from two sensors on the same 12T axle. Possible
  causes: the two lift motors' encoders disagreeing (`robot::lift`'s `MotorGroup::get_position()`
  averages them, so one motor undercounting would read as roughly half), or the rotation sensor
  isn't geared 1:1 with the motors after all. **(unclear from photos -- check the robot)**

## Claw and pivot

At the end of the lift arm is a small pivoting end effector. In
[`04-claw-arm-extended-cup-on-floor.jpg`](photos/04-claw-arm-extended-cup-on-floor.jpg) it is
a short bracket on a cross shaft carrying a single curved flat plate, hanging off the outer
end of the arm with the chain run feeding into it.

**The claw grips the pin, not the cup.** In
[`05-cup-in-claw-brain-screen.jpg`](photos/05-cup-in-claw-brain-screen.jpg) a yellow pin —
and nothing else, no cup with it — is held on its side between two angled grey bars just
above the brain. The jaws close on the pin's fluted body, in the waist between the hex head
and the flared base skirt; the skirt and the head stop the pin sliding out endways once the
jaws are shut. That is exactly the grip `claw.hpp` describes ("the pin arrive", "the pin is
between the jaws") and what `AUTOCLAMP_RANGE_MM` is meant to detect.

I still could not count the cylinders on the end effector itself from these angles.
**(unclear from photos)**

| Thing | Name | Port |
|---|---|---|
| Claw cylinder | `config::CLAW_PISTON_PORT`, `robot::claw` (`subsystems::Claw`) | ADI **D** |
| Pivot cylinder | `config::PIVOT_PISTON_PORT` (owned by `subsystems::Scoring`) | ADI **C** |
| Claw distance sensor | `config::CLAW_DISTANCE_PORT` (mounted in the claw; moved to 21 on 2026-10-03, was 11) | `21` |
| Left / right / rear distance sensors | `config::LEFT_/RIGHT_/BACK_DISTANCE_PORT`, `robot::leftDistance` etc. | `15` / `19` / `14` |

> **The two cylinders were on each other's ports in code until 2026-10-01.** The port the code
> called the claw ('C') is physically the **pivot**, and the port it called the pivot ('D') is
> physically the **claw**. The table above is the corrected assignment. Every polarity note written
> before that date was derived from the wrong pin and has been re-derived from scratch; `config.hpp`
> carries the full evidence chain on each constant. If anything about the claw or pivot in an older
> commit message contradicts this section, this section is right.

- **Claw** (ADI **D**), driver button **A** toggles clamp/open. **The jaws need air to grip, so the
  cylinder's unpowered position is OPEN**: `CLAW_CLOSED_IS_ADI_HIGH = true`. Derivation: every build
  ever run wrote both ADI pins LOW at construction, so power-on is LOW on both. On the `3723ef2`
  build, R1/INTAKE drove D HIGH (the old "pivot" object's BACK level) and the driver reported the
  clamp *engaged* at the intake where the code wanted it open -- so HIGH on D closes the jaws. The
  claw is therefore physically open at power-on, `Claw::State::OPEN`, and R1 works straight away
  without a special case. A pin preloaded for auton has to be gripped with A or L1; the claw cannot
  hold it with the program off. TODO: confirm with A -- the brain's `CLAW` line reads `OPEN D[0]` at
  power-on and must read `CLAMPED D[1]` with the jaws shut.
- **Pivot** (ADI **C**), driver button **X**, swings the end effector **OUT** and **BACK**. OUT
  (`true` in code, ADI **LOW**) = swung out: where it sits at power-on, at every stack level while
  scoring, and in the default down position after L2 / B. BACK (`false`, ADI **HIGH**) = pulled in
  toward the intake handoff, only at INTAKE (R1). `PIVOT_OUT_IS_ADI_HIGH = false`, confirmed twice
  from different runs: (1) at power-on C is LOW and the driver's own observation is that the pivot
  sits OUT; (2) on the `6b8165c` build `Claw::clamp()` drove C HIGH at every stack level, and the
  driver reported the pivot sat *pulled in* at every stack level. The brain's `CLAW` line shows
  `PIV OUT C[0]` / `BACK C[1]`. `PIVOT_BACK_AT_MATCHLOAD = false` (TODO). The only height tied to
  the pivot is the handoff zone, `INTAKE` to `INTAKE + PIVOT_HANDOFF_ZONE_DEG`: heading to or at
  the intake the pivot is BACK inside it and OUT above it, so it swings BACK as R1 brings the arm
  down into the zone and OUT as L1 lifts it out (or when the arm arrives at the level). It never
  gates the lift.
- Timing knobs: `PIVOT_TIME_MS = 250`, `CLAW_CLOSE_TIME_MS = 150`, `CLAW_OPEN_TIME_MS = 200`.
  **None of them has ever been measured against the cylinder it now controls** -- they were tuned
  while the ports were crossed, so treat all of them as `TUNE` starting values.
  `CLAW_CLOSE_TIME_MS` was put back up from 100 to 150 for exactly that reason. `CLAW_OPEN_TIME_MS`
  matters more than it used to: L2's scorer starts its "did the claw come free of the pin" check that
  long after it asks for the release, so if it is shorter than the real jaw-open time L2 will judge
  freedom while the jaws are still closing.

## Game elements: the pin and the cup

These are **two separate objects**. Both appear in the photos, and in three of them they
appear together, which is what made them easy to confuse for one two-tone part.

**The pin** — solid opaque **yellow**, and the thing the lift and claw actually carry. Shape,
from the top down: a faceted hexagonal knob head with a small central boss; a tapered fluted
body below it (deep vertical flutes, roughly a cone); and a wide flared circular skirt at the
bottom, like the base of a traffic cone. Seen clearest standing upright in
[`04-claw-arm-extended-cup-on-floor.jpg`](photos/04-claw-arm-extended-cup-on-floor.jpg) and
on its side in the claw in
[`05-cup-in-claw-brain-screen.jpg`](photos/05-cup-in-claw-brain-screen.jpg).

**The cup** — a separate tapered vessel with ribbed/fluted walls and a rim that flares out at
the bottom. It is a **different colour from the pin in every photo**: clear/translucent in
photo 04, red in photo 01. The pin drops down *into* the cup and sits there with its yellow
skirt resting on the cup's rim, the yellow body inside the cup and the hex head standing
proud above it.

| Photo | Pin | Cup |
|---|---|---|
| [`01`](photos/01-lift-full-up-rear.jpg) | yellow, held in a team member's hand by the head | **red**, the pin is seated in it |
| [`04`](photos/04-claw-arm-extended-cup-on-floor.jpg) | yellow, upright | **clear/translucent**, standing on the floor with the pin seated in it |
| [`05`](photos/05-cup-in-claw-brain-screen.jpg) | yellow, on its side, gripped by the claw | **none** — the pin is held alone |

So: **the claw carries the pin; the intake handles the cups.** Photo 05 is the decisive one —
the claw is holding a bare pin with no cup attached, which is only possible if they separate.

What is **(unclear from photos)** is the scoring relationship: whether cups stack onto a pin,
or a pin is placed into a stack of cups, or both — no stack appears in any photo.
`config.hpp`'s `STACK_LEVELS` comments now consistently say "pin" (they used to say "cup" in
one place, which was a leftover from before the two were told apart; fixed). Either way,
`lift_preset::STACK_LEVEL_COUNT = 5`, one preset lift height per level, measured from the
"Lift levels" photos above.

---

## Photo index

| Photo | Robot state |
|---|---|
| [`01-lift-full-up-rear.jpg`](photos/01-lift-full-up-rear.jpg) | Lift at full height, arm swung up and back well above the chassis; upper six-bar linkage and joint sprocket in view. Someone is holding a **yellow pin seated in a red cup** beside it for scale. Brain visible on the chassis below. |
| [`02-lift-raised-intake-front.jpg`](photos/02-lift-raised-intake-front.jpg) | Lift raised partway, looking down into the front of the robot: the roller intake with its black paddle blades, flexible flaps and orange flex-band roller, the chain drive along its side, and the angled corner guide-roller bars. |
| [`03-lift-down-stowed-rear.jpg`](photos/03-lift-down-stowed-rear.jpg) | Lift fully down and stowed over the chassis — this is the `0.0` power-on position. Rear three-quarter view showing the drivetrain spur gears on one side, a pneumatic cylinder across the frame, and the corner outriggers. |
| [`04-claw-arm-extended-cup-on-floor.jpg`](photos/04-claw-arm-extended-cup-on-floor.jpg) | Lift low, arm swung out sideways over the floor with the chain run and the pivoting end effector at its tip. A **yellow pin seated in a clear cup** stands on the floor beside the robot. |
| [`05-cup-in-claw-brain-screen.jpg`](photos/05-cup-in-claw-brain-screen.jpg) | Close front view, a hand steadying the lift. A **bare yellow pin, no cup**, is gripped on its side in the claw directly above the V5 brain; the brain screen shows the `JokrOverride` PROS program in slot 1. |

Note: the filenames of photos 04 and 05 say "cup". They were named before the pin and the cup
were told apart — the object in the claw in 05, and the yellow object in 04, are both the
**pin**. The filenames are left alone so existing links keep working.

All five JPEGs were re-encoded from the original HEICs with the camera orientation baked into
the pixels and **all EXIF/GPS metadata removed**.
