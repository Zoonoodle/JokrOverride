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
| Left drive | `config::LEFT_DRIVE_PORTS`, `robot::leftDrive` | `-2, -3, -1` (reversed) |
| Right drive | `config::RIGHT_DRIVE_PORTS`, `robot::rightDrive` | `8, 9, 10` |
| Chassis | `robot::chassis` (LemLib) | — |

Geometry constants: `DRIVE_WHEEL_DIAMETER_IN = 2.75`, `DRIVE_RPM = 450`,
`TRACK_WIDTH_IN = 11.5` (marked TODO — measure it), `HORIZONTAL_DRIFT = 2.0`.

Three grey omni wheels are visible along the left side at floor level in
[`02-lift-raised-intake-front.jpg`](photos/02-lift-raised-intake-front.jpg), consistent with
three per side. Whether any of the six are traction rather than omni is **(unclear from
photos)**.

## Odometry and sensors

- **Tracking wheel pod** — `ODOM_VERTICAL_ROTATION_PORT = 7`, exposed as
  `robot::verticalEncoder`. One vertical pod only. I could not positively identify the pod
  in any photo. **(unclear from photos)**
- **Inertial sensor — there is no IMU on the robot.** One is not fitted. `config.hpp` now sets
  `IMU_PORT = 0` (it used to be `8`, which was simply wrong -- port 8 is a right-drive port).
  `robot.cpp` only constructs a `pros::Imu` when `IMU_PORT != 0`; with it at `0`, `robot::imu`
  is a null `std::unique_ptr`, LemLib's `OdomSensors` gets a null IMU pointer (its own
  documented way to run on the tracking wheel alone), nothing calibrates it, and the brain
  screen shows `IMU  none (not installed)` instead of junk. Treat LemLib heading/odometry as
  tracking-wheel-only until a real IMU is mounted on its own port and `IMU_PORT` is set to it
  -- that's the only line that needs to change.
- **Lift rotation sensor** — `LIFT_ROTATION_PORT = 16`. The lift PID runs off this sensor,
  not the motor encoders. **It is mounted on the motored 12T axle**, not on the 48T arm axle,
  so it turns 4× for every degree the arm moves:
  **arm degrees = sensor degrees ÷ 4**, i.e. `LIFT_ROTATION_RATIO = 4.0` (`config.hpp` is
  fixed to this value now -- see the note under "Lift"). The brain's `ROT` line shows the
  converted arm degrees with the raw sensor degrees in brackets so the 4:1 can be checked by
  hand, and the lift reads the sensor with `Rotation::get_position()` (multi-turn), not
  `get_angle()` (wraps at 360°) -- at 4:1 the sensor turns more than a full revolution over
  the arm's ~105° of travel, so a wrapping read would make the lift think it jumped position.
- **Claw distance sensor (autoclamp)** — `CLAW_DISTANCE_PORT = 11`, `CLAW_DETECT_MM = 40`.
  `config.hpp` says it is not mounted yet, and I saw nothing at the claw that looks like a
  distance sensor. Autoclamp is off by default (`CLAW_AUTOCLAMP_AT_START = false`).

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

**Down** = the arm folded forward and low over the chassis, end effector near the intake
handoff in the middle of the robot
([`03-lift-down-stowed-rear.jpg`](photos/03-lift-down-stowed-rear.jpg)). This is where the
robot is switched on, and it defines `0.0` lift degrees.

**Up** = the arm swung up and back, reaching well above the robot — in
[`01-lift-full-up-rear.jpg`](photos/01-lift-full-up-rear.jpg) it is roughly head height on a
standing person. All preset heights are in **lift-arm degrees from that resting position**:

```
DOWN         0.0    resting / start
INTAKE       5.0    claw at the intake handoff (R1)
MATCHLOAD   15.0    claw at matchload height (L2)
PIVOT_CLEAR 25.0    lowest height the pivot may flip out
STACK_LEVELS 20 / 30 / 40 / 50 / 60 / 72 / 85   levels 1..7
MAX        105.0    hard stop, never command past this
```

Every one of those is marked TUNE — they have **not** been measured on this robot. The
README "Tuning the claw positions" is the procedure. The real top of travel is not known.
**(unclear from photos)**

Driving it: in driver control the lift is plain motor power (**L1** up, **L2** down, neither =
brake/hold). The PID, presets, stall guard and runaway guard only run in autonomous. See
"Safety rules" in `CLAUDE.md` — this split is deliberate and was hard-won.

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
between the jaws") and what `CLAW_DETECT_MM = 40` is meant to detect.

I still could not count the cylinders on the end effector itself from these angles.
**(unclear from photos)**

| Thing | Name | Port |
|---|---|---|
| Claw cylinder | `config::CLAW_PISTON_PORT`, `robot::claw` (`subsystems::Claw`) | ADI **C** |
| Pivot cylinder | `config::PIVOT_PISTON_PORT` (owned by `subsystems::Scoring`) | ADI **D** |
| Claw distance sensor | `config::CLAW_DISTANCE_PORT` (not mounted yet) | `11` |

- **Claw**: one pneumatic cylinder, driver button **A** toggles clamp/open.
  `CLAW_EXTENDED_IS_CLOSED = true` is a TODO — confirm that extending the cylinder actually
  closes the jaws before trusting any autonomous grab.
- **Pivot**: a second cylinder that flips the end effector out and back, driver button **X**.
  `PIVOT_ON_AT_INTAKE = true` (pivot out while intaking), `PIVOT_ON_AT_MATCHLOAD = false`
  (TODO). Interlock: the pivot only flips out once the lift is above `PIVOT_CLEAR`, and it
  always comes back in **before** the lift moves down, so it never swings while travelling.
- Timing knobs: `PIVOT_TIME_MS = 250`, `CLAW_CLOSE_TIME_MS = 150`, `CLAW_OPEN_TIME_MS = 200`.

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
or a pin is placed into a stack of cups, or both. `config.hpp` says both things — the
`STACK_LEVELS` comments call each level "the 1st through 7th cup on a stack" while also
calling the array "one entry per pin-stack height" — and no stack appears in any photo.
`lift_preset::STACK_LEVEL_COUNT = 7`, one preset lift height per level, either way.

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
