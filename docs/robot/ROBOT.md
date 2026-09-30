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

I could not count wheels per side or tell omni from traction with certainty from these
angles. **(unclear from photos)**

## Odometry and sensors

- **Tracking wheel pod** — `ODOM_VERTICAL_ROTATION_PORT = 7`, exposed as
  `robot::verticalEncoder`. One vertical pod only. I could not positively identify the pod
  in any photo. **(unclear from photos)**
- **Inertial sensor** — `IMU_PORT = 8`, `robot::imu`. **Port 8 is also a right-drive port.**
  That conflict is real and still unfixed in `config.hpp`; the code deliberately skips IMU
  calibration when no IMU answers on that port (commit `495bb66`) so it cannot stall startup.
- **Lift rotation sensor** — `LIFT_ROTATION_PORT = 16`. The lift PID runs off this sensor,
  not the motor encoders. I could not see where on the lift it is mounted, so I cannot tell
  you whether it sits on the 36T lift axle (`LIFT_ROTATION_RATIO = 1.0`) or the 12T motor
  axle (ratio `3.0`). **(unclear from photos)** — use the README "Lift direction check".
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

Game pieces are swept in at floor level at the front, carried up and rearward over the
paddles, and land in the middle of the robot, at the handoff spot where the claw picks them
up. The handoff height is `config::lift_preset::INTAKE` (currently `5.0` lift degrees, TUNE).

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

Driven by two motors on a high-strength axle, geared 12T → 36T (3:1 torque):
`config::LIFT_GEAR_RATIO = 36.0/12.0`. Red cartridges (commit `505ef6c`).

| Thing | Name | Port |
|---|---|---|
| Lift motors | `config::LIFT_PORTS`, `robot::lift` (`subsystems::Lift`) | `20, -13` (13 reversed) |
| Lift rotation sensor | `config::LIFT_ROTATION_PORT` | `16` |

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
end of the arm with the chain run feeding into it. In
[`05-cup-in-claw-brain-screen.jpg`](photos/05-cup-in-claw-brain-screen.jpg) a cup is captured
inside a V of angled bars just above the brain, which is the shape the claw closes around.

I can see it is a pneumatically actuated, chain-levelled end effector, but I could not make
out two opposing jaws or count the cylinders on it from these angles, so the exact grip
geometry is **(unclear from photos)**.

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

## Game element

The thing the robot picks up and stacks is a **cup**: a tapered, ribbed, translucent plastic
cup with a solid coloured cap moulded onto the top. The cap has a faceted, roughly hexagonal
top with a small central boss, and the cup body flares out below it with internal fins.

- In [`04-claw-arm-extended-cup-on-floor.jpg`](photos/04-claw-arm-extended-cup-on-floor.jpg)
  one is standing on the floor: yellow cap, clear body.
- In [`01-lift-full-up-rear.jpg`](photos/01-lift-full-up-rear.jpg) a team member is holding
  another: yellow cap, red body. So the caps and bodies come in more than one colour.
- In [`05-cup-in-claw-brain-screen.jpg`](photos/05-cup-in-claw-brain-screen.jpg) one is
  captured at the claw, sitting cap-up.

The cups nest, and the code stacks them seven high — `lift_preset::STACK_LEVEL_COUNT = 7`,
one preset lift height per level. `config.hpp` and `claw.hpp` also use the word **"pin"** for
the thing the autoclamp distance sensor is supposed to see between the jaws. Whether "pin"
means the cap boss on top of a cup, or a separate field stake the cups go onto, is **(unclear
from photos)** — no separate pin or stake appears in any of the five photos.

---

## Photo index

| Photo | Robot state |
|---|---|
| [`01-lift-full-up-rear.jpg`](photos/01-lift-full-up-rear.jpg) | Lift at full height, arm swung up and back well above the chassis; upper six-bar linkage and joint sprocket in view. Someone is holding a red cup with a yellow cap beside it for scale. Brain visible on the chassis below. |
| [`02-lift-raised-intake-front.jpg`](photos/02-lift-raised-intake-front.jpg) | Lift raised partway, looking down into the front of the robot: the roller intake with its black paddle blades, flexible flaps and orange flex-band roller, the chain drive along its side, and the angled corner guide-roller bars. |
| [`03-lift-down-stowed-rear.jpg`](photos/03-lift-down-stowed-rear.jpg) | Lift fully down and stowed over the chassis — this is the `0.0` power-on position. Rear three-quarter view showing the drivetrain spur gears on one side, a pneumatic cylinder across the frame, and the corner outriggers. |
| [`04-claw-arm-extended-cup-on-floor.jpg`](photos/04-claw-arm-extended-cup-on-floor.jpg) | Lift low, arm swung out sideways over the floor with the chain run and the pivoting end effector at its tip. A yellow-capped clear cup stands on the floor beside the robot. |
| [`05-cup-in-claw-brain-screen.jpg`](photos/05-cup-in-claw-brain-screen.jpg) | Close front view, a hand steadying the lift. A yellow-capped cup is captured in the claw directly above the V5 brain; the brain screen shows the `JokrOverride` PROS program in slot 1. |

All five JPEGs were re-encoded from the original HEICs with the camera orientation baked into
the pixels and **all EXIF/GPS metadata removed**.
