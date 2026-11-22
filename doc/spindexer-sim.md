# The Spindexer Sim

The **Spindexer Sim** tab simulates Reginald's spindexer: a two-arm spinner turns pollen round a round
spindexer, a ramp bar rises through a slot in the floor at the back and lifts a passing pollen into the
flywheel, and the flywheel and hood of the Launch Sim's `Launcher` throw it. You set both servos in
the tab, press *Feed*, and watch what the pollen do, with the spinner's and ramp's loads traced
against time. This page says what it simulates, where every number came from, what the physics
showed, and how to calibrate it against the real spindexer. Issue #180 asked for it, and
`doc/plans/spindexer-sim.md` is the plan it was built from.

Like the Launch Sim it runs on the simulator's own **core**, not on the robot's code: it runs
no robot code and nothing set here reaches a run.

## Contents

1. [What it simulates](#what-it-simulates)
2. [The spindexer's shape](#the-spindexers-shape)
3. [The spinner](#the-spinner)
4. [The ramp](#the-ramp)
5. [The flywheel and the hood](#the-flywheel-and-the-hood)
6. [The pollen and contact](#the-pollen-and-contact)
7. [Jams](#jams)
8. [The default feed](#the-default-feed)
9. [What the physics showed](#what-the-physics-showed)
10. [Ten times real time](#ten-times-real-time)
11. [Calibrating it against the real spindexer](#calibrating-it-against-the-real-spindexer)
12. [How it is tested](#how-it-is-tested)

## What it simulates

A **feed** starts with the pollen at rest against the spindexer's wall, the spinner still, the ramp
down and the flywheel at speed. Every 4 ms the servos are commanded, as the robot's loop does about
that often, and the engine moves everything between. A feed ends at the first of:

- **all left**: every pollen has left the spindexer, through the hood or out of a window;
- **at rest**: the spinner and ramp are still and every pollen has come to rest;
- **out of time**: the run's length has passed;
- **too much work**: it took more work than its budget (see [Ten times real time](#ten-times-real-time)).

Along the way it reports **events**: a pollen *thrown* from the hood (with where, how fast, at
what angle and with what spin), a pollen that *fell out* of the spindexer, and a *jam*.

The replay draws the spindexer cut away (floor, wall and top plate see-through, with their windows,
slot and notch), the spinner, the ramp bar, the flywheel with its spokes turning and the hood, and
each pollen with its 26 holes so its roll shows. Play, pause, step, scrub, and jump between the
events marked on the timeline. Under the view, four traces against time: the spinner's speed and
the load on it, and the ramp's angle and the torque on it. Click a trace to go to that moment.

Units inside are inches, seconds and radians. The settings use the units a team measures in: mm,
degrees, rev/s, N·m, seconds, and the flywheel's ticks per second.

| Setting | Default | Range |
|---|---|---|
| Pollen | 2 | 0, 1 or 2 |
| First and second pollen | 90° and 270° clockwise from the back | 0° to 360° |
| Spinner power | 1.0 | −1 to 1 |
| Spinner speed at full power, unloaded | 1.0 rev/s | 0.2 to 3 |
| Spinner stall torque | 1.1 N·m | 0.1 to 5 |
| Ramp down / up | 0° / 35° | 0° to 60°, down below up |
| Ramp raised at / lowered at | 0.30 s / 1.20 s | 0 to 10 s, raised before lowered, within the run |
| Ramp servo speed | 0.168 s per 60° | 0.05 to 1 |
| Back sill | 30 mm (the CAD's is 43.12) | 16 to 95 mm |
| Flywheel speed | 1050 ticks/s | 0 to the flywheel's top speed |
| Run length | 3 s | 0.5 to 10 |

A setting out of range, a ramp lowered before it is raised, or a ramp moved after the run ends is
refused with the rule it breaks, and the tab shows that rule beside the field.

## The spindexer's shape

Every measure is the CAD's: Onshape document `b57b76498365b6a963155c10`, part studio *Feeder*
(`e/0296dc8087bbd2d78083cd41`), read through the API on 16 November. In the **spindexer frame**, in
mm: z along the spindexer's axis, up; the origin on the axis at the floor slab's underside; −y toward
the robot's back; +x toward the robot's right.

| Part | Measure | Source |
|---|---|---|
| Floor | its top at z 15, cut by a slot x −32.17 to 47.83, y −57.45 to −50.45 | *FeederBottom*, CAD |
| Wall | a cylinder of inner r 82.27 (outer 88.62) from the floor to the top plate | *FeederBottom*, CAD |
| Front window | \|x\| < 63, sill at z 31.12 | CAD |
| Back window | \|x\| < 40.6, sill at z 43.12 in the CAD, **30 by default** (a setting) | CAD; issue #180's decision |
| Top plate | underside z 96.12, top z 113.12, cut flat at y −78.78 and 62.28 | *FeederTop*, CAD |
| Notch | \|x\| < 31.739, from y −26.843 out through the back | CAD |
| Spinner | a hub of r 10 and two arms 4 thick out to r 50, from z 17 to z 80 | not drawn: the team's default (plan, question 2) |
| Ramp bar | 5 tall, 6 wide, top edges rounded 1, from x −31.96 to the pad at 38, centred in the slot at y −53.95, turned about the ramp servo's shaft at x 57.54, z −13.99 | *Part 2*, CAD; its width and rounding inferred |

The spindexer is placed in the robot by the CAD's transform: tilted 16.46°, rising toward the back,
its origin 173.96 mm behind the chassis's front and 28.78 mm up. The flywheel's axle lands at
(y −84.13, z 132.82) in the spindexer frame, where the *Reginald* studio has it. Tests hold both.

The engine meets every face as a named collider: the floor, the wall, each window's sill and sides,
the top plate, the hub, each arm and the ramp bar. The viewer draws from the same measures, sent
with each feed, so the drawing cannot drift from the physics.

## The spinner

The spinner is a REV Smart Robot Servo (REV-41-1097) in continuous mode, on the top plate with its
spline on the axis. Its vendor figures are 0.14 s per 60° and 13.5 kg·cm at 6 V. Scaled to the
Hub's 5 V that is **1.0 rev/s unloaded and 1.1 N·m at stall**. It turns **clockwise seen from
above**: the team confirmed it, and the ramp only works that way round.

Its speed falls linearly with the load torque L opposing it:

  speed = full speed · power · max(0, 1 − L ÷ stall torque)

L is what the engine's contacts push against the hub and arms during the frame. A negative power
turns it the other way, and the load still holds it back.

## The ramp

The ramp servo is a second Smart Robot Servo, in angular mode, under the floor on the robot's right.
Its shaft runs front to back, so the bar swings up in the side-to-side plane and rises toward −x,
the end away from the servo. It slews toward the commanded angle at **0.168 s per 60°** (the vendor's
0.14 at 6 V, scaled to 5 V): 0° to 35° takes 0.098 s. It holds its angle whatever the load, and the
torque it holds against is reported.

Down at 0° the bar lies under the floor, as drawn. Its top reaches the floor's top at x 0 at 27.5°
and its free end at 17.5°. At 30° a centred pollen at the wall is lifted 12.2 mm, and at 35° 21.0
mm; by 35° every pollen at the wall from x 0 to x −30 reaches the wheel.

## The flywheel and the hood

They are the Launch Sim's `Launcher`, unchanged except that the ramp replaces its feed stand-in:
the flywheel's rim is a torus of 75.2886 mm about the axle with a tube of 35.56, in five rings 12.5
apart, radius 50 at its flanges and 39.73 at its groove's bottom; the hood runs from −64.8° to 20°
about the axle, up from straight back. Both squeeze the pollen compliantly. By default the flywheel
turns at stage 4's close launch of 1050 ticks/s, at 24.2 ticks a turn (2603 rpm). It holds its
speed.

## The pollen and contact

The pollen is the field's 70.61 mm ball (1.39 in radius), as `Launcher` uses, so the feed hands the
wheel the same ball a shot flies. They start seated on the floor against the wall at the angles
set, clockwise from the back.

The spindexer's faces meet a pollen with friction μ 0.30 and restitution 0.50, as `Launcher` uses.
That friction matters: at the frame's default of 0.35 a pollen wedges in the front window's corner
(at tan 19° ≈ 0.34) and falls out of the window.

## Jams

A **jam** is the spinner turning at under a tenth of its unloaded speed for a tenth of a second
while powered. It is reported once for each time it happens, naming the pollen touching the hub or
arms. The feed runs on, in case the ramp frees it. A pollen briefly pinned against a window's side
stalls the spinner for a single frame, which is not a jam.

## The default feed

Two pollen at 90° and 270°, the spinner at full power, the back sill at 30 mm, the ramp up from
0.30 s to 1.20 s, and the flywheel at 1050 ticks/s. It throws both pollen, one at a time, with the
ramp deciding each:

- Pollen 2 is thrown at 0.416 s and Pollen 1 at 0.987 s;
- each leaves the hood at about 228 in/s, 71° up, with backspin of about 1640 rpm;
- the feed ends *all left* at 0.988 s, after 257,930 units of work.

The plan's defaults were the CAD's sill and the ramp up from 0.40 to 0.80 s. Neither throws
anything: the CAD's sill holds a lifted pollen against the wheel, and that window misses both
pollen. On 21 November the session owner decided the defaults above (issue #180). The CAD's sill is
one setting away, and the tab shows the jam there.

## What the physics showed

These are findings for the team, recorded on issue #180, not physics tuned to an outcome.

- **The CAD's back sill is in the flywheel's path.** Its top is 89.70 mm from the axle and the
  groove's bottom 39.73, a gap of 49.97 for a 70.61 ball. With the wheel running a lifted pollen
  is held against the sill and the wheel, and the spinner stalls at 1.1–1.25 N·m. Sweeping the
  sill:

  | Back sill z (mm) | What the default spinner and flywheel do |
  |---|---|
  | 28 or lower | the arms push a pollen into the back window, where the wheel draws it out with no ramp at all |
  | 29 to 40 | the ramp decides: lifted pollen are thrown, others are not |
  | 42 | one thrown, the other falls out of the window |
  | 43.12 (CAD) | jam: held against the sill and the wheel |

  The CAD's sill is about 3 mm too high: at 40 a lifted pollen is still drawn out.
- **With the wheel stopped, the notch holds a lifted pollen down.** The ramp lifts it to z 66–68,
  its top above the plate's underside and up into the notch, which holds it while the spinner
  stalls. It falls back when the ramp lowers. This is the plan's question 4: with a stopped wheel
  the plate wins; with the wheel running and the sill lowered, the wheel takes the pollen first.
- **Timing.** With two arms a pollen passes the back station every half turn, 0.5 s at 1 rev/s. A
  window of 0.4 s catches at most one pollen, and only if it is placed right.
- **The arms push outward.** An arm meets a pollen about 52° ahead of it, pushing about 0.79
  outward, so a pollen rides the wall even at 0.5 rev/s. The plan's fling limit of 1.23 rev/s does
  not show. At the front window the pollen leans on the 16.12 mm sill past the arm's tip, and the
  sill holds it at 1 rev/s.

## Ten times real time

A feed counts its **work**, as a shot does: every integration of a ball and every query of the
surroundings. It is held to the engine's budget of **3,000,000 units per simulated second**; past it
the feed ends *too much work*. The limit is a count, not a clock, so the same settings give the same
feed on any machine.

The default feed needs about **261,000 units per simulated second** (257,930 over 0.988 s), under a
tenth of the budget. On the development container (arm64) it computed in 0.81 s, 0.82× real time. Each feed reports its own figure, and the telemetry store keeps the default's as
`feed.default.cost`.

## Calibrating it against the real spindexer

Film the spindexer from above and from the side at 240 fps or more, with a scale in view. Then:

1. **The spinner.** Time a turn with no pollen at full power: that is *spinner speed*. Hold an arm
   with a force gauge at a known radius until it stops: that is *stall torque*.
2. **The ramp.** Time the bar from down to up: that sets *ramp servo speed*. Read its angles off
   the side view against *ramp down* and *ramp up*.
3. **Friction.** Tilt the spindexer until a pollen slides on the floor and on the wall: the tangent of
   that angle is μ. The default is 0.30.
4. **The pass.** Note when each pollen passes the back station, and when the ramp lifts it, and set
   the tab's *raised at* and *lowered at* to the code's. A feed that throws in the tab and not on
   the robot, or the other way round, says which of these is off.
5. **The back sill.** Measure the real sill above the floor's top (the CAD's is 28.12) and set it.

## How it is tested

- **Core (`simcore`).**
  - `SpindexerShapeTest`: the CAD's shape is accepted; the static lifts and clearances above; a
    spinner reaching r 54, an arm taller than the room under the wheel, a bar wider than the slot
    and windows or notches outside the plate are refused; the transform puts the axle where the CAD
    has it.
  - `SpindexerServosTest`: the spinner's speed under load and at negative power; the ramp slews 0° to
    35° in 0.098 s and holds whatever the load; commands beyond the servos' ranges are refused.
  - `SpindexerAsAPartTest`: the spindexer keeps the contract every moving part keeps, at rest, with the
    spinner turning and with the ramp moving.
  - `FeedTest`: a stopped spinner leaves pollen at rest; the arms carry a pollen past the back;
    the front sill holds; the wheel takes a lifted pollen and the same feed with the ramp never
    raised throws nothing; two pollen are thrown one at a time; with the wheel stopped a lifted
    pollen stays in and falls back; the notch jam names its pollen; a small budget ends the feed;
    the default feed throws both and the CAD's sill jams it.
  - `FeedFingerprintTest` pins the default feed's work, frames, events and finish bit for bit.
  - `FeedSettingsTest` and `FeedDrawingTest`: the settings plan the default feed and refuse what
    they should; the drawing puts the spindexer and the launcher where the physics has them.
- **Server (`TeamCode`).** `SimFeedsTest`: the defaults throw both pollen, refusals name their rule,
  the last settings fed are kept across a restart, feeds are listed newest first, the frames replay
  the spinner, ramp and pollen, the viewer is served, and each feed carries its drawing.
  `DashboardPageTest`: the Spindexer Sim tab comes after Launch Sim. `SimSpindexerPageTest` holds the page's placeholders and the
  answers the browser check is given to the bench's shape.
- **Browser checks.** `tools/browser/spindexer.mjs` runs the bench's own default feed in the tab,
  sees both pollen thrown, replays it to the end and back, and sees a refused setting's rule beside
  its field. `tools/browser/dashboard.mjs` checks that the tab opens the viewer.
