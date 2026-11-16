# The Feeder Sim

A **Feeder Sim** tab, after the Launch Sim, that simulates Reginald's feeder
in fine detail. A 2-arm spinner turns pollen round a round feeder. A ramp
rises through the floor at the back and lifts a passing pollen into the
flywheel, and stage 4's `Launcher` takes it from there. The user sets both
servos in the tab and watches what the pollen do, as the Launch Sim lets them
watch a shot.

This plan replaces the *Feeder* and *Lift ramp* sections of
`reginald-internals.md`. That plan inferred a 4-arm rotor, a fence and a fin
under the hood from the 13 October CAD. The CAD now draws the feeder's wall,
both servos and the ramp, and none of those inferences survive (see
[What this retires](#what-this-retires)).

**Status: settled.** The team answered the [questions](#questions-for-the-team)
on 16 November.

## Contents

1. [What the CAD says](#what-the-cad-says)
2. [What we infer](#what-we-infer)
3. [Findings](#findings)
4. [Questions for the team](#questions-for-the-team)
5. [What the tab does](#what-the-tab-does)
6. [How it is simulated](#how-it-is-simulated)
7. [Phase 0 · settle and file](#phase-0--settle-and-file)
8. [Phase 1 · the feeder is a shape in the core](#phase-1--the-feeder-is-a-shape-in-the-core)
9. [Phase 2 · the servos](#phase-2--the-servos)
10. [Phase 3 · the feeder is a part](#phase-3--the-feeder-is-a-part)
11. [Phase 4 · feeds on the coding server](#phase-4--feeds-on-the-coding-server)
12. [Phase 5 · the tab](#phase-5--the-tab)
13. [Phase 6 · docs](#phase-6--docs)
14. [What this retires](#what-this-retires)
15. [Later](#later)

## What the CAD says

The Onshape document `b57b76498365b6a963155c10`, part studio *Feeder*
(`e/0296dc8087bbd2d78083cd41`), read through the API on 16 November. All
numbers are mm, in the **feeder frame**: z along the feeder's axis, up; the
origin on the axis at the floor slab's underside; −y toward the robot's back,
+x toward the robot's right.

| What | CAD |
|---|---|
| Variables | `PollenDiameter` 2.8 in (71.12), `FeederRadius` 55, `FeederHeight` 75, `FlywheelDiameter` 100, `FlywheelWidth` 50. `FeederRadius` and `FeederHeight` drive no feature |
| Floor (*FeederBottom*) | slab z 0–15, so the floor's top is z 15. A Ø19.15 bearing bore on the axis |
| Wall (*FeederBottom*) | inner r 82.27, outer r 88.62, z 15–96.12 |
| Front window | through the wall at \|x\| < 63, sill z 31.12: 16.12 above the floor. The entry from the scoop |
| Back window | through the wall at \|x\| < 40.6, sill z 43.12: 28.12 above the floor |
| Top plate (*FeederTop*) | z 96.12–113.12, cut flat at y −78.78 and y 62.28. 81.12 above the floor: one pollen plus 10 |
| Notch in the top plate | \|x\| < 31.739, from y −26.843 out through the back |
| Spinner servo | a REV Smart Robot Servo (REV-41-1097) on the top plate, z 96–140, its output spline on the axis pointing down, the shaft's end at about z 96.5 |
| Ramp servo | a second REV-41-1097 under the floor, on the robot's right. Its shaft points front to back (along y), at x 57.54, z −13.99, so its horn swings in the side-to-side plane |
| Horn (*DoubleServoArm*) | lies side to side, x 37.5–77.5, with holes at ±8 and ±16 from the shaft |
| Ramp bar (*Part 2*) | runs side to side, 5 tall (z −16.49 to −11.49 as drawn), at y ≈ −54, from x −31.96 to about 38. A pad from x 38 to 77 bolts to the horn. Its free end is 89.5 from the pivot |
| Floor slot | x −32.17 to 47.83, y −57.45 to −50.45 (7 wide), through the floor |
| Pocket under the floor | x 29.56–76.74, y −61.17 to −42.37, 10 deep: room for the pad and the horn |
| Spinner | **not drawn** |

**The ramp's motion.** The horn and the bar lie side to side, and swing up
together in the side-to-side plane about the ramp servo's shaft. Raised by θ from the drawn position, its top at x is at

  z = −13.99 + (57.54 − x)·sin θ + 2.5·cos θ

so it rises toward −x, the end away from the servo. Its top reaches the
floor's top at x 0 at θ 27.5°, and its free end at x −32 at θ 17.5°.

**Placed in Reginald.** The feeder frame maps to the *Reginald* part studio
(mm; +X left, +Y back, +Z up, origin at the chassis's front middle at floor
level) by

  R = (0, 173.96, 28.78) + x·(−1, 0, 0) + y·(0, −0.959, −0.2834) + z·(0, −0.2834, 0.959)

a tilt of 16.46°, rising toward the back. It is checked against the CAD: the
placed *FeederBottom* and *FeederTop*'s faces land at z 0, 15, 31.12, 43.12,
96.12 and 113.12, as in the feeder's own studio.

**The flywheel** (*FlywheelOuter*) has its axle along x through feeder
(y −84.13, z 132.82). Its rim is the groove stage 4's `Launcher` already
models: a torus of 75.2886 about the axle with a tube of 35.56, cut to
\|x\| ≤ 25, so radius 50 at its flanges and 39.73 at the groove's bottom.
Below the top plate it reaches into the feeder between y −50.2 and the wall,
\|x\| ≤ 25, down to z 82.85.

**Pollen placed in Reginald.** One at the feeder's front window, on its way
in from the scoop, and three on the floor ahead of the robot. None sits in
the feeder.

## What we infer

| Inference | Why | Confidence |
|---|---|---|
| The spinner servo runs in continuous mode, the ramp servo in angular mode | The team: "rotates continuously", "lifts a small arm". The Smart Robot Servo does both, set by REV's programmer | high |
| **The spinner turns clockwise seen from above** | The ramp rises toward −x. At the back a pollen moves toward −x only if the spinner turns clockwise from above. Turned the other way it would meet the ramp's high end first | high (question 1) |
| The spinner is a hub on the servo's shaft with two arms opposite, and nothing else | The team's description | high |
| **The arms reach no further than r 50**, for now | The raised bar stands at y −54: its nearest point to the axis is r 53.95. Arms that reach past it hit the raised bar, unless they are cut away above it | the team's default for now (question 2) |
| The arms run from 2 above the floor to z 80 | Under the flywheel's lowest point (82.85) with 2.85 to spare. A pollen's equator is at z 50.56, inside that | the team's default for now (question 2) |
| The ramp is down at θ 0, as drawn, and up at a θ the tab sets, 35° by default | At 0 the bar lies in the pocket under the floor. The lift table below puts every pollen at the wall within reach of the wheel by 35° | the team's default for now (question 3) |
| The pollen is the field's 70.61, not the CAD's 71.12 | `Launcher` already uses the field's (1.39 in), and the tab must hand it the same ball | high |

**The lift.** All the numbers in this plan are for the CAD's 71.12 mm
pollen; the field's 70.61 moves each by about half a millimeter. A pollen at the wall (its center at r 46.71) on the back half,
center x as given, lifted by the bar at θ. "Wheel" is the clearance to the
flywheel, "plate" to the top plate around its notch; negative means the
pollen meets it before the lift is reached.

| θ | x +20 | x 0 | x −10 | x −20 | x −30 |
|---|---|---|---|---|---|
| 0°: lift / wheel / plate | 0 / 7.1 / 10.0 | 0 / 11.9 / 10.0 | 0 / 7.9 / 10.0 | 0 / 7.1 / 10.0 | 0 / 10.1 / 10.0 |
| 30° | 0 / 7.1 / 10.0 | 11.0 / 3.8 / 11.3 | 16.3 / −5.2 / −1.5 | 18.2 / −8.5 / −10.9 | 14.9 / −2.3 / −16.3 |
| 35° | 4.8 / 2.9 / 5.2 | 20.3 / −2.9 / −5.2 | 24.8 / −11.2 / −10.1 | 25.3 / −14.2 / −17.5 | 20.6 / −6.7 / −23.5 |
| 40° | 12.7 / −3.9 / −1.8 | 29.6 / −10.7 / −11.9 | 32.3 / −15.8 / −14.2 | 31.3 / −18.8 / −20.8 | 24.7 / −9.9 / −27.0 |

At rest a pollen at the back station clears the wheel by 7.1 or more: the
wheel does not take a pollen the ramp has not lifted. Past x −35 the pollen
is off the bar's free end and drops back to the floor.

These are static numbers from a script in the session's scratchpad, not the
physics. Phase 1's tests restate the ones the shapes must give.

## Findings

**The notch's inner end.** A centered pollen at the wall meets the wheel at a
lift of about 16.3 mm, and its top meets the notch's inner end (y −26.843) at
about 16.1. Off center by 10–20 mm it meets the flange 2–4 mm of lift before
the plate. Whether the wheel pulls it out from under the plate, or the plate
pins it against the wheel, is for the physics to show (phase 3).

**The fling limit.** At the back station gravity pulls a pollen inward at
2.78 m/s², because the feeder rises toward the back. Above **1.23 rev/s** the
turn pulls it outward harder, so it rides the wall past the ramp. Below that,
it slides along the arm toward the hub, where the bar meets it far off its
center and pushes it inward rather than up. The Smart Robot Servo turns about
1.0 rev/s unloaded at the Hub's 5 V: under the limit. So the ramp may lift
pollen worse at lower speeds, and the tab should show it.

**The front sill holds.** At the front, gravity and the turn both push a
pollen outward, into the front window. Its 16.12 mm sill holds it until the
outward push passes about 14.3 m/s², which takes about **2.5 rev/s**. The
spinner cannot get there.

**Timing is easier with two arms.** A pollen passes the ramp every half turn:
0.5 s at 1.0 rev/s, against 0.25 s for `reginald-internals.md`'s 4-arm rotor.
The ramp's 35° takes 0.098 s each way at 5 V. Whether a ramp raised for a
fixed time lifts exactly one is a question the tab answers.

## Questions for the team

Asked and answered on 16 November.

1. **Direction.** Does the spinner turn clockwise seen from above (toward the
   robot's left as it passes the back)? *Yes.*
2. **The spinner's shape.** *Not decided: use the plan's for now*, a hub of
   r 10 and arms 4 thick from the hub to r 50, from 2 above the floor to
   z 80. Arms that reach past r 50 must be cut away where the raised bar is,
   and phase 1 refuses a spinner that does not.
3. **The ramp's angles.** *Use the plan's for now*: down at 0°, as drawn, and
   up at 35°. Both are settings in the tab.
4. **The notch.** The top plate's cutout, which lets the flywheel reach into
   the feeder, is 63.48 wide, narrower than a pollen, and stops 26.843
   behind the axis. A lifted pollen's top pokes up into it, and meets its
   front end or its sides at about the lift that reaches the wheel. *The
   plan keeps the CAD as drawn*, and the tab shows whether the wheel takes the
   pollen first or the plate holds it down. If the plate wins, a longer or
   wider cutout is the team's fix, and the shape is a measure in phase 1.

## What the tab does

A **feed** starts with the pollen in the feeder at rest and the flywheel at
speed. It ends when every pollen has left the feeder through the hood or come
to rest, when the run's length has passed, or when it has taken too much
work.

**Settings**, in the units a team measures in:

| Setting | Default | Range |
|---|---|---|
| Pollen | 2: the arms along the robot's length, one pollen each side of them, against the wall | 0, 1 or 2, and each one's starting angle |
| Spinner power | 1.0 | −1 to 1 |
| Spinner speed at full power, unloaded | 1.0 rev/s | 0.2 to 3 |
| Spinner stall torque | 1.1 N·m | 0.1 to 5 |
| Ramp down / up | 0° / 35° | 0° to 60°, down below up |
| Ramp raised at / lowered at | 0.40 s / 0.80 s | within the run, raised before lowered |
| Ramp servo speed | 0.168 s per 60° | 0.05 to 1 |
| Flywheel speed | stage 4's default | as the Launch Sim |
| Run length | 3 s | 0.5 to 10 |

The defaults for the servos are the REV Smart Robot Servo's 0.14 s/60° and
13.5 kg·cm at 6 V (vendor), scaled to 5 V. A setting out of range, or ramp
times out of order, is refused with the rule it breaks, as the Launch Sim
refuses.

**What it shows**, replayed with play, pause, step and scrub:

- the feeder cut away: floor, wall and top plate translucent, their windows
  and notch; the spinner; the ramp bar; the flywheel and the hood;
- each pollen with its holes, so its roll shows;
- events marked on the timeline: a pollen taken by the wheel, thrown from the
  hood, dropped off the ramp's end, held at a sill, or a jam;
- traces against time: the spinner's speed and load torque, the ramp's angle
  and the torque on it, the flywheel's speed;
- a summary: how many pollen were thrown, when, and how each left the hood
  (speed, angle, spin), as a Launch Sim shot starts.

## How it is simulated

| | Value | Source | Confidence |
|---|---|---|---|
| Engine | `Bodies`, as the Launch Sim: RK4 at 0.1 ms, contact events by bisection, Walton bounces, sequential impulses between pollen | the existing core | high |
| Floor | the slab's top face, cut by the slot | CAD | high |
| Wall | a cylinder patch of r 82.27 from the floor to the top plate, less the two windows; each window's sill and sides as flat faces with their edges named | CAD | high |
| Top plate | its underside less the notch, with the notch's edges named | CAD | high |
| Spinner | the hub as a cylinder; each arm a box; turned about the axis by the spinner servo | inferred (question 2) | medium |
| Ramp bar | a box 5 tall and 6 wide, its top edges rounded 1, turned about the ramp servo's shaft | CAD; its width and rounding inferred | medium |
| Flywheel and hood | stage 4's `Launcher`, placed by the transform above. The ramp replaces its feed stand-in | the existing core | high |
| Contact | the pollen's squash, μ 0.30, restitution 0.50, as `Launcher` uses | materials | low |
| Spinner servo | speed = full speed · power · max(0, 1 − L ÷ stall torque) toward the turn, with L the load torque opposing it, from the engine | vendor, scaled | low |
| Ramp servo | its angle slews toward the commanded one at the servo's speed, and is held whatever the load; the torque on it is reported | vendor, scaled | low |

A **jam** is the spinner stalled with a pollen squeezed: an arm presses a
pollen against an edge with more than the stall torque. The spinner stops,
the feed reports the jam and where, and runs on in case the ramp frees it.

**Ten times real time.** A feed is held to the Launch Sim's budget: work
counted in units, so a run's cost is exact and the same on any machine. Phase
3 measures a feed's units per simulated second and sets the budget there;
past it, the feed ends and says so.

## Phase 0 · settle and file

- Done: the questions went to the team, and their answers are recorded
  above. This plan and the change to `reginald-internals.md` are on
  `develop`, and the issue *Feeder Sim tab* works it.

## Phase 1 · the feeder is a shape in the core

`Feeder.Shape`, in `simcore`: the floor, wall, windows, top plate, notch, slot,
the spinner's shape and the ramp bar, each a Validated Type built from its
measures, refusing a measure that breaks its rule. A spinner whose arms reach
the wall, an arm taller than the room under the wheel, or a bar that does not
fit the slot is refused, naming the rule.

Tests first, one at a time:

1. The CAD's feeder is accepted. Its floor is 15 above the slab's underside,
   its wall r 82.27, its room 81.12 tall.
2. Two pollen fit opposite with the default spinner: each clears the hub by
   1.15 and the wall, seated against it, by 0.
3. A spinner whose arms reach r 54 is refused: its rule names the raised bar.
4. The bar's top is below the floor at 0°, reaches it at x 0 at 27.5° and at
   its free end at 17.5°.
5. A pollen at rest at the back station clears the wheel by 11.9 centered and
   by 7.1 at x ±20.
6. Lifted by the bar at 30°, a centered pollen at the wall rises 11.0; at 35°,
   20.3.
7. The transform puts the floor's top at the axis at Reginald
   (0, 169.71, 43.17) and the flywheel's axle at (0, 217, 180).

Then implement. The numbers above are the CAD's, typed into the tests, not
recomputed by them.

## Phase 2 · the servos

`Feeder.Spinner` and `Feeder.Ramp`, Total Core functions from a state, a
command and a load to the next state.

Tests first:

1. Unloaded at full power the spinner reaches 1.0 rev/s; at half power, 0.5.
2. Loaded with half its stall torque it turns at half speed; at its stall
   torque or more it stops.
3. At negative power it turns the other way.
4. The ramp slews from 0° to 35° in 0.098 s at the default speed, and stays
   at 35° whatever the load.
5. A command beyond the servo's range is refused, naming the range.

Then implement.

## Phase 3 · the feeder is a part

`Feeder` implements `Surroundings.Part`, carrying the spinner, the ramp and
`Launcher`. Its contact surfaces go through `PartContract`, as every part's
do.

Tests first, one at a time:

1. `PartContract` holds for the feeder at rest, with the spinner turning and
   with the ramp moving.
2. With the spinner stopped, a pollen at rest stays at rest for 1 s.
3. A turning spinner carries a pollen round; it passes the back station with
   the ramp down and stays in the feeder for two turns.
4. Above 1.23 rev/s a pollen passes the back station against the wall; at
   0.5 rev/s it leaves the wall there.
5. At 1.0 rev/s a pollen stays inside the front sill for two turns.
6. With the ramp up as the pollen passes, the wheel takes it, and it leaves
   the hood as stage 4's shots do: at 70 ± 3°, with backspin.
7. With the ramp up and the flywheel stopped, the pollen is lifted, does not
   leave, and drops off the bar's end.
8. Two pollen with the ramp held up leave one at a time, and never overlap.
9. A pollen pinned by an arm against a window's side stalls the spinner, and
   the feed reports the jam.
10. A feed run twice gives the same bits; a fingerprint test pins one feed's
    work and its result.
11. A feed past its budget ends and says so.

Then implement. If test 6 fails because the top plate holds the pollen down
at the notch, stop and report it with the numbers: that is question 4's
finding for the team, not the physics to tune.

Measure and record in the telemetry store: a default feed's units per
simulated second, and how long it takes.

## Phase 4 · feeds on the coding server

`SimFeeds`, after `SimLaunches`: settings in, a feed out, refusals named, the
last settings remembered in `feed-settings.json`, earlier feeds kept as
ghosts, frames sent in chunks.

Tests first:

1. The defaults give a feed that throws two pollen.
2. Each setting out of range is refused, naming its rule; the ramp lowered
   before it is raised is refused.
3. The settings are remembered across a restart of the server.
4. A feed's summary names how many were thrown, and when.
5. A feed's frames replay the spinner's and ramp's angles and each pollen's
   place.

Then implement.

## Phase 5 · the tab

`SimFeedPage` and `feeder.html` (three.js, after `launch.html`), and a tab
*Feeder Sim* after *Launch Sim* on the dashboard.

Tests first:

1. `CodingServerTest`: the tabs come in their order, with *Feeder Sim* after
   *Launch Sim*.
2. `tools/browser/feeder.mjs`: the page loads, runs the default feed, shows
   two pollen thrown, and replays it to the end and back.
3. `tools/browser/feeder.mjs`: a refused setting shows its rule beside the
   field.
4. `tools/browser/dashboard.mjs`: the tab opens the page.

Then implement.

## Phase 6 · docs

- `doc/feeder-sim.md`, after `doc/launch-sim.md`: what it simulates, every
  number and its source, the servos, the budget, calibration against the real
  feeder, how it is tested.
- `doc/CONTEXT.md`: **Feeder**, **Spinner**, **Ramp**, **Feed**, **Feeder
  Sim**.

## What this retires

From `reginald-internals.md`:

- the 4-arm rotor, its pockets, the r 93–96 fence and its entry guide: the
  CAD's wall at r 82.27 holds the pollen, and the spinner has two arms;
- the floor reaching r 96 and the widened notch: the CAD draws both;
- the fin under the hood, its pivot behind the wheel and its fixed piece: the
  ramp is a bar under the floor on its own servo, swinging across the robot;
- the timing finding's 0.21 s per pocket: 0.5 s per pass with two arms.

Still standing, restated above: the fling limit (now 1.23 rev/s at the wall),
the servo figures, and the ramp's torque, which the tab now reports rather
than estimates. Its batches 1 to 4 take the feeder from this plan once it
lands.

## Later

- A pollen entering through the front window from the scoop, at a speed and
  angle the tab sets.
- Running the feeder inside the robot simulation (`reginald-internals.md`
  batch 4), with `Launcher`'s ramp timing in the robot code.
- A sweep: the ramp's raise time against the spinner's angle, as a map of
  how many pollen each throws.
