# Reginald's internals on the shot physics

Today a pollen the intake takes jumps into a queue, and nothing inside the
robot has a shape. The flight plan puts every ball off the floor on the
Launch Sim's engine. Its stage 4 simulates Reginald's flywheel and hood, so
where a ball leaves, how fast and with what spin all come out of the physics.
Between the floor and the flywheel there is still nothing. Stage 4's
`Flight.fed` puts a chambered ball at the hood's lower end with a 0.1 m/s
feed, and a fixed hood extension and a stop hold it there while the wheel is
stopped.

This plan fills that gap. A pollen on the floor meets the roller and the
scoop, rolls into a pocket of the feeder, rides round, is lifted by the ramp
into the flywheel and thrown. It stays one body in one engine from the front
of the robot to the cell, and it spins the whole way. The two-wheel launcher
stays in the Launch Sim.

The scoop is the CAD's own shape. The CAD has no rotor, arms, fence or ramp,
so we infer them from what it does have, and say why. The flywheel and hood
are stage 4's. This plan changes them only where the ramp meets them, and
gives the flywheel inertia and a motor. Every number says where it comes
from: the CAD's exact faces (the 13 October snapshot), a source, or an
inference with its confidence. Nothing in the plan waits on the team. Each
open question has a default, and each measurement replaces one. Two steps
can stop for the user: batch 4's recalibration, if its band is narrower than
stage 4's, and `SimRunnerTest`, if the turning feeder pushes it past 1 s. The
second is likely, so its question goes to the user before batch 4 starts
(*What it costs*).

It builds on two plans and does not repeat them:
- `doc/plans/reginald.md`, batches 1 to 5: Reginald's shape, his CAD, his
  drive, his drawing, and no turntable. After its batch 5 there is no
  turntable, so stage 4's launcher frame is the robot's frame, and every part
  here is carried in it. Its batch 6 is superseded by the flight plan's
  stage 4. Its batch 7 is superseded here: the abstract `Feeder` is never
  built, and the robot-code half moves into batch 4 below.
- The flight plan, `doc/plans/runtime-flight.md`, stages 1 to 4, as built and
  as designed:
  - **Stage 1:** one engine, `Bodies`, for the Launch Sim and the realtime
    simulator. A body comes in through values that hold their rules: a
    `Name`, a `Motion` (a finite place, velocity and spin) and a `Start`
    (awake, asleep or fixed), on a validated `Step`.
  - **Stage 2:** `Surroundings` that move. Each `Part` says what it reaches,
    how fast it sweeps and whether it moves, and answers `near` with `Touch`es
    that carry the support's velocity and spin. A moving part wakes what it
    reaches, carries what rests on it, and never passes through a ball.
    `PartContract` is the test helper every part is run through.
  - **Stage 3:** `Flight` on the engine: inputs with their rules, the hives as
    a part, what stands in the flowers' bores as another (`Bores`), landing
    back in dyn4j, parts placed at `GridTime`s on the engine's step grid, a
    work allowance of 15,000 units each window of 50 steps on that grid, and
    the idle jump, `Bodies.idleThrough`, which is no work. A tip replaces the
    hives' part alone (`Surroundings.replacing`), and no refusal is dropped.
  - **Stage 4:** the launcher. The arc collider (`Arc`). Compliant surfaces,
    answered by a separate `squeezing` query and pushed inside RK4 at the
    engine's 0.1 ms step, with no sub-steps. The launcher frame that `Flight`
    carries from step to step, with the yaw unwrapped and `Flight.carriedTo`
    for teleports. `Flight.fed` with its entry check. Stage 3's core
    `Flywheel`, which clamps a command to the motor's top speed, with
    `TicksPerTurn` and `AngularSpeed`. `FLYWHEEL_TICKS_PER_TURN` in place
    of `LAUNCH_THROW`, which `Constant.Retired` lists, and protocol 7, whose
    hello says which constants a child reads.

  This plan uses all of these and plans none of them again.

**Frames.** The CAD frame and the robot frame are `reginald.md`'s. CAD: mm,
origin at the middle of the chassis front at floor level, +Y back, +X left,
+Z up. Robot: inches, +x forward, +y left, z up, origin 204 mm behind the
front. `x = (204 − Y)/25.4`, `y = X/25.4`, `z = Z/25.4`. Two more:
- The **feeder frame** (mm). Origin at CAD (0, 167.816, 49.596), robot
  (1.425, 0, 1.953) in. Axis n = robot (0.2834, 0, 0.9590): tilted 16.4645°,
  so the plates rise toward the back. e2 = robot (−0.9590, 0, 0.2834), in the
  plates' plane, toward the back. x along robot +y, in mm (CAD X). Heights d
  are along n. The **front station** is at e2 < 0 on the centreline, the
  **back station** at e2 > 0.
- The **flywheel angle** θ, about the flywheel's axle at robot
  (−0.512, ·, 7.087) in, from straight back, positive up. Straight down is −90°.

---

## How a pollen meets Reginald's parts

Every part in this plan is carried in stage 4's launcher frame, as the
flywheel and the hood are. Some also move within the robot: the rotor turns,
the ramp swings, and the roller and the flywheel spin.

A pollen meets every part through stage 4's compliant contact. Stage 4 makes
only the launcher's surfaces compliant; this plan adds every part inside the
robot to the same `squeezing` query. Every part uses stage 4's compliant
contact unchanged, with its own μ and restitution; the bands add their spring
in series.

Why compliant everywhere:
- The roller presses a pollen into the scoop with up to 13 N.
- An arm can close on a pollen held by the fence. A rigid solver has no
  answer when a moving part closes on a ball that a fixed part holds. The real
  answer is that the pollen squashes and the servo stalls, and a compliant
  contact gives exactly that.
- One law for every part keeps them consistent with each other.

**No sub-steps.** As in stage 4, every body integrates in the engine's 0.1 ms
pieces. The normal springs are easy. A pollen against a part has a period of
5 ms, 50 steps, and RK4 at 0.1 ms is stable up to stage 4's 19,000 N/mm.
Friction that sticks is harder. Regularised friction in stick is stable while
the step is under 2.785·v_reg ÷ (Σμ·N·(1/m + r²/I)). For a pollen,
1/m + r²/I is 103.5 per kg.

| Where | Σμ·N | Bound at 0.02 m/s |
|---|---|---|
| A pollen in a pocket, or on the scoop under its own weight | under 0.1 N | over 5 ms |
| Under the roller at the crest, built bands (6.7 N at μ 1.0, the scoop at μ 0.3) | 8.8 N | 61 µs |
| The same at `BAND_STIFFNESS`'s top, 0.5 N/mm (13.3 N) | 17.4 N | 31 µs |
| An arm jamming a pollen at the servo's stall (20 N, μ 0.3 both sides) | 12 N | 45 µs |
| Stage 4's wheel and hood, or the fin, at the full squeeze | about 36 N | 15 µs |

Friction saturates at μ·N, so a step past the bound does not blow up, but
jitters by an amount set by h·Σμ·N, not v_reg. Runs of stage 4's contact law
measured 2–2.5 mm/s of speed noise at 10 µs and 21 N. Scaled to 0.1 ms and
36 N that is about 40 mm/s, three times the rest speed, and at 0.1 ms the
same law gained energy in stick. So a held pollen never sleeps. Stage 4's
launch never sticks, but its own ball held against a stopped wheel does, at
the 15 µs bound, so stage 4 meets this first.

Here the crest, a jam and a pollen held on the fin against a stopped wheel
stick too: every row but the pocket is past the bound. So batch 2 writes the
stick test first: a pollen held still between a plate and the still bands at
13.3 N stays under the rest speed and gains no energy over 1 s at 0.1 ms. It
is red under a fixed 0.02 m/s. Batch 2 then adopts the rule below, or stage
4's rule if stage 4 has landed one: a body's regularisation speed is never
below what keeps stick stable at the engine's step,
2·h·Σμ·N·(1/m + r²/I) ÷ 2.785. Wherever the bound already holds it is stage
4's 0.02 m/s exactly, and stage 4's launch is insensitive to it (0.001 to
0.3 m/s).

**Joins.** A part built from several pieces names the joins between them. A
compliant contact pushes once for every touch, so a pollen across a join,
touching two pieces of one flat surface, would be pushed twice as hard, as it
is at each step of a staircase. Two rules, both in the shapes:
- A touch whose nearest point lies on one of its piece's join faces, or on an
  `Arc` end named as a join, is dropped. That removes the ghost edge. On the
  join itself, where both pieces' nearest points lie on it, the touch of the
  piece the join names first is kept.
- Of the touches from one part whose normals agree within 1°, only the deepest
  counts. That removes the double count where two flush faces meet.

A pollen in a concave corner still feels both faces, because neither nearest
point lies on the join. A join can also pair pieces of two parts: the fin
with the fixed piece, and the fixed piece with the hood's middle ring. There
only the first rule applies.

**Materials.** Named core constants with their sources, not settings, as
stage 4's are.

| Part | Material | μ | Restitution | Source | Confidence |
|---|---|---|---|---|---|
| Bands | natural rubber | 1.0 | 0.5 | rubber on PE is about 1.26 static (Forbo datasheet), less when sliding; restitution assumed | medium-low; low |
| Roller treads | polyurethane | 0.35 | 0.6 | PU 85A on PE (BEHAbelt datasheet); restitution assumed | low |
| Scoop | plastic; the CAD gives none | 0.30 | 0.50 | the Launch Sim's polycarbonate (0.29–0.33) and its walls' restitution | low |
| Plates, cross rail | aluminium (REV-41-1785) | 0.35 | 0.62 | the Launch Sim's metal and frame values | medium-low |
| Feeder plates, rotor, fence, fin, fixed piece | printed plastic, assumed | 0.30 | 0.50 | as the scoop | low |
| Flywheel, hood | stage 4 | 0.6, 0.3 | 0.6 | stage 4 | — |

**The pollen.** The field gives a pollen a radius of 1.39 in (70.61 mm), and
the engine flies that ball. Reginald's CAD is sized for its `PollenDiameter`,
71.12 mm (2.8 in, from AndyMark's am-5851 drawing, whose title block allows
±0.1 in). The tests that reproduce the CAD's measurements use 71.12 mm, as the
CAD does. Stage 4 placed its hood for the field's ball: 2.0 mm of squeeze on
it, 2.5 mm on the CAD's. Everywhere else the field's ball has 0.25 mm more
room on every side than the CAD planned for. Measuring ten pollen settles
which is right (calibration item 1).

## The parts

### Intake roller

**What the CAD says.**

| What | CAD (mm) | Robot frame (in) |
|---|---|---|
| Treads | two 90 mm grip wheels, 25 mm wide at \|X\| 84.54–109.54, radius 45.0 | \|y\| 3.33–4.31 |
| Axle | Y 25.4, Z 114.3. No shaft between the wheels | (7.031, ·, 4.500); front edge at x 8.803 |
| Bands | Parts 8–19: 12 strips at 6 angles 60° apart, each in two halves over X 0…±97.04. Each 10 × 1 mm, outer face at r 45.0, flush with the treads | |
| Free span | 169.08 between the wheels' inner faces (\|X\| ≤ 84.54) | |
| Gap to the floor | 69.30: a 71.12 mm pollen on the floor is squeezed 1.82 | |
| Gap to the scoop | 44.13, at the steep ramp's top corner (Y 47.29, Z 27.90) | |
| Overlap along the path | first touch with the centre at Y −7.5, lets go at Y 82; 22.18 under the axle; most 26.99, with the centre at (Y 38.5, Z 62.35) | |
| Rigid radius a pollen feels | 36.5 between strips, 45.3 on one | |

**A finding for the team: the roller cannot reach a pollen at the lip.**
- A floor pollen first touches the scoop's lip face with its centre at
  Y −14.96 (x 8.62 in). In effect that is a 9.36 mm step, with a contact
  normal 42.5° from vertical.
- At that moment the roller is **7.92 mm short**. The first strip touches only
  after the pollen has risen 5.23 mm, with its centre at (Y −7.54, Z 40.79).
- So the robot's own speed has to carry the pollen up the lip. It keeps 0.737
  of its approach speed along the face. It needs 0.43 m/s with no friction,
  0.47 m/s as a hop with a restitution of 0.5, and 0.56 m/s if the corner
  grips (μ ≥ 0.436). Call it **0.45 m/s, about 18 in/s**: a third of
  Reginald's 50 in/s top speed. Slower than that, the 42° face pushes the
  pollen ahead along the floor.
- To reach a pollen at the lip, the roller needs a radius of at least
  52.92 mm, or its axle no higher than 105.28 mm (4.14 in). Lowering the axle
  that far raises the overlap at the crest from 26.99 to 35.70 mm.
- The simulation does not fix this. It reproduces whatever the CAD has. The
  replay shows the pollen being pushed, and a change to the CAD shows up as a
  change in the replay.

**At the crest the bands bend 27 mm.** The overlap covers ±41.1° of the
roller, so one or two strips touch at once. Strips 0°, 15° and 30° off the
line to the pollen bend 27.0, 26.0 and 22.0 mm, which leaves them at a radius
of 18–23 mm. The design works only because the bands give. The rigid treads
stay at least 11.85 mm clear of a pollen inside the scoop's walls.

**A second hurdle on the steep ramp.** Where a pollen under the roller can
stall depends on the support's friction. The scoop's material decides it, and
the CAD does not give one.

| Pollen centre Y (mm) | Resting on | μ the support needs |
|---|---|---|
| −7.5 (first strip touch) | the lip's corner | 0.49 |
| +2 | the 15.6° slope | 0.31 |
| +10 | the 15.6° slope | 0.25 |
| about 18 | the 40.5° steep face | 0.44 |
| 26 | the steep face | 0.33 |
| the crest | the crest | about 0 |

At μ 0.3 a pollen stalls between Y −7.5 and +2 unless its approach carries
it, and between Y 16 and 27 unless its own speed carries it.

**What we infer.**

| Inference | Why | Confidence |
|---|---|---|
| The bands act as one cylinder of radius 45.0 mm over the free span, with a radial spring | They bend far more than the ripple between strips (36.5–45.3 mm rigid) | medium |
| Band stiffness about 0.25 N/mm per contact, between 0.10 and 0.44 | A neo-Hookean strip with G·A₀ ≈ 4 N gives k ≈ 2T₀/a = 0.10–0.22 N/mm per strip (table below), and one or two strips touch. 0.25 is λ₀ 2.0 with 1.5 strips touching on average | low |
| Positive power pulls a pollen in: the bands' bottoms move back | It is an intake | high |
| About 100 in/s at the bands at full power (540 rpm) | About twice the robot's top speed (FRCDesign's rule of thumb, reported on Chief Delphi). A DECODE team found 20 % faster than the robot enough | low |

| Pre-stretch λ₀ | Tension per strip | Force at 5 / 10 / 15 / 25 / 27 mm |
|---|---|---|
| 1.5 | 4.2 N | 0.5 / 1.0 / 1.6 / 2.8 / 3.1 N |
| 2.0 | 7.0 N | 0.8 / 1.7 / 2.6 / 4.5 / 4.9 N |
| 2.5 | 9.4 N | 1.1 / 2.3 / 3.5 / 5.9 / 6.5 N |

**How it is simulated.**

| | Value | Source | Confidence |
|---|---|---|---|
| Bands | a cylinder patch about the axle, R 45.0 mm, rounding 0, \|X\| ≤ 84.54 | CAD | high |
| Treads | two cylinder patches, R 45.0 mm, rounding 0, \|X\| 84.54–109.54 | CAD | high |
| Band contact | the band's spring in series with the pollen's squash. `BAND_STIFFNESS` 0.25 N/mm per contact, 0.05–0.5. It pushes along the line from the axle through the ball's centre, with depth 45 + r − d, d the centre's distance from the axle. A nectar over the steep ramp's top has its centre about 43 mm from the axle, inside r 45; a nearest-point patch would pull it toward the axle | band model | low |
| Tread contact | the pollen's squash, μ 0.35 | materials | low |
| Motion | spins about its axle; the shape stays still and only the surface moves. A touch's support velocity is ω × the ball's contact point at its deflected radius, d − r (about 18 mm at the crest), not at r 45. At r 45 it would draw a pollen 2.5 times too fast | | |
| Drive | a DC motor line: `INTAKE_RPM` 540 rpm free at full power on 12 V, scaled by the battery's volts; stall torque 1.5 N·m; loaded by the torque the bands and treads take | assumed | low |

The roller's inertia is small against its gearbox, so its speed follows the
motor line at once. On the scoop a pollen's centre moves at half the band's
speed at its deflected radius. At the crest a strip is at about 18 mm, so the
pollen crosses at about ω·9 mm: 40 % of what an unbent strip would give, and
0.51 m/s (20 in/s) at the default speed.

Tested in batches 1, 2, 3a, 3b and 5.

### Scoop

**What the CAD says.** *Part 40*, now *Part 41* in the live document with the
same geometry. Its section is the same at every X between the walls:

| Face | From (Y, Z mm) | To (Y, Z mm) | Slope |
|---|---|---|---|
| lip | (3.730, 4.522) | (9.080, 9.357) | 42.11° |
| low ramp | (9.080, 9.357) | (33.550, 16.173) | 15.57° |
| steep ramp | (33.550, 16.173) | (47.285, 27.896) | 40.48° |
| crest | (47.285, 27.896) | (60.235, 27.896) | flat |
| back slope, onto plate 1 | (60.235, 27.896) | (89.833, 15.875) | −22.10° |

- **Underside:** a solid wedge, not a sheet. It rises at 3.71° from 4.52 mm off
  the floor at the front edge. A hook face at Y 49.5 sits 0.5 mm ahead of
  plate 1's front edge (Y 50). It rests flat on plate 1 (Z 15.875) from Y 49.5
  to 89.83.
- **Walls:** two triangular blocks, Z up to 50. The inner face is the plane
  0.631777·X + 0.775150·Y = 78.704, at 50.8° to the fore-aft axis (101.6°
  between the walls). Its X is 120.000 at the front edge, 93.41 at the roller's
  axle (Y 25.4) and 50.928 at the walls' back face (Y 60.026).
- **Mouth:** 240.00 mm wide. X ±120, so robot y ±4.724 in at x 7.885 in, z
  0.178–1.969 in.
- **Throat:** 101.86 mm, which leaves 15.37 mm each side of a pollen.
- **Highest point:** 27.896 mm (1.098 in), the crest.
- Two mounting counterbores (r 7.5 mm at X −56 and +72, on the back slope) are
  cosmetic. A pollen rolling over one dips about 0.8 mm.

The section is not convex at (33.55, 16.17), so it splits into three convex
solids. Each section below is in robot inches (x, z), extruded over
y −4.724…+4.724:

| Piece | Section |
|---|---|
| F1, lip and low ramp | (7.885, 0.178), (7.674, 0.368), (6.711, 0.637), (6.711, 0.254) |
| F2, steep ramp | (6.711, 0.254), (6.711, 0.637), (6.170, 1.098), (6.083, 1.098), (6.083, 0.295) |
| F3, crest and back slope, on plate 1 | (6.083, 0.625), (6.083, 1.098), (5.660, 1.098), (4.495, 0.625) |
| Walls (left; mirror y for the right), z 0.178–1.969 | plan triangle (7.885, 4.724), (5.668, 2.005), (5.668, 4.707) |

**What we infer.** Nothing about the shape: it is exact. Two things about
how the engine should see it:
- The faces between F1 and F2 (x 6.711, the concave corner) and between F2
  and F3 (x 6.083, inside the crest) are joins. A pollen rolling along the
  crest then feels one flat surface, with no ghost edge at x 6.083 and no
  double count. A pollen in the concave corner feels both ramps.
- The counterbores are left out.

**How it is simulated.** F1, F2, F3 and the two walls as convex solids
carried with the robot, with the two joins named. A pollen meets them with
its own squash, μ 0.30, restitution 0.50.

Tested in batches 1, 2 and 5.

### Plates, and the path to the feeder

**What the CAD says.**

| What | CAD (mm) | Robot frame (in) |
|---|---|---|
| Plate 1, on the path | Y 50–95, X ±140, Z 12.87–15.875 | x 4.291–6.063, z 0.507–0.625 |
| Plate 2, under the feeder | Y 120–165 | x 1.535–3.307 |
| Plate 3, under the feeder | Y 190–235 | x −1.220–0.551 |
| Cross rail | | x −2.219…−1.628, top at z 2.397 |
| The feeder floor's front rim | top at (Y 89.524, Z 19.471): a 3.60 mm step up from plate 1, and 0.31 mm *ahead of* the scoop's back edge | |
| Where an arriving pollen stops | in the V between the back slope and the tilted floor, centre (Y 86.50, Z 55.66), 76.26 mm from the feeder's axis | (4.626, 0, 2.191) |

The path runs from the crest down the back slope onto plate 1 at Y 89.833,
then up the 3.60 mm step onto the feeder's floor. Plates 2 and 3 lie under the
feeder and are never on the path.

**A finding for the team: the CAD has interferences.** The feeder's floor
plate goes 1.02 mm into plate 1, its boss 2.96 mm into plate 2, and the floor
plate 0.40 mm into the cross rail. The floor's rim overlaps the scoop's back
edge by 0.31 mm.

**How it is simulated.** The plates and the cross rail as boxes, aluminium.
The interferences stay as drawn. Overlapping parts are fine for the engine,
and a pollen meets their union. The 3.60 mm step is on the path and stays.

Tested in batch 1.

### Feeder

**What the CAD says** (13 October).
- **Tilt:** 16.4645° about X, rising toward the back (the feeder frame above).
- **Bottom plate** (*FeederBottom*): a disc of r 83.62 mm (Ø = 2·PollenDiameter
  + 25 mm), top face at d −6.701, 5 mm thick, with a boss of r 63.894 reaching
  10 mm further down. It holds a REV-41-1683 bearing block: a 22.44 × 33.15 mm
  pocket 10.41 mm deep, a Ø19.15 mm bore, and holes at exactly the block's
  pattern scaled by 1.02.
- **Top plate** (*FeederTop*): r 83.62, underside at d 69.419, top at 86.419,
  17 mm thick. It holds a standard-size servo: a 20 × 41.65 mm pocket 14 mm
  deep, Ø4.545 mm holes on a 49.995 × 10.1 mm pattern, and a Ø13.13 mm shaft
  bore on the axis, 10.15 mm from the pocket's centre. The pocket, its ears
  and its holes lie within \|x\| ≤ 10 and e2 −37.42 to 17.12.
- **Gap between the plates:** 76.12 mm, one pollen plus 5 mm (Sketch 3,
  `#PollenDiameter + 5mm`). `FeederHeight` (75 mm) and `FeederRadius` (55 mm)
  are not used by any feature.
- **Notch** in the top plate: \|x\| < 31.739 from e2 26.843 out to the rim,
  63.48 mm wide.
- **Front flat** on the top plate at e2 −59.795. Without it a pollen coming
  over the crest would hit the rim by 6.09 mm. With it there is 5.07 mm.
- The flywheel's flanges dip 8.30 mm below the top plate's underside, into
  the pocket space: 67.82 mm above the floor at e2 84.13.
- **Nothing hints at pockets, arms or a fence.**

**The live CAD has moved on** (14 October, mid-edit). The bottom plate is now
*Part 1*: a 167.24 mm square slab, 15 mm thick, with the bearing block's
recess and no boss. A REV 15 mm inside corner bracket was imported.
*Transform 5* is in error, and Reginald still shows the slab at the origin,
untilted. *Sketch 6*, a 7.5 × 8.59 mm rectangle on the slab's side, is not
yet extruded. The top plate is unchanged.

**What we infer.**

| Inference | Why | Confidence |
|---|---|---|
| **Both plates are fixed, and a 4-arm rotor turns between them on a continuous-rotation servo** | The servo is bolted into the top plate, and a continuous servo whose body turned would wind up its cable. A bearing block in the bottom plate only makes sense with a shaft turning inside it. The notch and the front flat only work at one orientation. The live square slab cannot turn (its corners sweep r 118.3) and carries a frame bracket | high |
| **Floor:** the bottom plate's top face at d −6.701, reaching r 96, cut at the front at e2 −83.62 | A pollen in the entry V at \|x\| 50 touches the floor at r 91, past the CAD's 83.62. The fence needs a floor under it. The cut keeps the rim, and so the V, where the CAD has them | medium |
| **Lid:** the CAD's top plate, but for its notch | | high |
| **Notch:** \|x\| < 42 (the back window's width) from e2 26.843 out to the rim, not the CAD's 31.739 | From the side, a pollen crossing the raised fin meets the CAD notch's edge 3.48 mm deep. No fin that reaches the wheel clears it: a 1 mm blade whose crest just reaches the wheel is 1.10 mm into the edge. At \|x\| < 42 the tightest clearance is 1.49 mm (3.13 at \|x\| < 45), and a centred pollen on stage 4's squeezed path has 7.49 mm (6.83 through the CAD's notch). The servo's pocket and ears end at e2 17.12, so the wider notch leaves them whole | low |
| **Rotor:** a hub of r 12; 4 arms, 4.0 mm thick, from r 12 to 88, 22–62 mm above the floor and full height inside r 25 | Pollen to hub 5.56 mm. Arm top to lid 14.12 mm and to the flywheel's flange 5.82 mm. The arms' lower edge clears the raised fin by 2.21 mm. The arms push at the pollen's equator, 35.56 mm up | medium |
| **Fence:** a fixed ring, r 93–96, floor to lid. A front window \|x\| ≤ 50.9 (the throat) and a back window \|x\| ≤ 42. An entry guide spirals from r 112 to r 93 over 60°, on the side the arms sweep toward | At r 55 a pollen overhangs the plates by 7 mm, so something must hold it. The windows let pollen in at the front and up into the flywheel at the back. The guide carries an arriving pollen in from the V to the fence | medium |
| **Pockets:** 90° wedges between the arms. A pollen's centre sits between r 53.12 (seated against both arms) and 57.44 (against the fence); for the field's pollen, 52.76 and 57.69 | Follows from the rows above. It spans the CAD's unused `FeederRadius` of 55 | medium |
| Positive power turns the rotor counter-clockwise seen from above, so the guide is on the left of the front window | The CAD does not say. Either way works | low |

**Pocket numbers** (the CAD's 71.12 mm pollen unless it says otherwise).

| Pair | Clearance (mm) |
|---|---|
| Neighbouring pollen at r 55 | 6.66 |
| Pollen to a 4 mm arm, each side, at r 55 | **1.33**: 1.58 for the field's 70.61 mm pollen, 0.83 for one 1 mm large, 0.06 at 2.54 mm large |
| Pollen to arm, each side, against the fence | 3.06 |
| Pollen to fence (seated) / to hub | 4.32 / 5.56 |
| Pollen to lid | 5.00 |
| Arm top to lid / to the flywheel's flange | 14.12 / 5.82 |

**The R 58 option.** Pockets at r 58 with 4 mm arms leave 3.45 mm each side,
and need the fence at r 96. The default stays at the CAD's 55 with the fence
at 93 (calibration item 12).

**The fling limit.** At the back station gravity pulls a seated pollen inward
at 2.779 m/s², because the plates rise toward the back. Above **1.15 rev/s**
the turn pulls harder than that, and a pollen passes the back station against
the fence instead of seated. The back window is 84 mm wide, 13 mm wider than
a pollen, so it is pressed out into the window there. Whether it then leaves
the feeder, catches on the window's far edge or reaches the fixed piece is
for the physics to show (batch 3b).

**The front station** is the opposite case. Gravity's 2.779 m/s² points
outward there at any speed, and the front window leaves nothing to hold the
pollen. On each pass a pocket's pollen rolls out toward the V (r 76.26), and
the guide puts it back against the fence. While an arm lies on the front
centreline, the pockets either side of it keep their pollen: each rolls along
that arm onto the fence beside the window. The preload sits that way
(batch 4).

Vendor speeds, no load:

| Servo | Speed | rev/s | Seconds per pocket |
|---|---|---|---|
| goBILDA 25-2 Torque | 40 rpm at 4.8 V | 0.67 | 0.375 |
| REV Smart Robot Servo | 0.14 s/60° at 6 V | 1.19 | 0.21 |
| goBILDA 25-3 Speed | 90 rpm at 4.8 V | 1.5 | 0.167 |
| goBILDA 25-4 Super Speed | 180 rpm at 4.8 V | 3.0 | 0.083 |

The Hub's servo ports give 5 V. A Servo Power Module or a Servo Hub gives 6 V.

**How it is simulated.**

| | Value | Source | Confidence |
|---|---|---|---|
| Floor | three convex slabs, with their joins named: the front piece, and one each side of the fin's slot (\|x\| ≤ 10.5 from e2 28.6 out). Arcs as 72-sided polygons, within 0.1 mm of the circle | inferred | medium |
| Lid | three convex slabs, 17 mm thick, with their joins named: ahead of the notch, and one each side of it, with the front flat | the CAD's top plate with the notch widened (assumed) | low |
| Fence | two cylinder patches, R 94.5, rounding 1.5 (r 93–96), between the windows; the guide as four flat plates | inferred | medium |
| Rotor | the hub as a cylinder patch, R 12, rounding 0; each arm as two boxes, with their joins named, in the rotor's frame, turned about the feeder's axis | inferred | medium |
| Contact | the pollen's squash, μ 0.30, restitution 0.50 | materials | low |
| Drive | a continuous servo: speed = `FEEDER_SPEED`·sign(power)·max(0, \|power\| − L ÷ stall torque), with L the load torque opposing the turn. `FEEDER_SPEED` 1.0 rev/s at full power, 0.2–3. Stall torque 1.1 N·m | the REV Smart Robot Servo's 0.14 s/60° and 13.5 kg·cm at 6 V (vendor), scaled to the Hub's 5 V; under the fling limit | low |
| Load | the torque the arms and the hub took from pollen in the step, from the engine | | |

A jam: an arm that presses a pollen against a window's edge loads the servo
past its stall torque. The rotor stops at the next step, with the pollen
squashed by less than one step's travel of the arm (1.7 mm at 1 rev/s, r 55).

Tested in batches 1, 3a, 3b, 3c, 4 and 5.

### Lift ramp

**What the CAD says.** Nothing: there is no ramp and no servo. What it does
fix:

| What | CAD (mm) | Robot frame (in) |
|---|---|---|
| A seated pollen at the back station | Y 210.58, Z 92.33; 87.91 from the flywheel's axle, 4.2° ahead of straight down | (−0.259, 0, 3.635) |
| Rise to touch the wheel | 12.62 toward the axle, 13.65 straight along n (11.97 and 12.83 at r 55). The field's pollen, seated at r 52.76: 13.24 and 14.35 | |
| Room through the top plate's notch, lifted onto stage 4's squeezed path | 6.83 (7.09 for the field's pollen; 6.99 with the hood at r 108.849) | |

**What we infer.**

| Inference | Why | Confidence |
|---|---|---|
| A hinged fin under the hood, 12 mm wide (\|X\| ≤ 6), on a positional servo | The team's "a servo lifts a small ramp". A fin fits a floor slot and the notch | medium |
| Its crest continues stage 4's hood groove bottom: r 108.341 about the flywheel's axle, from θ −96° to −73° | A pollen on it is then squeezed stage 4's 2.0 mm between fin and wheel (2.5 mm for the CAD's pollen), so the wheel takes it | medium |
| Ahead of −96° the crest drops away from the wheel, round a nose of radius 27.624 mm tangent at −96°, to r 112.849 at the tip (θ −103°) | It keeps the tip 19.56 mm above the floor and the arms 2.21 mm clear of the raised fin. Without it the tip would stand 23.49 mm up, 1.49 mm into the arms' sweep | medium |
| The fin is a half-round bar: its top is round across its width with radius 6 mm, and its underside is flat 6 mm below the crest | At the back station the pocket moves across the robot, so a pollen meets the fin from its side, not from its front end. A round top lets an arm push the pollen up onto it. A top round with radius 8 across 16 mm would need the fin 8 mm deep, and lowered 37° that reaches the cross rail | medium-low |
| A fixed piece of the same section from −73° to −64.8°, part of the hood. It replaces stage 4's hood extension and stop (batch 4) | It joins the fin to the hood at the same crest radius. It clears a pollen passing against the fence by 2.48 mm (2.38 for the field's) and a seated one by 5.74 mm | medium |
| The pivot runs along y at CAD (Y 249.12, Z 74.95), robot (−1.776, 0, 2.951) in | It lies at the fin's back end, 1.51 mm from its crest at −73°, so lowering the fin hardly moves it at the fixed piece while its front swings down through the floor's slot | medium |
| Down is 37° front-down | It balances a pollen passing against the fence (1.50 mm clear; 3.79 mm seated) against the cross rail (1.53 mm). At 32° a pollen against the fence would clear by 0.59 mm, and at 35° by 1.19 mm | medium |
| Raised, the crest is 15.67 mm above the floor at θ −93° and 19.56 mm at the tip, both along n. A pollen pushed across it rises up to 15.67 mm, past the 13.65 mm along n (12.62 toward the axle) that brings it to the wheel; 14.35 for the field's pollen | | medium |
| Lowered, the fin's front drops about 24 mm at the back station and 37 mm at the tip. A pollen crossing the slot dips 1.59 mm | | medium |
| The ramp's servo takes 0.168 s per 60°: 0.104 s for the 37° stroke | The REV Smart Robot Servo's 0.14 s/60° at 6 V (vendor), scaled to 5 V | low |

So with the ramp up, the passing pollen rides over the fin's crest and the
wheel takes it there. With the ramp down it rides on round.

**A finding for the team: the notch.** The top plate's notch is 63.48 mm
wide, narrower than a pollen, and a pocket's pollen has only 5.00 mm under the
lid. A pollen that meets the raised fin from its side can rise more than 5 mm
only where its cap fits the notch. 20 mm off the centreline the CAD's notch
lets it rise at most 6.99 mm, but the fin lifts it 10.54 mm there. The
overlap with the notch's edge peaks at 3.48 mm, 17.9 mm off the centreline.
The arm pushes with only about 20 N, so the rotor jams. The plan widens the
notch to the back window's \|x\| < 42 (item 23).

**A finding for the team: timing.** A pocket passes every 1/(4·rev/s): 0.25 s
at 1.0 rev/s, 0.21 s at 1.19. Up and down together take about 0.21 s at 5 V.
The robot code does not know the rotor's angle, because a continuous servo
has no encoder. So a ramp raised for a fixed time may lift none or two,
depending on when it starts. The plan does not wait for the team's fix. Its
default uses a sensor the robot already has: the flywheel. A pollen the wheel
takes slows it by about 100 ticks/s (*Flywheel and hood*), twice the 50
ticks/s the default looks for and four times the 25 ticks/s a driver's power
step moves the target. So `Launcher` raises the ramp and lowers it as soon as
the flywheel's measured speed falls more than 50 ticks/s below its target, or
after `RAMP_UP_SECONDS` if no pollen came. The next pollen climbs high enough
to reach the wheel only in the last 13 mm before the station, about 0.038 s
before it at 1 rev/s. The fin must be down by then. The drop passes 50
ticks/s within the take's 40 ms; one loop to see it and 0.104 s to lower the
fin make at most about 0.16 s of the 0.21 s there is. A sensor on the rotor,
or a slower feeder for the shot, would replace this default (calibration
item 22).

**A finding for the team: the fin's torque.** While the wheel takes a pollen,
stage 4's 2.0 mm squeeze presses the fin with about 40 N (two contacts of
40 N/mm in series), 45 N at the peak, about 38 mm from the pivot at θ −93°.
That is 1.5–1.7 N·m, against a stall torque of 1.1–1.3 N·m for a Smart Robot
Servo. A servo's gears usually hold more than they drive, and the pollen
leaves the fin within milliseconds. So the plan holds the fin where its servo
puts it and reports the torque. If the torque is real, the team may want a
stronger servo or a hard stop.

**How it is simulated.**

| | Value | Source | Confidence |
|---|---|---|---|
| Fin top | stage 4's arc collider rounded to 6 mm: its centre line at r 114.341 about the axle, so its face toward the axle, the crest, is at 108.341, θ −96° to −73°; ahead of it a second arc rounded to 6 mm, radius 21.624 mm about a centre 135.965 mm (108.341 + 27.624) from the axle along θ −96°, to the tip at −103°. Joined at −96° | inferred | medium |
| Fin section | half-round, 12 mm wide and 6 mm deep. The rounded arcs are whole tubes, 12 mm deep, but a pollen only ever meets their top half, so the clearance tests read the section, not the arcs | inferred | medium |
| Fixed piece | the same top and section, −73° to −64.8°, carried with the robot. Joined to the fin at −73° and to the hood's middle ring at −64.8° | inferred | medium |
| Motion | turns about the pivot: up is 0°, down 37° front-down. The angle slews toward the commanded one at the servo's speed, and is held whatever the load | inferred | medium |
| Positions | `Launcher`'s two ramp positions are down and up. A position between them is the angle between, linearly | read from `Launcher`, so there is no second copy | high |
| Contact | the pollen's squash, μ 0.30 | materials | low |

Tested in batches 1, 3a, 3b and 4.

### Flywheel and hood

Stage 4's wheel and hood stand as designed (groove bottoms r 39.729 and
108.341).

This plan changes three things.

1. **The flywheel slows when it throws.** Stage 4 takes the wheel's speed from
   the command. Here the wheel has inertia and a motor, and the hub's velocity
   controller drives the motor with the robot code's coefficients. The
   `AngularSpeed` that `Flight` gets is the simulated wheel's. So a shot takes
   speed off the wheel, `flywheelReady` sees that, and the next shot waits.
2. **The ramp feeds it.** `Flight.fed` and its entry check go with the
   chamber: no ball is put at the hood's lower end. The pollen comes up from
   its pocket on the fin, already in the full squeeze.
3. **Below −64.8° the fin and the fixed piece replace stage 4's stand-ins.**
   - The fixed piece replaces the hood extension. Its crest continues the
     groove bottom at r 108.341, so the fin, the fixed piece and the hood's
     middle ring make one curve.
   - The stop goes. A stopped wheel still holds a pollen: on the fin when the
     ramp is up, and in its pocket when it is down.
   - The lead-in's middle ring goes, because the fixed piece is there. Its
     four side rings stay, so the groove's sides still open gently.

**What the CAD says about the drive:** nothing. It has no motor and no
gearing, and its printed parts have no material, so it gives no inertia. The
robot code sets the velocity PIDF to (250, 0, 0, 12.9) and the close launch to
1050 ticks/s (`Launcher`). The simulator ignores the coefficients, and sets
the measured velocity to the commanded one (`SimRobot`).

**What we infer.**

| Inference | Why | Confidence |
|---|---|---|
| A 6000 rpm, 28-tick motor, like a goBILDA 5203: stall torque 0.144 N·m. It turns the wheel once per `FLYWHEEL_TICKS_PER_TURN` ticks | The code's F of 12.9 means 32767 ÷ 12.9 ≈ 2,540 ticks/s at full power, by the SDK's convention F = 32767 ÷ top speed. That is a 6000 rpm, 28-tick motor at about 90 % of its free speed. Stage 4's calibration fixes the ticks per turn, about 24.4: a 1.15:1 step-up. Batch 4 calibrates it again | low |
| Inertia 0.95 lb·in² (2.78·10⁻⁴ kg·m²) | The Launch Sim's default wheel inertia. A printed 100 × 50 mm wheel at about 40 % fill is about 0.8 | low |
| The hub's controller: power = (P·e + I·∫e + D·ė + F·target) ÷ 32767, clamped to ±1, run every substep (at most 5 ms) | The SDK's PIDF convention. The hub's own loop period is not published | medium for the form, low for the period |

Stage 4 expects the calibration's centre near 24.2, from runs of its model
with the hood at r 108.849. With the hood placed for the 1.39 in pollen, as
stage 4 now places it, the same model threw 220.9 in/s at 25 ticks per turn
instead of 219.2. That puts the centre for 226 in/s nearer 24.4, and 28 ticks
per turn throws about 197 in/s.

**What follows.** At 1050 ticks/s and 24.4 ticks per turn the wheel turns at
270 rad/s and holds 10.2 J. A pollen at 226 in/s with stage 4's backspin
(S about 1.05) carries about 0.69 J: 0.41 moving, 0.28 spinning. Stage 4's
simulated launch puts 38 % of the wheel's work into the ball, so the wheel
loses about 1.8 J: **9 % of its speed, about 100 ticks/s**. The robot's
ready window is ±15 ticks/s. At full power the motor gives the speed back in
roughly 0.1 s.

**How it is simulated.**

| | Value | Source | Confidence |
|---|---|---|---|
| Speed | stepped once per Flight step, in two parts. The motor's: J·(ω′ − ω₀) = h·G·τ_motor at ω₀. The balls': ½J(ω₁² − ω′²) = −W, where W is the work the wheel's surface did on the balls in the step at the speed ω₀ it held. ω₁ keeps ω′'s sign, and is 0 if W is more than ½Jω′². G = `FLYWHEEL_TICKS_PER_TURN` ÷ 28 | | |
| Motor | τ = τ_stall·(power·V/12 − ω_motor/ω_free) | a DC motor line | medium |
| Constants | `FLYWHEEL_INERTIA` 0.95 lb·in², 0.05–5. The default motor's free speed and stall torque are core constants | vendor; the Launch Sim | low |
| What `Flight` gets | the wheel's speed as stage 4's `AngularSpeed`, signed, finite, clamped to the motor's top speed | stage 4 | high |
| What the code reads | ω·T ÷ 2π ticks/s, rounded as the hub reports it | **Clocked encoder** | high |

So the energy the balls take comes off the wheel exactly, and the motor alone
still starts the wheel from rest and turns it backward, which an energy rule
for the motor could not.

Tested in batches 3a, 3b and 4.

## Handoffs

| From | To | Where | What passes |
|---|---|---|---|
| dyn4j (the floor) | the engine | the intake box | position, velocity, rolling spin, and dyn4j's spin about z, as a `Motion` through stage 3's `Flight.with` |
| the engine | the engine | everywhere inside the robot, and out through the hood | nothing: one body from the box to the cell |
| the engine | dyn4j | on the floor outside the box, by stage 3's landing rule | (x, y) and (vx, vy). Spin about a level axis is dropped, as in stage 3 |

**The intake box.** In the robot frame: centres with \|y\| ≤ 4.724 in − r,
so the whole ball is inside the mouth's 240 mm; on the floor; from the lip
(x 7.885) out to the footprint's front edge at the roller (x 8.803) plus the
ball's radius plus a margin. The margin is one substep's closing travel, and
never less than 0.25 in, today's tolerance at the front. The box reaches a
ball's radius past the footprint's front at the roller, where dyn4j's
footprint ends.

**Which balls.** Every ball whose centre enters the box: pollen or nectar,
intake on or off. The engine decides what happens next, as the real robot
would. Today's rule (a pollen, the intake on, room left) goes. A ball that
straddles the mouth's edge stays in dyn4j and is pushed, as today.

**Two engines meeting.** A dyn4j ball that touches a ball the engine holds in
the box is handed over too, so the two never pass through each other.
Elsewhere they meet only as in stage 3: a ball still bouncing outside the box
passes through dyn4j's balls.

**The robot does not feel the balls in the engine.** A pollen pinned between
the lip and a wall does not need it to: the lip's 42° face and a vertical wall
make a 48° wedge that opens upward. A pinched ball stays only if friction
reaches tan 24° ≈ 0.45. At the walls' 0.30 it climbs over the lip. dyn4j still
pins whatever is outside the mouth. Two-way coupling is under Later.

**A ball that falls through the robot** to the field floor (one pressed out
through the back window, say) lands by stage 3's rule, and dyn4j pushes it
out of the footprint. That holds until the chassis is a collider (Later).

**Never landed while touching the robot.** A compliant touch is invisible to
stage 3's landing rule. So the rule gains one condition: a ball with any
compliant touch does not land.

**Held and flying.** A ball in the engine is **held** while its centre is in
the box or inside the footprint's prism and it has not touched the flywheel
since it came in. Otherwise it is **flying**, as stage 4's fed ball is from
the hood's lower end. `held()` counts the first.

## What it costs

From the Launch Sim's measured work: free flight costs 4 units per awake ball
per piece. Under stage 4's count, a ball in compliant contact costs 8: one
RK4 step, four compliant queries (one per stage) and three queries of the
field. At the engine's 0.1 ms step that is 40,000 and 80,000 units per
simulated second.

| Scenario | Units per simulated second | Source |
|---|---|---|
| Nothing moving inside the robot (feeder still; robot still or cruising) | bounds checks only | asleep in the robot (batch 3c) |
| Four pollen riding a turning feeder | about 320,000 | 4 × 80,000 |
| The same, and a fifth pollen under the roller | about 400,000 | 5 × 80,000 |
| One shot through the hood (about 40 ms in contact) | about 3,000 per shot | stage 4's count: about 370 RK4 steps and 1,480 compliant queries |
| Four balls flying | 160,000 | measured |
| A launch rattling in a cell among nectar, busiest 20 ms | 418,000 | measured |
| Four riding, a fifth taken, two flying and a rattle | about 0.9 million | sum |

Warm, a unit costs 0.46–0.79 µs. So four riding pollen cost 0.15–0.25 s of CPU
per simulated second.

What keeps it there:
- **The internals are `Surroundings.Part`s.** Stage 2 asks a part only when
  it `reaches` the ball, so a ball outside the robot never queries the
  internals' 36 pieces, and the field's 120 colliders are not scanned again
  for them.
- **Compliant contacts are not located by bisection** (stage 4), so pocket
  pollen add no events. Ball against ball inside the robot still bisects.
- **Asleep in the robot** (batch 3c): a still feeder in a cruising robot costs
  only bounds checks, and `Bodies.idleThrough` still jumps.
- Stage 3's work allowance, 15,000 units each 5 ms window of steps, still
  bounds a burst.

One cost grows. Every bisection integrates every awake body again, so four
riding pollen make each event elsewhere on the field about five times dearer
than it is with one ball flying.

**The risk.** `SimRunnerTest`'s 2 s run must finish in under 1 s. It runs
`WaitingAuto`, which builds Reginald's `Robot`, whose `Intake` starts on.
Under batch 4's rule the feeder turns whenever the intake runs, so the four
preloaded pollen ride round: about 640,000 units, 0.3–0.5 s warm and 1.3 s at
the 2.0 µs a unit that short cold runs measured, so red is likely. The levers
above do not reach riding pollen: they are awake, in compliant contact, which
never bisects, and far under the work allowance. The test does not change.
Whether a feeder should turn while nothing needs feeding is the robot code's
question, and so the team's. The default is `reginald.md`'s: it turns
whenever the intake or the flywheel runs. So before batch 4 starts, put this
question to the user with these numbers. Batch 4 then measures it, and if the
test is red, stops and reports the measurement.

**Pinned work.** `FlightTest` pins the exact work of three scenarios: four
pollen riding one turn at 1 rev/s (batch 3b), one ramp-fed shot (batch 4),
and one pollen taken at 30 in/s (batch 5). A change in cost then shows as a
diff, not as a slower suite. Each batch records its suite times and `RunCost`
before and after in the telemetry store.

## Findings for the team

| # | Finding | Numbers | What follows |
|---|---|---|---|
| 1 | The roller cannot reach a pollen at the lip | 7.92 mm short; the pollen rises 5.23 mm before a strip touches; it needs an approach of 0.43–0.56 m/s, about 18 in/s | Slower, the scoop pushes pollen ahead. A roller of r ≥ 52.92 mm, or an axle at ≤ 105.28 mm (4.14 in), would reach; the lower axle raises the crest overlap to 35.70 mm |
| 2 | The bands bend up to 27 mm at the crest | 26.99 mm; 3–13 N | The intake works only through the bands' give. Measure them (item 4) |
| 3 | A second hurdle on the steep ramp | stalls between Y 16 and 27 mm at μ 0.3 | The scoop's support friction limits draw-in, not the bands' |
| 4 | The CAD's hood squeezes 0.29 mm and has no way in | 0.289 mm; a sharp lower edge | Stage 4 squeezes 2.0 mm; here the fin feeds it |
| 5 | The CAD has interferences | 1.02 (floor, plate 1), 2.96 (boss, plate 2), 0.40 (floor, cross rail); rim 0.31 mm over the scoop, 3.60 mm step | Kept as drawn |
| 6 | The arms are tight | 1.33 mm each side at r 55; 0.83 for a pollen 1 mm large; 0.06 at 2.54 mm | R 58 gives 3.45 (item 12) |
| 7 | The fling limit | 1.15 rev/s; a Smart Robot Servo at 6 V turns 1.19 | Above it a pollen passes the back station pressed into the window. Choose the servo and its voltage under it |
| 8 | The ramp's timing | up and down about 0.21 s; a pocket every 0.21–0.25 s; no rotor sensor | A fixed ramp time may lift none or two. The default lowers the ramp once the flywheel's speed drops (item 22) |
| 9 | The fin's torque | 1.5–1.7 N·m against a stall of 1.1–1.3 | The servo may give way |
| 10 | The flywheel slows per shot | about 9 %, 100 ticks/s, at 0.95 lb·in²; the ready window is ±15 | Shots wait about 0.1 s to recover |
| 11 | The close launch's power | Under drag, Magnus and contacts the close shot scores from 40 in at 224.0–228.0 in/s. Stage 4's hood throws about 0.425 of the 100 mm surface speed, so at 1:1 on a 28-tick motor 1050 ticks/s throws about 197 in/s and lands short. The shot needs about 1193–1214 ticks/s: 21 ticks/s wide, narrower than the ±15 window (30 wide) and the 25 ticks/s power step | The robot code's. Stage 4 calibrates the simulator's ticks per turn instead. A ready window of ±8 or less is worth having |
| 12 | A nectar may be drawn onto the scoop | 91.44 mm against the roller's 69.30 mm gap: 22 mm on the bands. It cannot pass the 76.12 mm feeder gap, and the intake has no reverse | A prediction. Batch 5's replay shows what happens |
| 13 | Two pollen sizes | the field's 70.61 mm; the CAD's 71.12 mm | Item 1 |
| 14 | A fifth pollen meets every arm | a pollen waiting in the V sits at r 76.26, its near side at r 40.7, and the arms reach r 88 every quarter turn; each passing pocket's pollen also rolls out there, and with the next pocket full the guide wedges the fifth against it | Predicted to stall the rotor. Batch 5's replay shows what happens |
| 15 | The top plate's notch is narrower than a pollen | 63.48 mm wide; a pollen crossing the raised fin from the side meets its edge 3.48 mm deep, and no fin that reaches the wheel clears it | The plan widens it to \|x\| < 42, the back window (item 23). Widen it, or say how a pollen rises through it |

## Facts in reginald.md that are now known to be wrong

Stage 4 writes the corrections it lists into `reginald.md`: the mouth, the
height, the exit, the hood's squeeze, the retired throw and batches 5, 6
and 7. Batch 1 writes the rest.

| `reginald.md` says | Now known | Source |
|---|---|---|
| The 2-D check scores at 205–223 in/s; the throw 0.204 in/s per tick/s is 46 % of the surface speed | Under drag, Magnus and contacts it scores at 224.0–228.0 in/s, and 0.204 is a knife edge. Stage 4's simulated hood throws about 0.425 of the 100 mm surface speed | Launch Sim runs; inferred: stage 4's launcher throws 220.9 in/s at 1050 ticks/s and 25 ticks per turn, where the surface moves at 519.5 in/s |
| The feeder is tilted about 17°, its plates `FeederHeight` ≈ 75 mm apart, with pockets at `FeederRadius` | 16.4645°. The gap is 76.12 mm (pollen + 5 mm). Neither variable is used by any feature | the CAD's features |
| The discs sit on a servo hub | Both plates are fixed; a rotor turns between them | this plan |
| *Part 40* is the funnel | It is *Part 41* in the live document, with the same geometry. The reduction must find it by its geometry, not its name or id | the live document |
| "The roller pulls a pollen up the funnel" | The robot's own speed, about 0.45 m/s, carries a pollen over the lip; the roller first reaches it 5.23 mm up | the CAD's exact faces |

## Measurements and confirmations

Every item has the default the plan uses until it is answered. Each answer
replaces that default: a mechanism constant on the admin page, a core
constant with its new source, an `assumed` entry in `robot.json`, or the
robot code. After each, run stage 3's acceptance set again.

| # | Measure or confirm | How | Default until then | Replaces |
|---|---|---|---|---|
| 1 | Pollen diameter | 10 balls, calipers, 3 axes each, one across the seam: mean, least, most | 70.61 mm (the field's); 71.12 for the CAD's clearance tests | the field's pollen radius; batch 1's clearance tests |
| 2 | Pollen squash between flat plates | force at 1, 2, 4, 6 and 8 mm of travel, and where it snaps | stage 4's 40 N/mm per contact (16–23 N/mm between plates) | the pollen's contact stiffness |
| 3 | Pollen squash in grooves | as 2, between blocks grooved at r 36 and r 35.56 | as 2 | the wheel's and hood's contact stiffness |
| 4 | Band force | push one strip at mid-span with a pollen, then with a rigid 71 mm ball: force at 5, 10, 15 and 25 mm; the strand's installed and free length | 0.25 N/mm per contact | `BAND_STIFFNESS` |
| 5 | Friction | three pollen taped into a triangle, tilted on the scoop's material and on a REV plate; μ is the tangent of the angle where it slides | 0.30 plastic; 0.35 aluminium | the materials |
| 6 | Roller speed | video at full power, empty and with a pollen under it | 540 rpm | `INTAKE_RPM` |
| 7 | The intake's motor and gearing, and which way positive power turns it | confirm | stall 1.5 N·m; positive pulls in | the motor's constants |
| 8 | The feeder: two fixed plates and a 4-arm rotor on a continuous servo? Which servo, at 5 V or 6 V? How is the top plate held up? What are the corner bracket and *Sketch 6* for? | confirm | as inferred; a Smart Robot Servo at 5 V | the `assumed` feeder |
| 9 | The feeder's speed | 120 fps video, a tape mark, frames over 10 turns, empty and with four pollen | 1.0 rev/s | `FEEDER_SPEED` |
| 10 | The arms | how many, how thick, how tall above the bottom plate, what material, straight or swept, to what radius | 4; 4.0 mm; 22–62 mm; plastic; straight; r 12–88 | the `assumed` rotor |
| 11 | A fence or housing | inner radius, height, openings at the intake and at the flywheel | r 93–96, floor to lid, windows \|x\| ≤ 50.9 and 42, a guide | the `assumed` fence |
| 12 | Pocket radius | 55 or 58 mm | 55, fence at 93 | the `assumed` fence |
| 13 | The feeder's direction | which way positive power turns it | counter-clockwise seen from above | the `assumed` rotor |
| 14 | The ramp | a hinged fin under the hood? Its hinge, length, width and section, its servo, its two positions in degrees, how it sits against the notch | the fin above, 12 mm wide and half-round; 37° | the `assumed` fin |
| 15 | The ramp's stroke | 120 fps video, up and down | 0.168 s per 60° | `RAMP_SERVO_SECONDS` |
| 16 | The flywheel's motor and gearing | confirm | a 6000 rpm, 28-tick motor; ticks per turn as batch 4 recalibrates them. Expect fewer than stage 4's 24.4: the wheel has lost part of its 9 % by the time the ball leaves | the motor's constants; `FLYWHEEL_TICKS_PER_TURN` |
| 17 | The flywheel's inertia | weigh and measure the wheel and its hub | 0.95 lb·in² | `FLYWHEEL_INERTIA` |
| 18 | The hood's squeeze as built | the gap at the groove bottom | 2.0 mm on the 1.39 in pollen (try 1.5–3) | stage 4's hood, and the fin's crest with it |
| 19 | The throw | 240 fps video: exit speed, angle and spin against rpm, at three squeezes | stage 4's model | checks the whole launcher; fits 2, 3 and the wheel's μ |
| 20 | The bottom plate | where the live 15 mm square slab's top sits against the 13 October floor | the 13 October floor plane | the `assumed` floor |
| 21 | Reverse | will the intake ever run backward, to clear a nectar or a jam? | no: the code has on and off | nothing; the simulator already takes negative power |
| 22 | How a launch picks out one pocket | a sensor on the rotor, a slower feeder for the shot, or the flywheel's drop | lower the ramp once the flywheel's measured speed falls 50 ticks/s below its target, or after `RAMP_UP_SECONDS` | `Launcher` |
| 23 | The top plate's notch | its width as built, and how a pollen the ramp lifts rises through it | \|x\| < 42 | the `assumed` notch |

## Asks of the CAD

No batch waits on these. Each makes the next update cheaper:
- Draw the rotor, the arms, the fence, the fin and both servos, so a later
  reduction reads them and the `assumed` entries go.
- Widen the top plate's notch to the back window's \|x\| < 42, or say how a
  pollen the ramp lifts rises through it (finding 15).
- Finish the bottom plate (*Transform 5* is in error, and Reginald shows it at
  the origin), and make it reach the fence.
- Resolve the three interferences and the step at the rim.
- Name the scoop. The reduction will still find it by its geometry.
- Put mate connectors at the feeder's axis and at the ramp's pivot.
- Trim the hood at +20°, as `reginald.md` already asks, and place its groove
  bottom at r 108.341 for 2.0 mm of squeeze on a 70.61 mm pollen, or say what
  squeeze the robot is built with (item 18).

---

## Batch 1 · Reginald's internals are shapes in the core

Nothing moves in a run. The internals become a validated value read from
`robot.json`, and the engine gains the shapes it lacks. Every measurement in
this plan's tables is asserted from the committed geometry, so a wrong number
fails at the earliest layer that can name it.

- `robot.json` gains **`internals`**: each part's numbers in its own frame
  (robot, feeder, rotor, ramp). Each entry is marked `measured`, from the
  CAD's exact faces in the 13 October snapshot, naming the part, or `assumed`,
  inferred here, naming why. Nugget's file has none.
- In the core, **`Internals`** holds the scoop, the plates, the cross rail,
  the roller, the feeder's plates, the fence, the rotor, the fin and the fixed
  piece, with the fin's half-round section. It refuses, naming the rule:
  - a radius or a slab's thickness that is not positive, or a rounding that
    is negative;
  - arms that do not reach from the hub to inside the fence;
  - a floor and lid that are not parallel and square to the rotor's axis, or
    no further apart than the ball they are built for;
  - a window wider than its fence;
  - a fin or fixed piece whose crest (its radius less its rounding) is not
    stage 4's hood groove bottom, so the three cannot drift apart;
  - a fin section whose top is not round across its whole width: a width
    other than twice the rounding, or a depth less than the rounding.
- **Cylinder patch**: a cylinder limited to an angular span and an axial span,
  with a rounding. Its nearest point is `Arc`'s, taken at the axial position
  clamped into the span, then offset by the rounding, so there is no second
  closed form to keep in step. It is total on the axis, as `Arc` is. It is
  the bands and the treads (R 45, rounding 0), the fence (R 94.5, rounding
  1.5) and the hub (R 12, rounding 0).
- **`Arc`** (stage 4) gains a rounding, zero for the wheel's and the hood's
  rings, which then give stage 4's answers exactly. The fin and the fixed
  piece are arcs of R 114.341 rounded 6 mm, and the fin's nose is one of
  R 21.624 about its own centre, rounded 6 mm.
- **Joins**, as *How a pollen meets Reginald's parts* says. A convex solid
  may name faces it shares with a neighbour, and an `Arc` may name either end.
  A join may pair pieces of two parts. A touch whose nearest point lies on a
  join face, or on an `Arc` end named as a join, is dropped; on the join
  itself, the touch of the piece the join names first is kept. Of a part's
  touches whose normals agree within 1°, only the deepest counts.
- `SimChassis` reads the internals into the core types. A refused part fails
  the load, naming its rule.
- `reginald.md` gains the corrections in *Facts in reginald.md that are now
  known to be wrong* that stage 4 has not written.

Tests first:

1. `CylinderPatchTest`:
   - Gaps and normals from inside, from outside, past each end of the angular
     span, past each end of the axial span, and at a corner.
   - A point on the axis gets a gap and a normal.
   - A full turn has no angular ends.
   - Inside its axial span it answers exactly as `Arc` does in that plane.
   - A radius that is not positive, an empty span or a value that is not
     finite is refused, naming its rule.
2. `ArcTest`: a rounding comes off every gap. Zero rounding gives stage 4's
   answers exactly, bit for bit. A negative rounding is refused, naming the
   rule.
3. `JoinTest`:
   - A pollen rolled along the scoop's crest across the F2/F3 join keeps one
     touch, a straight-up normal and the same depth, on the join itself too.
   - A pollen resting in the F1/F2 corner keeps two touches.
   - A pollen rolled along the fin's crest across its −96° join, and with the
     fin up across its −73° join to the fixed piece, keeps one touch.
   - A solid naming a join face it does not have is refused, naming the rule.
4. `InternalsTest`: each rule above is refused, naming it.
5. `InternalsMeasurementsTest`: with the CAD's 71.12 mm pollen, the committed
   geometry gives each of these, and each assertion's message names the
   measurement:
   - the mouth 240.00 mm, the throat 101.86 mm, the crest 27.896 mm up;
   - the roller 69.30 mm off the floor, and its overlap 26.99 mm at most, with
     the centre at (Y 38.5, Z 62.35);
   - first contact with the lip at Y −14.96, where the roller is 7.92 mm short;
   - the entry V 76.26 mm from the feeder's axis, and the 3.60 mm step at the
     rim;
   - the plates 76.12 mm apart; a pocket's pollen between r 53.12 and 57.44;
     1.33 mm each side of an arm at r 55; arm top to lid 14.12 mm and to the
     flange 5.82 mm;
   - at the back station, 87.91 mm from the axle, 12.62 mm short of the wheel
     toward it and 13.65 mm along n; 7.49 mm of room through the widened
     notch on stage 4's squeezed path;
   - a pollen carried across the raised fin clears the lid at every offset,
     by 1.49 mm at the tightest;
   - the fin's and the fixed piece's crest at r 108.341, and the nose's tip at
     r 112.849, θ −103°; raised, the crest 15.67 mm above the floor at θ −93°
     and the tip 19.56 mm; the arms 2.21 mm above the fin; the fixed piece
     2.48 mm clear of a pollen passing against the fence; lowered 37°,
     1.50 mm clear of a pollen passing against the fence and 1.53 mm clear of
     the cross rail, all read from the fin's half-round section;
   - each to 0.01 mm where this plan gives two decimals.
6. `SimChassisTest`:
   - Reginald's internals load.
   - Nugget's file has none.
   - A refused part fails the load, naming the rule.
7. Every existing test and golden trace passes unchanged, the Launch Sim's
   fingerprints among them.

Then implement, and update `doc/CONTEXT.md`:
- new entries: **Internals**, **Join**
- **Reginald** gains his internals; **Arc** gains its rounding

## Batch 2 · what a pollen feels against Reginald's parts

This batch is in the core and moves nothing in a run. The engine meets the
internals through stage 4's compliant query, tried in small worlds with the
robot's frame held still.

- **The internals are `Surroundings.Part`s** carried in stage 4's frame: one
  for the parts that do not move within the robot (the scoop, the plates, the
  cross rail, the feeder's floor and lid, the fence and the fixed piece), and
  one each for the roller, the rotor and the fin. Each answers stage 4's
  `squeezing` query and nothing through `near`. Its ids are its pieces' names,
  which stage 2's `Surroundings.of` already keeps unique. They are held
  beside the hives, the bores and the launcher, so a tip, which replaces the
  hives' part alone, and `amid`, which rebuilds the surroundings, both keep
  them.
- Every touch between a ball and a part of the internals is compliant, as
  *How a pollen meets Reginald's parts* says: the pollen's own squash, the
  band's spring in series for the bands, damping for each part's restitution,
  and friction toward the part's surface velocity at the ball's contact point.
  The bands push away from their axle (*Intake roller*). Stage 4's wheel and
  hood are unchanged.
- The materials are named core constants with their sources.
- **`BAND_STIFFNESS`** joins the mechanism constants: newtons per millimetre
  per contact, 0.25 built, 0.05 to 0.5. Stage 4's hello lists what a child
  reads, so a new constant needs no protocol bump.
- No sub-steps; the regularisation floor in *How a pollen meets Reginald's
  parts*, with its test.

Tests first:

1. `RobotContactTest` (core):
   - Against the bands, the force at a deflection δ is that of the two springs
     in series, within 1 %. At the CAD's 26.99 mm, with the built stiffness,
     it is 6.7 N, inside the 3–13 N the band model gives.
   - A ball whose centre is inside r 45 is pushed away from the axle.
   - Each material's μ and restitution reach the contact: dropped onto each
     part, a pollen bounces with that part's restitution, within 0.05, and
     sliding on it, it slows as that part's μ says, within 2 %.
2. Closed forms, each with the real scoop or a plate:
   - **Draw-in.** A pollen pinched between a band surface moving at u and a
     still plate, meeting at an angle α (their normals 180° − α apart), is
     drawn through when tan(α/2) is below the plate's μ, and stalls spinning
     when it is above. Checked 2° either side of 2·atan μ.
   - **Centre speed.** At the crest's 27 mm deflection, between the bands and
     the still scoop, once it grips the centre moves at ω·9 mm, within 2 %.
     Stage 4 already proves (u+w)/2 for a rigid support.
   - **The lip.** With no roller, and the scoop's friction and restitution set
     to zero, a pollen meeting F1's lip at 0.46 m/s rises past 5.23 mm (where
     the first strip would reach it), and at 0.40 m/s it does not. The
     threshold is √(2gΔz)/cos β, with Δz 5.23 mm and β 42.5°: 0.43 m/s.
3. The step:
   - **Convergence at the crest.** A pollen drawn over the crest by the bands,
     at the built `BAND_STIFFNESS` and at 0.5, with the scoop's μ at 0.5 so
     neither run stalls, takes the same time from rest at Y +10 on the low
     ramp (μ 0.25 needed) to the V, and arrives at the same speed, within 2 %,
     at stage 1's `Step` of 0.1 ms and of 0.05 ms.
   - **Stick.** A pollen held still between a plate and the still bands at
     13.3 N stays under the rest speed and gains no energy over 1 s at 0.1 ms.
     It is red under a fixed 0.02 m/s, and green under the floor.
   - A pollen resting on a plate comes to rest by stage 4's rule and stays put
     exactly.
4. Energy: a pollen rattling between still parts for 1 s never gains energy.
5. Each part passes stage 2's `PartContract`. These parts answer only
   `squeezing`, which the contract does not read, so the contract is run over
   `squeezing`'s touches too (stage 4 adds that, or this batch does).
6. `ConstantsTest` and `SimConstantsTest`: `BAND_STIFFNESS`'s range and built
   value. `SimRunStreamTest`: the child's hello lists `band_stiffness`.
7. The Launch Sim's fingerprints, every Launch Sim test and stage 4's launcher
   tests pass unchanged.

Then implement, and update `doc/CONTEXT.md`:
- new entries: **Robot contact**, **Carried part**
- **Flight** (a ball against Reginald's parts), **Simulation constants** (the
  bands)

## Batch 3a · the drives and the flywheel's motion

This batch is pure core. It gives the moving parts their drives and the
flywheel its inertia and controller, and nothing moves in a run yet.

- **Drives**, pure functions in the core:
  - **a DC motor line**, τ = τ_stall·(power·V/12 − ω/ω_free): the roller's
    motor and the flywheel's motor;
  - **a continuous servo**: speed = free speed·sign(power)·max(0, \|power\| −
    L ÷ stall torque), with L the load torque opposing the turn: the rotor;
  - **a positional servo**: the angle slews toward the commanded one at the
    servo's speed, and holds whatever the load: the fin;
  - **the flywheel**: stage 4's `Flywheel` gains the wheel's motion, stepped
    once per Flight step as *Flywheel and hood* says: the motor's torque
    first, then the work the balls took, which never carries the wheel
    through zero. Its power comes from **the hub's velocity controller**,
    (P·e + I·∫e + D·ė + F·target) ÷ 32767, clamped to ±1. Its speed is an
    `AngularSpeed`, so it stays finite and under the motor's top speed.
- **Mechanism constants**, guesses until measured:
  - `INTAKE_RPM`: 540 built, 100–2000;
  - `FEEDER_SPEED`: revolutions a second at full power, 1.0 built, 0.2–3;
  - `RAMP_SERVO_SECONDS`: the ramp servo's seconds per 60°, 0.168 built,
    0.05–0.5;
  - `FLYWHEEL_INERTIA`: 0.95 lb·in² built, 0.05–5.

  The default motors' and servos' stall torques and free speeds are core
  constants, with their sources.

Tests first:

1. `DrivesTest` (core):
   - The motor line gives its free speed at full power and its stall torque
     at rest, both scaled by the volts.
   - The continuous servo turns at its free speed at full power and stops at
     its stall torque. A larger load holds it still and never turns it
     backward. At zero power it stops. Negative power turns it the other way
     at the same speeds, and a load opposing that turn slows it the same.
   - The positional servo reaches the commanded angle after the stroke divided
     by its speed. A load does not move it.
   - Each refuses a parameter that is not finite or not positive, naming the
     rule.
2. `FlywheelTest` (stage 4's, core):
   - From rest, with the code's (250, 0, 0, 12.9) and 1050 ticks/s, it settles
     within ±15 ticks/s and stays there.
   - With the motor's torque at zero, a load that takes a shot's energy slows
     it by exactly that energy, to 1e-9 J. With the motor on, the controller
     brings it back inside ±15.
   - A load larger than the wheel's energy stops it at zero, not past it.
   - Its power never leaves ±1.
3. `ConstantsTest` and `SimConstantsTest`: the four constants' ranges and
   built values. `SimRunStreamTest`: the child's hello lists `intake_rpm`,
   `feeder_speed`, `ramp_servo_seconds` and `flywheel_inertia`.
4. Every existing test, the Launch Sim's fingerprints and stage 4's launcher
   tests pass unchanged.

Then implement, and update `doc/CONTEXT.md`:
- **Flywheel** (its inertia and the hub's controller), **Simulation
  constants**

## Batch 3b · the parts move within the robot

This batch is in the core. Parts move in the robot's frame, each has a drive
from batch 3a, and the engine tells each drive the load it took.

- **Carried parts that move.**
  - The rotor turns about the feeder's axis, and the fin about its pivot.
    `Flight` keeps each one's angle where its last step ended, as it keeps
    stage 4's launcher frame, and each step is given the angles it ends at.
    Between them the angle at engine time t is interpolated linearly, as stage
    4 does for the frame, extrapolated past the end and never clamped.
  - The roller and the flywheel spin about their own axes. Their shape stays
    still, and only their surfaces move, so their `sweep` is 0.
  - `sweep` counts the arms' tips (22 in/s at 1 rev/s) and the fin's
    (14 in/s). Neither adds pieces.
- **The step's mechanisms.** `Flight.step` takes a validated value,
  `Flight.Mechanisms`, in place of stage 4's end frame and wheel speed: the
  frame the step ends at, the rotor's and the fin's angles at the end, the
  roller's speed, and the flywheel's `AngularSpeed`. It refuses a value that
  is not finite, or a rotor that turns more than a quarter turn in a step,
  naming the rule. `SimRobot` steps `Flight` every substep (at most 5 ms), so
  a quarter turn is far above any real step. Nugget, with no launcher and no
  internals, passes none, as in stage 4. (Stage 2's test part is already
  called `Carried`, so this value is not.)
- **What comes back.** `Flight.step`'s `Stepped` gains, beside the landings,
  each moving part's torque about its axis over the step (the sum over its
  contacts of force times lever), and the work the flywheel's surface did on
  the balls.
- **The launcher with the internals.** Built with Reginald's internals, stage
  4's launcher changes below −64.8° (*Flywheel and hood*): the fixed piece
  takes the place of the hood extension, and there is no stop and no middle
  lead-in ring. The hood's middle ring names its −64.8° end a join, with the
  fixed piece. Built without them, as `SimRobot` still builds it until
  batch 4, it is stage 4's launcher, `Flight.fed` and all.

Tests first:

1. Engine tests, with the internals and the robot's frame still, and the
   field's pollen:
   - A pollen in a pocket turning at 1 rev/s rides round between r 52.76 and
     57.69 outside the front window; in it, it rolls out no further than the
     V and is back inside 57.69 after the guide. It never passes through the
     floor, the lid, the fence or an arm.
   - **The fling limit comes out of the physics.** With the ramp down, at
     1.05 rev/s a pollen passes the back station seated, within 0.5 mm of
     r 52.76. At 1.25 rev/s it passes there against the fence or out in the
     window, beyond r 57.19.
   - **A jam stalls the rotor.** A pollen held against a window's edge by an
     arm stops the rotor within one step, squashed by less than one step's
     travel of the arm. With the pollen taken away the rotor turns again.
   - **The fin lifts a passing pollen.** With the fin up and no flywheel, a
     pollen pushed across it by an arm at the pocket's speed comes within
     75.03 mm of the flywheel's axle, close enough to reach the wheel, and
     never touches the lid. With the fin down it dips at most 1.59 mm, and
     touches only the floor, the fence, the lid and the arm.
   - **A stopped wheel holds a lifted pollen.** With the wheel stopped and the
     fin up, a pollen lifted into the squeeze comes to rest on the fin by stage
     4's rule and stays put exactly. Spinning the wheel up wakes it and throws
     it.
   - **The torques.** The rotor's torque for a pollen an arm pushes is the
     arm's force times its lever, within 1 %. With the motor's torque at zero,
     the work the flywheel's surface does on a pollen is the energy the
     flywheel loses, to 1e-9 J.
2. `Mechanisms` refuses a value that is not finite and a rotor turning more
   than a quarter turn in a step, naming each rule.
3. The Launch Sim's fingerprints, every Launch Sim test and stage 4's launcher
   tests pass unchanged: they build the launcher without the internals.
4. The launcher with the internals: its parts below −64.8° are the fixed
   piece and the four side rings of the lead-in, and a pollen carried up the
   fin by the wheel crosses −73° and −64.8° with one touch on its bottom at a
   time.
5. Cost: `FlightTest` pins the exact work of four pollen riding one turn at
   1 rev/s.

If the fin test is red, first check the lid: a pollen crossing the fin must
clear the widened notch, as batch 1 measures. If it is red because the
pushed pollen stalls against the round top at the built μ, confirm that this
is why. The next shape to try is a fin whose leading side is a wedge rising
at 30° in the direction the pockets move. The fin's row in *How it is
simulated* records which shape stands.

Then implement, and update `doc/CONTEXT.md`:
- new entries: **Rotor**, **Fin**, **Roller**
- **Flight** (parts that move, the torque they take, `Mechanisms`), **the
  launcher's physics** (with the internals)

## Batch 3c · asleep in the robot

This batch is in the core, and is the riskiest change to the engine, so it
lands by itself.

- **Asleep in the robot.** Stage 4 lets a ball rest against compliant
  supports only while the frame is still. Here a ball whose contacts are all
  with parts of the robot that do not move within it, or with balls asleep in
  it, sleeps in the robot's frame once its motion relative to the frame stays
  under the rest speed for the rest time.
  - `Bodies` keeps where it rests in the frame. Its place and velocity are the
    frame's there, and it is not integrated. `Flight.carriedTo` moves it with
    the frame, as it moves balls in the launcher.
  - It wakes when a part it touches moves within the robot, when something new
    touches it, or when the frame's acceleration at it passes 2 in/s²: the
    rate that would carry it past the rest speed (0.5 in/s) within the rest
    time (0.25 s).
  - The frame's own motion at a constant velocity does not wake it, where
    stage 2 wakes a ball on a moving support. A ball asleep on the field near
    the moving robot still wakes as stage 2 says.
  - It does not stop `Bodies.idleThrough`: the parts carried with it that do
    not move within the robot are not moving faces for it, so a cruising
    robot with four pollen asleep in still pockets still jumps.

Tests first:

1. Asleep in the robot:
   - A pollen seated in a still pocket of a robot driving at a constant
     velocity sleeps, and its path matches an awake run to 0.01 in.
   - When the robot brakes it wakes that step. When the rotor starts it wakes.
   - A robot driving for 2 s with four pollen asleep in still pockets does no
     engine work beyond the bounds checks, and `idleThrough` jumps.
   - `Flight.carriedTo` moves the four with the robot.
2. The Launch Sim's fingerprints, every Launch Sim test and stage 4's launcher
   tests pass unchanged.

Then implement, and update `doc/CONTEXT.md`:
- new entry: **Asleep in the robot**
- **Asleep and awake** (in the robot), **Idle and allowance** (a ball asleep
  in the robot does not stop it)

## Batch 4 · the feeder and the ramp replace the gates

This batch changes the robot code as well as the simulator. It is the
robot-code half of `reginald.md`'s batch 7, with the physics doing what its
simulator half did. Before it starts, the question in *What it costs* goes to
the user.

- **Hardware.** `topGate` and `bottomGate` give way to **`feeder`**, a
  `CRServo`, and **`ramp`**, a `Servo`. The Control Hub's configuration must
  name both before this lands on the robot. Hardware is whole or not at all,
  so a hub without them will not start an op mode. Changing the hub's
  configuration is the team's job.
- **`Launcher`** owns the feeder and the ramp, as it owned the gates:
  - It turns the feeder whenever the intake or the flywheel is running.
  - A launch waits for the flywheel and raises the ramp. It lowers the ramp
    as soon as the flywheel's measured speed falls more than 50 ticks/s below
    its target, because the wheel took a pollen, or after `RAMP_UP_SECONDS`
    (1.0 s, a turn of the feeder) if none came.
  - The drivers tune `RAMP_UP_SECONDS` with gamepad 2's bumpers, which tuned
    the bottom gate's wait.
- **Fakes.** `fakes` gains `FakeCRServo`; `SimDevices` gains the feeder and
  the ramp. `FakeDcMotorEx` keeps the velocity PIDF it is given.
- **`SimRobot`:**
  - It builds Reginald's internals into `Flight`, carried with his pose. The
    preload is four pollen, one in each pocket, with an arm on the front
    centreline so no pocket faces the front window, asleep in the robot.
  - Each substep it reads the feeder's power, the ramp's position, the
    intake's power and the flywheel's target. It steps the drives, hands
    `Flight` the step's `Mechanisms`, and takes back the torques and the
    flywheel's work.
  - The ramp's angle comes from `Launcher`'s two ramp positions, read from
    `Launcher`.
  - The launcher's measured velocity is the flywheel's, rounded as the hub
    reports it. It no longer equals the command.
  - The hopper, the chamber and the gates go.
  - `pieces()` still gives a held ball as `null`. A ball the wheel has taken
    is flying, and shows, as in stage 4. Batch 6 shows the rest.
  - **For this batch only**, a pollen the mouth takes by today's rule is set at
    rest in the entry V, at robot (4.626, 0, 2.191) in, if nothing lies within
    2r of that spot. Otherwise it is pushed. Batch 5 replaces this with the
    roller and the scoop.
- **Stage 4's stand-ins go.** `SimRobot` now builds the launcher with the
  internals (batch 3b). `Flight.fed` and its entry check, the hood extension,
  the stop and the lead-in's middle ring then have no user, and go.
- **Calibration.** Stage 4's `FLYWHEEL_TICKS_PER_TURN` is calibrated again by
  stage 4's rule in T, now that the flywheel slows and the fin feeds it: steps
  of at most 0.11 % of T, a common band at least 1.77 % of T wide, reported in
  ticks and in in/s. It uses stage 3's acceptance set. For the four-shot test
  it keeps whatever stage 4 settled with the user. If the band is narrower
  than stage 4's, stop and report it with the numbers.

Tests first:

1. `LauncherTest`:
   - A launch waits for the flywheel, raises the ramp, and lowers it as soon
     as the flywheel's measured speed falls more than 50 ticks/s below its
     target.
   - A change of target by one power step during a launch does not lower it.
   - With no shot it lowers the ramp after `RAMP_UP_SECONDS`.
   - The feeder turns while the flywheel runs and while the intake runs, and
     stops when neither does.
   - The bumpers tune `RAMP_UP_SECONDS`.
2. `HardwareTest` and `SimDevicesTest`: the hardware has a feeder and a ramp,
   and no gates. `FakeDcMotorExTest`: it gives back the coefficients it was
   given.
3. `SimRobotTest`:
   - With the ramp down, a full feeder turning for two turns throws nothing
     and still holds four.
   - Twenty runs from the preload, with the launch started at rotor angles
     4.5° apart over a quarter turn; each throws exactly one ball. Each leaves
     the hood at 70 ± 3°, with backspin antiparallel to the wheel's spin and
     0.9 ≤ S ≤ 1.2, as stage 4 asks.
   - After a launch the launcher's measured velocity drops and
     `flywheelReady` goes false, and both come back.
   - Stage 4's launcher tests that fed a ball are restated for the fin:
     - with the flywheel stopped and the ramp up, the pollen stays on the fin,
       inside the robot and off the floor, and is thrown once the flywheel
       spins up;
     - with the ramp held up and four pollen, they leave one at a time and
       never overlap (closest centres at least 2r − 0.04 in);
     - the robot's velocity carries into the throw;
     - a reversed wheel throws nothing.
4. `LauncherSimTest`:
   - Four preloaded and four launches throw four balls, one each, at their
     real cadence.
   - The close shot scores, for Blue and for Red.
5. `IntakeSimTest`: four pollen go in while the feeder turns, each into its
   own pocket.
6. The calibration's band test: the chosen ticks per turn, moved by 0.885 %
   either way (about 2 in/s), scores and stays for Blue and for Red.
7. Cost:
   - `SimRunnerTest`'s 2 s run still finishes in under 1 s, unchanged.
     Record the units per simulated second with four pollen riding.
   - `FlightTest` pins the exact work of one ramp-fed shot.
8. Golden traces:
   - Both Flight goldens stay byte-identical: `FlightTraceTest` has no robot.
   - Read the diffs of the others (the gates give way to the feeder and the
     ramp), then regenerate.

If a launch at some rotor angle still throws none or two, confirm why from
the map of angles and take the next default: the feeder turns at half power
while the ramp is up, which doubles the time between pockets. Record the map
in the telemetry store and report it with finding 8. The test stays as
written: one ball per launch is the behaviour we want.

Then implement, and update `doc/CONTEXT.md`:
- new entries: **Feeder**, **Ramp**
- **Intake** (into the feeder's pockets, not a hopper), **Hardware**,
  **Simulated robot** (preload, holds, gates → feeder and ramp, the flywheel
  slows), **the launcher's physics** (fed by the fin; no stand-ins below the
  hood), **Simulation constants**

## Batch 5 · the roller and the scoop take pollen off the floor

- **The intake box** replaces today's rule at the front (*Handoffs*). Every
  ball whose centre enters it goes to the engine: pollen or nectar, intake on
  or off.
- **Into the engine:** through stage 3's `Flight.with`, as a `Motion`: the
  ball's position on the floor, its velocity, the spin of rolling without
  slipping, and dyn4j's spin about z. It starts awake.
- **Back to dyn4j:** stage 3's landing rule, with the ball's centre outside
  the box and no compliant touch.
- A dyn4j ball that touches a ball the engine holds in the box is handed over
  too.
- The roller and the scoop are live. Batch 4's interim rule for the V goes,
  and so does today's capacity rule.

Tests first:

1. `SimRobotTest`, the handoff:
   - A pollen rolling at the mouth is handed over before dyn4j touches it, at
     the same position and velocity, rolling.
   - A pollen outside the mouth, or straddling its edge, is pushed by dyn4j as
     today.
   - A ball the engine sends back out of the box, onto the floor, lands in
     dyn4j.
   - A ball touching the scoop does not land, even with its centre outside the
     box.
   - A dyn4j ball that rolls into a ball the engine holds in the box is handed
     over, and the two meet.
2. `IntakeSimTest`:
   - Driven onto a pollen at 1.3 times the lip's threshold, and at least
     12 in/s, with the intake on, Reginald takes it over the lip, under the
     roller, over the crest, into the V and into a pocket. It arrives
     spinning. The threshold is computed from the committed geometry,
     √(2gΔz)/cos β, not written as a number.
   - With the intake off, a pollen that gets over the lip stops under the
     still roller and goes no further.
   - Four pollen, as in batch 4.
3. `SimRobotTest`:
   - A ball wider than the feeder's gap, a nectar, driven into the mouth never
     enters a pocket.
   - A pollen pinned between the mouth and a wall never passes through either.
   - The pushing tests outside the mouth pass unchanged.
4. Cost: pin the work of one pollen taken at 30 in/s.
5. Golden traces: read the diffs, then regenerate.

Then implement, and update `doc/CONTEXT.md`:
- new entry: **Intake box**
- **Intake**, **Simulated robot** (what the front does; a nectar is no longer
  turned away by a rule), **Flight** (a ball touching the robot never lands)

## Batch 6 · the ticks and the replay show the internals

- `pieces` gives every ball's position, inside the robot too. No ball is
  `null` for being held.
- `held` counts the held balls (*Handoffs*).
- For a ball in the engine that turned since the last tick, the tick carries
  its orientation as a quaternion: four numbers, to 3 decimals. Balls on the
  floor carry none, because dyn4j has no turn about a level axis.
- A tick carries the parts: the rotor's angle, the ramp's angle, the roller's
  and the flywheel's angles, and the flywheel's speed in ticks a second.
- A ball that went through the launcher since the last tick carries its path
  every 4 ms, the Launch Sim's frame interval. A 20–33 ms loop would miss the
  throw, and the replay draws it from the path.
- Ticks only gain fields. A page given a tick without them draws as today, so
  a run from an older child still replays.
- **Field scene:** the parts the CAD lacks (the rotor, the fence, the fin) and
  the hood as stage 4 placed it are drawn from `robot.json`'s internals, in the
  robot group, turned by the tick's angles. Held balls are drawn where they
  are. Each ball has a mark, so its turning shows.
- **Flat drawing:** held balls are drawn inside the robot's outline.
- Nugget has no internals and draws none.

Tests first:

1. `SimRunStreamTest`:
   - positions for held balls; turns only for balls that turned; the parts; a
     launch's 4 ms path;
   - every double to 3 decimals.
2. `SimReplayPageTest`: every placeholder is filled, and the page carries the
   internals.
3. `tools/browser/replay.mjs`:
   - A held ball is drawn inside the robot, at its position.
   - The rotor is drawn at the tick's angle.
   - A ball's mark turns by the tick's quaternion.
   - A throw is drawn along its path.
   - A tick with none of the new fields replays.
4. `check.mjs`:
   - More than 20 balls are still drawn.
   - A Nugget run draws no internals.
5. `cost.mjs`: the budget includes the internals, at no more draws than they
   have materials. `scene-budget.json` is regenerated.
6. The stream of a 150 s game is measured and recorded; it grows by no more
   than a fifth.

Then implement, and update `doc/CONTEXT.md`: **Tick**, **Replay**, **Field
scene**, **Flat drawing**, **Simulated robot** (holds).

## Later

- **The chassis as a collider for balls in flight.** The footprint's prism,
  open at the mouth and over the launcher, carried like the internals. A ball
  landing on Reginald, or bouncing off a cell back onto him, would meet him. A
  ball that falls through him would stay in the engine, instead of landing
  inside his footprint and being pushed out by dyn4j. The launched ball starts
  inside the hull, so the start check must skip the openings.
- **Two-way coupling.** The robot would feel the balls in the engine: each
  step's impulse on the carried parts applied to the dyn4j body over the next
  substep. Nothing needs it yet: a pinned pollen climbs the lip.
- **Reading the internals from Onshape.** Once the CAD draws the rotor, the
  fence and the fin, a reduction reads them, and refuses an `assumed` entry
  the CAD supplies, as `reginald.md`'s batch 2 does. It must find the scoop by
  its geometry, because the scoop was renamed from *Part 40* to *Part 41*.
- **The CAD's roller and flywheel drawn turning.** `robot.glb` would keep them
  as nodes of their own.
- **Battery sag** from the flywheel and the intake, beside the drive's.
- **The bands as six strips**, if calibration shows that the ripple between
  them (36.5–45.3 mm) matters.
- **A spread of pollen sizes.** The manual says pollen "may vary in size".
- **Past the snap-through.** A softer slope in the contact law beyond about
  8 mm of travel, if a jam or the hood ever squeezes that far.
