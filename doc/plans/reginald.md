# Reginald: the simulated robot takes the shape of the CAD

Today the simulator has no robot shape. It has one number, 18 inches, and
uses it as each robot's length, width and height: Reginald and Nugget are
the same 18-inch body. The cube is built again by `Chassis.SIZE_IN`, by
`Placement.corners`, by the `SimRobot` rectangle (`SimPlacement.ROBOT_SIZE_IN`),
by the replay page (`robotFaces`, `insideTheWalls`, `under`), by
`fieldscene.js` `#robotOf` (which falls back to 18), by `model.mjs`, by the
browser harness (`bench.mjs`, `replay.mjs`), and by `step_to_field.py`, which
cuts the field's obstacles at its own `ROBOT_HEIGHT_IN = 18`. The launch point
is three more constants in `Launch`: 6 in ahead of the turntable's axis,
14 in up, at 70°.

Reginald is the robot as its CAD draws it: the part studio *Reginald* in the
Onshape document *FTC 2026*
(`b57b76498365b6a963155c10` / `w/d59ab971c5581582ffbe1052` /
`e/8e46242050eb2931586f309b`), with sizes taken from *Variable Studio 1*.
The document is not finished. This plan says what the CAD gives us, what the
team decided where the CAD is unfinished or wrong, what we still infer, and
the batches that turn the cube into Reginald. Each batch lists its tests
first. Nugget has no CAD and keeps the 18-inch cube throughout.

---

## What the CAD says

In the CAD frame the origin is the middle of the chassis front, at floor
level. +Y points back, +X points to the robot's left, +Z points up, and units
are mm. The **robot frame** used below follows Road Runner: inches, +x
forward, +y left, z up from the floor. Its origin is the assumed **centre**,
204 mm behind the front, which is the midpoint of the side rails. The
conversion is `x = (204 − Y)/25.4`, `y = X/25.4`, `z = Z/25.4`.

| What | CAD evidence | Robot frame |
|---|---|---|
| Side rails | two REV 408 mm C-channels; outer faces at X ±143.5; bottom at `Clearance` = 5/8 in | x −8.03…+8.03, y ±5.65 |
| Cross rail | one 248 mm C-channel at `MidCtoFront` = 10.25 in | x −1.6…−2.2 |
| Floor | three 280 mm flat plates, starting at `PlateToFront` and spaced `PlateSpacing` apart | z 0.51–0.63 |
| Intake roller | two 90 mm grip wheels joined by 12 rubber-band strips. Axle `IntakeToFront` = 1 in behind the front and `IntakeHeight` = 4.5 in up. Overall width `IntakeWidth` = 8.625 in | axle at x 7.03, z 4.5; front edge at x 8.80; y ±4.31 |
| Funnel | *Part 40*, a scoop on the floor that narrows toward the middle. Its mouth is 245 mm wide | mouth at x 7.89, y ±4.83, z 0.18–1.97 |
| Feeder | *FeederBottom* and *FeederTop*: two coaxial discs of diameter `2·PollenDiameter + 25 mm` = 6.58 in, set one pollen + 5 mm apart (`FeederHeight` ≈ 75 mm). They sit on a servo hub and a bearing block, and are tilted about 17° so they rise toward the back. The top disc has a notch at the back, under the flywheel. `FeederRadius` = 55 mm | axis near x +1.3 |
| Flywheel | *FlywheelInner* and *FlywheelOuter*: `FlywheelDiameter` 100 mm, `FlywheelWidth` 50 mm, on the centreline. Its axle runs across the robot, 217 mm behind the front and 180 mm up | axle at x −0.51, z 7.09 |
| Hood | *Hood*: a grooved arc behind the flywheel, from 64.8° below horizontal to 41.9° above. The ball's centre travels 75.3 mm from the axle, 10 mm closer than the free pollen's | top at z 10.26; back at x −6.41 |
| Launcher mount | one REV 420 mm 15 mm extrusion and one bearing block, on the left side only | y +1.65…+2.24, z 7.3–7.9 |
| Reference pieces | four pollen, each made of two *Pollen Half* parts: three on the floor at the front and one on the plates | not robot |

**How a ball moves through it**, from the CAD and the team. The roller pulls a
pollen up the funnel onto the plates and into a pocket at the feeder's low
front. The feeder turns and carries its pockets round. Under the feeder and
the flywheel, a servo lifts a small ramp. With the ramp up, a pollen passing
under the flywheel is lifted into it, pinched against the hood, carried up
the back of the flywheel and thrown forward over the top. With the ramp down,
the pollen rides on round. As drawn, the hood throws at 48.1° above
horizontal, from 9.07 in up and 2.72 in behind the centre. The robot will
throw at 70° (see *What the team decided*).

The CAD **does not have**:
- a drivetrain or any motor, servo, hub, battery or camera
- a rear cross-member
- the right-hand launcher mount
- the feeder's pockets
- the ramp and its servo

Two things about the export that the pipeline has to handle:
- The part-studio glTF export carries every part, hidden or not. (38 of the
  42 were hidden on 13 October. Visibility in the studio is someone's view of
  it, not the robot.)
- It carries each *90mm Grip Wheel* composite **beside** its *HUB* and *TREAD*
  constituents. That is 227k triangles drawn twice. The full export is 639k
  triangles. Snapping to a twentieth of an inch with the duplicates and the
  reference pollen dropped leaves about 89k.

## What the team decided

Answered on 13 October.

| Question | Decision | What follows |
|---|---|---|
| Launch angle | **70°, as today.** The 48° the hood draws is not the robot's angle. | The ball leaves where a 70° throw leaves Reginald's own flywheel: its centre 75.3 mm from the axle, 20° above horizontal behind it, which is **(−3.30, 0, 8.10) in**. A 2-D check against the upturned cell's mouth scores from `Nav.LAUNCH_DISTANCE_INCHES` = 40 at 205–223 in/s, so the launch distance stays. |
| Turntable | **None.** | The robot aims by turning. `Turntable`, its motor, its simulation constant and the camera "on the turntable" all go. |
| Feeder | **Four pockets. It replaces both gates.** A second servo, under the feeder and the flywheel, lifts a ramp that puts a passing pollen into the flywheel. | The ramp is the trigger. The gates let one ball through at a time; now a ball is thrown when the ramp is up while one pocket passes. |
| Drive | **75 mm mecanum, outside the rails.** | The robot is ≈16.8 × 14.8 in. |
| Back | **The rails' end.** The CAD's 420 mm extrusion is longer than the real robot's. | The reduction and the drawing end the robot at the rails, and say where the CAD differs. |

## What we infer

| Inference | Why | Confidence |
|---|---|---|
| **The wheels' axles are 38.4 mm up, on the rails' outside faces** | A rail whose bottom is at `Clearance` has its centreline at 15.9 + 22.5 = 38.4 mm, which is a 75 mm wheel's axle on the floor. Wheels there put the track near Road Runner's tuned 13.55 in (`trackWidthTicks · inPerTick`). | high |
| **≈16.8 in long and ≈14.8 in wide**: the rails, the roller ahead of them, and the wheels' outer faces at about y ±7.4 | Follows from the line above | high |
| **The centre is the midpoint of the rails**, on the centreline | Nothing in the CAD says where the drivetrain's centre is | low; only the dead-wheel offsets would show the difference |
| **Height 10.26 in** (the top of the hood) | Measured from the CAD | high, for what is drawn |
| **The feeder's servo turns continuously** | Pollen ride round the feeder and pass the flywheel again and again, and four pockets need more than a turn. A positional servo stops short of one turn. | medium-high |
| **Pollen go in at the front and come out at the back** | The funnel meets the feeder's low front. The flywheel's axle is 1.8 in behind the feeder's axis, over a pocket 55 mm behind it. | high |
| **The four pockets sit at `FeederRadius` = 55 mm** | Four pollen there fit with 6.7 mm between them | medium |
| **The ramp's servo is positional**: down or up | It lifts and lowers | high |
| **Mass 15 kg, as today** | The CAD's mass is 0.92 kg, because only the REV aluminium has a material | — |
| **Camera as today**, (0.75, 4, 16) in at −68°, now fixed to the robot | Not in the CAD | — |

## Asks of the CAD

No batch waits on these. Each makes the next update cheaper:
- Move the hood's upper end to 20° above horizontal, so that the CAD throws
  at 70°. Batch 2's reduction then drops its override.
- Shorten the extrusion to the real robot's.
- Draw the pockets, the ramp and its servo, and the drivetrain.
- Name *Part 40* (the funnel) and *Part 8–19* (the intake bands).
- Put a mate connector at the drivetrain's centre and one at the launch exit,
  so that the reduction reads them rather than inferring them.

---

## Batch 1 · each robot has a shape, not a size

This is a refactor, and nothing moves. Each robot's shape becomes a validated
value in the core. It is read from a committed file beside `field.json`:
`robot.json` for Reginald and `robot-nugget.json` for Nugget, named by
`TeamRobot.ownName`. Both start as today's 18-in cube, so every existing test
and every golden trace passes unchanged. That unchanged result proves the
seam is right before any number changes.

In the core:
- **`Footprint`**: a convex, counter-clockwise ring in the robot frame that
  strictly contains the origin. Any other ring is refused, naming the rule.
- **`Chassis`** becomes a robot's shape: footprint, **height**, and
  **intake**. The intake is either a **mouth** (half-width and reach on the
  front edge) or none, since Nugget has no intake. A mouth wider than the
  footprint's front is refused.
- `Chassis.againstTheFront` asks the mouth instead of `SIZE_IN / 2`.
- `Placement` takes the `Chassis`:
  - For the walls it clamps by the footprint's support along each wall's
    normal at the heading. That support is `size/2·(|cos|+|sin|)` only for
    the square.
  - For the obstacles it pushes out the footprint's own ring.

In the shell:
- `SimChassis` reads the run's robot's file into the core types, as `SimField`
  reads `field.json`. A refused shape fails the load. There is no fallback
  cube.
- `SimRobot` builds the dyn4j body from the footprint polygon and uses
  `Reaches(0, height)`. It sets the mass at the pose origin with the
  footprint's inertia about it, so an asymmetric footprint does not move the
  point the drive pushes about.
- `step_to_field.py` reads every robot's height from the robot files and
  cuts each obstacle's footprint at each distinct height, keyed by it.
  `SimRobot` takes the footprints for its own height. Both robots are 18 in
  now, so `field.json` gains one key and nothing else changes.
- The pages:
  - `__ROBOT_IN__` becomes `__ROBOT__`, the run's robot's shape as JSON.
  - `robotFaces`, `insideTheWalls` and `under` draw and hit-test the
    footprint.
  - `fieldscene.js` and `model.mjs` extrude the footprint to its height, and
    lose the fallback 18.
- `bench.mjs` and `replay.mjs` read the robot files.

The launch does not move into `robot.json` in this batch. That is batch 6.

Tests first:

1. `FootprintTest`: a ring that is not convex, that is clockwise, or that
   leaves the origin outside or on its edge is refused, and each refusal
   names its rule. A ring that is refused cannot be used to build a `Chassis`.
2. `ChassisTest`:
   - A mouth wider than the front edge is refused.
   - A pollen touching the front just outside a narrower mouth is not taken;
     one just inside is.
   - A chassis with no intake takes nothing.
   - The cube's mouth gives today's answer for every case in today's test.
3. `PlacementTest`: take a footprint 16.8 long, 14.8 wide, reaching 0.77 in
   further forward than back. Beyond each wall at 0°, 90°, 180°, 270° and
   30°, it lands with its own furthest point on the wall. Pushed out of an
   obstacle, it ends clear of it. The square gives exactly today's answers.
4. `SimChassisTest`:
   - Each robot's file loads into the core types.
   - A file whose footprint is refused fails the load, naming the rule.
   - A robot with no file fails, naming the file.
5. `SimRobotTest`:
   - An asymmetric footprint turning in place keeps its pose's position,
     because the mass is at the origin.
   - Driven into a wall, its front-most point stops at the wall.
   - Nugget runs on its own file.
6. `SimFieldTest`: `field.json` has obstacle footprints for every robot's
   height. A missing height fails, naming the height and the command that
   regenerates the file.
7. `test_step_to_field.py`:
   - The heights come from the robot files.
   - An obstacle that widens above a robot's height is cut at that height.
8. `SimReplayPageTest`: every placeholder is filled, and the page carries the
   run's robot's shape.
9. `tools/browser/replay.mjs`:
   - The field scene's robot spans the footprint and height to within
     0.01 in.
   - The placement page's drag hit-tests the footprint, not a square.
10. Every existing test and golden trace passes unchanged.

Then implement, and update `doc/CONTEXT.md`:
- new entries: **Chassis** (a robot's shape), **Footprint**, **Intake mouth**,
  **Robot frame**
- entries that say "18-inch": **Simulated robot**, **Simulated Nugget**,
  **Placement**, **Flat drawing**, **Field**

## Batch 2 · Reginald comes from Onshape

This batch adds tooling and moves nothing. It fetches Reginald, builds what
he looks like, and computes what he measures. The physics stays on the
committed cube until batch 3 changes it deliberately. The field already works
this way: the model a page draws is fetched, and the model the physics runs
on is committed and pinned, because golden traces are taken against it.

- `Onshape` gains Reginald's document, workspace, part studio and variable
  studio.
- **`RobotAssets.download`** fetches the part studio's glTF export (every part,
  whether it is hidden or not) and the variable studio's variables into the
  state directory. Nothing is written until both have arrived and been
  checked.
- **`RobotAssets.build`** turns the export into `robot.glb`, at the resolution
  the page draws, in the robot frame. It:
  - drops game pieces by name (a part whose name says pollen or nectar);
  - draws each composite part once: the composite or its constituents, never
    both;
  - ends the extrusion at the rails' end, by moving its vertices behind the
    rails' end onto it. An extrusion has the same section all along, so its
    outline stays exact;
  - names `Keep` and `Shading` at every call, as `FieldGlb` does, sharing
    that code by taking the frame change as a parameter.
- The **reduction** works out a candidate `robot.json` from the same export
  and variables. The geometry is in the core, as pure functions over points;
  the shell only reads the files. It computes:
  - the footprint: the plan-view hull of every kept part, with the
    overrides and assumptions below
  - the height: the top of every kept part
  - the intake mouth: from `IntakeWidth`, `IntakeToFront` and the grip
    wheels' front edge
  - the launch exit, which batch 6 uses: the flywheel's axle, the ball's path
    radius from the hood's groove, and the launch angle
- What the CAD **does not have** goes under **`assumed`**, each entry named
  for what it stands in for: `centre`, `drivetrain` and `mass`. The reduction
  **refuses an assumption once the CAD supplies the thing**, for example
  parts named for mecanum wheels.
- Where the team says the CAD **is wrong**, the entry goes under
  **`overridden`**. Each entry names what the CAD shows beside what the robot
  has:
  - `back`: the rails' end. The CAD's extrusion runs 40 mm past it.
  - `launchAngle`: 70°. The CAD's hood throws at 48°.

  The reduction **refuses an override that the CAD has come to agree with**.
  So neither block can outlive the drawing that settles it.
- The admin page's Download, Build and Refresh do the robot alongside the
  field. The page shows the candidate `robot.json` next to the committed one,
  with its overrides. It says per asset whether it is drawing the fetched
  model or the stand-in.
- Commit a **stand-in** `robot.glb` (low resolution, about 89k triangles)
  built from today's CAD. The tests draw it with no network.

Tests first:

1. `GltfTest`: a small committed part-studio export (a handful of parts,
   including a composite with its constituents and a hidden part) reads into
   named parts with their triangles in the studio frame.
2. `RobotGlbTest`:
   - A point on the CAD's front centreline lands at x = +centre.
   - +X lands at +y.
   - Game pieces are gone.
   - A composite is drawn once.
   - The extrusion ends at the rails' end.
   - A build that names no resolution does not compile.
3. `RobotReductionTest` (core):
   - Hull and height of known points.
   - A missing named part or variable stops the build, naming it.
   - An `assumed` drivetrain beside parts named for mecanum wheels is refused.
   - A `launchAngle` override of 70° beside a hood that ends at 20° is
     refused, because the CAD now agrees.
   - The exit for an axle at (−0.51, 7.09), a path radius of 75.3 mm and 70°
     is (−3.30, 8.10).
4. `RobotAssetsTest`:
   - A download writes nothing until both the export and the variables have
     arrived.
   - A build with nothing downloaded says so and does not fetch.
   - The stand-in is served per asset when nothing has been fetched.
5. `CodingServerTest`: the admin routes do the robot, and no test reaches
   the network.

Then implement, and update `doc/CONTEXT.md`:
- new entries: **Reginald**, **Robot model**, **Reduction**, **Assumed**,
  **Overridden**
- **Refresh assets**, **Download** and **build**, and **Stand-in** gain the
  robot

## Batch 3 · Reginald drives

This batch commits the reduction's `robot.json`. Reginald becomes about
16.8 × 14.8 in and 10.26 in tall, with an 8.63 in intake mouth at the roller.
The launch is still today's. Nugget's file stays the cube.

Tests first. Each reads its numbers from the committed robot files, not from
literals.

1. `SimRobotTest`:
   - Driving into each wall stops the roller, the sides and the back at the
     wall.
   - A pollen that arrives at the front inside the mouth is taken. One that
     arrives at the front outside the mouth is pushed.
   - The height is Reginald's, so `Reaches` filters by it.
2. `SimPlacementTest`: placed beyond a wall at 30°, Reginald lands touching
   it with his own furthest point.
3. `field.json` is regenerated with footprints cut at 10.26 in and at 18 in,
   and `SimFieldTest` and `test_step_to_field.py` pass. Reginald no longer
   meets an obstacle that clears his height. Nugget meets exactly what it
   met before.
4. Golden traces:
   - Nugget's pass unchanged.
   - Reginald's: run them, read the diffs, then regenerate. Expect the drive
     traces to differ where the robot touches something or where the inertia
     changed, and nowhere before that.
5. `tools/browser/scene-budget.json` is regenerated for the new footprint
   prism.

Then implement, and update `doc/CONTEXT.md`: **Simulated robot** gives
Reginald's size, and **Field** gives the heights its obstacles are cut at.

## Batch 4 · Reginald is drawn as Reginald

- **Field scene:**
  - `robot.glb` loads at the page's resolution together with the field.
  - It goes under the robot group, which is placed at the pose and turned to
    the heading. It is batched by material within that group, since the group
    moves as one.
  - The heading arrow stays on top, 0.1 in above Reginald's height.
  - The driver's view watches the middle of Reginald's height, not 9 in up.
- **Flat drawing** (the colliders): the footprint prism, with its front edge
  in the front colour and the intake mouth marked on it.
- The webcam view still hides the robot.
- A page that cannot fetch `robot.glb` draws the prism and says on the page
  that it fell back, as the field does.
- Nugget has no model, and is drawn as its prism with no fall-back notice,
  because nothing failed.

Tests first:

1. `replay.mjs`:
   - In the field scene, the model's bounds in the robot frame match
     `robot.json`'s footprint and height to within the snapping grid. This
     ties the picture to the physics, so a part the reduction missed fails
     here.
   - At heading 0 and at heading 90° the model's front points along +x and
     +y respectively.
   - A Nugget run draws Nugget's prism and no notice.
2. `replay.mjs`: the flat drawing puts the front colour at the roller's edge.
3. `check.mjs`: when `robot.glb` returns 404, the page draws the prism and
   says it fell back.
4. `cost.mjs`:
   - The budget includes the robot.
   - The robot costs no more draws than it has materials.
5. `bench.mjs` and `dashboard.mjs` pass unchanged.

Then implement, and update `doc/CONTEXT.md`: **Field scene**,
**Flat drawing**, **Driver's view**, **What a frame costs**.

## Batch 5 · Reginald has no turntable

This batch changes the robot code as well as the simulator.

- `Turntable` goes, from the robot code and from the core, with its tests.
- The `turnTable` motor leaves `Hardware` and `SimDevices`.
- `Brain` stops aiming the turntable each loop. `Nav.launchPose` already
  faces the target, so the auto-shoot plans and the driver's held aim turn
  the robot.
- A sighting is no longer turned back by the turntable's offset, because the
  camera is fixed to the robot.
- `Driver` loses the controls that parked the turntable and set it following
  the goal (gamepad 1's cross and circle), and the nudges (gamepad 2's d-pad
  left and right). `OpMode`'s channel toggle loses `Turntable`.
- `Launch.from` loses its turntable angle. The throw leaves along the
  heading.
- `TURNTABLE_SPEED` leaves the simulation constants. A settings file that
  names it must still load: a settings file the server cannot read stops it
  from starting, and a retired constant is not unreadable. The server drops
  it, and the log and the admin page say that it was dropped.

Tests first:

1. `SimConstantsTest`:
   - A settings file naming `turntable_speed` loads, drops it and says so.
   - A name that was never a constant is still refused.
2. `LaunchTest`: the throw leaves along the heading, from 6 in ahead of the
   pose. That is still today's point; batch 6 moves it.
3. `BrainTest`:
   - A sighting becomes the robot's pose as the camera saw it, not turned.
   - The auto-shoot plans turn the robot to face the target.
4. `DriverTest`: the held aim turns the robot to face the target.
5. `HardwareTest` and `SimDevicesTest`: the hardware is whole without a
   `turnTable`.
6. `SimRobotTest`: the close shot still scores, because this batch moves no
   launch point.
7. Golden traces: read the diffs (the turntable's motor goes), then
   regenerate.

Then implement, and update `doc/CONTEXT.md`: remove **Turntable**; update
**Sighting**, **Launch pose**, **Simulation constants**, **Hardware** and
**Simulated robot**.

## Batch 6 · Reginald throws from his hood, at 70°

- `robot.json` gains the launch from batch 2's reduction: the exit at
  (−3.30, 8.10) and 70°.
- In the core, **`LaunchExit`** is part of the `Chassis`. An angle that is
  not strictly between 0° and 90°, or a point outside the footprint or above
  the robot's height, is refused, naming the rule.
- `Launch.from` takes the exit instead of `HEIGHT_IN`, `AHEAD_IN` and
  `ANGLE_RADIANS`, which go.
- The throw is recalibrated the way the realtime flight's move onto the
  Launch Sim's engine calibrated it: a scan in 0.25 in/s steps for the widest
  band of throws at which every scored shot the tests ask for (both
  alliances' close shots, the ball at rest against the back, the shot after
  a tip, one launch and four at their real cadence) goes in and stays, its
  middle built in.
  - From today's launch point, with drag and no spin, that band ran from 192
    to 206.75 in/s at 1050 ticks/s when balls first flew through air, so
    0.1899 in/s per tick/s is built in (0.189 before). The band's edges are
    history; what a test holds is that the built throw and 2.1 in/s either
    side of it score and stay for every one of those shots, the four launches
    at their cadence going into an empty cell and all coming to rest in it.
    The rebuilt launch keeps a test of that shape.
  - The 2-D check's 0.204 in/s per tick/s (band 0.195–0.212) was worked out
    in a vacuum for the hood's exit and no longer holds: the hood's exit has
    to be scanned again.
  - About 0.2 is 45 % of a 100 mm wheel's surface speed on a 28-tick motor
    at 1:1, at the top of what hooded wheels are reported to throw.
  - `LAUNCH_THROW`'s built value moves. An admin who set it keeps what they
    set, as far as its range now lets it go: one saved past that is used at
    the nearer end and shown on the admin page beside what was saved.

Tests first:

1. `LaunchTest`: the ball starts at the exit turned by the heading, at the
   exit's angle, with the robot's velocity added.
2. `ChassisTest`: an exit outside the footprint, above the robot, or at 0° or
   90° is refused, naming its rule.
3. `SimChassisTest`: `robot.json`'s launch loads.
4. `SimRobotTest.aCloseShotFromTheLaunchDistanceScoresInTheAlliancesUpturnedCell`:
   - At 0.1899 from the hood's exit, see whether it is red and why before
     scanning.
   - At the recalibrated throw it is green.
5. `ConstantsTest`: the throw's built value.
6. Flight golden traces: read the diffs, then regenerate.

Then implement, and update `doc/CONTEXT.md`: **Simulated robot** (the launch
point and the throw) and **Simulation constants**.

## Batch 7 · the feeder and the ramp

This batch changes the robot code as well as the simulator.

- **Hardware.** `topGate` and `bottomGate` give way to **`feeder`**, a
  `CRServo`, and **`ramp`**, a `Servo`. The Control Hub's configuration must
  name both before this lands on the robot. Hardware is whole or not at all,
  so a hub without them will not start an op mode. Changing the hub's
  configuration is the team's job.
- **In the core, `Feeder`:**
  - It holds four pockets, each empty or holding a ball, and the feeder's
    angle.
  - Pollen enter at the front station and are thrown at the back station.
  - Turning by an angle moves every pocket.
  - A pocket that passes the back station with the ramp up gives up its
    ball. With the ramp down, the pocket keeps it.
  - A pollen at the mouth goes into the pocket nearest the front station if
    that pocket is empty. If it is full, the pollen is pushed.
  - It is total for any step, so a step that passes two pockets throws both.
- **`FEEDER_SPEED`** joins the mechanism constants: turns a second at full
  power, a guess until measured (1).
- **`SimRobot`:**
  - The feeder replaces the hopper, the chamber and the gates. The preload
    fills all four pockets.
  - The ramp counts as up past halfway between `Launcher`'s two ramp
    positions. They are read from `Launcher`, as the turntable's ticks were,
    so there is no second copy.
  - `SimDevices` gains a `FakeCRServo`.
- **The robot code.** `Launcher` owns the feeder and the ramp, as it owned
  the gates.
  - It turns the feeder whenever the intake or the flywheel is running.
  - A launch waits for the flywheel, raises the ramp for the time one pocket
    takes to pass (`RAMP_UP_SECONDS`), and then lowers it.
  - The drivers tune that time with gamepad 2's bumpers, which tuned the
    bottom gate's wait.

Tests first:

1. `FeederTest` (core): each rule above, including a step that passes two
   pockets, and turning past a full turn.
2. `LauncherTest`:
   - A launch waits for the flywheel, raises the ramp for
     `RAMP_UP_SECONDS`, and lowers it.
   - The feeder turns while the flywheel runs and while the intake runs, and
     stops when neither does.
   - The bumpers tune `RAMP_UP_SECONDS`.
3. `SimRobotTest`:
   - With the ramp down, a full feeder turning for two turns throws nothing.
   - With the ramp up for one pocket's passing, one ball leaves.
4. `IntakeSimTest`: the intake takes four pollen into the four pockets while
   the feeder turns, and a fifth is pushed.
5. `LauncherSimTest`:
   - Four preloaded and four launches throw four balls, one each.
   - The close shot scores.
6. `HardwareTest` and `SimDevicesTest`: the hardware has a feeder and a ramp,
   and no gates.
7. `ConstantsTest` and `SimConstantsTest`: `FEEDER_SPEED`'s range and built
   value.
8. Golden traces: read the diffs (the gates give way to the feeder and the
   ramp), then regenerate.

Then implement, and update `doc/CONTEXT.md`:
- new entries: **Feeder** and **Ramp**
- **Simulated robot** (preload, holds, gates → feeder and ramp),
  **Hardware**, **Simulation constants**

## Later

- **Moving parts in the drawing.** Ticks would carry whether the intake is
  on, the flywheel's speed, the feeder's angle, which pockets hold a ball,
  and the ramp. The roller and flywheel would turn, and held balls would be
  drawn in the feeder, where today `pieces` gives them as `null` and nothing
  is drawn.
- **The dashboard overlay.** Road Runner's `Drawing` draws a 9 in circle for
  the robot. It would draw the footprint instead.
- **The camera's pose.** Once the CAD has a camera, `webcam.js` and
  `AprilTagWebcam` should read the same source. Today the pose is written
  out in both places, and nothing checks that the two agree.
- **The Launch Sim.** A shot could start from Reginald's exit, with his
  flywheel as wheel one and the hood as a wheel two that does not turn. It
  could also draw `robot.glb`. Today it draws no robot.
