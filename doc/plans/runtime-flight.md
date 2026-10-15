# Balls in flight on the Launch Sim's engine, and Reginald's launch simulated

The Launch Sim flies a ball through air with drag, Magnus lift and
spin-down, and bounces it off every collider of the field with Walton's
contacts. Until this plan, a run did neither. Its balls flew under gravity
alone in `Flight`, and its launch was a formula: `Launch.from` put the ball
14 in up and 6 in ahead of the turntable's axis, at 70°, at a speed
proportional to the flywheel's command, with no spin.

The team asked for three things:
- "The intention is for flying objects to use the advanced physics added in
  the Launch Sim."
- "The more flexible and nuanced two-wheel launcher can stay in the Launch
  Sim instead of changing how the robot simulates its mechanisms."
- "Spin should be part of the simulation. The release point should be based
  on the simulated physics."

So a ball off the floor in a run moves on the Launch Sim's engine, and the
launch is not a formula: the ball is fed into Reginald's flywheel and hood,
and where, how fast and with what spin it leaves comes out of the same
engine. `WheelPair` stays in the Launch Sim. The rest of Reginald's insides
(the roller, the scoop, the feeder, the ramp, the flywheel's inertia) are
`reginald-internals.md`'s.

Stages 1 to 3 are built, and so is stage 4: the arcs and the compliant
contact the launcher is made of, the launcher itself, tested on its own,
the launcher in a flight, which a flight may carry, step with each tick,
feed and take along in a teleport, and the launcher in a run, stage 4c's
runtime, which carries it in Reginald's flight from the start of every
run. With it in the run the two shots into an empty cell are red, so
stage 4c's runtime waits off `develop` until the team reduces the hood's
backspin (*Calibration*). Every stage keeps the
Launch Sim's results bit for bit: its fingerprint tests pin the exact work
and the raw bits of every field of every story, in the simulator's core and
on the field, and no stage changes them.

---

## What is built

**Stage 1 · one engine, two users.** `Bodies` is the Launch Sim's engine,
taken out of `Shot`: RK4 on a fixed grid of 0.1 ms steps, pieces of at most
0.1 in of travel, events located by bisection, Walton contacts with
sequential impulses, Nathan's air, islands that sleep and wake. A body comes
in as a `Name`, a `Motion` (a finite place, velocity and spin) and a
`Start` (awake, asleep or fixed). `ShotRun` builds the Launch Sim's story
from the engine's spans. The fingerprints were computed on the code before
the move and pass unchanged after it.

**Stage 2 · surroundings that move.** `Surroundings` holds the colliders
that stand still and the `Part`s that move. A part says what it reaches,
how fast it sweeps and whether it moves, and answers `near` with `Touch`es
that carry the support's velocity and spin. A moving part wakes what it
reaches, carries what rests on it and never passes through a ball. A part
wakes a body only through a touch that moves. `PartContract` is the test
helper every part is run through. An engine with nothing awake and nothing
a moving face could reach idles to a later step at once (`idleThrough`).

**Stage 3 · `Flight` flies on the engine.** A ball off the floor in a run
moves on `Bodies` among the field's colliders, without the hives, and the
hives are a part that moves while one tips.
- A ball lands back on the floor, in dyn4j, once it rolls there touching
  only what the floor's own world has.
- A flower's base plate is floor, so a ball resting there lands where the
  floor's world keeps the flower's nest. What stands in a flower's bore (its
  stack, and any ball whose centre is in the bore) stands in the flight too,
  as still balls on the plate (`Bores`), so a flown ball meets the stack and
  never passes through it.
- A ball is out once its centre is past a wall.
- A hive tips only once its upturned cell holds a full load pressing in it.
  The tipped hives take the hives' place in the surroundings under the same
  names (`Surroundings.replacing`), leaving every other part as it was.
- The engine's work is allowed fifty steps (5 ms) at a time on its own step
  grid, 15,000 units a window. Idling and a step with nothing awake are no
  work. So however a run's time is cut into ticks, a flight comes out the
  same to the bit, its work included.
- Parts are placed at `GridTime`s, whole steps and a part of a step, and a
  tip is timed in whole steps, so it lasts exactly its second wherever it
  begins.
- `Flywheel` clamps a command to the motor's top speed, 2,800 ticks a
  second, and reads one that is not a number as 0. `LAUNCH_THROW` was held
  to at most 0.29, so the fastest throw off the fastest robot (180 in/s) was
  within the 1,000 in/s a flight takes; stage 4 retired it. A launch the
  flight refuses stops the run, saying why.
- A constant saved past where its range now lets it go no longer stops the
  server: runs use the nearer end of its range, and the admin page shows
  what was saved (`Constants.Restored`) until the constants are saved again.
- The core's PMD gate refuses a `Checked` that nothing looks at, used as a
  statement, as what a lambda or an arrow arm does for its effect, or behind
  a method reference where nothing is returned. A tip or a ball the engine
  refuses stops the run, saying why.
- `LAUNCH_THROW` was calibrated again for air and contacts: the widest band
  at which every scored shot the tests ask for goes in and stays ran from
  192 to 206.75 in/s, and its middle, 0.1899 in/s per tick/s (199.4 in/s at
  1050 ticks/s), was built in, and tests held the built throw and 2.1 in/s
  either side of it for every shot in that set. Stage 4 retired the throw
  and the formula, and their band test with them: no band test holds the
  launch today.

**Stage 4a · arcs and compliant contact.** Nothing in a run uses them yet.
- `Arc` is an arc of a circle, whole or a part of a turn, nearest any point
  on its circle within its span and at its nearer end outside it; one whose
  radius, axis or span could not be one is refused, naming the rule.
- `Compliance` is a surface's validated stiffness, restitution, friction and
  regularisation speed, with the launcher's from the table below, each named
  beside its source. Its damping is the one at which a spring and dashpot
  that never pulls returns a ball at the restitution.
- A part answers `squeezing` with overlaps: a name, a compliance, a unit
  normal, a depth and how what squeezes moves (`Support`). An overlap names
  no point: the surroundings meet the ball midway through it, at the centre
  and radius they were asked about. The part contract holds a depth to the
  ball's overlap, shrinking as the ball moves out along the normal and
  growing as its radius does, and holds a part to squeezing only within its
  reach, under its own names and never under a name it touches by.
- A ball within a tenth of an inch of a compliant part's reach is pushed
  inside each RK4 stage, with the parts placed at that stage's own time; any
  other ball is integrated exactly as before. Each evaluation and each
  asking of what squeezes is a unit of work.
- The stick floor and the rest rules below are built as written. A ball
  never gains energy against still supports where each surface is damped at
  least k·h/2 and all of them together at most m/h, as the launcher's ten
  rings are for the pollen at 0.1 ms. A lone still surface returns a ball at
  its restitution to within c·h/m + k·h²/(12·m), while c·h/m ≤ 1 and
  k·h²/m ≤ 2.
- In a world of one spinning ring and the still hood, a fed ball is thrown
  at 215.5 in/s and 72.8° up, and the same to 0.05 % and 0.01° at half the
  step.

**Stage 4b · the launcher, tested on its own.** Nothing in a run uses it yet.
- `Flywheel.TicksPerTurn` is 18 to 10,000 ticks of the motor to a turn of
  the flywheel; any other count, one that is not a number included, is
  refused, naming the rule. `Flywheel.AngularSpeed.from` turns a command
  into radians a second: the command clamped as `turnsAt` clamps it, times
  2π over the ticks per turn, signed, positive when the back of the wheel
  moves up, about +y in the launcher's frame. It is total and bounded by the
  motor's top speed over 18 ticks a turn, 977 rad/s.
- `Launcher.Frame` is a place within 10,000 in of the field's centre either
  way and a finite yaw. `Launcher.Course` holds the frames a step begins and
  ends at, with their `GridTime`s, and places the frame as *The launcher's
  frame* below says, carrying it from whichever end is nearer in time at the
  course's velocity and turn, so a moving course is exactly the frame it
  begins at as it begins and the frame it ends at as it ends, even where
  carrying one end across the course would miss the other by rounding, and
  the next moving course from that frame begins exactly where it ended. A
  course that does not move gives exactly no velocity and no turn and leaves
  the frame exactly at its end from its start: equal frames, frames whose
  difference is too small to give a rate, and frames whose yaws are half a
  turn either way among them. A course whose faces would sweep faster
  than a flight takes a ball (its speed plus its turn times the 5.26 in the
  farthest face is from the turning axis) is refused. The course keeps both
  rates, bounded so, rather than dividing at each placing, and every
  `GridTime` is finite, so whatever frames, times and wheel speeds the types
  admit, however short the course (down to the least time above none a step
  admits), the launcher is placed somewhere finite, what squeezes a ball
  within its reach moves at a finite velocity where it meets it, and no
  squeeze is dropped.
- `Launcher` is a part of the surroundings with the faces in the tables
  above. A hood ring is its arc and its lead-in, answered as one by
  whichever is nearer, so a ball is squeezed once by each ring. The lead-in
  takes the hood extension's place: an arc of the hood ring's radius about a
  centre of its own for each ring, from −64.8° down to −73°, 8.2°, where
  its radial gap to the groove bottom is the pollen's diameter less 0.25 mm.
  Its normal turns by 6.4° to 7.1° where it meets the hood, an edge the
  ball rolls over. The stop is a rigid face across the path one pollen's
  radius below the entry, from the flange out to the lead-in's lower end
  and across the hood's groove, answered through `near` as the frame's
  metal. A wheel ring moves as the frame does at the hub, its velocity plus
  its turn across the hub's arm, and spins at the frame's turn about the
  upright plus the wheel's about the axle; a hood ring and the stop move
  with the frame alone. A still frame with a stopped wheel squeezes and
  touches with exactly nothing moving. It reaches where some face is within
  the distance asked and nowhere else, so a ball elsewhere on the field
  moves and costs as if the launcher squeezed nothing. It keeps the part
  contract still, sliding, turning and across ±π, with its wheel stopped,
  spinning and reversed, densely across the lead-in's join and the stop,
  and each lead-in is held to its arc between its ends.
- The entry is at −73°, the ball's centre one radius off the groove bottom,
  touching the stop; the feed is the frame's velocity there plus 0.1 m/s up
  the path. The ball overlaps the lead-in there by more than 0.25 mm, since
  the lead-in slopes across the radius it is measured along.
- In a world built as `Flight` builds its own, at 1050 ticks/s and T 24.2,
  the wheel speed the launcher's tests turn it at and not Reginald's, whose
  T is 28 (*Calibration*), a fed ball leaves at 227.7 in/s, 0.424 of u50,
  and 71.0° up; the first sample with nothing of the launcher touching it is
  8.5° past the hood's upper end, at (−3.11, 0, 8.49) in in the frame,
  31.2 ms after the feed and 5,615 units of work later. It spins back at
  171.5 rad/s about the axle, a spin ratio of 1.05. The loads peak at 33 N
  on the wheel and 38 N on the hood, where the stick floor reaches 0.21 m/s.
  The hood's upper end stays at +20°: 71.0° is within the degree that would
  have moved it.
- The throw hardly depends on friction: 227.4 to 228.5 in/s for μ 0.3 to
  1.0 on the wheel and 0.05 to 0.8 on the hood, though a frictionless hood
  throws 208 in/s. A smoother hood carries the ball faster through the
  groove, spinning less for its speed (223.5 against 223.1 in/s, S 1.083
  against 1.085, at μ 0.15 and 0.3). Past the hood's upper end the release
  turns that by a tenth of a percent: at the first sample with no contact
  the smoother hood's ball is the slower, 227.5 against 227.7 in/s.
- In a vacuum, a frame moving at a constant velocity throws as a still one
  does with that velocity added, to 2·10⁻¹¹ in/s. In air the robot's velocity
  is also the ball's through the still air: at 180 in/s it moves the throw
  by a further 0.05 in/s. At T 18 and 2,800 ticks/s the ball leaves at
  420 in/s, and at 516 off the fastest robot along the throw.
- With its wheel stopped the launcher holds a fed ball against the stop,
  where it falls asleep; a wheel that then spins up throws it. Without the
  stop a reversed wheel would drive the ball out of the lead-in's lower end.

`doc/CONTEXT.md` defines each of these terms as built.

---

## Stage 4 · Reginald's launch is simulated through his flywheel and hood

When the bottom gate lets a chambered ball go, it is fed into the launcher:
put at the lower end of the hood's lead-in, touching the wheel, in the
launcher's frame, with a small feed speed along the path. The engine
carries it from there. It is flying the whole way, so a tick shows it in
the hood. Whatever the physics gives as it leaves is the launch: the point,
the velocity and the spin.

### What the CAD says

The 13 October snapshot's exact faces. CAD frame and robot frame as in
`reginald.md`. θ is the angle about the flywheel's axle, from straight back,
positive up.

| What | CAD (mm) | Robot frame (in) |
|---|---|---|
| Flywheel axle | Y 217, Z 180, along X | (−0.512, ·, 7.087) |
| Flywheel groove | a torus: R 75.2886 about the axle, tube 35.56, over \|X\| ≤ 25. The flanges are at r 50, the groove bottom at r 39.729 | |
| Hood | a torus about the same axle, from θ −64.8° to +41.9°. Its groove bottom squeezes a 71.12 mm pollen against the wheel's by 0.29 mm | |
| Exit as drawn | 48.13° above horizontal, from 9.07 in up and 2.72 in behind the centre | |
| Hood extension | fixed, from −73° to the hood's lower end | |

The "10 mm closer" that `reginald.md` gives for the hood is the groove's
depth, not a squeeze. A hood that squeezes 0.29 mm does not grip.

### What we change, and why

The team allowed the flywheel and hood to be adjusted for a 70° exit and a
pollen that fits.

| Change | Value | Why |
|---|---|---|
| The hood's groove | tube 36.0 mm; groove bottom at r 108.341 (R 72.341) | 2.0 mm of squeeze on the simulated 70.61 mm pollen (2.5 mm on the CAD's 71.12 mm one), so the wheel grips |
| The hood's upper end | +20°, not +41.9° | A ball leaving tangentially at +20° leaves at 70°. Exit centre about (−3.26, 0, 8.09) in. The robot's top is 9.084 in after the trim |
| A lead-in | one eccentric arc per ring, in the hood extension's place: opening from the main arc at −64.8° to a gap of the pollen's diameter less 0.25 mm at its lower end, −73°, over 8.2° | Opened to the full 2.0 mm, a pollen fed slower than 0.15 m/s is not drawn in, and with the wheel stopped it drops through the robot. Opened to 0.25 mm it is drawn in from a feed of 0, and held while the wheel is stopped |
| A stop | rigid, at the extension's lower end, −73° | With the wheel stopped a fed ball stays in the launcher, inside the robot and off the floor |
| The feed | 0.1 m/s along the path, at the lead-in's lower end, the ball's centre one radius off the wheel's groove bottom | Stands in for the ramp, which `reginald-internals.md` simulates |

Both grooves are sampled across their width by five rings, at X = 0,
±12.5 and ±25 mm: the wheel's as full circles of radius
75.2886 − √(35.56² − X²), the hood's as arcs of radius 72.341 + √(36² − X²)
over its span. One ring throws 5 % slower than five; nine throw the same as
five.

### How it is simulated

**Arcs.** `Arc.of(centre, axis, radius, span)` with the span `Whole` or a
`Part` from an angle over a width between 0 and 2π, Checked, naming the
rule. Its nearest point is on the circle within the span and at the nearer
end outside it, across the ±π seam. It is total:
- on the axis, the start point, at distance √(h² + a²);
- on the circle itself, which the X = 0 rings allow, the in-plane radial
  direction is the normal, as `Collider.away` does at zero distance.

**Compliant surfaces.** Only the launcher's surfaces are compliant, and they
answer a separate query, `squeezing`, with overlaps, as stage 4a built it.
`near`, `over` and `crossed`
never return them, and a compliant touch never enters the impulses, the
impacts, the touching sets, the islands, the landing rule or the projection.
- The force acts inside RK4, each stage placing the frame at its own time:
  normal k·δ + c·δ̇, never pulling; friction opposing the tangential part of
  (v + ω × arm) − (the support's velocity at the contact), of size
  μ·F_n·min(1, |slip| ÷ v_reg), with torque arm × F on the ball.
- A body beyond a tenth of an inch of every compliant part's reach goes
  through the existing RK4 unchanged. Adding a zero force is not unchanged:
  −0.0 + 0.0 is +0.0.
- No sub-steps: compliant bodies integrate in the engine's 0.1 ms pieces.
  The normal springs are stable to about 19,000 N/mm there, and the launch
  is the same from 5 to 100 µs.
- Each RK4 evaluation of each body is one unit of work, and so is each
  compliant query.

| Constant | Value | Source |
|---|---|---|
| Stiffness | 40 N/mm per surface, shared by its rings | `doc/launch-sim.md` takes the pollen as an indoor 26-hole pickleball. Pickleball Science's force-gauge tests put one between two plates at 16.4–22.8 N/mm: two contacts in series, so 33–46 N/mm a contact |
| Restitution | 0.6 | a pickleball on stone is about 0.65 (Pickleball Science) |
| μ, wheel and hood | 0.6, 0.3 | rubber-like tread and printed plastic, assumed. In a model of this launcher, the throw moved 0.2 % over 0.3–1.0 on the wheel and 0.4 % over 0.15–0.5 on the hood |
| v_reg | 0.02 m/s | the launch is the same from 0.001 to 0.3 m/s |

**Stick.** The launch never sticks: every ring slips until the ball leaves.
A ball held against a stopped wheel does. Regularised friction in stick is
stable while the step is under 2.785·v_reg ÷ (Σμ·N·(1/m + r²/I)). For a
pollen 1/m + r²/I is 103.5 per kg, and at the full squeeze Σμ·N is about
36 N, so at 0.02 m/s the bound is 15 µs, and at 0.1 ms the held ball
jitters by about 40 mm/s and gains energy. So a body's regularisation speed
is never below what keeps stick stable at the engine's step,
2·h·Σμ·N·(1/m + r²/I) ÷ 2.785, taken over its compliant touches. Where the
bound already holds it is 0.02 m/s exactly.

**Rest.** A body may sleep against compliant supports when it is slower
than the rest speed and every compliant support is still: the frame still
and the wheel stopped. A body touching a compliant surface that moves does
not sleep. A sleeping body with a compliant touch wakes when that support's
frame moves or its wheel turns.

**The launcher's frame.** It is the robot's pose turned by the turntable,
while the code has one. `Flight` keeps the frame its last step ended at, and
each step is given the frame it ends at, so the frame is continuous by
construction.
- Between them the pose at engine time t, on stage 3's step grid, is
  interpolated linearly, extrapolated past the end and never clamped,
  carried from whichever end is nearer at the step's velocity and turn, so
  a moving step begins exactly at the frame the last one ended at. A step
  that does not move is at the frame it ends at from its start, less than a
  rate can count from where it began or facing the same way, so the only
  jump meant is a teleport's. The step keeps its velocity and turn, which
  the sweep bounds, so it is finite however short the step.
- The yaw change is `StrictMath.IEEEremainder(yawEnd − yawStart, 2π)`, so
  crossing ±π never spins the hood. The frame's spin is the yaw's rate.
- A step of no time gives the end frame at rest.
- `Flight.carriedTo(frame)` is for a placed robot (`SimRobot.setPose`): no
  sweep, the launcher at rest there, and the balls in the launcher move
  with it. `Flight.carriedMovingAsItWasTo(frame)` is for the turntable's
  encoder written while the robot drives: no sweep either, and the launcher
  keeps the velocity and turn it had, so the balls it holds keep the
  robot's speed and lie in it as they did.
- The launcher is a part of its own, so its motion never wakes the hives and
  never stops the idle jump while nothing is near it. It is held beside the
  hives and the bores: `amid` keeps it when it rebuilds the surroundings,
  and a tip, which replaces the hives' part alone, never touches it.

**The flywheel's speed.** Stage 3's `Flywheel` already clamps a command to
the motor's 2,800 ticks a second and reads NaN as 0. It gains:
- `TicksPerTurn.of(ticks)`, refusing a value outside the constant's
  range, 18 to 10000;
- a total `AngularSpeed` from that clamped command: signed, positive when
  the back of the wheel moves up (the motor's direction).

`Flight.step` takes the `AngularSpeed`, never a double. A reversed wheel
drives the ball down and does not throw it.

**Feeding.** `Flight.fed(ball, kind, radius)` is Checked, and what it
gives is a `Feeding`: `Fed`, with the flight the ball is in, or
`EntryNotClear`, a wait that says why ("the launcher's entry is not
clear") while a ball lies within its own radius and the fed ball's, plus
the touching distance, of the entry spot. `SimRobot` then keeps the ball
chambered and tries again the next substep. A feed is refused, naming the
rule, for a ball that is not a pollen of the launcher's 1.39 in, for a
flight with no robot, and for a ball that would come into flight faster
than a flight takes one (`Flight.MOST_SPEED_IN_PER_S`, 1,000 in/s): the
frame's velocity at the entry and the feed's 0.1 m/s up the path together
pass that while the launcher sweeps near its bound the way the feed goes.

**Constants and the run stream.** The throw is no longer a constant.
- `Constant.FLYWHEEL_TICKS_PER_TURN` (`flywheel_ticks_per_turn`), a
  mechanism, "Flywheel ticks per turn", in ticks per turn of the flywheel,
  18 to 10000, built at 28, the motor's ticks a turn, since the flywheel
  is direct drive (*Calibration*). It says: "How many encoder
  ticks Reginald's flywheel motor counts as the flywheel turns once, gearing
  included: the flywheel turns at its motor's ticks a second over this, and
  the throw and spin follow from the simulated hood."
- Its least value keeps the fastest throw within the 1,000 in/s a flight
  takes, as stage 3's bound on `LAUNCH_THROW` did. The throw is about
  5511 ÷ T in/s at 1050 ticks/s and grows more slowly than the wheel: at
  T 18 and 2,800 ticks/s it is 420 in/s, and 516 off the fastest robot
  along the throw.
- `Constant.Retired` lists `LAUNCH_THROW`, with its label and why it went.
  `Constant.named` refuses a retired name, saying it is no longer a
  simulation constant and what replaced it.
- Stage 3's `Constants.Restored` already restores a value saved past its
  range at the nearer end and keeps what was saved until the constants are
  saved again. It gains the retired names: loading the saved settings sets
  them aside, whatever finite number they are, and keeps every other rule.
  A number no double holds, such as 1e999, stops the server as it does for
  a current constant. A name that was never a constant still stops the
  server. A PUT and the start line stay
  strict. `asSaved()` writes what was set aside back until a PUT succeeds,
  and the listing and the admin page say "Dropped from the saved settings".
  Nothing is printed at startup, since nothing could test it.
- **Protocol 7:** the child's hello lists the constants it reads and the
  ones it retired. A version 6 child reads the 17 names it read before the
  list. A run whose changed constants the child does not read is refused,
  naming them and the fix: "pull develop" when the child's sources are
  older, "the server is older" when the child retired one. A hello of 7 or
  more without a readable list is refused. A start line the child cannot
  take becomes a named outcome, not "could not read the driver station".

**What goes with `Launch`.** `Launch.java` and `LaunchTest` go. Its four
turntable tests move first, to a new `TurntableTest`. `FlightTraceTest`
keeps its own 14 in. `Launch` leaves the strictness allow-list in the same
commit. `SimRobotTest`'s uses of the launch height and reach restate as
"above the robot's top". `Launch.MOST_ROBOT_SPEED_IN_PER_S` moves to
`Chassis`, where `SimRobotTest` already holds the robot to it.
`ConstantsTest`'s bound on `LAUNCH_THROW` goes with the constant, and
`LauncherTest`'s fastest-throw test takes its place.

### Calibration

There is no scan in T. The team settled the flywheel's ticks per turn: "The
flywheel is direct drive, 1:1 on the motor." Its motor counts 28 ticks a
turn (`Flywheel.MOTOR_TICKS_PER_TURN`: a 6,000 rpm 28-tick motor like a
goBILDA 5203, as the code's F of 12.9 implies), so T is 28, a fact and not a
calibration parameter. `Constant.FLYWHEEL_TICKS_PER_TURN` is built at 28 and
stays a constant, 18 to 10000, so a regeared robot can say so.

Stage 3's acceptance set is held at T 28 and the code's own commands, with
no band around it: both alliances' close shots, the ball at rest against
the back, the shot after a tip, one launch, the preload that tips the hive,
and four at their real cadence with all four at rest in an empty cell.

**What stage 4c's runs found.** The code's close launch was 1050 ticks/s,
tuned in October 2025 on an earlier launcher. Through the simulated
launcher at T 28 it throws about 198 in/s, and every shot in the set falls
short. Run at every whole command from 1000 to 1400 ticks/s at T 28, the
shots into a cell holding nectar (both close shots, the ball at rest
against the back, one launch) pass from 1211 to 1231 ticks/s, and in
shorter runs below and above that, and the preload that tips the hive from
1218 to 1227. The two shots into an empty cell pass
elsewhere: the shot after a tip in short runs, 1148 to 1150, 1195 to 1202
and 1263 to 1267, and at 1282; the four at their cadence only at 1148, 1200,
1202 and 1267. So the whole set passes only at isolated commands near 1148
and 1267 ticks/s, and one tick a second either side of them the four at
their cadence fail. Into an empty cell the ball bounces back out at nearly
every command: the cell's friction turns the hood's backspin, about
170 rad/s, into speed back out of the cell.

The user decided two things:
- **Set 1220, check on the robot.** The code's close launch is 1220
  ticks/s (`Launcher.closeLauncherPower`), which the simulated launcher
  throws at 228.6 in/s, 70.9° up, with 172 rad/s of backspin, and the team
  test-fires it on the real robot. `SimRobotTest` and `FlightTest` each
  hold the close launch they use to the one the code commands.
- **A robot problem: keep the tests.** The two empty-cell tests,
  `SimRobotTest.aHiveThatTipsDropsWhatWasInItAndHoldsUpItsOtherCell` and
  `LauncherSimTest.fourLaunchesAtTheirCadenceAllComeToRestInAnEmptyCell`,
  stay exactly as written. With the launcher in the run they are red, and
  they stay red, taking the simulator to be right that the robot's backspin
  throws a ball back out of an empty cell, until the team changes the robot
  to reduce it.

So stage 4c's runtime waits. A red test holds `main` for everyone, so the
flywheel's ticks per turn, protocol 7, `SimRobot` feeding the launcher, the
retired throw and the close launch of 1220 stay off `develop` until a
change to the robot reduces the backspin. What lands before them is the
launcher in `Flight`: the engine's idle surroundings and moved bodies, the
launcher carried, stepped, fed and teleported in a flight, the feed that
waits while its entry is not clear, and the launcher's frame from the
robot's pose and the turntable. The flywheel's ticks per turn and protocol
7 land with `SimRobot` feeding the launcher, not before: until then no run
reads the ticks per turn, so the admin page would offer a constant that
changes nothing, and a protocol 7 child would list it as read, which no
later server could tell from one that reads it. Protocol 7's test of a
start line the child cannot take starts one more child JVM, and the retired
throw's server tests hash one more password, so the suite's cost budget is
pinned again with them.

Any later change to the stiffness, the friction, the rings or the hood
changes what those commands throw, and the acceptance set says again whether
they score.

### Tests first

1. `ArcTest`: gaps and normals inside, outside and past each end of the
   span, across the ±π seam, on the axis and on the circle. A radius that is
   not positive, an axis that is not a unit vector, or a width outside
   (0, 2π) is refused, naming its rule.
2. `CompliantContactTest`:
   - A ball pinched between a still plate and a support moving at u rolls at
     u/2 once it grips, within 2 %.
   - Against still supports it never gains energy over 1 s.
   - Squeezed between still supports at the full squeeze, it falls asleep
     and stays put exactly. It is red under a fixed 0.02 m/s, and green under
     the floor in *Stick*.
   - Spinning the wheel up wakes it and throws it.
   - Held in the squeeze for 0.1 s it records no hit and no impact.
   - Halving the step changes the exit speed by less than 0.5 % and the
     angle by less than 0.2°.
3. `LauncherTest` (core), at 1050 ticks/s and T 24.2, as in stage 4b, a
   wheel speed that exercises the launcher and not Reginald's, since the
   test needs only a spinning wheel:
   - It is drawn in from a feed of 0, and at μ 0.3 on the wheel and 0.5 on
     the hood.
   - It leaves at 0.366·u50 ≤ v ≤ 0.584·u50, where u50 is the surface speed
     at the 100 mm diameter, at 70 ± 3°. The first sample with no launcher
     contact is at θ 20–30°.
   - Its spin is backspin, antiparallel to the wheel's, with 0.9 ≤ S ≤ 1.2.
   - A faster wheel throws strictly faster, with more spin. A lower μ on the
     hood carries it faster through the groove, spinning less for its speed;
     at the first sample with no contact the release past the hood's edge
     turns that by a tenth of a percent. No fixed ratio of throw to wheel
     speed is asserted: the hood's grip falls as the wheel speeds up.
   - With the wheel stopped, the ball stays in the launcher, inside the
     robot and off the floor, and is thrown once the wheel spins up.
   - A reversed wheel throws nothing.
   - The robot's velocity carries into the throw. Turning the turntable by θ
     turns the exit's direction by θ about z.
   - A frame whose yaw crosses ±π throws as one that does not.
   - A NaN command leaves every ball finite.
   - At the least ticks per turn, the motor's top speed and the fastest
     robot, the ball leaves at most `Flight.MOST_SPEED_IN_PER_S`.
4. `FlightTest`: `fed` waits, saying why, while a ball lies just within
   the two radii and the touching distance of the entry, and feeds once it
   lies just beyond; it is refused for a nectar or a ball of another size,
   and for a feed faster than a flight takes a ball while one just slower
   goes in, naming the rule. With both gates open, balls leave one at a
   time and never overlap. A ball in the launcher stays where it was
   through a hive's tip and through a change in what stands in the bores.
   `FlightTest` turns the launcher's wheel at the built T.
5. `FlywheelTest` gains: the `AngularSpeed` is |1050|·2π ÷ T exactly, and
   its sign follows the command. Its clamp tests stay as they are.
6. `SimRobotTest`:
   - `theLauncherThrowsAsHardAsTheLauncherThrowConstantSays` becomes
     `theFlywheelTurnsAsFastAsTheTicksPerTurnConstantSays`, with
     `aFasterFlywheelThrowsFaster`.
   - `theMostTheLauncherThrowConstantAllowsStillThrowsTheCodesCloseLaunch`
     becomes the same at the least ticks per turn.
     `aFlywheelCommandedPastItsMotorsTopSpeedStillThrowsTheBallAsItsTopSpeedWould`
     stays.
   - `withTheFlywheelStoppedTheBallJustDropsOutOfTheRobot` becomes "with the
     flywheel stopped the ball stays in the launcher and is thrown once the
     flywheel spins up".
   - A ball held in the launcher stays where it lies in it when the
     turntable's encoder is written while the robot drives.
   - The acceptance set is held at T 28 and the code's own commands, with
     no band around it, and passes there but for its two shots into an
     empty cell, which stay as written and red (*Calibration*). The close
     launch it uses, and `FlightTest`'s, is held to the one the code
     commands.
7. Constants and the stream: `ConstantsTest`, `SimConstantsTest`,
   `SimDriverStationTest`, `CodingServerTest`, `SimRunStreamTest`,
   `SimBenchTest`, `SimChildTest` and `tools/browser/admin.mjs`, for each
   rule under *Constants and the run stream*. Every test and fake that names
   `LAUNCH_THROW` names the new constant. The tests of a constant saved past
   its range use another constant, and a saved `launch_throw` is set aside as
   retired, whatever finite number it is, and refused as 1e999 or -1e999.
   Each constant the child's hello lists as read changes a run of the robot
   the child builds from it (`SimChildTest`), a child of the first version
   to list them runs on each it reads, from the hello it printed
   (`SimRunStreamTest`), and a run refused both a retired and an unread
   constant names both in its outcome.
8. `TurntableTest`: `LaunchTest`'s four turntable tests, unchanged.
9. Both Flight goldens and the wheel-power traces stay byte-identical:
   `FlightTraceTest` has no robot. The Launch Sim's fingerprints pass
   unchanged.

Then implement, and update `doc/CONTEXT.md`:
- **Launch** becomes **the launcher's physics**; new entries **Arc**,
  **Compliant contact**, **Flywheel**
- **Simulated robot** (fed, not thrown), **Simulation constants** (the ticks
  per turn; retired and dropped constants), **Run stream** (version 7)

And `reginald.md`: its batch 6 is superseded by this stage, and its batch 7
by `reginald-internals.md`. Its facts are corrected: the scoop's mouth is
240 mm, the robot is 9.084 in high after the trim, the exit is about
(−3.26, 0, 8.09) in, and the CAD's hood squeezes 0.29 mm, not 10.
