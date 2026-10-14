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

Stages 1 to 3 are built. Stage 4 is planned here. Every stage keeps the
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
  second, and reads one that is not a number as 0. `LAUNCH_THROW` is held
  to at most 0.29, so the fastest throw off the fastest robot (180 in/s) is
  within the 1,000 in/s a flight takes. A launch the flight refuses stops
  the run, saying why.
- A constant saved past where its range now lets it go no longer stops the
  server: runs use the nearer end of its range, and the admin page shows
  what was saved (`Constants.Restored`) until the constants are saved again.
- No refusal is dropped. The core's PMD gate refuses a `Checked` that
  nothing looks at, and a tip or a ball the engine refuses stops the run,
  saying why.
- `LAUNCH_THROW` was calibrated again for air and contacts: the widest band
  at which every scored shot the tests ask for goes in and stays ran from
  192 to 206.75 in/s, and its middle, 0.1899 in/s per tick/s (199.4 in/s at
  1050 ticks/s), is built in. Tests hold the built throw and 2.1 in/s either
  side of it for every shot in that set.

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
| A lead-in | one eccentric arc per ring, opening from the main arc at −64.8° to a gap of the pollen's diameter less 0.25 mm at its lower end, over at least 5° | Opened to the full 2.0 mm, a pollen fed slower than 0.15 m/s is not drawn in, and with the wheel stopped it drops through the robot. Opened to 0.25 mm it is drawn in from a feed of 0, and held while the wheel is stopped |
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
answer a separate query, `squeezing`: the touch's id, surface, normal, depth
and the support's velocity at the contact. `near`, `over` and `crossed`
never return them, and a compliant touch never enters the impulses, the
impacts, the touching sets, the islands, the landing rule or the projection.
- The force acts inside RK4, each stage placing the frame at its own time:
  normal k·δ + c·δ̇, never pulling; friction opposing the tangential part of
  (v + ω × arm) − (the support's velocity at the contact), of size
  μ·F_n·min(1, |slip| ÷ v_reg), with torque arm × F on the ball.
- A body with no compliant touch goes through the existing RK4 unchanged.
  Adding a zero force is not unchanged: −0.0 + 0.0 is +0.0.
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
  interpolated linearly, extrapolated past the end and never clamped.
- The yaw change is `StrictMath.IEEEremainder(yawEnd − yawStart, 2π)`, so
  crossing ±π never spins the hood. The frame's spin is the yaw's rate.
- A step of no time gives the end frame at rest.
- `Flight.carriedTo(frame)` is for teleports (`SimRobot.setPose`): no sweep,
  and the balls in the launcher move with it.
- The launcher is a part of its own, so its motion never wakes the hives and
  never stops the idle jump while nothing is near it. It is held beside the
  hives and the bores: `amid` keeps it when it rebuilds the surroundings,
  and a tip, which replaces the hives' part alone, never touches it.

**The flywheel's speed.** Stage 3's `Flywheel` already clamps a command to
the motor's 2,800 ticks a second and reads NaN as 0. It gains:
- `TicksPerTurn.of(constants)`, refusing a value outside the constant's
  range;
- a total `AngularSpeed` from that clamped command: signed, positive when
  the back of the wheel moves up (the motor's direction).

`Flight.step` takes the `AngularSpeed`, never a double. A reversed wheel
drives the ball down and does not throw it.

**Feeding.** `Flight.fed(key)` is Checked. It is refused with "the
launcher's entry is not clear" while a body lies within 2r plus the
touching distance of the entry spot. `SimRobot` then keeps the ball
chambered and tries again the next substep.

**Constants and the run stream.** The throw is no longer a constant.
- `Constant.FLYWHEEL_TICKS_PER_TURN` (`flywheel_ticks_per_turn`), a
  mechanism, "Flywheel ticks per turn", in ticks per turn of the flywheel,
  18 to 10000, built at the calibrated value. It says: "How many encoder
  ticks Reginald's flywheel motor counts as the flywheel turns once, gearing
  included: the flywheel turns at its motor's ticks a second over this, and
  the throw and spin follow from the simulated hood."
- Its least value keeps the fastest throw within the 1,000 in/s a flight
  takes, as stage 3's bound on `LAUNCH_THROW` did. The throw is about
  5474 ÷ T in/s at 1050 ticks/s and grows more slowly than the wheel, so at
  T 18 and 2,800 ticks/s it is at most about 810 in/s. Off the fastest
  robot that is at most about 990.
- `Constant.Retired` lists `LAUNCH_THROW`, with its label and why it went.
  `Constant.named` refuses a retired name, saying it is no longer a
  simulation constant and what replaced it.
- Stage 3's `Constants.Restored` already restores a value saved past its
  range at the nearer end and keeps what was saved until the constants are
  saved again. It gains the retired names: loading the saved settings sets
  them aside, whatever their value, and keeps every other rule. A name that
  was never a constant still stops the server. A PUT and the start line stay
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

The throw is about 5474 ÷ T in/s at 1050 ticks/s, for T ticks per turn, so
the scan is in T: steps of at most 0.11 % of T (about 0.25 in/s), over
stage 3's acceptance set: both alliances' close shots, the ball at rest
against the back, the shot after a tip, one launch, and four at their real
cadence with all four at rest in the cell. The widest band at which every
one passes must be at least 1.77 % of T wide (about 4 in/s). Its middle is
built in, and reported in ticks and in in/s.

Expect it near T 24.2: a 1.16:1 step-up over a 28-tick motor. At T 28 the
throw is 196 in/s and lands short. The Launch Sim's runs of this exit put
the band for the close shot at 224–228 in/s, with four shots in a row
failing at two speeds inside it. Stage 3's band came out three times wider
than the same kind of estimate, so the scan decides. If the band is
narrower than 1.77 % of T, stop and report it to the user with the numbers,
rather than build in a knife edge.

Any later change to the stiffness, the friction, the rings or the hood moves
T, and is calibrated again.

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
3. `LauncherTest` (core), at 1050 ticks/s and the built T:
   - It is drawn in from a feed of 0, and at μ 0.3 on the wheel and 0.5 on
     the hood.
   - It leaves at 0.366·u50 ≤ v ≤ 0.584·u50, where u50 is the surface speed
     at the 100 mm diameter, at 70 ± 3°. The first sample with no launcher
     contact is at θ 20–30°.
   - Its spin is backspin, antiparallel to the wheel's, with 0.9 ≤ S ≤ 1.2.
   - A faster wheel throws strictly faster, with more spin. A lower μ on the
     hood throws faster, with less spin. No fixed ratio of throw to wheel
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
4. `FlightTest`: `fed` is refused while the entry is not clear, naming the
   rule. With both gates open, balls leave one at a time and never overlap.
   A ball in the launcher stays where it was through a hive's tip and
   through a change in what stands in the bores.
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
   - The acceptance set passes at the built T, and the band test holds it at
     T·(1 ± 0.885 %) for every shot in it. `LauncherSimTest`'s four launches
     at their cadence are held at the built T and at T·(1 ± 0.885 %) too.
7. Constants and the stream: `ConstantsTest`, `SimConstantsTest`,
   `SimDriverStationTest`, `CodingServerTest`, `SimRunStreamTest`,
   `SimBenchTest`, `SimChildTest` and `tools/browser/admin.mjs`, for each
   rule under *Constants and the run stream*. Every test and fake that names
   `LAUNCH_THROW` names the new constant. The tests of a constant saved past
   its range use another constant, and a saved `launch_throw` is set aside as
   retired, whatever its value.
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
