# The Launch Sim

The **Launch Sim** tab simulates one pollen thrown by a pair of flywheels, on the field, in as
much detail as we could find a measured basis for. It flies through air that drags it, lifts it by
its spin and slows that spin. It bounces off every collider of the field, and its spin changes each
bounce. It knocks the pieces where a match starts out of the way. This page says what the
simulation is, where every number came from, what we chose not to model and why, and how to
calibrate it against a real robot. Issue #175 asked for this.

It is **separate from the realtime robot simulation**: nothing here changes `Flight`, `Hives`,
`SimRobot`, `Launch` or the golden traces. A run of an op mode still throws its balls the way it
always did. The Launch Sim shares the field model (`Field`, read-only) and the pages' assets.

## Contents

1. [What it simulates](#what-it-simulates)
2. [The pollen](#the-pollen)
3. [The two wheels](#the-two-wheels)
4. [The air](#the-air)
5. [Bounces](#bounces)
6. [The field it bounces off](#the-field-it-bounces-off)
7. [The pieces where a match starts](#the-pieces-where-a-match-starts)
8. [Numerics](#numerics)
9. [Ten times real time](#ten-times-real-time)
10. [Options we weighed](#options-we-weighed)
11. [Calibrating it against a real robot](#calibrating-it-against-a-real-robot)
12. [Known deviations from the real field](#known-deviations-from-the-real-field)
13. [How it is tested](#how-it-is-tested)
14. [Sources](#sources)

## What it simulates

A **shot** starts at the moment the pollen leaves the wheels. Its position, velocity and spin at
that moment come from the wheel speeds and the launcher's pose. It ends when one of these happens:

- the pollen comes to rest, with nothing else still moving: in a cell (which cell, and whether that
  cell is upturned so the ball scores), in a flower's bore, on the floor, or on some named collider;
- it leaves the field over a named wall;
- the longest flight allowed has passed;
- it has taken too much work (see [Ten times real time](#ten-times-real-time)).

The page replays it. You can play, pause, step and scrub. The ball is drawn with its 26 holes, so
you can see it spin. You also get the path, a mark at every bounce, the apex and the launch point,
and readouts at every moment. You can compare a shot with earlier launches, which are drawn as
ghost paths.

Units inside are inches, seconds and radians. The settings use the units a team measures in:
rpm, grams, degrees, °C, metres of elevation and lb·in².

## The pollen

FIRST describes pollen as "approximately 2.8 in. (7.1 cm) Gopher ResisDent polyethylene balls"
that "are not perfectly spherical and may vary in size" (game manual §9.8). AndyMark's part
(am-5851) gives more detail:

- **Size.** The drawing shows Ø2.8 in, a 0.07 in polyethylene wall, and 26 holes of Ø0.4 in. The
  STEP file has an outer radius of 1.400 in and an inner radius of 1.330 in.
- **Weight.** It weighs 0.055 lb, which is **24.9 g**.
- **Nectar.** Nectar is the same ball at 3.6 in, weighing 0.091 lb (41.3 g). Its wall is 0.09 in
  thick and it also has 26 holes.

The ball is hollow, so its mass sits near its surface. A thick shell's moment of inertia is
I = (2/5)·m·(R⁵ − r⁵)/(R³ − r³). For pollen that gives **I = 0.634·m·R²**, close to a thin
shell's 2/3 and far from a solid ball's 2/5. Nectar comes out at 0.635. Holes spread evenly over
the ball do not change this to first order.

Settings: *Pollen diameter* 2.8 in (2.6–3.0), *mass* 24.9 g (20–30), *inertia* 0.634 m·r²
(0.4 to 2/3), *nectar mass* 41.3 g (30–55).

Pollen is not foam. It is a perforated, rigid plastic ball, so the closest published flight data
is for the **indoor 26-hole pickleball** (2.87–2.97 in, 22.1–26.5 g, USA Pickleball's standard).
Smooth spheres, table-tennis balls and baseballs only bracket it.

## The two wheels

The issue describes the launcher this way: each side of the ball leaves at the speed of the wheel
touching it, and the difference between the two wheel speeds spins it. That is the classic
two-wheel pitching machine. In Nathan's words, "the speed of the ball is determined by the average
tangential speed of the two wheels and the spin is determined by the difference."

The surface speeds and the release are:

    u₁ = π·D₁·rpm₁/60,   u₂ = π·D₂·rpm₂/60          (in/s)
    v  = (u₁ + u₂)/2   along the launch direction d
    ω  = (u₁ − u₂)/D_eff   about n₁ × d

Here n₁ points from the ball's centre to wheel one, and D_eff is the ball's diameter less any
compression.

**Which way it spins.** The launch direction is d = (cos p cos y, cos p sin y, sin p) for yaw y
and pitch p. *Roll* turns the wheel pair about d:

- At roll 0, wheel one is on top. If wheel one is faster, the ball gets **topspin** and its lift
  pushes it down.
- If wheel two (the bottom wheel) is faster, the ball gets **backspin** and its lift holds it up.
- At roll 90°, wheel one is on the left, and the ball gets **sidespin**.

The preview names the kind of spin. It does this by the direction the spin lifts the ball, judged
against "up" as seen across the flight path, so a steep lob with the bottom wheel faster still
counts as backspin.

**Options, all off or neutral by default** (the issue's model is the default):

- *Speed efficiency* and *spin efficiency* (0.3–1). Teams report hooded single wheels throwing at
  about 30–45 % of surface speed, and two opposing wheels "at roughly the surface speed". No
  published measurement gives the fraction for two wheels, so both default to 1.
- *Compression* (0–0.6 in) shrinks D_eff and raises the spin by D/(D − c). The mean speed does not
  change.
- *Wheels slow as they throw.* A wheel of finite inertia gives up speed to the ball. Solving the
  two impulse balances, with the ball leaving at the wheels' new surface speeds, gives:

      (m + M₁ + M₂)·V + (M₁ − M₂)·s = M₁u₁ + M₂u₂
      (M₁ − M₂)·V + (m_s + M₁ + M₂)·s = M₁u₁ − M₂u₂

  where M = I_wheel/r_wheel², m_s = α·m·R²/r_c² and s = r_c·ω. With a bare goBILDA 96 mm Hogback
  wheel this gives about 80 % of the ideal speed. With a steel flywheel added it gives about 91 %.
  That is the most physically grounded source of "efficiency", and it needs no fitting. *Wheel
  inertia* is per wheel, with its flywheel, in lb·in². The default of 0.95 is a Hogback with an
  82 mm steel flywheel.
- *Robot velocity*: the robot's own motion is added to the ball's (shooting on the move).

*Ticks per revolution* (28 for a goBILDA 5203 at 1:1) changes nothing about the shot. It only shows
each wheel's speed as the encoder ticks per second a team's code would command.

## The air

We use Nathan's form, which most published trajectory calculators use. Forces act on the velocity
relative to the air, v_rel = v − wind, with V = |v_rel|:

    drag   a_D = −K·C_D·V·v_rel,          C_D = C_D0 + k_S·S
    lift   a_L =  K·C_L·V²·(ω × v_rel)/|ω × v_rel|,
                  C_L = k·S/(1 + k·S/C_max)
    spin   dω/dt = −λ·V·ω,                λ = β·ρ·π·R²/(α·m)
    K = ρ·A/(2·m_eff),   S = R·|ω⊥|/V,   g_eff = g·(m − ρ·V_ball)/m_eff,   m_eff = m + C_a·ρ·V_ball

S is the spin parameter. Only the part of the spin across the motion lifts the ball; spin about the
direction of travel (rifle spin) does not. Both forms are written so that nothing divides by V.
They are exactly zero for a ball that is not moving through the air, and continuous everywhere
else.

**Drag.** *Drag* is C_D0 = 0.45, the 26-hole indoor pickleball's free-flight value (Lindsey 2025:
mean 0.45, standard deviation 0.09, over 86 trajectories at 5–13 m/s). Other balls bracket it:

- the 40-hole outdoor pickleball: 0.30–0.33;
- a Wiffle ball: about 0.6;
- floorball CFD at hole sizes like pollen's: 0.39–0.72;
- a community BIOBUZZ calculator's guess: 0.55 ("anywhere in 0.4–0.8").

We do not model a drag crisis. Pollen's Reynolds number stays under 10⁵, and the perforated balls
measured show flat drag over that range.

*Drag rise with spin* is k_S = 0.15. Measured slopes for spinning spheres, tennis balls and
baseballs range from 0 to 0.5.

**Lift.** *Lift slope* k = 2 is dC_L/dS at small spin. *Lift limit* C_max = 0.4 is the lift
coefficient it levels off at. That curve passes:

- smooth-sphere LES results (C_L ≈ 0.17 at S = 0.1);
- table-tennis CFD (0.2 at S = 0.25);
- pickleball backspin data (≈ 0.2);
- a foam ball (0.35 at S ≈ 1).

Published presets in the same family include the tennis ball (k = 1, C_max = 0.5) and Nathan's
baseball fit (k = 1.9, C_max = 0.48). We use the normal Magnus sign. Smooth spheres show an
inverse Magnus effect in pollen's Reynolds range, but the holes very likely trip the boundary
layer, and measured pickleballs lift the normal way.

**Spin-down.** *Spin-down* β = 0.012 comes from the golf-ball torque coefficient
(Tavares/Smits, as used by Nathan). It gives pollen a spin time constant of about 27 s at 10 m/s.
Holes acting as paddles could raise it, but no one has measured a perforated ball. Spin-down
changes the range by under a millimetre, and it matters mostly for how much spin is left at the
first bounce.

**Buoyancy and added mass.** *Added mass* C_a = 0.5 is a sphere's. The displaced air is 0.9 % of
the ball's mass, and the added mass 0.45 %. Together they lengthen a 5.7 m shot by about 7 cm.

**The air itself.** *Temperature*, *elevation* and *humidity* set the density:

    p = 101325·(1 − 0.0065·h/288.15)^5.25588
    ρ = (p − p_v)/(287.058·T) + p_v/(461.495·T)
    p_v = RH·610.78·exp(17.27·t/(t + 237.3))

This gives 1.204 kg/m³ at 20 °C and sea level, and 18 % less at Denver's 1609 m. *Wind* defaults
to none. FTC plays indoors, but an HVAC draught is a real effect, and you can set one.

## Bounces

Each contact is an instantaneous impulse following **Walton's three-coefficient law**:

- *bounce* e is normal restitution;
- *friction* μ is the Coulomb friction coefficient;
- *grip* β₀ is tangential restitution.

This is Garwin's and Cross's model of a ball that grips the surface, capped by Coulomb friction, so
it also reproduces Brody's sliding case. Let n be the contact normal, toward the ball's centre, and
c the velocity of the contact point (spin included). Then:

    approach v_n = −c·n,    slip c_t = c − (c·n)·n
    a contact that is new and approaches faster than the *threshold* bounces:
       normal: the contact point leaves at e(v_n)·v_n
       across: the contact point leaves at −β₀·c_t, if friction can do that (|J_t| ≤ μ·J_n)
               otherwise it slides, with |J_t| = μ·J_n
    any other contact holds: the contact point does not go into the surface, and rolls without
    slipping where friction allows

**Effective mass across a contact.** Across the contact, a sphere's effective mass is
m·α/(1 + α), which is how spin and slip trade. For one contact this is exactly Walton's map. With
β₀ = 0 it is Nakashima's 3-D sliding and rolling model, which was validated on table-tennis balls.

**What the law reproduces:**

- Backspin kicks a bounce up and topspin makes it skid (a 45° impact at 6 m/s leaves at 61° with
  60 rad/s of backspin, against 38° with topspin).
- Sidespin throws a ball sideways off a wall: at 100 rad/s, about 1.6 m/s along the wall.
- A sliding ball comes to roll at (v₀ + α·R·ω₀ × n)/(1 + α).
- With e = 1 and full grip, a solid ball leaves at 3/7 of its slip speed with 10/7 as much spin
  (Garwin's superball).

**Restitution falls as impacts get harder:**

    e = clamp(e_ref − loss·(v_n − 244 in/s)/100, min(e_ref, 0.1), max(e_ref, 0.9))

244 in/s (6.2 m/s) is the pickleball drop test. *Bounce loss* is 0.038 per 100 in/s, which is the
table-tennis slope of 0.015 per m/s. The clamp keeps a bounce below 1 unless you set 1 yourself, so
a dropped ball always comes to rest.

**Spin about the contact normal** is lost to the contact patch. *Impact twist* (0.1 in) is the
lever arm for a bounce: for a Hertz patch of Coulomb slip it is (3π/16)·μ·a, and pollen's patch
radius is about 19 mm at 6 m/s. *Spin twist* (0.06 in) is the lever arm while resting or rolling.
Neither ever reverses the spin.

**Rolling resistance** (*rolling resistance* 0.04) slows a rolling ball at C_rr·g. Measured values
are 0.04–0.05 for a golf ball on carpet and 0.04 for a tennis ball.

**Six surfaces** (bounce / friction / grip):

| Surface | What it is | e | μ | β₀ | Basis |
|---|---|---|---|---|---|
| Floor | EVA foam tiles (am-2499) | 0.55 | 0.50 | 0.20 | estimate; EVA μ 0.5–0.7; soft surfaces raise grip |
| Walls | polycarbonate panels, aluminium rails | 0.50 | 0.30 | 0.15 | Lexan μ 0.29–0.33; panels flex |
| Frame | A-frame, hive tubes and brackets (metal) | 0.62 | 0.35 | 0.15 | pickleball on granite 0.59–0.66; acrylic on aluminium μ 0.5 |
| Flowers | HIPS pipes and plates | 0.55 | 0.35 | 0.15 | polystyrene μ 0.3–0.5 |
| Cells | the hives' cell walls and backs | 0.50 | 0.30 | 0.15 | skin material unpublished; as the walls |
| Pieces | ball on ball (polyethylene) | 0.55 | 0.30 | 0.15 | PE on PE μ 0.2–0.4 |

These are the first numbers to measure on a real field (see
[Calibrating it](#calibrating-it-against-a-real-robot)).

## The field it bounces off

The shot meets what the colliders view draws. The Launch Sim's colliders view and overlay are drawn
from the shot's own collider list, not rebuilt on the page, so the drawing cannot drift from the
physics. The colliders are:

- the floor, z = 0;
- the four walls, as 1 in thick boxes outside the field's 141.17 in, up to the wall's 11.64 in. A
  ball that clips the top of a wall is turned by its edge.
- every field element as a closed convex solid: the A-frame's legs, bar, corners, ACM panels, axle
  holders and feet; each flower's four pipes, its layers and its backstop; and the artifact trays.
  Every element is checked to be convex and closed, to 0.02 in, before it is used.
- each hive's parts (churro tubes, base tubes, pivot brackets, tag plates) as convex solids, and
  its cells' walls and back as two-sided plates. The hives sit at their rest tilt, or at the other
  tilt if you mark that hive *tipped*.

A cell is open at its mouth. A ball whose centre comes to rest inside the cell is in it. The cell
scores when its mouth faces up.

The robot's 2.5-D obstacle footprints and the tape are not colliders here. They exist for the
robot's planar simulation.

## The pieces where a match starts

With *pieces where a match starts* on (the default), the shot shares the field with the pieces:

- **Loose pollen and the nectar in the cells** are bodies that sleep where they lie until something
  touches them. Then they move under the same air and contacts as the launched pollen, and they
  sleep again where they come to rest. A tipped hive's cells have emptied, so their nectar is not
  placed.
- **The pollen standing in the flowers** are held in their bores, so they are fixed spheres.

Two balls meet with the *pieces* surface. Momentum is conserved in every ball-on-ball contact,
which a test checks.

## Numerics

**Integration.** Every awake ball is integrated with classic RK4 at a fixed step h, on a grid
t_k = k·h counted by an integer. *Step* is 0.1 ms by default (0.01–2 ms). Each step is cut into
equal pieces short enough that no ball moves more than 0.1 in, so nothing can pass through a
plate.

**Contacts as events.** Within a piece, a contact that was not touching at the piece's start is
found as an **event**:

1. Bisect the least gap of every such pair 48 times, re-integrating from the piece's start each
   time.
2. Keep the side where the ball has not yet touched.
3. Move there and resolve the contact.
4. Integrate the rest of the piece.

So a bounce happens where and when the ball meets the surface, not wherever it happened to be at
the end of a step. Detecting contacts only at step ends is first order, and its error at 0.1 ms is
0.4 mm. With the event, it is below 10⁻⁵ mm.

**Resolving contacts.** Contacts are resolved together with sequential impulses (20 iterations,
accumulated clamps), so a ball wedged in a corner of a cell is handled. Any penetration left over
is projected out along the normals.

**Accuracy.**

- Free flight converges at RK4's fourth order. At 0.1 ms it is converged to about 10⁻⁹ mm.
- Sliding and rolling come out first order in the step, about 4 µm at 0.1 ms, because friction is
  applied as an impulse each piece. The speed at which a slide turns to a roll, and rolling's
  deceleration, are exact.
- A ball **rests** once it touches something and moves slower than 0.5 in/s for 0.25 s. A contact
  that approaches slower than the threshold does not bounce, which ends a dropped ball's endless
  series of ever-smaller bounces.

**Determinism.** Every class of the Launch Sim uses `StrictMath` and never `java.lang.Math`,
because Math's results may differ by an ulp between JVMs and machines. So a shot is the same bit
for bit wherever it runs, however its time is split into chunks. A test scans the launch sim's
class files to hold that.

## Ten times real time

We read "completing within 10x realtime" as: **computing a shot takes at most ten times as long as
the shot lasts**. The defaults are far inside that.

- **Measured cost.** One RK4 integration of one ball, with the field's colliders culled by their
  boxes, takes about 1 µs here. A shot at the default step costs about 0.01× real time, and at the
  finest step (0.01 ms) about 0.1×. Each launch reports its own figure: "computed in 12 ms,
  0.01× real time".
- **The limit is a count, not a clock.** Deciding by the wall clock would make the same settings
  give different shots on a busy server, so the limit is deterministic instead. A shot that needs
  more than **2,000,000 integrations per simulated second** is stopped as *too much work*, which
  says to make the step longer. That budget is about twenty times under ten times real time at the
  measured cost. Only a pathological storm of events could reach it.
- **How it is tested.** The tests pin this count. The wall time is printed and never judged,
  because a time is the machine's.

## Options we weighed

| Area | Option | Chosen? | Why | Cost against the budget |
|---|---|---|---|---|
| Launcher | Ideal no-slip release (the issue's model) | yes, default | literal to the issue; Nathan's two-wheel machines | nothing |
| Launcher | Finite wheel inertia | yes, switch | physically grounded, needs no fitting; explains most "efficiency" | nothing |
| Launcher | Efficiency factors | yes, default 1 | teams measure exit speeds; no published value for two wheels | nothing |
| Launcher | Compression | yes, default 0 | changes the spin by D/(D − c) | nothing |
| Launcher | Time-resolved contact with the wheels (5–15 ms) | no | no published wheel-on-PE friction or stiffness to calibrate it | small |
| Launcher | Launch angle depending on the speed difference | no | applies to curved hoods, not two opposed wheels | — |
| Air | Quadratic drag with constant C_D0 | yes | measured perforated balls show flat C_D over pollen's Re | tiny |
| Air | C_D rising with spin | yes, 0.15 | measured on spheres, tennis balls and baseballs | tiny |
| Air | Drag crisis C_D(Re) | no | Re < 10⁵ and perforated balls show none | — |
| Air | Saturating Magnus lift C_L(S) | yes | fits every family of balls up to S ≈ 1, which two wheels can reach | tiny |
| Air | Inverse Magnus effect, lift crisis | no (normal sign) | holes trip the boundary layer; pickleballs lift normally | — |
| Air | Spin-down from aerodynamic torque | yes | changes the spin left at a bounce | tiny |
| Air | Buoyancy and added mass | yes | about 1 % of the range | tiny |
| Air | Wind | yes, default none | HVAC draughts | tiny |
| Air | Knuckle (side force from hole orientation) | no | reported for ResisDent fuel, but no coefficient published | — |
| Contact | Rigid impulse, Walton's three coefficients | yes | validated in 2-D and 3-D; exact closed forms make tight tests | tiny |
| Contact | Restitution falling with speed | yes | measured on hollow plastic balls | nothing |
| Contact | Twist friction, rolling resistance | yes | spin about the normal and rolling to a stop | nothing |
| Contact | Compliant contact (spring-damper with tangential stiffness) | no, not yet | reproduces friction reversing during contact, but needs a 50 µs step while touching and stiffness and damping nobody has measured; β₀ stands in for it | ~50× more steps in contact; affordable |
| Contact | Normal-force offset D | no | Cross measures 0.5–3.8 mm; the threshold and rolling resistance cover its visible effects | nothing |
| Field | Every element as a convex solid, walls as boxes, hive cells as plates | yes | the colliders the simulator already has | tiny with box culling |
| Field | Hives tipped | yes, switch per hive | a tipped hive is scored at its other end for much of a match | nothing |
| Field | Hives tipping during the shot | no | one pollen cannot fill a hive (3 nectar + 1 pollen is 29 of 40) | — |
| Pieces | Fixed spheres | for the flowers' pollen | they are held in their bores | tiny |
| Pieces | Movable sleeping bodies | yes, for loose pollen and cells' nectar | a launched pollen knocks them, and lands on them in a cell | small |
| Numerics | RK4, fixed step, bisection to each new contact | yes | deterministic, a fixed time grid, exact bounce times | ~1 µs a step |
| Numerics | Adaptive Dormand–Prince | no | cheaper per unit of accuracy, but its large steps need swept collision tests; compute is not the constraint | — |
| Numerics | Detecting contacts at step ends | no | first order: 0.4 mm at 0.1 ms | — |
| Spread | Seeded scatter of rpm, size, angles (k of N score) | not yet | worthwhile next: balls "may vary in size" | N × a shot |

## Calibrating it against a real robot

Film with a phone at 240 fps or more, side on and against a measured background. Then:

1. **Drag.** Fire with both wheels at the same speed, so the ball has no spin. Fit *drag* to the
   path, then set the *speed efficiency* from the exit speed. This is how Nathan, Steyn and Lindsey
   measured theirs.
2. **Lift.** Fire with the wheels at different speeds. Fit *lift slope* and *lift limit*.
3. **Bounce.** Drop pollen onto a tile and onto a wall panel from 0.5 m and 2 m. The rebound
   heights give each surface's *bounce* and the *bounce loss*.
4. **Friction and grip.** Throw pollen at the floor at 30–60°, with and without spin. Cross's
   e_x = −(v_x2 − Rω₂)/(v_x1 − Rω₁) gives *grip*. The angle where slide turns to grip gives
   *friction*.
5. **Rolling resistance.** Roll a ball across the tiles and time how it slows: the deceleration is
   C_rr·g.
6. **Spin twist.** Spin a ball on the spot and time it to a stop.

The *Download frames* link on a replay gives the frames as CSV, to set beside video tracking.

## Known deviations from the real field

- **Flower layers.** Layers B and C are convex hulls of holed plates. As colliders they close the
  flower's bore (B at 3.9–5.2 in, C at 20.3–21.4 in), so a ball dropped into a flower rests on a
  lid that is not there. Layer B also pokes 0.05 in past the wall plane.
- **Pipes.** The pipes are 8-cornered hulls, not cylinders, so a ball glancing off one sees flat
  faces.
- **Thin parts.** Thin parts (ACM panels, the backstop, the tag plates) are extruded slabs. The
  cells are zero-thickness plates without their rim ribs.
- **Walls.** The walls are 1 in boxes. The real perimeter's rail profile is not in the model.
- **Trays.** The artifact trays stand outside the walls. A ball leaves the field before it could
  reach them.
- **Pieces.** Nectar's radius is the CAD's, 1.8 in. A pollen piece is the launched pollen's size.
- **No robot.** The robot is not drawn and does not collide. The ball starts clear of it.

## How it is tested

- **Core tests (`simcore`).** These use closed forms throughout:
  - flight: the vacuum parabola and its landing time; quadratic-drag fall against the exact tanh
    solution; a dropped ball's series of bounces and its rest time; slide-to-roll speed; rolling
    deceleration; RK4 convergence to the first bounce;
  - bounces: Walton's map against its formulas and its invariants over 10,000 random impacts
    (energy never rises; angular momentum about the contact point is conserved);
  - spin: Garwin's 3/7; Cross's spin reversal at normal incidence; backspin kicks up and topspin
    skids;
  - the wheels: kinematics and impulse balances; the density of air;
  - geometry: the colliders' closest points and normals;
  - determinism: a shot is identical bit for bit however it is chunked.
- **Field tests (`TeamCode`).** On the real field:
  - the default launch scores in blue's upturned cell, clear of the nectar there;
  - a shot over a wall names the wall;
  - a shot at the underside of a cell never gets inside it;
  - sidespin throws a ball sideways off a wall;
  - a launched pollen wakes a loose one;
  - a tipped hive is scored at its other end;
  - the finest step stays within the work budget;
  - every setting at either end of its range still launches.
- **Browser checks.** `tools/browser/launch.mjs` checks the replay page, and `dashboard.mjs`
  checks the tab.

## Sources

- FIRST, *BIOBUZZ game manual* §9: https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-09
- AndyMark, *BIOBUZZ scoring elements* (am-5851, am-5852; drawings and STEP): https://andymark.com/products/biobuzz-scoring-elements
- USA Pickleball, *Equipment Standards Manual* (2025): https://usapickleball.org/docs/rules/USAP-Equipment-Standards-Manual.pdf
- Lindsey, *Pickleball aerodynamics* (2025): https://twu.tennis-warehouse.com/learning_center/pickleball/pickleball_aerodynamics.php
- Steyn et al., *Pickleball trajectories* (2025): https://arxiv.org/pdf/2501.00163
- Nathan, *The effect of spin on the flight of a baseball*, AJP (2008): https://baseball.physics.illinois.edu/AJPFeb08.pdf
- Nathan, *Trajectory analysis* and the trajectory calculator: https://baseball.physics.illinois.edu/TrajectoryAnalysis.pdf
- Nathan, *Spin decay*: https://baseball.physics.illinois.edu/spindown-rev1.pdf
- Muto, Tsubokura and Oshima, *Negative Magnus lift on a rotating sphere* (2012): https://eprints.lib.hokudai.ac.jp/repo/huscap/all/49222/PoF24-1_014102.pdf
- Ito and Kamijima, *Table-tennis ball aerodynamics* (2025): https://www.jstage.jst.go.jp/article/tjsst/17/1/17_25/_pdf/-char/en
- Kim et al., *Inverse Magnus effect* (JFM 2014): https://cambridge.org/core/journals/journal-of-fluid-mechanics/article/inverse-magnus-effect-on-a-rotating-sphere-when-and-why/0E9FEF2880AA45C8B12587374C6F10EE
- Cross, *Sports ball aerodynamics*: https://www.physics.usyd.edu.au/~cross/TRAJECTORIES/Sports%20Balls.pdf
- Cross, *Measurements of the horizontal coefficient of restitution*, AJP 70, 482 (2002): https://www.physics.usyd.edu.au/~cross/PUBLICATIONS/20.%20HorizCOR.PDF
- Cross, *Grip-slip behavior of a bouncing ball*, AJP 70, 1093 (2002): https://www.physics.usyd.edu.au/~cross/PUBLICATIONS/21.%20GripSlip.PDF
- Cross, *Bounce of a spinning ball near normal incidence* (2005): https://www.physics.usyd.edu.au/~cross/PUBLICATIONS/31.%20Spin.pdf
- Cross, *The rolling ball*: https://www.physics.usyd.edu.au/~cross/PUBLICATIONS/55.%20RollingBall.pdf
- Cornell impact group, *Walton's model and measured coefficients*: https://grainflowresearch.mae.cornell.edu/impact/impact.html
- Nakashima et al., *3-D model of a bouncing table-tennis ball*: https://nagoya.repo.nii.ac.jp/records/12580
- Biber, Champneys and Szalai, *Compliant contact and friction reversal* (2022): https://arxiv.org/abs/2208.11685
- Catto, *Sequential impulses* (GDC 2006): https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf
- Catto, *Continuous collision* (GDC 2013): https://box2d.org/files/ErinCatto_ContinuousCollision_GDC2013.pdf
- FRC 449, *Flywheel calculations*: https://robot.mbhs.edu/media/whitepapers/2026/flywheel-calculations-latest.pdf
- Chief Delphi, *flywheel exit velocity*: https://www.chiefdelphi.com/t/determine-flywheel-velocity-for-ball-exit-velocity/394940
- goBILDA flywheels and wheels: https://www.gobilda.com/flywheels/
- OpenJDK, *StrictMath*: https://docs.oracle.com/en/java/javase/21/docs/api/java.base/java/lang/StrictMath.html
- Wikipedia, *Density of air*: https://en.wikipedia.org/wiki/Density_of_air
