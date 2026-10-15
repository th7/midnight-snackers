# Glossary

The words this project uses for the coding server, the simulator and the
robot, as the code uses them. When a term here and a name in the code
disagree, fix one of them. Robot-side vocabulary is covered only where a
module has been deepened; plans, steps and the other subsystems are not yet.

## The robot

**Where things live** — Each robot's code is a package of its own, its
**own files**: `org.firstinspires.ftc.reginald` is Reginald and
`org.firstinspires.ftc.nugget` is Nugget (see **Nugget**). `reginald` holds
the **Robot** and nothing but the subsystems it is made of, so the list of
files is the list of parts. Underneath it: `opmode` (what the driver station
lists and how a plan becomes one), `hardware` (the devices and what is wired
to what), `control` (the controllers and filters a subsystem steers by) and
`roadrunner` (trajectories, which we own rather than vendor).
`org.firstinspires.ftc.teamcode` holds what every robot's code is built on:
`base` (what a subsystem *is* — `Loopable` — where it prints — `Prints`,
`Channels` — and the `Alliance` a run plays for), `planrunner` (plans and
steps), and what both robots' Road Runner needs — the **clocked encoder** and
the drawing of a robot in `roadrunner`, the drive runner that follows an action
a loop at a time in `control`, and the dashboard it draws on in `hardware`;
its test sources hold the simulator, the coding server and the fakes.
Nothing outside a robot's package uses that robot's code, and every op mode
is in a robot's package, and a test says both: a class moves into `teamcode`
when a second robot needs it, rather than being reached for across.
Class: `EachRobotKeepsToItsOwnPackageTest`.

**Subsystem** — One part of the robot, and now only that: something the
robot ticks, which is to say a `Loopable` that lives directly in
`reginald`. There is no base class to extend. A subsystem is built with
everything it needs, sets itself up in its own constructor, does its work in
`loop`, and prints to the **channel** it was handed. What used to be three
hooks a base class asked for is one method an interface asks for, and the
package says which classes are subsystems — which is the rule this glossary
already opened with, now doing a job.

**Whoever owns it, ticks it.** Anything that must be ticked is ticked by
whatever it belongs to, on the first line of that thing's own loop: a
subsystem's plan runner or drive runner in its `loop`, an auto's plan runner
in the auto's `onLoop`, a TeleOp's driver in the TeleOp's. Registering them
to be ticked by somebody else read tidily and cost more than it saved. The
method that is supposed to say what a thing does each tick stopped saying it
— `Drive`, which follows a trajectory every loop, had an empty one — and the
robot ended up ticking an op mode's plan runner, which meant it had to
accept something that was not a subsystem and then ask which it had been
given.

So the chain is plain the whole way down: an op mode's `init` builds the
**Robot**, which builds each subsystem and hands it what it needs; an op
mode's `loop` ticks the robot, which ticks its subsystems in the order of
the one list in its constructor, and then the op mode's own `onLoop`, which
is where anything the op mode owns is ticked. `loop` is final at both
levels, so the order is not a subclass's to rearrange, and an auto's plan
advancing after every subsystem is the shape of it rather than a position in
a list. That every subsystem is ticked at all is mechanical rather than
remembered, and a test says so: it finds every `Loopable` in `reginald` and
fails naming any the op modes never reach. Classes: `Loopable`;
`Robot.loopOrder`; `SubSystemsAreTickedTest`.

**Channel** — The driver station's telemetry divided by the name of whoever
is printing. Each subsystem is handed its own when it is built and prints to
it and nothing else, always, knowing nothing about whether anyone is
reading; the channel puts its own name on every caption, so two subsystems
cannot print the same one. Telemetry costs a loop and a crowded screen, so a
channel is off until a driver asks for it, and which button reaches which
channels is said once, in the op mode.

That the decision lives there and nowhere else is the point. It used to be a
flag on every subsystem, a gate in a base class and a method on each of them
that the op mode called by name; a subsystem carried the machinery for a
question it had no business answering. **Nowhere** is a channel that reaches
no screen, which is what Road Runner's tuning op modes are handed, said out
loud rather than left as a null. Classes: `Channels`; `Prints`;
`Prints.NOWHERE`.

**Handed, not fetched** — A subsystem is given everything it needs when it
is built — its devices, its clock, what it coordinates, and somewhere to
print — and reaches for nothing else, ever. Nothing is handed to it
afterwards, so there is no moment between being built and being ready. So
the order in Robot's constructor is the order
they depend on each other in, and javac says so: a subsystem built before
one it is given does not compile, and one that asks for something the
robot does not have does not compile either. Nothing reaches back into the
robot, so there is no list to keep in step and no null to find on a field.

**Hardware** — Everything the op modes touch outside their own code: the
configured devices, the camera's detections, the dashboard, and the
**clock**, the time in nanoseconds that every timer in the robot code
reads and nothing else does: a step that waits, the trajectory followers,
the camera's check of a detection's age, and the dead wheel encoders'
measure of their own speed. On the robot the clock is the
system's; in the simulator it is the simulated clock. A timed step must
name its clock, so a forgotten one is a compile error, and no part of the
robot may reach for Road Runner's wall-clock encoder, which a test
enforces rather than leaving to care.

It is **whole or not at all**. Both sides of the seam wire it by hand — the
robot from its configuration, the simulator from its fakes — and nothing
but this class can say the two agree, so it is built through a builder that
names each device and refuses one that is missing any, naming what is
missing. A device added here that an adapter has not kept up with then
fails where that adapter is written, rather than as a null inside whichever
subsystem reaches for it first, which on the robot is in the middle of a
match. The devices are named as they are set rather than counted out in
order, since nine of them are motors and a positional list would let a
wheel be quietly swapped for its neighbour. Class: `Hardware`;
`Hardware.Builder`; `Robot.clock`.


**Wheels** — The four wheels, and the only thing that turns numbers into
them turning. Everything that wants the robot to move — a driver's sticks,
the drive steering itself toward a pose, a Road Runner trajectory being
followed — ends here. It is also the one place three numbers become four:
forward, left and counterclockwise, mixed by Road Runner's own mecanum
kinematics, so there is one answer to what a mixed command means. A
command asking more of a wheel than it has is **scaled down whole**, not
clipped: clipping the one wheel that ran out would leave the others as
they were, which is a different command than the one given, and the robot
would go somewhere other than where it was pointed. Scaled, it goes where
it was pointed, slower. That there is one such place is the point: while two
of them wrote the same four motors, what the robot did came down to which
ran last in the loop, and nothing in the code said which that was. It also
does the wiring the motors need once, when the robot is built: they brake
when asked for nothing, and the two on the back run the other way round
because of how they are mounted. A test asks the motors themselves who
wrote them and holds the answer to this class. Class: `Wheels`;
`DriveOwnsTheWheelsTest`.

**Drive** — The subsystem that moves the robot. Whoever is driving gives it
one **intent** per loop and it asks the **Wheels** to turn: **manual**
(straight, strafe and turn powers from the sticks), **toward** (a pose the
drive steers to on its own, answering whether the robot has arrived and come
to rest), or **follow** (a Road Runner action, run loop by loop until it is
done or cancelled). An action being followed owns the wheels: manual and
toward do nothing until it is done or cancelled, and a stick pushed past the
takeover deflection cancels it. **Cancelling stops the robot**: it is
whoever was driving saying they are done, and a robot nobody is driving
should not still be driving, so the wheels are asked for nothing, which
brakes them. Whoever cancels may give an intent in the same loop, and that
is what the robot does. Class: `Drive`.

**Held axis** — While the drive steers toward a pose, an axis the driver
holds (straight, strafe or turn) is driven at the driver's power and the
rest are the drive's to steer. Under the left bumper the driver may hold
strafe and turn; under the right bumper they hold straight and strafe and
may hold turn. Class: `Drive.Held`.

**Pose** — A position and heading on the field, in Road Runner coordinates
(+x forward, +y left, facing the field from the audience), as the alliance
plays it. Everything Nav takes and gives is a pose; Road Runner's own
`Pose2d` inside it is read only by Road Runner and the drive's controllers.
Class: `Nav.Pose`.

**Localizer** — Where the robot believes it is, and the sensors it
believes it from: the two dead wheels and the IMU. It keeps what that
update measures -- the pose, the speed, and the recent **trail** -- because
all three are answers about where the robot is; a trajectory being followed
asks it rather than working them out again. It moves that belief on
**once** per loop and is the first subsystem ticked, so one tick of the
robot is one moment — everything that reads the pose during it reads the
same pose, and the encoders and the IMU are asked once between one loop
and the next. A second update inside the same loop asks the encoders again
for a delta that has barely happened and leaves two subsystems that ran
either side of it disagreeing about where the robot was at a single
instant; that it is once is held to by a test. What the belief *means* —
the field, the alliance's half of it, where the goal is — is Nav's. Class:
`Localizer`.

**Clocked encoder** — The hub reports a motor's velocity in sixteen bits,
so a wheel turning quickly wraps the count, and the true one is recovered
by measuring the wheel a second way: how far it turned, over how long. The
*how long* is a clock, and it is the robot's, like every other timer.
Road Runner's own overflow encoder reads the wall clock for it, which is
right on the robot and wrong in the simulator, where a loop is twenty
milliseconds of the robot's time and a fraction of a millisecond of the
machine's: the estimate comes out tens of times too large and the wrong
wrap is chosen, so a robot rolling to a stop measures itself doing a
hundred and seventy inches a second. The pose survives that, being built
from position rather than velocity; what does not is anything that asks
how fast the robot is going, such as whether a trajectory has finished.
The arithmetic is Road Runner's, unchanged, so on the robot this measures
what its own encoder measures. The tuning op modes are the one exception
and keep Road Runner's: they run on the robot and nowhere else, and its
ramp loggers find the raw encoders underneath by looking for that exact
class. Class: `ClockedOverflowEncoder`.

**Nav** — Where the robot is on the field, and where the places worth going
are.
`pose(x, y, heading)` makes a pose from coordinates given the blue way, and
the alliance's mirroring (y and heading negated for red) happens there and
nowhere else. It answers the **current pose** (where the localizer believes
the robot is), whether the robot is **near** a pose (within 3 inches and
6 degrees), and the **launch pose**. It says where, not how: the strafing
and backward paths the plans follow are built by the drive, from the
robot's own model of itself, and a test holds Road Runner's drive out of
Nav. Classes: `Nav`; `Drive.strafeTo`; `Drive.backwardTo`;
`NavTest.navDoesNotHoldTheDriveThatBuildsPaths`.

**On the field** — Whether the robot's pose means anything beyond distance
travelled since it was switched on: whether somebody has said where it is.
Two things say so and they say the same thing — `placeAt`, a person setting
the robot down or a plan's first step naming its start pose, and a
**sighting**, the camera's answer. Until one of them has, there is nothing
to aim at and the turntable points straight ahead.

It used to be raised by the sighting alone, so an auto that placed itself by
hand aimed straight ahead for the whole run unless the camera happened to
see a tag. That it is one fact with one meaning, however the robot got
there, is the point: a caller that knows where the robot is should not also
have to know which of two doors marks it. It follows that a sighting
arriving after a placement is a *later* sighting and nudges rather than
placing, which is what we want — a start pose somebody measured is worth
more than the first frame the camera agreed on. Methods: `Nav.placeAt`;
`Nav.sighted`.

**Turntable** — What the launcher and the camera ride on, and the only thing
that decides where it points. Each tick the brain gives it an **aim**, how
far from straight ahead the goal is; what it does with one depends on what
it is **following**: the goal, straight ahead (the driver parked it), or the
driver's own hand (the driver nudged it, ten ticks a press). An aim is a
standing request, not an order, and only `followTheGoal` gives the turntable
back after parking or a nudge. It starts following the goal.

That the mode lives with the target it guards is what makes a nudge stick.
It used to be two booleans on the brain over a target field on the turntable,
set from a third module: a nudge survived only because the line above it in
`Driver` flipped one of them, so deleting that line still compiled, still
ran, and lost the nudge on the brain's next tick twenty milliseconds later.
There is now no ordering to forget, because a nudge *is* the driver taking
it over. Class: `Turntable`.

**Launch pose** — Where to launch from: 40 inches short of the alliance's
goal on the line from the robot to it, facing the goal. There is none when
playing for no alliance, which has no goal; then the bumpers just drive,
and the brain's auto-shoot has nowhere to go.

**Intake** — The subsystem that takes pollen off the floor and into the
hopper. It is **on** or **off** and nothing else: on, its motor runs at full
power; off, the motor stops. It starts on, from the moment the robot is built
rather than from its first loop, so a robot nobody has told anything is taking
pollen in. That is all it is for now — nothing reverses it to clear a jam and
nothing senses a full hopper — and the motor is the whole of what the
simulator, or anyone else, can see of it. Class: `Intake`.

**Sighting** — Where the goal's AprilTag says the robot is, as the camera
faces, which is the turntable's heading. The brain turns it back by the
turntable's offset and, when playing for an alliance, hands it to Nav.
Class: `Camera.sighting()`.

**Agreement** — What the camera waits for before it will say where the robot
is: the last three detections of the goal's tag placing it within an inch of
one another, the newest of them less than a tenth of a second old on the
robot's clock. The filter answers that one question and every way of asking
it is **total** — a filter that has seen nothing answers that it has seen
nothing. It used to publish the newest detection's age as a bare number, which
threw when there was no newest detection, so a driver who turned the camera's
telemetry on before the robot had looked at anything ended the op mode in the
middle of a match. The age is still there, and still readable exactly when
nothing is agreeing, which is when it is worth reading; it is now an answer
rather than an exception. Classes: `DetectionFilter`;
`DetectionFilter.Agreed`.

## Nugget

**Nugget** — The team's second robot, whose users work on `nugget-develop`
(see **Robot**, under the coding server). Its code is a package of its own,
`org.firstinspires.ftc.nugget`, beside Reginald's `org.firstinspires.ftc.reginald`
and the shared `teamcode` (see **Where things live**), and its teammates
always edit every file in it (see **Own files**). Its op modes extend `NuggetOpMode`, which is how the simulator knows
them for Nugget's and hands them Nugget's hardware rather than the robot
controller's configuration. Class: `NuggetOpMode`.

**Tank drive** — Nugget's two drive motors, `leftDrive` and `rightDrive` in
the robot's configuration, one to a side. They are mounted facing each other,
so one turns opposite the other: the left is reversed, and a positive power
drives either side forward. Straight and counterclockwise are mixed into a
left and a right power and, like Reginald's **Wheels**, a command asking more
of a side than it has is scaled down whole rather than clipped, so the robot
turns as sharply as it was pointed, slower. Both brake when asked for nothing.
Its TeleOp drives it from the sticks Reginald's drivers use: the left stick
forward and back, the right stick across to turn. Nugget's hardware is those
two motors, the battery's voltage, the dashboard and the **clock**, which is
the robot's own as Reginald's is and the simulated one in the simulator, and
it is **whole or not at all**, as Reginald's is. Classes: `TankDrive`;
`NuggetHardware`; `NuggetTeleOp`.

**Tank localizer** — Where Nugget believes it is, from how far each side has
gone by its drive motor's encoder: both sides ahead take it ahead the way it
faces, and the right side going further than the left turns it
counterclockwise by the difference over the track. It has nothing else to go
on, so a side that slips sideways, as a tank does in a tight turn, moves Nugget
somewhere it does not know about. Like Reginald's **Localizer** it reads the
encoders once a loop, on Nugget's own **clock**, and whoever owns it ticks it
first. What it measures in — the inches a tick and the track — is Road
Runner's model of Nugget, and nobody has measured the real one yet, so it is
the **simulated Nugget**'s, which a test holds it to. Classes:
`TankLocalizer`; `Trajectories.Params`.

**Trajectories** — How Nugget drives along a path Road Runner has built: from
the pose the **tank localizer** believes, a path of splines and turns within
Nugget's limits of speed, turning and acceleration, followed a loop at a time
by an action that asks the **tank drive** for each side's power. A tank cannot
move sideways, so a Ramsete controller steers it back onto its path by turning.
What the path asks of its speed and turning, and of how fast each is changing,
is fed forward through Nugget's model of its motors, so it keeps up with the
path rather than lagging behind it and swinging wide of the corners. A turn in
place is an action too, corrected toward the heading it should have by then.
What Nugget is, to Road Runner — the inches a tick, the track, its motors, its
limits and its gains — is one set of parameters, and until someone measures
the real Nugget it is the **simulated Nugget**'s. Classes: `Trajectories`;
`Trajectories.Params`.

**RoadRunner example** — Nugget's autonomous op mode, listed on the driver
station and the Simulate tab as *Nugget RoadRunner Example* under Nugget, and
the example of a **plan** that drives with Road Runner. Set down on the blue
side at (-24, -44) facing the back wall, it drives a lap of the field around the
middle — along the blue side, the back wall, the red side and the audience
wall — and spins once where it started. Each of those is a **step** of its
plan, which builds its path from wherever Nugget believes it is when the step
begins, so no leg inherits where the one before meant to finish. It ticks its
localizer, then whatever action is being followed, then its plan, and it is
done when its plan is, well within the autonomous period. In the simulator, set
it down where its plan says it starts. Class: `RoadRunnerExample`.

**Simulated Nugget** — Nugget as the simulator runs it: the same 18-inch
rigid body Reginald is, on the same field, pushed by two motors rather than
four. Each side's motor turns both of that side's simulated wheels, so the
robot drives and turns and cannot strafe, nor slide out of a turn: a tank's
wheels grip the floor sideways, as hard as the floor's traction allows, where
Reginald's mecanum rollers let it slide and only its motors push it that way.
The two motors are mounted
mirror-image, the left one reversed: code that forgets to reverse it spins
Nugget where it stands, as it would on the floor. A side is one motor, so both
its wheels share that motor's **noise**, drawn as the front wheel's. It has no
hopper, so it starts holding no balls, and nothing else of Reginald's: no
intake, launcher, turntable, camera, dead wheels or IMU. Its motors are
modelled as Reginald's, tuned the same, because nobody has measured Nugget's.
Its drive powers are recorded left then right. Classes: `Drivebase`;
`Drivebase.sideways`; `Sides`; `SimRobot`.

**Drive encoders** — What Nugget knows of how it has moved: each drive motor
counts how far its side has gone, in the ticks Reginald's motors are modelled
in, and reports it as the SDK does, the way the motor turns. So a motor
reversed to match its mounting counts ahead as its side goes ahead, and one
that is not counts ahead as its side goes back. The sides turn as the
simulator's own wheels do, each as far off the centre as Reginald's track says,
and a hub reports their speed to the nearest twenty ticks a second, as it does
a dead wheel's. Classes: `DriveEncoders` and `Encoder`, in the core.

## The coding server

**Coding server** — The host process teammates reach over the LAN to edit
the robot code and run simulations. One machine runs it with
`./gradlew :TeamCode:codingServer`. It has two listeners: the **admin
listener** on loopback only (port 21987, `CODING_ADMIN_PORT`), and the
**user listener** on every interface (port 21986, `CODING_USER_PORT`).
Class: `CodingServer`.

**Routes** — What each listener answers, said once as a table of method,
path pattern and handler: the user listener's public routes, then the
**approved routes** behind one guard (403 for anything but an approved
session); the admin listener's routes. A path no route knows is a 404, a
path known by another method a 405. The bench's routes are one table too,
**mounted** at the root of the bench page and under `/sim` on the user
listener, per user, so a run started there is recorded as theirs. Class:
`Router`.

**Admin** — The person at the host machine. Only they can reach the admin
listener, where they decide logins, pick files for the editable set and set
the **simulation constants**. There is no admin login; being on the machine is
the credential.

**User listing** — `GET /admin/users` and the roster it draws on the admin
page: every user who has ever logged in, in the order they first did, one
row to a name, each with their **work** on every robot and the **sessions**
they have made, oldest first, each saying which robot it is on. A user's
work on a robot says whether they work on it (**enabled**, a box on the row
the admin ticks or clears) and, while they do, the worktree, its branch, its
**status** (the same changed, ahead and behind that `GET /git/status` gives
the user; null until the worktree exists, and null with a **statusError**
when git cannot read it, so one broken worktree does not blank the list),
how its last pull or push ended, and a Pull button. What is the user's
rather than any one robot's is said once, on the user: whether they are
**deletable** (whether a **delete** would go through rather than be refused
— said by the server, since the server is what enforces it, counting the
work kept on a robot they were taken off as well, and false for a user git
cannot be read for, because what a delete would throw away is exactly what
could not be counted), and a Delete button. Logging in again, on either
robot, adds a session to the user, never a second row; a **delete** takes
the row away, and a login after that is a new row for the same branches.

**Fold** — A user's sessions on the admin page, folded away under their
row and opened by the caret. A user the admin has not folded by hand is
open exactly while a session of theirs is pending, so a login waiting to
be decided is never hidden; folded shut, the row still carries the
pending count, the branch, the status and what its sessions have open.

**Robot** — One of the team's robots: **Reginald** or **Nugget**. Each robot
works its own **develop branch**, and keeps its own worktrees, user branches
and editable set. Reginald keeps every name it had before there were two
robots, so a server started on an old state directory finds its users, their
worktrees and their files where they were, and a login stored before then is
Reginald's. Which robots there are, what each is called and what its lines
are named is said once, in the **core**; `GET /admin/info` lists them for the
admin page, and a test holds the login page's choice to the same list.
Class: `TeamRobot`, in the core.

**Robots a user works on** — Which robots the admin lets a user work on:
either or both, and never none once they have been let in. A teammate picks
the robot to work on when they ask to join, and none is picked for them; the
login says which (`POST /login?username=<name>&robot=reginald` or
`robot=nugget`), and one that does not is refused before any session exists.
Approving it **lets the user onto** that robot, on top of any they had. The
admin lets them onto the other, or takes them off one, with the robot's box
on their row, `POST /admin/users/robots?robot=<robot>&username=<name>&enabled=true|false`;
letting them on makes their worktree there, so git's refusal, if any, is the
admin's to see, and taking them off their last robot is refused, saying to
revoke their logins or delete them instead. Taking them off a robot keeps
their work on it — the worktree, the branch, anything uncommitted — for when
they are let back on, stops their run on it, and moves every approved login
of theirs on it onto a robot they still work on. A user who has only ever
asked works on none, and may be let onto one before any login is approved.
An approved login is always on one of its user's robots, which the server
holds to whatever it is told, and reaches that robot's work and none of the
other's. Which robots a user works on, which one a login is moved to, and
whether a switch may happen are decided in the **core**. Class: `Robots`, in
the core.

**Switch** — `POST /robot?robot=<robot>`, from the robot picker in the
dashboard's header: the login moves to another robot its user works on, and
the page comes back on that robot's files, branch and simulator. A robot the
admin has not let them onto is refused (403), and the login stays where it
was. Only the login that asked moves, so one browser can be on Reginald while
another is on Nugget. The page waits for its save to land before it asks,
and a page whose login was moved under it — by another tab, or by the admin
taking a robot away — notices on its next look at `GET /me` and comes back on
the robot it is on now. A user on one robot alone sees it named, with
nothing to pick.

**User** — A teammate on the LAN who has logged in with a username. Users
reach only the user listener. A user is their username, whichever robot they
are on, so one user is one row of the listing however many times they log
in. Their **work** on a robot — the worktree, the branch and what is on it —
is owned by the pair of the username and the robot, so the work on each
robot is kept apart, and pulling, pushing or simulating on one never touches
the other.

**Session** — One login by one user, identified by a small integer **id**
the admin sees, and proven by a **token** the browser holds in the
`session` cookie as `<id>.<secret>`. A session moves through four
**states**: *pending* (asked, not yet decided), *approved* (may edit and
simulate), *denied* (the admin said no), and *revoked* (was approved, no
longer is). Only approved sessions reach files, builds, and the simulator.
A session carries what is its own alone: where it logged in from, how long
ago, the robot it is on (the one it asked for, until a **switch** or the
admin moves it), the file it has open, and its state, which the admin
decides per session at `POST /admin/logins/<id>/approve|deny|revoke`. A
revoked session is still a session; only a **delete** takes sessions away,
and it takes all of that user's at once.

**Editable set** — The files a **robot**'s users may edit: every one of
that robot's **own files** in the user's worktree, and on top of them the
files the admin has picked for that robot, each named by its **key**. The
admin picks for one robot at a time, and every `/admin/files` route names it
with `?robot=`; `GET /admin/files` lists what was picked, and a robot's own
files are never among them. A robot's own files cannot be picked, for it or
for the other robot, nor removed: `POST /admin/files/add` refuses one,
saying whose it is, and `/admin/files/remove` refuses to take one away. A user
may only ever name a file by exact match against this set; nothing a user
sends is resolved against the filesystem. The admin picks from the host
checkout; the key means the same path in every worktree, and a picked file
the user's branch does not have yet is listed all the same and answers 404
naming it. The admin page shows each robot's own package as always editable,
and marks the files of either robot's package in the picker as whose they are
rather than offering them. Class: `EditableSet`.

**Own files** — A robot's own package, which its users always edit without
the admin picking anything: every regular file under
`TeamCode/src/main/java/` and `TeamCode/src/test/java/` of its package
(`org/firstinspires/ftc/reginald/`, `org/firstinspires/ftc/nugget/`), and
of the packages under it, as they are in that user's worktree — so a file
that arrives there by a pull is editable from then. Found by walking the
worktree rather than by anything a user sends; a link is not followed, so a
link in the package, or a package that is a link out of the worktree, gives
nothing. Which directories are whose, and so what the admin may pick, is
said once, in the **core**. `GET /admin/info` gives each robot's as `own`.
Methods: `TeamRobot.javaPackage`; `TeamRobot.ownDirectories`;
`TeamRobot.owns`; `TeamRobot.picking`.

**Key** — A file the server has vouched for, named by its **root-relative
path** with `/` separators (`TeamCode/src/main/java/.../Plans.java`), and
the only kind of thing the server resolves against a worktree. There are
two ways to make one and no others: relativising a real path the server
found itself, and a root-relative string that is neither absolute nor
climbing out of the root — which is what the **editable set** and the
**source set** hand back when a user's string matches one of theirs
exactly. So a string off a request becomes a path only by being recognised,
and the rule above is held by javac rather than by the one `if` that used
to hold it, with a test that fails if a third way to make a key is ever
added. The same check runs over what comes back out of the **state
directory** and over the names git gives in a diff, since neither is more
trustworthy than anything else read off disk. Class: `Key`.

**Project root** — The repository checkout the coding server serves: the
top of a git working tree with a `develop` branch. All paths users see are
relative to it, and the editable set and the worktrees are stored per
project root. A user's save never writes under it; only git does, under
`.git`.

**Develop branch** — The permanent branch every user's work starts from,
pushes to, and pulls from: `develop` for Reginald's users and
`nugget-develop` for Nugget's, so wherever this glossary says `develop` of a
Nugget user, it is `nugget-develop`. The server requires `develop` to exist;
a repository without `nugget-develop` gets one at `develop`'s tip when the
server starts, said on its console. It never deletes or rewrites either: it
only adds merge commits to them. Getting `develop` to `main` is CI's: a **run
for develop** whose every step is green moves `main` to the commit it judged,
unless `main` is already past it; a run whose commits since `main` change a
workflow file holds `main` red, naming those files, until a maintainer pushes
`main` to the judged commit by hand and re-runs the failed jobs. Anything that
moves between the two robots' lines is the coach's job. `GET /git/status` names
the one a user's **ahead** and **behind** are counted against, as **develop**.

**Run for develop** — What CI does each time `develop` is pushed. Runs go one
at a time: a push that arrives during a run waits, and a run still waiting
when a newer push comes is cancelled rather than judged, so it is never red,
and the pushes that arrived meanwhile are judged together by the next run. A
run judges `develop`'s tip as it is when the run starts, not the commit whose
push started it, with the same commands as a local run:
`./gradlew spotlessCheck`, `python3 tools/run_tests.py` and
`./gradlew :TeamCode:testDebugUnitTest`. When every step of the judging is
green, `main` has no commit `develop` lacks and no commit from `main` to the
judged commit changes a workflow file, CI pushes the commit it judged to
`main` with the built-in token its checkout keeps: a plain push, never forced
and never a merge, so a `main` that moved meanwhile refuses it. A `main` that
already has commits of `develop` the judged commit lacks stays where it is, and
the run is green but leaves the issue as it is, open or not, since only a run
that judges the commit `main` stands at can say whether that commit is good.
Anything else is a red run, and `main` stays where it was. A red run opens the
issue **IMPORTANT: develop CI Failures**, or comments on it when it is already
open, naming the commit judged, whether its judging was green, and what held
`main` — the step that failed, a setup step or the report upload as much as a
check; the commits `main` has that `develop` lacks,
with the merge that brings them in; the workflow files that hold `main` for a
maintainer; or git's own words for a refused push — and the run; a commit that
reached `main` by itself keeps every run red until `develop` holds it. No step
reads a secret, and the built-in token may not move a branch past a change to a
workflow file, so when any commit from `main` to the judged commit changes a
file under `.github/workflows`, even one a later commit undoes or one a merge
changes in resolving it, the run pushes nothing and is red, naming those files,
a renamed one by its old name and its new. No commit to `develop` and no session
can clear that: it waits on a maintainer, who pushes `main` to the judged
commit by hand, never forced, and re-runs the failed jobs of the run. The next
run that moves `main`, or finds it already at the commit it judged, as that
re-run does, closes the issue, saying `main` now stands at that commit. A pull
request is still judged, in a queue of its own, and never moves `main` or
touches the issue. What a run decides is the core's, and the workflow only
gathers what the core needs and does what it says, which a test holds it to.
Files: `.github/workflows/tests.yml`; `tools/ci/develop_run.py`;
`tools/ci/develop_issue.py`; `tools/ci/test_workflow.py`.

**Which git** — The coding server is handed the git it works the repository
with, rather than making one. What a real git does is `RealGitTest`'s, over a
contract `FakeGit` passes too, and what worktrees do over a real one is
`WorktreesTest`'s; a test of the server itself takes the fake and forks nothing,
except where its question is one only git can answer. Whether a directory is
still a worktree is on that interface for the same reason: asked of the
filesystem, by looking for the `.git` marker git leaves, it was a question only
one of the two could be right about, and the other quietly remade the worktree
under whoever was saving into it. The fake keeps a file as text and refuses by
name one it cannot, since a model mangled in a worktree is a test passing for a
reason nobody meant.

**Worktree** — A git worktree of the project root's repository, one per
**username**, on its own **user branch**. All of that user's edits are
written there and every run they start is compiled from there. Owned by
the username, not the session, so logging in again after a restart or a
revoke-and-reapprove finds the same work. Made when the admin approves
the login, so git's refusal, if any, is the admin's to see. Class:
`Worktrees`.

**User branch** — `coding/<slug>` on Reginald and `nugget/<slug>` on
Nugget, created at the tip of the robot's **develop branch** when the
worktree is made. Saves are uncommitted changes in the worktree
until the user presses Commit.

**Delete** — `POST /admin/users/delete?username=<name>`, the Delete button
on the user's row: the user leaves the listing, with their work on every
robot. Every session they have is forgotten, so their browsers are logged
out, their benches and their children stop, which robots they work on is
forgotten, and their worktree directory on each robot goes. Their **user
branches** stay, and so does the mapping to them, so logging in again and
being approved rebuilds the same worktree on the same branch with everything
they committed, under the same slug, on the robot they are approved for.
Refused, changing nothing, while they have work a robot's `develop` does not
have — uncommitted files, or commits ahead, on any robot, including one they
were taken off — naming it robot by robot, until the admin asks again with
`force`; the page's Delete anyway is that second ask, so what the admin
reads is what the server enforced and not a warning the page worked out for
itself. The refusal is decided before the benches are stopped, so a user who
keeps their work keeps their runs.

**Commit** — `POST /git/commit` with a message: the **formatter** runs over
every uncommitted `.java` file in the worktree first, and then every
uncommitted change there becomes one commit on the user branch, authored
by the username. The reply names the files the formatter changed
(**formatted**), and the Edit tab reloads what is open so the editor shows
the formatted text; a file with unsaved typing is left alone and its next
save overwrites the formatting, which the next commit puts back. Nothing
to commit is a success that says so. An empty
message is refused. `GET /git/status` reports the uncommitted files
(**changed**), the commits the branch has that `develop` lacks
(**ahead**), the commits `develop` has that the branch lacks
(**behind**), and the branch's tip (**head**), which moves on a commit
and on any pull or push, whoever asked for it. It also says whether a
**push** would land the user's work (**pushable**), which is what the Push
button is offered by.

**Formatter** — palantir-java-format, run in the server's own process over
one file's text: imports ordered, the unused ones removed, then the text
formatted — the three steps, in the order, that Spotless's
`palantirJavaFormat` step runs in CI, at the one version
`TeamCode/build.gradle` names for both. So what a commit writes is what
`./gradlew :TeamCode:spotlessCheck` accepts, and a teammate's push never
fails CI on formatting alone. Shelling out to `spotlessApply` per click
would cost seconds of Gradle startup and would format files the user never
touched. Only Commit formats: a save writes exactly what the editor sent.
A file it cannot read or parse is committed as the user wrote it and named
in the reply's **warning**, since a commit is a save point and a save point
that refuses half-written code is no use. It reads javac's own trees, so
the JVM running it needs the `--add-exports` that `javacExports` passes;
the server checks it can format before it starts, rather than failing on
someone's first Commit. Class: `JavaFormatter`.

**Push** — `POST /git/push`: the user branch is merged into `develop` with
a merge commit, and then the user branch and worktree are fast-forwarded
to the new `develop`, so the user carries everyone's pushed work from
then on. Where `develop` is checked out, normally the host checkout, the
merge runs there so that working tree shows the pushed work; git refuses,
changing nothing, if an uncommitted edit there would be overwritten.
Checked out nowhere, only the branch moves. It needs a clean worktree
(commit first) and refuses, changing nothing, on a **merge conflict**.
A clean worktree with a commit `develop` lacks is **pushable**: the same
question the push asks before it merges anything, asked in `GET
/git/status` so the Push button is offered for exactly the presses that go
somewhere — said by the server, since the server is what enforces it, as
with **deletable**. A conflict counts as pushable: the press is how the
user is sent to their coach.
Then `develop` is pushed to the remote named `origin`, when there is
one, whether or not anything was merged, so origin is current whenever
the network allows; nothing is sent when origin already has it. A remote
push that fails (the robot's wifi has no way out; origin has moved on)
leaves the local merge in place, is never forced, and is reported to
the user and, with the command to run, to the coach on the admin page.

**Pull** — `POST /git/pull`: `develop` is merged into the user branch, in
the user's worktree, a fast-forward when it can be and a merge commit
otherwise. `develop` does not change. Uncommitted edits stay uncommitted
and ride along, except in a file the merge would change: those would be
overwritten, so the pull refuses (commit first), naming only those files
and changing nothing. It also refuses, changing nothing, on a merge
conflict. The Pull button pulses, and wears the count, while the branch
is behind. The admin can press Pull for a user too,
`POST /admin/users/pull?robot=<robot>&username=<name>`: the same merge in
that user's worktree on that robot, with the same refusals, so a teammate
who has walked away from a stale branch is brought up to date without their
browser. Each robot a user works on has its own Pull on their row, and a
robot they do not work on is a 404. Their editor
notices by the **head** in the status moving under it: a file with
nothing typed since its last save is reloaded; one with unsaved typing
is left alone, and its next save is the usual conflict with a reload to
offer.

**Merge conflict** — `develop` and the user branch changed the same lines
since they diverged. Not the save **conflict** (a stale base version on
`PUT /files`), which keeps its name. Push and Pull hit the same conflict,
since both merge the same two branches, and both find it with
`git merge-tree`, which touches no working tree, so either changes
nothing anywhere, names the files, and tells the user to ask their
coach. The admin page shows the coach the **recipe**, with the real path:

```
cd <the user's worktree>
git merge develop         # resolve the conflicts in an editor
git add -A && git commit  # then the user presses Push
```

The coach's merge *is* the pull, so Push is all that is left, and
`develop` is never checked out to resolve anything. The commands are git's
and the server writes them, like every other judgement it enforces; the
page prints the lines it is handed rather than composing them itself.

**Merge report** — How a **pull** or a **push** went, said once. It is
handed the merge, which of the two it was, whom it happened to and whom it
is being said to, and works out the rest: the status (200 when something or
nothing happened, 409 otherwise), the sentence in the right **voice** (the
user's own, or the admin's about them), the **recipe**, and the
**severity** — how bad it is, and the one thing a page needs in order to
draw it.

Severity is why it exists. Both pages used to work it out for themselves,
from the status code and the outcome's name, and they disagreed: a push
that landed but could not reach `origin` was a warning on one and was
folded in with conflicts on the other. Before that, the reply was a
six-parameter method two of whose parameters were finished English
sentences the caller composed, so adding an outcome meant touching the
enum, a switch, three callers' prose and two partitions written in
JavaScript. It is now a function of values, with no git and no listener in
it, which is also how it is tested. Classes: `MergeReport`;
`Worktrees.Remote.Outcome`.

**Slug** — The username lowercased, every run of characters outside
`[a-z0-9]` replaced by one `-`, trimmed of leading and trailing `-`, at
most 32 characters, never empty. It names the branch and the worktree
directory. Two usernames that collapse to one slug get `-2`, `-3`, … so
the mapping is decided once and stored, never recomputed. It outlives the
worktree: a deleted user's mapping stays, which is what gives them their
own slug and their own branch back when they return.

**Worktrees directory** — Where the worktrees live: under the state
directory, at `worktrees/<root name>-<8 hex of SHA-256(root path)>/<slug>`
for Reginald and `worktrees/<root name>-<8 hex>-nugget/<slug>` for Nugget,
so two checkouts on one machine keep their worktrees apart and the admin
can still read the directory name.

**Version** — The SHA-256 of a file's bytes, as hex. A read returns the
content, its version and the robot it was read on; a save carries the **base
version** it was edited from and that robot, and is refused as a
**conflict** (409, with the current content) when the file on disk has moved
on, whether from another browser or from an IDE on the host. A save for a
robot the login is no longer on is refused too (409, naming the robot it is
on now), writing nothing: a file both robots edit can be byte for byte the
same on each, so the version alone cannot tell the two apart, and a save
typed on one must never land on the other.

**State directory** — Where the coding server keeps what outlives the
process: `sessions.json` (each with the robot it is on), `users.json` (the
robots each user works on; a state directory from before it existed lets
each user onto the robots of their approved logins), `editable.json`,
`worktrees.json` (username to slug, path, and branch, per project root) —
Reginald's, and `editable-nugget.json` and `worktrees-nugget.json`
Nugget's — `settings.json` (what the admin
set: the **resolution** the pages draw and the **simulation constants** they
changed), the assets it
fetched, and the worktrees themselves.
It follows the XDG Base Directory convention:
`$XDG_STATE_HOME/midnight-snackers/coding-server`, else
`~/.local/state/midnight-snackers/coding-server`. Owner-only. Session
secrets are stored there only as salted scrypt hashes. A store that
exists but cannot be read stops the server from starting. Class:
`StateStore`.

**Editor bundle** — CodeMirror 6, built once by `tools/codemirror/build.sh`
into `codemirror.js` under the test resources and served by the user
listener at `/static/codemirror.js`. It is the one **static asset**: the
`/static` route serves only names on its allowlist. The page reaches the
bundle only through `window.CM`.

**Part** — One piece of the dashboard page that starts on its own: the
editor, the branch panel, the simulator, the launch sim, the header, the
file list, the tabs. A part that throws is caught, named in a banner at the top of the
page and recorded in `window.codingPage.broke`, and the others still
start — so a browser that cannot run one part of the page still gives the
rest, and never leaves the shell it was served as, which is what "nothing
to show you" looks like too. The page asks for nothing an **older
tablet's** browser has not got: an iPadOS 13 `MediaQueryList` carries
`addListener` and not `addEventListener`, and the page feature-tests it
the way CodeMirror feature-tests its own. Held by
`tools/browser/dashboard.mjs`, which opens the page on an engine with
those APIs taken away; see `doc/browser-tests.md`.

**Diagnostic** — A build problem shown in the editor itself, in the lint
gutter and under the text of the open file. The problems list under the
editor shows the same problems for every file.

**Navigator** — Answers, for a position in one of the worktree's main
sources, which **symbol** is there (its qualified name and kind), where
it is **defined** (a file, line and column in the sources, or nowhere
when it comes from the SDK or the JDK), and every **usage** of it across
the sources (a reference, never the declaration itself). The JDK's own
compiler answers, through the `com.sun.source` Trees API, from the same
kind of task the build runs, stopped after `analyze()`; no language
server, no extra process. Cached by the build's source fingerprint, so
an unchanged tree costs nothing. A file that does not compile still
answers for the parts that do. Lines and columns are 1-based and count
characters, never tab stops. In the Edit tab: F12 or Ctrl-click for the
definition, Shift-F12 for the usages, at `GET /nav/definition` and
`GET /nav/usages`. Class: `SourceNavigator`.

**Source set** — Every `.java` file under the worktree's main source
root, named by its root-relative key like the editable set. A user may
**view** any file in it, read-only, at `GET /source/<key>`, so a jump to
a definition lands somewhere the user can read; a user may **edit** only
the editable set, as before, and the file list still shows only that.
The source set is enumerated and matched exactly, never resolved against
the filesystem.

**Build** — Compiling a project's main sources as they are on disk with
the JDK's own compiler against the **libraries**, together with that
project's **simulator** — everything under its `TeamCode/src/test/java`
that is not a test, with the resources next to it copied along — cached
by a fingerprint of all three trees. The simulator runs in the child against the sources it was
just built with, so a simulator that does not fit them fails the build
naming the seam, rather than the child failing at run time with a
linkage error nobody can read. The Edit tab asks for a build after every
save and shows the **problems** (file, line, message); a problem in the
simulator has no file the user can open and says so in its message. In
the coding server the project is the user's worktree, so one user's
broken edit breaks only their own build, and a worktree on another
version of the code than the server runs its own simulator, not the
server's. Not the Android build: no Kotlin, no desugaring. Class:
`SimBuild`.

## The simulator

**Core** — `simcore`: the simulator's geometry and physics — the field, the
hives and flight, the robot's motors and the **drivebase** they turn its wheels
through, dead wheels, intake and launch, the
flowers' stacks and nests, and the **noise** a run's robot is drawn with — and
the bench's decisions about a run — how it ends, what the **watchdog** holds its
child to, and what Stop does — in a module whose build proves every function
total. What is left outside it is the rigid-body engine and what reads and
writes it. A project's own core is built with its simulator. Classes:
`GatesTest`; `SimBuild`.

**Drivebase** — How a robot's drive motors turn the four wheels of the rigid
body the simulator pushes, said once per robot in the core. Reginald's is
**mecanum**: a motor to each wheel, the back pair mounted mirror-image to the
front. Nugget's is a **tank**: a motor to each side turning both of that side's
wheels, the left mounted mirror-image to the right. A drivebase also names its
motors, in the order their powers are recorded, and says which of the noise
drawn per wheel each motor has. Classes: `Drivebase`; `TeamRobot.drivebase`;
`Drivetrain`.

**Bench** — The simulation core shared by the bench page and the coding
server's Simulate tab: the catalog, the runs so far, and the routes that
start a run and follow it. One run at a time per bench. The bench page
has one; the coding server makes one per worktree through a **bench
factory**, so one run at a time per user and two users may run at once,
each in its own child JVM, and a user's status lists only their runs. A bench
is for one **robot**, the user's, so a user's Simulate tab lists and runs
their own robot's op modes and never the other's.
Given a project it builds that project's robot and simulator before
every run, and refuses a project with no simulator of its own, since the
child would fall through to the server's; without one (the tests) it
runs on the current classpath. Class: `SimBench`.

**Waits** — How long a bench waits for each thing it waits for, said once by
name: a game's **autonomous period** and **TeleOp period**, both in simulated
seconds; the **kill grace** a child has after Stop; the **startup** the op
mode's time has to begin, which a child JVM's loading is inside; the **silence**
the child may say nothing at all for mid-run before it is killed for hanging;
how long a run may go **unwatched**; and the **view wait**, how long a run that
**waits for its view** waits for it. It is made from what the bench waits for a
person watching a run — a match's periods, room to load, thirty seconds for a
page to draw the field, and five minutes to come back to a run they left — and
said differently where a test means something different. Five bare seconds in a
row at a call site was five chances to hand the wrong one to the wrong wait, and
the silence was the one nobody could say at all, so a test of a hung op mode sat
through the real five. Every one but the periods is judged on the bench's own
clock, by the **watchdog**, so a test moves that clock rather than sitting
through a wait. Class: `SimBench.Waits`.

**Watchdog** — What a bench holds a run's child to while the run goes on: the op
mode's time must begin within the **startup**; once it has, the child must say
something at least every **silence**, and somebody must ask about the run within
the **unwatched**; and once the child is told to Stop, it must end within its
**kill grace**, which is then the only wait on it. A child **held** for its view
is judged by none of those: it is placed without its view once the **view wait**
is over, and its startup runs from its placing, since the time it was held was
the bench's and not its own. Each is one answer read off how far the run has got
— *building*, *held*, *starting*, *running*, *stopping*, or over — the time on
the bench's clock, and the waits, so each rule is tested with no thread and no
clock. The bench asks every tenth of a second until the run is over, and kills
the child when an answer says to, or places it when the answer is that its view
is late. Classes: `Watchdog`; `RunState`.

**Waits for its view** — A run started from the Simulate tab begins only once
the live view under it is **ready**: it has fetched and built the field it
draws, and the goal tags on it, and has drawn a frame of it, which is when the
browser compiles what it draws with. That is seconds of a page doing nothing
else — on the committed low model, drawn in software, a second and a half to
build the field and nearly two to draw its first frame — and a run already
under way was watched in stutters, the robot frozen and then leaping ahead to
catch up, and a TeleOp driven blind. So the bench builds the run and starts its
child as before, overlapping the page's loading, and **holds** the child's start
line until the view says it is ready; the op mode's time then begins with the
view already drawing, and the page says, meanwhile, that the run begins once it
has drawn the field. A view that is not ready within the **view wait**, thirty
seconds from pressing Run — a tab closed, a browser that never draws — is not
waited for longer: the child is placed without it, and the view catches up when
it can. Stop tells a held child as it tells any other, and it ends unplaced. A
run nobody asked to wait (`begin=now`, or nothing said) begins as soon as its
child can be placed, as every run did before; a child from before placement
places itself, so it is never held. `POST /run?opmode=<name>&begin=ready` and
`POST /runs/<id>/ready`. Classes: `View`; `RunState.Held`; `SimBench.Begin`.

**Game** and **free play** — The two ways a run is made, chosen per run: `POST
/run?opmode=<name>&mode=game`, or `mode=free`, which is also what a run that
says neither is. A game is played as a match is: an auto has the **autonomous
period** (30 s) and a TeleOp the **driver-controlled period** (2:00), and its
live view stands you where its drivers stand, in the **driver's view**. Free
play has **no limit**: an auto goes on until its plan is done and a TeleOp until
the driver presses Stop, and it is watched from wherever the viewer likes. The
child is told the one number either way, the period or `Infinity`, which is how
Java writes and reads no limit. The Simulate tab's **Game mode** box says which
the next run is, and the status says which each run was. Class: `SimBench.Mode`.

**Unwatched** — A run nobody has looked at for five minutes is stopped, and says
so in its message. Looking is anything asked about the run — its page, its
ticks, its log, a press of the controller — and a live view asks several times a
second while it is open, so a run is unwatched once no page has it open. Free
play has no end of its own, and one that somebody walked away from would
otherwise hold a child JVM and grow its ticks in the server, which every user
shares, until one of them ran out of memory. Method: `SimBench.Run.lookedAt`.

**What builds and starts** — `SimSources`: the classpath this JVM runs on, a
project on disk, or, in a test of the bench itself, `FakeSources`, which is
neither and need not be. The bench is handed one rather than picking it, so
everything the bench does with a build and with a child runs without a project
copied onto disk, a compile, or a JVM. What a real build reports is
`SimBuildTest`'s, what a real child prints is `SimChildTest`'s, and what the
handshake decides is `SimRunStreamTest`'s; the bench's own tests hold what the
bench does with each. A `FakeChild` is handed the lines a child would print,
written by `SimRunStream`'s own writers, so the fake speaks the protocol by
construction rather than by a string somebody typed.

**Child** — The JVM a run, or a listing of the catalog, happens in, and
where a teammate's robot code runs; the server never runs it itself. It
is fenced against the accidents robot code makes on a laptop, not
against anyone trying: it stands in its own **scratch** directory, which
is its working directory, home and temp and goes when it ends, so a file
written by a relative path never lands in the host checkout; it is given
none of the server's environment beyond what a JVM needs to start; its
heap is capped, and it exits rather than limping on when that runs out;
whatever it starts is killed with it — by the server when it is killed,
by the child itself when it ends any other way, and by the server's own
exit for any still running then; and no more run at once than the
machine has processors, one `JvmChild` being shared by every user's
bench, so a start past that is refused and said. Classes: `JvmChild`;
`ProcessTree`, which reaches `ProcessHandle`, missing from the
`android.jar` the tests compile against.

**Status** — What the bench is doing, as one moment: the runs newest
first, each taken in one hold of that run's own lock, and **running**
read from the newest of those same snapshots — it is running exactly
when that run has no outcome yet. Asking the run again would let one
that finished in between be reported as running and done at once, which
is a moment that never was and what the Simulate tab would then draw.
Method: `SimBench.statusOf`.

**Libraries** — What a project is built against, navigated against, and
run with: the jars on the server's classpath, and none of the server's
own code. The server's code is the directories on its classpath (its
simulator and tests), the jar its robot classes come from, and the jar
its **core** comes from; the FTC
SDK, Road Runner, and the FtcRobotController module's jar, which no
project rebuilds, are libraries. So a class a project lacks is missing —
in its build, in its child, and to the navigator — rather than quietly
the server's. Method: `SimBuild.libraries()`.

**Catalog** — The op modes the simulator can run, exactly as the robot
controller lists them for one robot: every concrete op mode of that robot's
own kind, under that robot's own package, that carries the
`@Autonomous` or `@TeleOp` annotation — Reginald's `opmode.OpMode` under
`reginald`, Nugget's `NuggetOpMode` under `nugget` — plus whatever a
**registrar** (a
static method annotated `@OpModeRegistrar`) registers, found by calling
the registrar itself. The autos are registered that way: each `@Auto`
annotation on a plan method in `Plans` is one op mode, with no class of
its own. Entries are keyed by the op mode's **name**, which the robot
controller requires to be unique, and carry a **kind** (*auto* or
*teleop*), **where** a person finds the code (a class, or the plan
method), and the **alliance** it plays for (*Blue*, *Red*, or none for a plan
relative to wherever the robot starts). An op mode says its alliance only once
it is made, so the catalog makes one to ask; one that cannot be made is listed
all the same and says why when it is run. A listing from a child from before
alliances plays for none. Nested classes are never listed, so test op modes
stay out of the real bench, and Road Runner's vendored code is not ours to
simulate.
The child reports the catalog after each build, so a newly written op
mode appears without a restart. Class: `SimCatalog`.

**Child** — The fresh JVM each run executes in, launched on the newly
built classes — the project's robot sources and its simulator, built
together — and the libraries, nothing else. Every class identity is
consistent, static state starts clean, and a hung op mode is a process
that can be killed. It is told which **robot** it is with `--robot=nugget`, and
is Reginald when it is told nothing, which is what every child was before
there were two. It says its **protocol** first, builds that robot's catalog,
waits to be **placed**, then prints the run stream; the op mode's own output
goes to stderr, after a line naming the robot the run is on, and which of its
imperfect selves. Its standard
input is the driver station, one line at a time: the first places the
robot and says which robot (its seed, or none for the exact robot), and
until it comes Stop ends the run unstarted; when the input ends, the run ends stopped. Class: `SimChild`.

**Run stream** — The lines the child prints, one JSON object per line, in
order: which protocol it speaks, one that the op mode's time has begun,
each tick as it happens, and finally the outcome. Written and read in one
place, and every way a run can end is named there. But the child runs the
project's simulator, which may be another version of this code than the
bench, so the lines carry a **protocol** version: the bench refuses, by
name and with whose the fix is, a child newer than itself or older than
the oldest it still reads, rather than misreading it; and a test pins
what a child of that oldest version printed. A child from before the
version line prints content first and is that oldest version. The
version also says what the child reads: from version 3 a child waits to
be placed, so the bench places it first thing; an older child places
itself at the origin, so the bench lets it run when that is the start
pose and refuses it by name, with the fix, otherwise. From version 4 the
start line may carry the op mode's seed and the child runs the robot
drawn from it; an older child runs the exact robot whatever it is told,
so the bench lets it run when that is the op mode's robot and refuses it
by name, with the fix, when a seed is set. From version 5 the child simulates
whichever robot it is told; an older child simulates Reginald alone, so the
bench runs Reginald on it and refuses Nugget by name, with the fix, which is
to pull `nugget-develop` — or, when that is older too, for the coach to merge
`develop` into it. From version 6 the start line may carry the **simulation
constants** the admin changed, and only those; an older child runs on the
constants it was built with, so the bench lets it run when none were changed
and refuses it by name, with the fix, when any were: pull the robot's develop
branch, or have the coach put the constants back as built. A tick's
line is also the form the replay page reads, so a run the bench knows
only by its lines is the same page the child wrote from its own
recording: the page reads either **source**. Every one of those decisions —
which protocol the child speaks, whether this run can be made on a child of
that version, what to send it, and the refusal with whose the fix is — is
one answer read off the child's first line, the **handshake**, and is made
where the versions are defined rather than by the bench that asks. Class:
`SimRunStream`; `SimRunStream.Handshake`; `SimReplayPage.Source`.

**Run** — One execution of one op mode on a fresh simulated robot, on the
simulation's own clock: the world moves one **loop period** between one op
mode loop and the next, whatever the machine is doing, so a run is the
same tick for tick every time it is made. The period is 20 ms exactly on
a robot without noise, and on a robot with **noise** it varies the way a
real loop does, the same sequence for the same seed. A run's **pace** says whether it is held to real time:
a run somebody drives or watches live is (`REAL_TIME`), a run nobody
watches, such as a test, goes as fast as the machine can (`FASTEST`).
A run has a **phase** (*building*, *starting* while the child JVM
loads the catalog and, for a run that **waits for its view**, until the view is
ready, *running* from the moment the op mode's time begins,
*finished*), a
**message** (compile errors, or why it was killed or stopped), and when
finished an **outcome**: *done*, *stopped*, *timed out*, *build failed*, *wrong
protocol*, or *child exited with code N*. It starts from the op mode's
**start pose**, on the op mode's **seed**, as they were when the run was
started, and it is a **game** or **free play**. An **auto run** is done when
its plan is, and in a game times out when the plan is not done within the
autonomous period (30 s of simulated time). An op mode **runs a plan**, whichever
robot's it is, by saying whether its plan is done and which step it is on, and
the run records that step every loop; Reginald's autos do, and so does
Nugget's **RoadRunner example**. A **TeleOp run** has no plan: it is
driven from the controller until the driver presses Stop, or in a game is done
when its **period** is over (2:00, a match's driver-controlled period). In free
play neither kind has a limit. Either kind is *stopped* when the driver presses
Stop, and by the bench when it goes **unwatched**. Whether a run loops again,
and if not how it ends, is asked before every loop, of whether Stop was pressed,
where the plan has got, and the simulated time against the run's **budget** —
its period, or no limit. A run that times out says so in its outcome like any
other ending; only a test of an auto, through `SimRunner.run`, fails for it.
Classes: `SimRunner`; `SimRunner.Pace`; `Ending`; `Budget`; `RunsAPlan`.

**Stop** — What pressing it does depends on how far the run has got. A run still
building ends there, stopped, since there is no child to tell, and the child its
build then starts is killed rather than left waiting to be placed. A run with a
child, held for its view or not, tells it once, and its **kill grace** runs from
then; a child that cannot be told is killed. A run that is over, or already
told, is left as it is. Class: `Stop`.

**Driver station** — What the driver does during a run: the state of the
two **gamepads** and the **Stop** button; and before it, where the robot
is placed, the **start line** the bench sends first and the child waits
for. The controller page sends the gamepads and Stop to
the bench (`POST /runs/<id>/gamepad`, `POST /runs/<id>/stop`), the bench
checks it and relays it to the child on standard input, and the runner
copies each gamepad's state into the op mode's gamepad before every loop
through the SDK's own packet copy, so `crossWasPressed()` and the other
edge detectors behave as on the robot. A state holds until the next one
replaces it, like a real gamepad. Class: `SimDriverStation`.

**Gamepad state** — One gamepad's inputs as a value, carrying only what
is not neutral, named as the SDK names its fields: the PlayStation names
for the buttons (`cross`, `dpad_up`, `left_bumper`, `share`, ...), the
sticks as `left_stick_x` and the like, and the two triggers. A stick
pushed forward reads negative y, as on the robot. A name that is not a
gamepad input is refused, by the bench (400) before it reaches the run,
and by the child (the run's outcome) should it ever get there. Class:
`SimDriverStation.State`.

**Controller** — The PlayStation-style gamepad drawn under the live view
of a TeleOp run. Every input is a button to click or a stick to drag, and
every one is labelled with its keyboard key; a key held is a button held,
a stick's four keys are its directions. The page shows and sends one
gamepad at a time, **gamepad 1** or **gamepad 2**, picked with `1` and `2`;
Stop is `Esc`. After the run the controller replays what the driver
pressed, tick by tick, and the keys go back to play/pause and stepping.

**Tick** — One entry in a run's recording, one per op mode loop: the true
pose, the plan's current step (empty for a TeleOp), the drive powers — one per
drive motor, in the order the robot's **drivebase** names them, `LF`, `RF`,
`LB`, `RB` on Reginald and `L`, `R` on Nugget, which is how the replay page
labels them — the
dashboard packets drawn that loop, where the balls are (each as x, y, z, or
nothing for one held in the robot), how many balls the robot **holds**, how
many each alliance has **scored**, and for a TeleOp what each gamepad read.
Class: `SimRecording`.

**Golden trace** — What the robot code wrote to the four wheel motors,
tick by tick, kept in a file under
`TeamCode/src/test/resources/.../sim/traces` and compared against the next
run. A run is the same tick for tick every time it is made, so the powers
are too, and a change in how any intent reaches the wheels — the mixing,
the saturation, the feedforward, the voltage compensation — arrives as a
diff with a tick number on it. It holds only what the code **commanded**,
never where the robot went: the pose is the physics engine's answer to the
powers, and a rigid-body simulation compounds a difference in its last
digit into a visibly different path. For the same reason a trace is a
prefix of a run, long enough that every intent runs many times over and
short enough that a difference too small to be ours has not grown into
one. A trace is a change detector, not a judgement: it cannot tell a
regression from a change somebody meant, so an intended diff is read and
then **regenerated**, and a regenerating run fails, because a run that
wrote the answer down has not checked it. A missing file fails too, rather
than passing for want of anything to compare. Classes: `SimTrace`;
`WheelPowerTraceTest`.

**Drive intents** — The test TeleOp a golden trace of the driver's side is
taken from: it gives the drive each axis alone, then all three at once
hard enough that a wheel saturates, then a pose to steer to, on a schedule
of its own rather than from a gamepad, so a run of it is the same tick for
tick. Between it and an auto that follows a trajectory, every way an
intent reaches the wheels is traced. Class:
`TestTeleOps.DriveIntentsTeleOp`.

**Replay** — A run's ticks written as a single self-contained HTML page
with the field, the true pose, and play/pause/scrub controls, named after
the op mode under `TeamCode/build/sim`. The field is the **flat drawing**.
Class: `SimReplayPage`.

**Flat drawing** — The field drawn into a 2D canvas by the page itself, in
three dimensions from a camera that orbits it (drag to turn, scroll to
zoom, double-click for the audience's view): the walls at their height, the
field elements, tape and game pieces of the collision model, the hives
where each tick says they lean, the balls the simulator moves where each
tick puts them, the robot as a cube turned to its heading, and the
dashboard's field overlay projected onto the floor. The obstacles are
outlined on the floor. The model and the sizes are the simulated robot's,
so it draws what the simulator collides — and it draws it out of what the
page already carries, which is why it is what a page with nowhere to fetch
from draws.

**Live view** — The same page in live mode, following a run while it is still
adding ticks, and drawing the **field scene** rather than the flat drawing. It
is the *only* page that draws the field: what used to be a second page at
`/sim/field` is retired, and what it could do the live view is asked for in
its query string — `?view=camera` for the **webcam's view**, `?view=flat` for
the flat drawing, `?cost` for **what a frame costs**, `?resolution=low|medium|high`
for another than the **resolution** it draws by default. The dashboard passes
them through, so `#simulate?resolution=low` reaches the view it embeds. All of
it is also **under the field**, because what you are looking at is a thing to
change while looking at it rather than a sentence about what somebody put in a
query string: one selector for *drawn as* — the **colliders** first, then the
three resolutions, cheapest first — and a box for **measure rendering cost**,
which is `?cost` by another name and measures whichever model is drawn.
Choosing navigates, carrying everything else asked of the page along. The
controls sit apart from the page's messages, since a cost reading or a model
that would not load writes over a message and must not write over them. The
box is unavailable on the colliders, which are drawn on a 2D canvas with no
renderer to ask for a frame. Classes: `SimLiveServer`; `SimBench` serves the
same page per run. Files: `webcam.js`; `framecost.js`.

**Driver's view** — How a **game** is watched: from where its drivers stand,
the middle of their **alliance area** five feet up, **watching the robot**. A
driver turns their head — drag — to look somewhere else, and does nothing else:
no orbiting, no zooming, no walking round. An op mode that plays for no alliance
is watched from blue's, since it plays the blue way round. The bench works out
where, from the field, and tells the page as the run's **match**, with the
period; the page stands there in the field scene and in the colliders alike, and
its clock says how long the game has left.
The colliders are drawn by projecting points the page works out itself, and from
a driver's area some of them are behind the eye — the wall they stand behind,
the floor at their feet — so the page cuts each shape where it crosses a plane
just in front of the eye. Free play is watched from the orbit, as every run was
before there were games. Methods: `SimBench.Run.match`; `FieldScene.standAt`.

**Watching the robot** — Where a driver looks until they look somewhere else:
at the middle of their robot, wherever the tick shown has it, so the view turns
to follow it as it drives, as it plays and as it is scrubbed. Before there is a
tick there is no robot to watch, and the view looks at the middle of the field.
A driver who drags to look elsewhere keeps looking there while the robot moves;
double-clicking watches the robot again. While it watches, where the view looks
is worked out from the tick shown and nothing else, so scrubbing back to a tick
sees it as it was seen the first time.

**Webcam's view** — The goal tags as the robot's camera would see them: the
tags where they are, in the perspective the lens gives, and nothing else. The
tag artwork is drawn on the field whatever the view, since it is printed on
the goal; asking for the webcam is what swaps the camera for the lens and
hides everything that is not a tag. What the lens is and how big the printed
tag is are assumptions written at the top of `webcam.js`, and must be measured
before a pose read off these frames means anything.

**Field scene** — The field as it looks — `field.glb`, the Onshape
assembly's own tessellation — drawn with three.js by `fieldscene.js`, which
is the one renderer of it: the field page and the live view both build
their picture from it and add their own. It needs the assets, so a page
gets it only when it is served somewhere they resolve.

**Shading** — What of the CAD's own surface a model carries, and the second
thing **resolution** decides. *A colour apiece* is the lowest model, as it
always was: no normals, and one flat colour to a part. *As the CAD drew it* is
what medium and high carry: **normals** and whole materials. A normal says which
way a face points, and which edges are sharp and which are smooth is the
CAD's to say: a page that computes its own averages across every face a
vertex belongs to and rounds the sharp ones off. Normals are turned by the
frame change and scaled by none of it, and two triangles meeting at a hard
edge share the point but not the normal, so the fold survives being written
out. Whole means the export's material as it came — name, base colour,
metallic and rough factors, alpha mode, double-sidedness — rather than a
colour a part is painted with. They roughly double a model, which is why
only the model that is already the expensive one carries them: 20 MB became
42, and the cheap one is 4 MB either way. The export carries **no UVs and no
images**, so there is nothing of that kind to keep; the goal tag artwork is
drawn on quads of the page's own, by `webcam.js`.

**What the page supplies** — Whatever the model did not. A model with no
normals gets computed ones and the page's own roughness and metalness, which
is what the cheap model has always been drawn with; a model that brought its
own gets nothing painted over it. The page reads which by asking the
geometry, never by asking which file it loaded.

**Game pieces** — Drawn from the full model where there is one. The
simulator says where every piece is each tick, so a piece has to be an
object the page can move, which the **batched** field is not; so one of each
kind is kept as it came from the CAD and cloned for each piece the simulator
reports. The shape is the CAD's, the size is scaled to the radius the
simulator gives, and the colour is the simulator's — which alliance a nectar
belongs to is not something the one CAD nectar can say. Out of the cheap
model they stay spheres: its pollen is snapped and flat-shaded, which a
sphere draws better and cheaper.

**Drawn floor** — The dark plane the field scene puts under everything, and
scenery rather than geometry: the field's own floor plane is z = 0, where the
tape lies and the game pieces rest. The drawn floor sits `FLOOR_DROP_IN`
below it, because **high resolution** brings the CAD's soft tiles whose top
surface *is* z = 0, and two opaque surfaces in one plane fight for the same
pixels — the floor came out in radial slivers, worse the further out the
camera. How far below is derived rather than chosen: a 24-bit depth buffer
resolves about `z² / (near · 2²⁴)` at distance z, which at the far end of the
orbit is about 0.06 in, and the drop is three times that and 0.14% of the
field. The browser check recomputes that quantum from the camera and the
orbit rather than pinning the number, so moving the camera's near plane moves
what the floor must clear. CI draws the **stand-in**, which has no tiles and
cannot show the fight; what it holds is the rule that prevents it.

**Batched by material** — The field's 305 parts share 11 materials, so the
scene merges the geometry of the ones that share into a mesh apiece rather
than drawing a part at a time. What cannot be merged says so by what is asked
of it: a goal tag is read back by name to work out where it hangs, and
anything that moves or hides on its own — a hive's parts, the tape, the game
pieces — merges only within the group that moves it. A merged mesh keeps what
it was made of in `userData.from`, so the scene can still be asked whether the
perimeter is in it, and a merge that lost a triangle fails the load rather
than drawing a field quietly missing parts.

**What a frame costs** — What the field scene costs to draw, measured rather
than guessed: the **measure rendering cost** box, or `/sim/runs/<id>/?cost`
which is the same thing, takes the reading in whatever browser
opens it, which is how a tablet is measured, and `node tools/browser/cost.mjs`
takes the same one here. A reading says how long one frame takes to **submit**
— the CPU's share — and how long to **finish**, which is the GPU's, with and
without the shadow pass; and how many **draws** the frame really made, counted
at the context rather than asked of three.js, whose own count is the colour
pass and so misses that the shadow pass is nearly half of them. The probe
renders flat out rather than once a refresh, waits for the GPU with a one-pixel
read, and doubles the renders in a block until the block outlasts the clock's
resolution — and when even the cap is too quick it says it could not judge,
because a measurement nobody could take must not read like one that was.
Counts are pinned in a **budget** and times are only ever printed: a count is
the scene's and a time is the machine's. Files: `framecost.js`;
`tools/browser/cost.mjs`; `tools/browser/scene-budget.json`.

**Drawn in software** — What this container and CI always do, having no GPU
between them, and what every time they report is a time of. A reading says
which it was, and the two are never allowed to be confused: the page marks
itself when it is drawn in software, `cost.mjs --gpu` fails rather than hand
back SwiftShader's times as a GPU's, and a run without `--gpu` that finds
itself on hardware fails too, since the correctness checks were promised a
software rasteriser to come out the same on two machines. One rule tells them
apart — `inSoftware` — and a table of real renderer names holds it to it. The
counts do not care, so a budget is checked on whatever ran; only the times need
a GPU to mean anything, which is a laptop's job or the tablet's own.

**Budget** — `tools/browser/scene-budget.json`: the draws, colour-pass calls
and triangles one frame of the field scene makes. The same deal as a **golden
trace** — a change detector, not a judgement — so a missing budget fails rather
than passing for want of anything to compare, a regenerating run fails because
a run that wrote the answer down has not checked it, and an intended change is
read and then regenerated with `--regenerate`.

**What the suite costs** — What a run of the unit tests spends on the few
things that are dear: a **child JVM** started, a **compile** of a project's
sources, a **git** process, a session secret put through **scrypt**, a file
**formatted**, a **node** process, and a **project copied** onto disk. Each is
counted where it is spent, inside the one adapter that reaches it, so the
ledger is complete for the same reason nothing else may reach any of them at
all: `CostIsCountedWhereItIsSpentTest` pins who may reach each one and holds
each of them to counting it, off the bytecode, where a fully qualified name
cannot dodge either half. `./gradlew :TeamCode:testDebugUnitTest` prints the
ledger and the slowest test classes every run, and a run of one class prints
the ledger too, so what any one test class costs is one command:
`./gradlew :TeamCode:testDebugUnitTest --tests '*SimBenchTest*'`. That holds
for a class that spends nothing counted as surely as for one that does: the
test JVM is started with `Cost` as its agent, which is what writes the ledger
up on the way out, so whether it is written never waits on some test having
touched it — and `CostTest` holds the build to starting it that way.

The counts are pinned in a **suite budget** and the times are only ever
printed: a count is the suite's and a time is the machine's. The same deal as
the scene **budget** above — a change detector, not a ceiling, so fewer misses
it as surely as more, a missing budget fails rather than passing for want of
anything to compare, and a regenerating run fails because a run that wrote the
answer down has not checked it. A ledger is one JVM's, so a run of part of the
suite, or of the suite split across JVMs, says it **could not judge** rather
than passing.

A count that is the size of the codebase rather than what the tests do with it
is printed and not pinned: every Java file in the project goes through the
formatter once, so that one would move on any commit that adds a file, and a
gate that fires on every such commit is one people turn off. The report stars
the ones it only prints, and a budget line for one of them is refused, since it
would judge nothing. Files: `Cost`; `suite-budget.json`.

**A file of the repository** — A **suite budget**, a **golden trace**: written
where somebody reads it in a diff, and holding nothing secret. None of them goes
through the **state directory**'s store, which makes the directory it writes
into owner-only because session secrets live there — right there, and wrong on a
source tree, where it leaves a checked-out directory only the person who last
regenerated something can read. Class: `RepoFile`.

**Refresh assets** — The admin page's button, and what a coding server does
for itself at startup when it has fetched none: ask Onshape for the field
assembly, build the field as it looks, fetch the five images the CAD cannot
carry, and write them under the server's state directory. Nothing is written
until every one has arrived and been checked, so a rename upstream stops a
refresh rather than leaving half of one behind. The coding server serves what
it fetched and falls back, asset by asset, to the **stand-in** committed for
the tests — so a server that has never fetched still draws, and the admin page
says which of the two it is drawing. The fetch is `main`'s to start rather than
the server's, so no test that builds a coding server reaches the network.
Classes: `Onshape`; `Gltf`; `FieldGlb`; `FieldAssets`; `CodingServer.Assets`.

**Download** and **build** — The two halves of getting a field model, asked
for separately because they cost such different things. A download is tens of
seconds and eleven megabytes of Onshape; it fetches the assembly and the
textures and **keeps the export** at `export.gltf` in the state directory. A
build is arithmetic on what the download left behind, under a second, and is
handed no Onshape at all — so a build that finds nothing downloaded says so
rather than quietly fetching, and javac holds that rather than a comment.
Changing the resolution, or rebuilding after the pipeline changes, is a build
and not another download. **Refresh** is still there and is the two in one,
which is what a server with nothing fetched runs at startup. At
`POST /admin/assets/download`, `POST /admin/assets/build?resolution=…` and
`POST /admin/assets/refresh?resolution=…`, with a button apiece on the admin
page, and `POST /admin/assets/default?resolution=…` for the **resolution the
pages draw**. A build asked for no resolution in particular builds that one,
and so does the startup fetch: a server set to draw the cheap model does not
fetch the dear one for nobody to look at.

**Resolution** — Which field a page draws, in three steps, each adding one
thing to the one below it:

| | parts | triangles | bytes |
|---|---|---|---|
| **low** | 305 | 316,000 | 4.0 MB |
| **medium** | 305 | 1,020,000 | 27.5 MB |
| **high** | 1,029 | 1,474,500 | 41.7 MB |

**Low** is the field as it plays: the hardware dropped, snapped to a twentieth
of an inch, a colour to a part. **Medium** is those same parts at the points
and in the materials the CAD gave them — the jump in triangles is the snapping
let go of, and the jump in bytes is **normals**. **High** adds back every part
the export holds, which is the two thirds of the assembly that holds the field
together and that nothing draws once the field is up.

**Which one a page draws when it asks for none is the admin's**: *Pages draw*
on the admin page, kept with the rest of the server's state so a restart draws
what was set. It is **high** until somebody sets another — a field that does
not look like the field is the thing worth avoiding, and whoever cannot afford
it is the one who knows that — and a query string still says otherwise for one
page with `?resolution=low`, `medium` or `high`. The setting reaches the pages
as the one line they import for it, `field-default.js`: a coding server writes
that line from the setting, every other server serves the copy committed beside
the models, and the line is `FieldAssets`' to write either way, so what a page
draws and what a server builds cannot drift apart by being said twice. A page
that asks for one nobody built draws the low one **and says on the page that it
fell back** — which, on a server that has built nothing, is every page, said
out loud rather than quietly. The **build** makes any of them, or `all`, from
one **export**, and keeps them side by side, so changing resolution on a
tablet is a query string rather than another fetch.

The two things resolution decides are `FieldGlb.Keep` — which parts — and
`FieldGlb.Shading` — how much of their surface. Both are named at every call
rather than defaulted, so a model that quietly drops two thirds of the CAD, or
its normals, cannot be built by forgetting to say.

**Stand-in** — The `field.glb` in the repository: the model the tests draw,
frozen, so the browser checks and the **budget** come out the same on any
machine with no network. It is not what a refreshed server draws, and it is
not regenerated — a revision of FIRST's CAD reaches a page by being fetched,
not by being committed. The collision model `field.json` is different again
and stays committed and pinned: it is an input to deterministic physics, which
a **golden trace** is taken against, so it may not change under a running
season without somebody meaning it.

**Where the assets are** — What a served page is told, and a written one is
not: the base its `field.glb`, its `fieldscene.js` and its three.js resolve
against, relative to where that page is served (`assets/` from the live
server's root, `../../assets/` from the bench's `/runs/<id>/`). Being told
is what makes a page draw the field scene; being told nothing is what makes
the replay file self-contained. A page told of assets it then cannot fetch
falls back to the flat drawing and says on the page why, rather than
showing an empty field. `?view=flat` asks for the flat drawing outright.
Each server's base is checked against its own routes rather than asserted
in prose: `SimLiveServerTest`, `SimBenchTest`.

**Devices** — A robot's devices as fakes, and a clock: everything the robot
code can reach through its **Hardware**, and nothing that moves on its own.
A motor reads back what was last written to it, and time passes only when
someone says `advance`. This is the second adapter at the hardware seam —
the **simulated robot** is the first, and holds one of these, writing its
sensors from where the physics put the robot. A test of one subsystem wants
neither the physics nor the field (a turntable turns because its own motor
says so) and wants time it can move by hand; before there were two, both
came only as a side effect of building a rigid-body world, so a test of the
intake loaded the ball model and a launcher waiting a tenth of a second
moved the balls to get there. Class: `SimDevices`.

**Placement** — Where the field lets a robot be: a pose beyond a wall or
inside an obstacle comes back pushed against it, at the heading it was
given, clear of the obstacles first and inside the walls second so a robot
pushed out of an obstacle at the wall still ends inside the field. It is
the season's field and an eighteen-inch square and nothing else — no world,
no bodies, no time, no balls — so the **placement page**, the run stream's
hive tilts and the replay page's sizes cost a polygon overlap rather than a
rigid-body engine.

That it is only geometry is the point, and a test holds it rather than a
comment: it loads the class with dyn4j and the **simulated robot** both
forbidden and makes it answer anyway, and checks the same loader still
refuses the simulated robot, so the gate cannot pass by being toothless.
Living inside `SimRobot`, it meant that dragging the robot on the placement
page loaded fifteen hundred lines of simulator and twelve dyn4j classes to
clamp one pose. Classes: `Placement`, in the **core**; `SimPlacement`;
`PlacementTest`; `SimPlacementTest`.

**Simulated robot** — The robot, the walls, the field's obstacles and the
balls as rigid bodies in a **dyn4j** world, driven by the model Road Runner
was tuned with: each wheel's motor, at its commanded power, pushes its wheel
toward the speed the tuned kS and kV give, at the rate the tuned kA
allows, so the robot takes time to get up to speed and to stop, and its
true pose comes out of the engine integrating that push against whatever
it runs into. The localizer's sensors are written back from the true
pose, so the dead wheels read nothing while the wheels spin against a
wall. The robot is an 18-inch cube; a wall or an obstacle stops it and
lets it slide along, a robot pushed into something off-centre pivots on
it, and a robot placed beyond a wall or in an obstacle is placed against
it, at rest. The balls are pushed ahead of the robot, roll on with the
speed they were given, slow to a stop, and stop at the walls, the
obstacles and each other with a little bounce; a ball pinned against a
wall stops the robot short of it, since nothing goes through anything.
A ball grips whatever it touches, where the robot and the field slide on
one another: a ball the robot presses against a wall and then drives along
it rolls along between the two, slipping on neither, so at half the
robot's speed. Gripping sets a ball spinning, and a ball is at rest when no
part of it, centre or surface, moves faster than the **rest speed** (half an
inch a second), so a spinning ball stops when a rolling one would.
The robot starts with four pollen (its **preload**) in its **hopper**,
which with the three nectar a hive is set up with is enough to fill one.
Four is also all it **holds**, hopper and chamber together: while the
**intake** is on, a pollen that touches the front of the robot — the front
face, anywhere across its width — goes into the hopper, until there is no
room for another, so a robot that starts preloaded takes nothing in until it
has launched. A nectar is the bigger ball and no intake of ours takes one, so
a nectar the robot meets, and a pollen it meets with the intake off or with
no room, is a ball it pushes.
The launcher is on the turntable, and its gates feed it as the robot
code drives them: with the top gate open a ball drops from the hopper
into the **chamber**, and with the bottom gate open the chambered ball
drops into the flywheel and leaves at a speed set by the flywheel's, in
**flight**. The flywheel turns as fast as the code commands it, up to its
motor's top speed of 2,800 ticks a second either way (a goBILDA 5203 at 6,000
rpm with no load, 28 ticks a turn), and the throw is never set so high that the
fastest ball it throws from there, off the fastest robot the noise allows, is
one a flight refuses: so every launch leaves the robot, and one a flight did
refuse would stop the run saying why. The model is planar apart from flight and the flowers'
stacks: only the robot's footprint collides, nothing goes over a wall or
under a hive by being low, and a ball in flight meets what a **shot**'s
ball meets — the floor, the walls, every element of the field, the hives
as they turn and the other balls in flight — and what stands in a flower's
bore, its stack and any ball on the floor whose centre is in the bore, but
never the robot or the other balls on the floor. What a body on the floor
meets is what it can **reach**: a
ball rolls under an obstacle whose underside clears it — a flower's pipes
begin four inches up — and the robot, being eighteen inches tall, runs into
the same obstacle. No
sensor noise, ever: the sensors read exactly what the robot did, as the
hub would report it, which for an encoder's velocity is to the nearest
20 ticks per second (Road Runner's encoder wrapper reads the remainder
as a count of the hub's 16-bit overflows, so anything finer is read as
tens of inches a second). The
launcher's throw and the turntable's speed are guesses until measured on
the robot. The throw, a ball's speed for each tick a second the flywheel
turns, leaves the launcher with no spin. It is built at 199.4 in/s for the
code's close launch of 1050 ticks a second, and that throw and one 2.1 in/s
slower or faster each score and stay in every shot the tests score from the
launch distance: a close shot for each alliance, one that comes to rest
against the back of the cell, the shot after a tip, and the preload's four
launches at the code's own cadence into an empty cell, all four coming to
rest in it. The throw is a steep one, into a mouth a hive holds five feet
up. Both are **simulation constants**,
so whoever measures them sets them on the admin page. The world moves in steps of at
most 5 ms whatever the loop rate, so nothing is jumped over. Classes:
`SimRobot`, which holds the engine; `Drivetrain`, `DeadWheels`, `Chassis`,
`Launch` and `Turntable`, in the **core**.

**Flight** — A ball off the floor during a run: launched, falling, or in or on
a hive or a flower. A flight is made over a field, with the field's hives at rest
and their clock and its own starting together, so it is never given hives part
way through a tip. It flies on the **Launch Sim**'s engine, with what a **shot**
has by default: the same air, bounce, grip and rolling on every surface, steps of
0.1 ms with each new contact found where it happens, and the same sleeping and
waking. A ball is the size of the piece it is, a pollen 24.9 g and a nectar
41.3 g. It meets the field as a shot does, except that the hives are a part of
its **surroundings** that turns as one tips, carrying and spinning what it
touches, and what the floor's own world has standing in a flower's bore, its
**stack** and any ball on the floor whose centre is in the bore, is another part:
each a still ball standing on the bore's base plate where that world has it,
moved there once it has moved more than 0.02 in from where the flight has it.
The robot and the other balls on the floor are not in it. A ball **lands**,
back on the floor with the robot, once it presses on the floor rather than
striking it, rolls there without slipping or has fallen asleep there, and
touches nothing the floor's own world lacks: a wall, the side of an obstacle
standing on the floor, the side of a ball standing in a bore and another ball
landing with it are all it may touch, so balls that come to rest together land
together. A flower's base plate, the floor of its bore with the **nest**'s ring
in it, is floor here as it is to the floor's own world. A ball rolled along it
into a bore that holds a stack meets the stack's bottom pollen there, as it
would the real one, and lands clear of it, so no stacked pollen is moved; one
that comes to rest on the plate of an empty bore lands there and the nest takes
it. A ball resting on another ball, on top of any other
element or on a flower's top stays in flight; a flower's top closes its bore from
above, so a ball gets into a bore only under the flower's layers, along its base
plate. A ball whose centre is past a wall's outer face is out. A
ball counts toward tipping a hive only while it presses on its upturned cell's
walls or back or on a ball in the cell, never while it crosses the mouth in the
air. The nectar a hive is set up with lie asleep where the field puts them,
cleared of what they overlap; a ball put in a cell is awake and falls asleep as
any ball does; and a flight with nothing awake and no hive tipping does no work
however long it waits, going on at once to where stepping would have left it.
Its time is counted in whole nanoseconds and its steps in whole numbers, and a
tick is at most a quarter of a second. Its work is allowed by the steps, not by
the ticks: each window of fifty steps (five milliseconds), the windows lying end
to end from its first step, may do fifteen thousand units of the engine's work,
three million a second; once the work done since a window's first step reaches
that, contacts in the rest of the window are resolved at the ends of pieces
without being located, and the next window locates them again. So ticks that add
up to the same nanoseconds leave a flight the same to the bit, however they are
cut: every landing, every ball still flying and the work counted. A hive tips by
its tipped self taking the hives' place in the surroundings under the same names,
so what stands in the bores stays as it was. Were the surroundings to refuse a
tip, or the engine a ball, the flight says so to whatever stepped it or put the
ball in, and a run stops there saying why, rather than a full cell staying
upturned or a ball going missing. A flight is a value.
Classes: `Flight` and `Bores`, in the **core**; `SimRobot.standInTheBores`, which
tells the flight what the floor's own world has in the bores each step.

**Noise** — How a run's robot differs from the tuned model, the ways a
real robot does, drawn once per run from a **seed**: the same robot for
the same seed, another robot for another. The noise is in the mechanisms
only, never the sensors, so a gap between where the robot is and where
its localizer believes it is comes from the robot code. Each drive
motor's kS, kV and kA are within a tenth of the tuned values, each its
own way, so the robot drifts under equal power. The **battery** starts
anywhere from flat (12.0 V) to fresh (13.8 V), sags in proportion to the
drive power commanded and drains as the run goes on; the sensor reads it
exactly and the motors get what it reads. The floor gives only so much
**traction** (0.3 to 0.6 g): a wheel that asks for more, driving or
braking, slips. A robot **set down** at a start pose lands near it, as a
hand puts it, about half an inch and two degrees off. The loop period is
about 33 ms (thirty hertz), varied, with the odd **hiccup** of 80 to
200 ms. A robot without noise is the tuned model exactly, with the loop
at 20 ms: what every run had before there was noise. A seed draws what
`java.util.Random` would, so it is the robot it always was. Every range, rate
and chance above is a **simulation constant**, so a seed draws the robot from
the constants a run was started on. Classes: `Noise` and `Draws`, in the
**core**; `SimNoise`, which names the robot in a run's log; `SimRobot.setDown`.

**Simulation constants** — The numbers the simulator was built with that are
the admin's to change: the **noise** a seed draws a robot from (how far a
motor strays from its tuning, the battery, its sag and drain, traction, how a
hand sets the robot down, and the loop and its hiccups), and the
**mechanisms** nobody has measured yet (the launcher's throw and the
turntable's speed). Each has a range it may go in, and the ones that come in
pairs — the flattest and freshest battery, the least and most traction, the
shortest loop, the loop, and the shortest and longest hiccup — must be the
right way round, so a set is checked whole and is either taken whole or
refused, saying which and why. The world's physics — gravity, a ball's bounce
and grip — and the robot's own tuning are not among them: the one is the
field's, the other the robot code's. The **Launch Sim**'s bounce, grip, drag and
lift are something else again: **shot settings**, chosen per launch by whoever
launches, kept per user, and never reaching a run: a run's **flight** flies on
their defaults, which are the field's physics. The admin page lists them under
*Simulation constants*, each with what it is, its unit, its range and what it
was built as, and a set is saved whole, so what is left out goes back to how
it was built; `GET /admin/constants` and `PUT /admin/constants` with what
differs as `{"<name>": <number>}`. The server keeps them with the rest of its
settings, so a restart runs what was set, and a settings file it cannot read
stops it from starting. A saved constant that is a number past where its range
now lets it go, as a throw saved before the throw's range was narrowed is, does
not stop the server: it is used at the nearer end of its range, and the admin
page shows it beside what was saved until the admin saves the constants again.
Saving any other setting, as the field the pages draw is, keeps it saved as it
was, so it is still shown after a restart. Every run a user starts is made on the constants as
they were when it was started: its child is told the ones that were changed on
its start line, its log names them, and `/status` says which they were. The
core's calls that draw a seeded robot, launch a ball or turn the turntable
name the constants rather than defaulting them, so none of them is made on the
built ones by forgetting to say. Classes: `Constant` and `Constants`, in the **core**;
`SimConstants`, which reads and writes them; `SimBench.Run.constants`.

**Seed** — Which robot an op mode's runs are made on, kept by the bench
beside the op mode's start pose (in `start-poses.json`, so per user on
the coding server): a whole number, drawn into a robot by the noise, or
none for the exact robot. Seed 1 until someone sets it, so a run is made
on an imperfect robot unless someone asks for the exact one. Shown and
edited in the op mode's row on the bench page and the Simulate tab (a
blank seed is the exact robot), answered by `GET /seed?opmode=<name>`
and set by `PUT /seed` with `{"seed": <n or null>}`, listed with each op
mode in the catalog, carried by each run (`/status` says which seed a
run was made on, and the run's log names the robot drawn) and sent to
the child on the start line. Class: `StartPoses`; `SimBench.Run.seed`.

**Simulated clock** — The simulation's time, in nanoseconds since the
world was made, advanced only when the world is stepped. The robot code
reads it through its hardware's clock, so a simulated run need not take
real time and comes out the same every time (for the same seed, on a
robot with noise). Method: `SimRobot.nanoTime`.

**Field** — The season's field as the simulator has it, reduced from
FIRST's CAD by `tools/field/step_to_field.py` to `field.json`: the
**size** between the walls (141 inches) and the walls' height, each
**field element** as a low-poly convex shape with its colour, the two
**hives**, the **game pieces** where a match starts, the gaffer **tape**
on the floor, and the
**obstacles**: the convex footprint of every part of an element that
stands lower than the robot is tall, which is what the robot runs into,
each saying how high it **stands** and how far it **clears** the floor.
Every part blocks on its own, so what is driven through between blocks
nothing: the frame in the middle of the field is an obstacle leg by leg
and foot by foot, and a flower is one pipe by pipe. What stands lower than
the floor's lip (half an inch) is no obstacle at all, being part of the
floor: a flower's base plate is driven and rolled over. A game piece is a
**pollen** or a **nectar**, the bigger ball; the pieces on the floor in
the open are **loose**, the simulator's to roll, the nectar in the hives is
the hives' to hold, the pollen in the flowers is the flowers', and the rest
(the rows outside the walls) stay put. Everything is in
the field frame Road Runner uses, in inches: the origin at the centre, +x
away from the audience, +y to the audience's left. Classes: `Field`, in the **core**; `SimField`, which reads it.

**Alliance area** — Where an alliance's drivers stand: the box its tape marks on
the floor outside the walls, the tape being the colour of its hive, and the tray
its human player loads from is in it. Blue's is behind the -y wall and red's
behind the +y wall, each the other in a mirror. It is worked out from the tape
when the field is loaded, and a field that marks none for an alliance does not
load. Class: `Field.AllianceArea`.

**Flower** — One of the four towers at the walls, whose four pipes make the
**bore** a stack of pollen stands in: a circle on the floor, the **axis**
midway between the pipes and the radius the nearest of them leaves clear,
with the **gap** between two neighbouring pipes narrower than a pollen, so
what is in the bore stays in it. The bore's wall begins at the **lip**, the
height the pipes start at; below that it reaches nothing but the **nest**.
Class: `Field.Flower`.

**Nest** — The ring in a flower's base plate that the ball at the bottom of
its bore sits in the middle of. To leave, that ball has to roll up over the
ring carrying whatever rests on it, so the nest pushes it back toward the
axis — hardest at the bore's rim, not at all in the middle — and the weight
on it drags it to a stop there rather than letting it roll about. The
middle is the axis to within the engine's contact tolerance, so a ball
rolled back comes to rest rather than creeping at the axis forever. A nest
holds one ball, the one nearest its middle; another in the bore is on the
plate around it and free. Classes: `Nest` and `Seat`, in the **core**;
`SimRobot.holdTheNests`, which pushes on the engine's ball as they say.

**Stack** — The four pollen a flower is set up with, standing one on another
in its bore from the floor up. Each rests on what is under it and falls onto
it under gravity when there is nothing there, landing and settling, and one
that lands hard bounces first, as a ball in flight does; the stack
does not move until something takes the bottom pollen out of the nest. What
can is the robot's own push, directly or through the balls between: its drive
is behind that push and the nest's ring is no barrier to it, and once a push
has rolled a ball up off its seat the ball is out of the nest until it comes
to rest again. A loose ball rolling in is far too light — it knocks the
pollen a little way up the ring and the ring rolls it back. When the bore's
floor does come clear, what is left comes down one place and stands again; a
ball that comes to rest in a bore holds a stack up as well as a pollen of its
own does. Class: `Stack`, in the **core**.

**Hive** — An alliance's see-saw, hanging over the middle of the field
from the axle the frame's top bar holds, with a **cell** at each end.
Everything a hive is made of is given in the hive's own frame — the
origin on the axle, +x along the beam toward the scoring cell with the
beam level, +y the field's and +z up — and the **tilt** it leans at, in
degrees above level, is all that says where that is on the field. The
field is set up with the blue hive leaning 30 degrees toward its scoring
cell and the red hive 30 degrees the other way; a hive that **tips** turns
over to lean the same 30 degrees the other side of level. A tip takes a
second, the ten thousand steps of a flight's engine after the step it began on,
wherever on the steps that was: it starts from rest, is fastest as the hive passes level and comes
to rest again, so it throws nothing, and whatever the hive touches it
carries along as it turns, at the speed that part of the hive moves. A hive
finishes a tip before it can start another, and a tip is timed by the whole
steps since the step it began on and the part of a step since, never by adding
up the times since or by taking one reading of a clock from another; before
that step the hive rests where it was. In a run's **flight**
the hives are turned in their own frame and meet a ball under the same names
and with the same gaps as the **shot's colliders**, at rest and tipped. Every tick says where each hive has got to, so a replay or a live
view shows it turning. Classes: `Field.Hive`; `Hives`; `Lean`, in the **core**.

**Cell** — The basket at one end of a hive, which a launched ball scores
in: the opening the CAD's goal ribs frame — the **mouth**, twenty inches
across and fourteen high — swept twelve inches to the **back** that
closes it, with a **wall** between every pair of the mouth's corners. A
ball whose centre is between the mouth, the back and the walls is in the
cell. The mouth is open and the rest is solid, from either side, so what
goes in stays in until the cell turns it out. Each hive has an **audience**
cell and a **scoring** cell,
named for the end of the field they face. Class: `Field.Cell`.

**Upturned** — Of a cell: its mouth faces up, so it holds what goes in,
resting on the floor at the back. The cell at the other end of the same
hive is **downturned**, mouth facing down, and whatever is in it rolls
out of the mouth and falls to the floor — which is what a hive tipping
does to the cell that goes under. What is in that cell stays in it while
its floor still slopes toward the back, and rolls out once the hive is
past level and the floor slopes toward the mouth. One cell of each hive is
upturned at a time, the one whose mouth faces higher, and that is the one
an alliance can score in; as a tipping hive passes level it is the cell
the hive is turning up. Class: `Field.Cell.upturnedAt`;
`Hives.upturnedCell`.

**Load** — How full a hive is: what its upturned cell holds, where one is
full: a nectar is a fifth of it and a pollen an eighth, so five nectar fill
a hive, or eight pollen, or a combination worth as much — the three nectar
a hive is set up with are three fifths of it, and four pollen finish the
job. A hive tips once what presses in its upturned cell — on its walls or
back, or on another ball in it — would fill it, so a ball still crossing the
mouth in the air counts toward the load but not yet toward the tip; and
tipping empties it. What is still
rolling out of the cell that went under is not the hive's load, being in a
downturned cell, so it cannot tip the hive straight back. What a hive has
**scored** is every ball in either of its cells. Method: `SimRobot.load`;
`Flight.fill`.

**True pose** — Where the simulated robot actually is, as opposed to where
the localizer believes it is.

**Start pose** — Where the robot is placed on the field before a run: its
true pose as the op mode's time begins, in the same frame as every tick.
It says nothing to the robot code, which believes what its localizer and
its plan tell it, exactly as when a person sets the robot down on the
real field. One per op mode, by name, kept by the bench in
`start-poses.json` under its output directory (`TeamCode/build/sim`, the
worktree's own on the coding server) so it outlives the server and each
user has their own; the origin until someone places the robot.
`GET /start?opmode=<name>` answers it and `PUT /start` places it, where
the field allows, so a pose beyond a wall or in an obstacle lands
against it. Chosen on
the **placement page** (`GET /place?opmode=<name>`, the Place button
next to Run): the replay page's field with the robot where the run will
start, dragged to move it and its handle dragged to turn it, or typed
in inches and degrees; every move is saved. Class: `StartPoses`;
`SimReplayPage.placement`.

## The launch sim

**Launch Sim** — The dashboard's third tab, after Simulate: one pollen thrown by a
pair of flywheels, on the field, in as much detail as there is a measured basis
for, and replayed. Its settings stand on the right of its view, with its launches
under the view; under 760 px wide the page stacks them, the view first, then the
settings, then the launches. `#launch`, with the view's options after a `?` as on
Simulate. Like a run, a launch is a bench's, so a user's launches are theirs on the
robot they are on. It runs no robot code, starts no child and changes nothing a run
does; a run's balls in **flight** move on its engine and its defaults, but nothing a
user sets here reaches a run. It runs in the coding server's own process, on the server's own
**core** rather than the worktree's, since it is the server's physics and not a
teammate's code. What the model is, where every number came from, what was left
out and why, and how to calibrate it is `doc/launch-sim.md`. Classes: `Shot`, in
the **core**; `SimLaunches`; `SimLaunchPage`. Files: `launch.html`.

**Shot** — One launched pollen from the moment it leaves the wheels until it comes
to rest with nothing else still moving, leaves the field over a wall, runs out of
time, or has done too much work. It flies through the **air**, bounces off the
**shot's colliders** and the pieces where a match starts, and says how it ended in
words a team uses: at rest in which cell (and whether that cell is upturned, so it
scores), in a flower's bore, on the floor, or on a named collider or piece; over
which wall. A shot is a value: advancing it gives a new one, a shot that is still
flying can be advanced and one that has finished has a **finish** and nothing else, and the same
settings give the same shot bit for bit on any machine however its time is cut up,
since every class of it uses `StrictMath` and a test reading their class files holds
that. Integration is RK4 on a fixed grid of steps; each new contact is found by
bisecting the gap to the moment of touching, so a bounce happens where and when the
ball meets the surface. Each piece's **fate** says whether it stayed on the field or
when and over which wall it left. Classes: `Shot`, `ShotRun`, `Bodies`, `World`, in the **core**.

**Ball** — In the launch sim, any ball's size, mass and how far out its mass is: the
launched pollen is one, and so is each piece, a nectar included, and so is each ball
in a run's **flight**, the size of the piece it is. Class: `Ball`, in the
**core**.

**Wheel pair** — The two flywheels a shot is thrown by. Each side of the pollen
leaves at the surface speed of the wheel touching it, so the centre leaves at their
mean and the difference spins it: with wheel one on top (*roll* 0) and faster it is
topspin, with the bottom one faster backspin, and at roll 90 sidespin. Speed and
spin efficiencies, the pollen squeezed between the wheels, and wheels that slow as
they throw are there to match a real launcher and are off until somebody sets them.
Not the **launcher**, which is Reginald's subsystem. Class: `WheelPair`, in the
**core**.

**Air** — What a shot flies through: drag on the air-relative velocity, Magnus lift
from the part of the spin across the motion, the spin's own slowing, buoyancy and
added mass, and wind, at a density worked out from the temperature, the elevation
and the humidity. The drag is a 26-hole indoor pickleball's, the nearest ball anybody
has measured in flight; the lift curve sits among the measured ones for spheres,
table-tennis balls, pickleballs and a foam ball (`doc/launch-sim.md` says which).
Class: `Air`, in the **core**.

**Bounce** — How a shot meets anything: an impulse with a **restitution** that falls
as impacts get harder, Coulomb **friction**, and **grip**, the tangential
restitution that lets backspin kick a bounce up and topspin make it skid; spin about
the contact's normal and rolling are slowed as they would be. A contact bounces only
when it is new and faster than the threshold, which is what ends a dropped ball's
endless ever-smaller bounces. Each **surface** has its own: the floor's foam tiles,
the perimeter walls, the metal frame, the flowers' HIPS, the hives' cells, and
another ball. A surface's *bounce* setting is its restitution at 244 in/s. Classes:
`Contacts`, `Bounce`, `Surfaces`, `Surface`, in the **core**.

**Shot's colliders** — The field as a shot meets it: the floor, the four walls as
boxes an inch thick with tops a ball can clip, every field element as a convex solid
(each checked convex and closed before it is used), and each hive's parts and its
cells' walls and back, at its rest tilt or tipped. Each is named apart from every
other, since a shot tells what a ball touches by its name, so a field in which two
share a name is refused. They are what the **colliders**
view of the launch page draws, from the shot's own list rather than drawn again on
the page, so the drawing cannot drift from the physics. The robot's obstacle
footprints and the tape are not among them. Class: `Colliders`, `Collider`, in the
**core**.

**Arc** — A circle of some radius about a centre and a unit axis, whole or a part of a
turn from an angle over a width more than nothing and less than a whole turn. One
whose radius is not positive and finite, whose axis is not a unit vector, whose centre
or starting angle is not finite, or whose width is outside that, is refused, saying
which. Its angles run about the axis from the level way across it (straight back for
an axle along the robot's y) toward the axis crossed with that (up), and from x for an
upright axis. Every point has a point of it nearest: on the circle straight out from
the axis while that is within the span, and the end nearer round the circle once it is
not, across the seam at ±π as anywhere else. A point on the axis is nearest the start
(a whole circle's angle 0), as far from it as √(h² + r²); the normal runs from the
nearest point to the point, and for a point on the circle itself straight out from the
axis in the circle's plane. Class: `Arc`, in the **core**.

**Surroundings** — What the balls of a shot meet besides one another: the **shot's
colliders**, which never move, and the parts that may, each listed by itself under the
names it gives when it comes in, with where it reaches, how fast its shape sweeps
through space (nothing for a part that only spins about its own axis, as a flywheel
does) and whether it is moving. A part takes the place of the one held under exactly
its names, in any order; one under any other names is refused, saying so. A part is
placed at a time given as the engine's whole steps and a part of a step, so a part
that moves for so many steps does so wherever on the steps it starts. Whatever a ball
touches says how fast the surface goes there and how it spins, so a ball rides what
carries it and is dragged along by what slides or turns under it; a face of a moving
part that does not itself move says it is still. A touch has a unit normal and a
finite gap, velocity and spin, and a part's touch is kept only within the reach asked
for and under one of the names the part came in with. A step is cut finer while a part
sweeps fast at the step's start or at its end, and each new contact with a part is
found where its face meets the ball, so a part that is moving at either end of a step
passes through no ball. What a part promises the engine (its names given once and the
same wherever it is placed, the same answers however it was placed, touches only where
it says it reaches, a gap that changes as the face's velocity along its normal says
and no faster than its sweep, a normal that turns as its spin says, and nothing moving
while it says it is not moving or on a face that is still) is checked by a test helper
that every part is run through. A part may also squeeze: it says so, and answers what
squeezes a ball (`squeezing`) with overlaps, under names it never touches by, which
the surroundings make into compliant touches (**compliant contact**) where the ball
they were asked about meets each; they ask only the parts that squeeze, and only
where they reach. Only the **launcher** squeezes, and the same helper holds a part
to squeezing only where it says it reaches, under its own names, only if it says it
squeezes, with what moves at a finite velocity where the ball meets it, and with a
depth that shrinks along its normal and changes as the face's velocity along it
says. In a shot nothing moves; in a run's **flight** the hives do,
as they tip. Class: `Surroundings`, in the **core**.

**Compliant contact** — How a ball meets a surface that gives, as Reginald's flywheel
and hood do in the **launcher**: by a force while it is squeezed, never by an impulse. A part answers
what squeezes a ball with an overlap: the name of what squeezes, its surface's
**compliance**, a unit normal, a depth more than nothing, and how what squeezes moves,
as a point of it goes and as it spins, each finite; an overlap says nothing of where
the ball meets it. The surroundings make each overlap a compliant touch for the ball
they were asked about, at the centre and radius they were asked with: it meets the
ball midway through their overlap, the centre less the radius less half the depth
along the normal, and takes the velocity of what squeezes there, finite or no touch
at all, so a part names no point: the ball is met where the part's surface overlaps
it, and the part contract holds a depth to being that overlap, shrinking as the ball
moves out along the normal and growing as its radius does. A touch moves while that velocity
or that spin is not nothing. A compliance is the whole
surface's stiffness in newtons a millimetre, shared evenly by the rings it is sampled
by, a restitution more than 0 and at most 1, a friction coefficient not below 0 and a
regularisation speed more than 0; anything else is refused, saying which. The
launcher's are 40 N/mm, 0.6, 0.6 on the wheel and 0.3 on the hood, and 0.02 m/s, each
named beside where it came from. A ball within a piece's travel, a tenth of an inch,
of where a compliant part reaches is pushed inside each stage of its integration, with
the parts placed at that stage's own time: along each normal by the stiffness times
the depth plus a damping times how fast the depth grows, never pulling, so that the
push ends once it reaches nothing, before the depth does; the damping c is the one at
which a ball of its mass m striking the whole surface alone comes back at the
restitution, as such a spring and dashpot does, and the engine returns it so to within
c·h/m + k·h²/(12·m) at a step h, what the damping pushes in a step and what RK4 misses
of the spring k as the squeeze begins and as it ends, at any restitution, 1 included,
while c·h/m is at most 1 and k·h²/m at most 2; and against the slip of its surface
over what squeezes it (its velocity and its spin across the arm to where they meet,
less the velocity of what squeezes there, along the surface) by the friction coefficient times
that push, in proportion below the regularisation speed, turning it about its centre.
A ball's regularisation speed is never below 2·h·Σμ·N·(1/m + r²/I) ÷ 2.785 over what
squeezes it, twice the least at which RK4 holds a grip stable at the engine's step h,
and is the surface's own wherever that already holds. So a ball pinched between a
still plate and one sliding along rolls at half the slide's speed. Where the step
resolves what squeezes a ball, each surface damped at least half its stiffness times
the step (c ≥ k·h/2) and all of them together at most its mass over the step
(Σc·h ≤ m), as the launcher's five rings on the wheel and five on the hood are for
the pollen at a **flight**'s 0.1 ms, it never gains energy against supports that are
still, held in them, sliding along them or arriving at them from beyond their reach
within a piece; one damped less may gain a little as it arrives, since RK4's last
stage alone sees a squeeze begun within half a step of its end. A wheel and hood
with the launcher's surfaces throw a ball the same to within a half of a percent and
a fifth of a degree at half the step. A compliant touch is never a hit or a press,
never among what a body touches, never part of an island or of a **flight**'s landing,
and never pushes a ball out of a face. A ball squeezed by supports that are all still
rests as one on a still face does, falls asleep, and then stays put exactly; one
squeezed by a support that moves or turns never rests, and rests only from when it
stops; and a sleeping one wakes when a support squeezing it moves or turns, so a wheel
that spins up throws the ball it held. Each evaluation in the integration of a ball so
near a compliant part is a unit of work, and so is each asking of what squeezes a
ball, as in each check of whether what squeezes a sleeping ball moves; a ball further
from every compliant part moves exactly as it would, and costs exactly as it would
among parts that squeeze nothing. Classes: `Compliance`, `CompliantContact`, `Surroundings`,
`Bodies`, in the **core**.

**Flywheel** — Reginald's flywheel as the simulator turns it. Its motor, a goBILDA 5203
at 6,000 rpm with no load and 28 ticks a turn, turns at most 2,800 ticks a second
either way: a command past that turns it at that, and one that is not a number does
not turn it. Its **ticks per turn** are how many of its motor's ticks turn the flywheel
once, gearing included, from 18 to 10,000; any other count is refused, saying so. It
turns at its clamped command times a whole turn over its ticks per turn, in radians a
second, signed as the command is: positive when the back of the wheel moves up, the
motor's direction, which in the **launcher**'s frame is about the way to its left. So
no command turns it faster than 2,800 ticks a second over 18, 977 radians a second.
A run's launch does not turn it yet: it still leaves at a speed in proportion to the
command (**Simulated robot**). Classes: `Flywheel`, with `Flywheel.TicksPerTurn` and
`Flywheel.AngularSpeed`, in the **core**.

**Launcher** — Reginald's flywheel and hood as a part of the **surroundings**, its
faces **arcs** a ball meets by **compliant contact**, as stage 4 of
`doc/plans/runtime-flight.md` gives them; no run uses it until stage 4c, when a
**flight** holds it and feeds it. Its **frame** is the robot's pose turned by the
**turntable**: a place within 10,000 in of the field's centre either way and a finite
yaw, kept within half a turn either way; anything else is refused, saying which. In it,
in inches with x forward, y to the left and z up from the floor, the flywheel's axle
runs along y, 13 mm behind the origin and 180 mm up, and angles about it run from
straight back, up. The wheel's groove is sampled across its width by five whole rings,
at 0, ±12.5 and ±25 mm, of radius 75.2886 − √(35.56² − X²) mm, which turn with the
wheel; the hood's by five arcs of radius 72.341 + √(36² − X²) mm from −64.8° to +20°,
each carried on down to −73°, the hood extension's lower end, by a **lead-in**: an arc
of the same radius about a centre of its own, opening to a gap of the pollen's
diameter less 0.25 mm at −73°, whose normal turns by 6.4° to 7.1° where it meets the
hood, an edge a ball rolls over. A hood ring answers with whichever of its arc and
its lead-in is nearer, so it squeezes a ball once. The wheel's rings share the
wheel's compliance and the hood's rings the hood's. A rigid **stop** stands across the
path one pollen's radius below the **entry**, from the wheel's flange out to the
lead-in's lower end and across the hood's groove, and a ball touches it as it would
the field's metal frame. A launcher is placed for a step: it holds the frame the step
begins at and the frame it ends at, with their times, and the wheel's speed, which
stays as it was given wherever it is placed. Placed at any time, its frame is on the
straight line through the two, on past them and never clamped, carried from whichever
of the two is nearer in time at the step's velocity and turn, so a step that moves is
exactly the frame it begins at as it begins and the frame it ends at as it ends, and
the next step that moves from that frame begins exactly where it ended; its yaw
changes by the difference of the two yaws less whole turns, so it turns the short way
across ±π, at that rate. A step of no time leaves it at rest where it ends, and a
step that does not move leaves it exactly at the frame it ends at from its start: two
equal frames, two whose difference is too small to give a rate, and two whose yaws
are half a turn either way among them, so it starts less than a rate can count from
where it began, or facing the same way. A step whose frame would sweep a face faster
than a flight takes a ball, its speed plus its turn times the 5.26 in the farthest
face stands from the turning axis, or that ends before it begins, is refused, saying
so; the step keeps its velocity and turn, which that bounds, and every time on the
grid is finite, so whatever the types admit, however short the step, it is placed
somewhere finite, and what squeezes a ball within its reach moves at a finite
velocity where it meets it. A new wheel speed or a new frame comes as a new launcher
in its place. A wheel ring moves as the frame does at the hub, plus the wheel's turn
about the axle; a hood ring and the stop move with the frame alone; a still frame with
a stopped wheel squeezes and touches with exactly nothing moving, so a ball held in it
falls asleep there. It reaches where some face of it is within the distance asked and
nowhere else, so a ball elsewhere on the field moves and costs as it would were the
launcher to squeeze nothing. A fed ball goes in at the entry, at −73° with its centre
one radius off the wheel's groove bottom, touching the stop, at the frame's velocity
there and 0.1 m/s up the path, which stands in for the ramp that
`doc/plans/reginald-internals.md` simulates. At 1,050 ticks a second and 24.2 ticks a
turn, on a flight's step and in its air, it throws the pollen at 227.7 in/s, 0.42 of
the wheel's surface speed at its 100 mm diameter, 71.0° up, with backspin of 171.5
radians a second about the axle, a spin ratio of 1.05; the first sample with nothing
of it touching the ball is 8.5° past the hood's upper end, 31.2 ms after the feed and
5,615 units of the engine's work later. A faster wheel throws faster with more spin;
a smoother hood carries a ball faster through the groove, spinning less for its speed;
a frame moving at a constant velocity throws as a still one does with that velocity
added, exactly in a vacuum, and a turned frame throws turned as much about the
upright, across ±π as anywhere; a stopped wheel holds a fed ball against the stop,
inside the robot and off the floor, until it spins up and throws it; and a reversed
wheel drives it onto the stop and never throws it. Classes: `Launcher`, with
`Launcher.Frame`, `Launcher.Course` and `Launcher.Faces`, in the **core**.

**Asleep** and **awake** — Of a piece in a shot, or a ball in a run's **flight**: the
loose pollen and the nectar in the cells lie **asleep** where a match starts them until
something moving touches them, and are **awake** from then until they come to rest
again; the pollen standing in the flowers are held in their bores and never move. A
moving body wakes every
piece it reaches through pieces touching one another, passing through any that are
awake but resting, and a piece it was holding up; a body at rest wakes nothing, so
pieces resting against each other stay asleep. Before a step moves anything, a part
of the **surroundings** moving at the step's start or at its end wakes every sleeping
body one of its moving faces could reach within the step, or that a support squeezing
it moves or turns at either end (**compliant contact**), so a ball asleep in a wheel's
squeeze moves in the first step in which the wheel turns; and so do new surroundings
the moment they are given; after each piece, a part moving then wakes what it could
reach within the next step, and the wake spreads from what it woke in the same piece.
A body at rest within a step's reach of a moving face does not fall asleep, and a
body pressed against a surface that moves or spins, or against an awake body that is,
is never at rest, so a stack riding a slowly moving part rides it awake. A ball asleep
on a still face of a moving part sleeps on, and a part that moves nowhere near a
sleeping body wakes nothing. Classes: `Islands`, `Bodies`, in the **core**.

**Idle** and **allowance** — Of the engine a shot moves its balls on: it is **idle**
through to a later step when nothing is awake and no moving face could reach a
sleeping body before then, nor a support squeezing one move or turn now or then, and
it then goes there at once, exactly where stepping
there would leave it. Its work counts every integration (every evaluation in one, for
a ball near enough a compliant part to be pushed by it, as **compliant contact**
says), every query of its surroundings (asking what squeezes a ball among them), every
check of a moving part's bounds and every placing of the parts in a step in which some
body is awake once what the moving parts reach ahead of the step is woken; going idle,
and a step in which nothing is then awake, are no work, so whether an engine idles
through a stretch or steps through it never changes its work. A step can be given an
**allowance**: so many units of work in each window of so many steps, the windows
lying end to end from the first step. Once the work done since the first step the
engine took in a window reaches its allowance, contacts in the rest of that window are
resolved at the ends of pieces without being located, as they are once a piece has
used up its events, and the next window locates them again. A shot's engine is allowed
work without limit. Class: `Bodies`, in the **core**.

**Shot settings** — Every number a launch is made from, each with its group, label,
unit, range, default and a sentence saying what it does, and checked whole: the
launcher, the pollen and the pieces, the field, the air, the flight model, the
bounce of each surface, the contact and the simulation. They are chosen per launch
on the tab's right, the last ones launched are remembered per user and robot in
`launch-settings.json` beside the start poses, with how many launches there have
been, so a launch's number is never used again for another; a remembered one that no
longer fits goes back to its default and the tab says so. `GET /sim/launch/settings`,
`POST /sim/launch/preview` (how the wheels will throw, without launching),
`POST /sim/launch`, `GET /sim/launches`, `GET /sim/launches/` (the view, with the
launches to show in its fragment, `#7` or `#7,5,3` with the others as ghosts),
`GET /sim/launches/<id>/shot` and `GET /sim/launches/<id>/frames.csv`. Classes:
`ShotSetting`, `ShotSettings`, in the **core**; `SimLaunches`.

**Too much work** — How a shot is held to computing within ten times real time: by a
count rather than a clock, so the same settings always give the same shot. The count
is the shot's **work**: every integration of a ball and every query of the field's
colliders. A shot that needs more than three million units of it a second of its own
time is stopped and says to make the step longer; at the most a unit was measured to
cost here that is about four times real time, and every launch says what it actually
cost ("computed in 112 ms, 0.06x real time"), which is printed and never judged. Method:
`Shot.tooMuchWork`.

