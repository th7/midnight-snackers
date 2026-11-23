# Glossary

The words this project uses for the robot gateway, the simulator and the
robot, as the code uses them. When a term here and a name in the code
disagree, fix one of them. Robot-side vocabulary is covered only where a
module has been deepened; plans, steps and the other subsystems are not yet.

## The robot

**Where things live** — The robot's code is a package of its own, its
**own files**: `org.firstinspires.ftc.reginald` is Reginald. A teammate opens
two folders of it: `subsystems`, which holds nothing but the parts the robot
is made of, so the list of files is the list of parts, and `opmodes`, which
holds nothing but what the driver station lists, so the list of files is the
list of TeleOps and autos. Adding, switching off or removing a part, a TeleOp
or an auto touches that one file and no other. Beside them: `framework` (the
**Robot**, what a subsystem and an op mode *are*, and how the driver station
comes to list them), `tools` (tuning op modes for the people who tune),
`hardware` (the devices and what is wired to what), `control` (the
controllers and filters a subsystem steers by), `roadrunner` (trajectories,
which we own rather than vendor) and `oldcode` (**old code**).
`org.firstinspires.ftc.teamcode` holds what every robot's code is built on:
`base` (where a subsystem prints — `Prints`, `Channels` — its `Gamepads`,
and the `Alliance` a run plays for), `planrunner` (plans and
steps), and what both robots' Road Runner needs — the **clocked encoder** and
the drawing of a robot in `roadrunner`, the drive runner that follows an action
a loop at a time in `control`, and the dashboard it draws on in `hardware`;
its test sources hold the simulator, the robot gateway and the fakes.
Nothing outside a robot's package uses that robot's code, and every op mode
is in a robot's package: a class moves into `teamcode` when a second robot
needs it, rather than being reached for across. No test holds the
subsystems or the op modes to this or to anything else: they are the team's
to change. The tests and CI judge the simulator, the robot gateway and
`framework`, which is what makes a file in `subsystems` a part and a file in
`opmodes` an op mode, and which no teammate needs to open.

**Subsystem** — One part of the robot, and now only that: a class in
`subsystems` that extends `SubSystem`. A subsystem names everything it needs
as its one public constructor's parameters, sets itself up in that
constructor, gets ready in `init`, reads the driver's buttons in `teleOp`,
does its work in `loop`, and prints to the **channel** it was handed. It
offers an auto its **steps**, such as `drive.to(24, 0, 0)` or
`intake.spin()`. It never builds, registers or ticks itself, and nobody does
it by hand: the **robot** builds every subsystem in every op mode, calls its
`init` once every part is built, in a TeleOp its `teleOp` every loop with
the gamepads, and its `loop` every loop after that. Each hook does nothing
unless the subsystem says otherwise. Which buttons do what is said in the
subsystem they move, so there is no driver class to keep in step with it.

**Built because it is there** — Every subsystem the robot finds in
Reginald's code is built in every op mode, and, first, everything its
constructor asks for; a class that is no subsystem is built only because
something asks for it. Nobody lists the parts: a file in `subsystems` is
built because it is there, and one taken out is not. A part is built once
and shared: the localizer `Drive` steers by is the one
`MecanumDrive` reads. What the robot does not build it supplies: the
`Hardware` (switched off for a **switched-off** part), the `Alliance`, the
`Gamepads`, the `Dashboard`, and each part's own channel, named after its
class. It builds only Reginald's own classes, through their one public
constructor, and an interface is the one class it found that implements it,
as `Localizer` is for `PoseEstimate`. A part that needs something the robot
neither supplies nor builds, an interface no class or two classes implement,
or itself, fails at init naming the chain that led to it. The robot finds
Reginald's classes by listing the app's own code at registration, every
class under `org.firstinspires.ftc.reginald`, the way the SDK finds
annotated op modes. Classes: `Robot`; `Discovered`.

**Whoever owns it, ticks it.** A subsystem is ticked by the robot, after
everything it was built from: what a part needs is built first, so it is
ticked first, and the localizer leads because the parts that steer by its
pose need it; parts that need nothing of each other tick in the order of
their names. Anything else that must be ticked is not a subsystem, and is
ticked by whatever it belongs to on the first line of that thing's own loop:
a subsystem's plan runner or drive runner in its `loop`, an auto's plan
runner in the auto's `onLoop`. The tick order used to be one list in the
robot's constructor, written apart from the parts it named; the intake fell
out of it and was built and never ticked. Now there is no list, so there is
nothing to keep in step.

So the chain is plain the whole way down: an op mode's `init` builds the
**Robot**, which builds every subsystem and calls each part's `init`; an op
mode's `loop` hands a TeleOp's gamepads to each subsystem's `teleOp`, ticks
the robot, which ticks its subsystems, and then the op mode's own `onLoop`,
which is where anything the op mode owns, such as an auto's plan, is
ticked. `loop` is final at both levels, so the order is not a subclass's to
rearrange, and an auto's plan advancing after every subsystem is the shape of
it rather than a position in a list. Classes: `SubSystem`; `Robot`.

**Channel** — The driver station's telemetry divided by the name of whoever
is printing. Each subsystem is handed its own when it is built and prints to
it and nothing else, always, knowing nothing about whether anyone is
reading; the channel puts its own name on every caption, so two subsystems
cannot print the same one. Telemetry costs a loop and a crowded screen, so a
channel is off until a driver asks for it, and which button reaches which
channels is said once, in the op mode. Every channel the robot makes has a
button; a button may also name a channel of a subsystem that is switched
off, which then reaches nothing. Class: `TelemetryTogglesReachEveryChannelTest`.

That the decision lives there and nowhere else is the point. It used to be a
flag on every subsystem, a gate in a base class and a method on each of them
that the op mode called by name; a subsystem carried the machinery for a
question it had no business answering. **Nowhere** is a channel that reaches
no screen, which is what Road Runner's tuning op modes are handed, said out
loud rather than left as a null. Classes: `Channels`; `Prints`;
`Prints.NOWHERE`.

**Handed, not fetched** — A subsystem is given everything it needs when it
is built — the hardware it asks for its devices and clock, what it
coordinates, and somewhere to print — and reaches for nothing else, ever.
What it needs is its constructor's parameters, so it says so in one place,
and the robot hands each of them over already built. Nothing is handed to
it afterwards; `init` runs once everything is built, and before the first
loop. Nothing reaches back into the robot, so there is no null to find on a
field.

**Hardware** — Everything the op modes touch outside their own code: the
configured devices, the dashboard, and the
**clock**, the time in nanoseconds that every timer in the robot code
reads and nothing else does: a step that waits, the trajectory followers,
and the dead wheel encoders' measure of their own speed. On the robot the clock is the
system's; in the simulator it is the simulated clock. A timed step must
name its clock, so a forgotten one is a compile error, and no part of the
robot may reach for Road Runner's wall-clock encoder, which a test
enforces rather than leaving to care.

A device is **asked for by name, where it is used**. A part is handed the
hardware and asks it for the motors and servos it drives by their configured
names, `motor("intake")`, in its own constructor; the hardware holds no list
of its own. So a device goes from the configuration to the part that drives
it in one step, and a device nothing asks for is never fetched. On the robot
the hardware map answers, and in the simulator an SDK `HardwareMap` of its
**devices** does, so the simulator builds and runs whatever the robot code
asks for. A device asked for that is not configured, or not of that kind,
fails at init with the SDK's own message naming the device, on the robot and
in the simulator alike, rather than as a null inside whichever subsystem
reaches for it first, which on the robot is in the middle of a match. Devices are named rather than counted out in
order, since nine of them are motors and a positional list would let a
wheel be quietly swapped for its neighbor. Class: `Hardware`;
`Hardware.fromHardwareMap`; `Hardware.nanoClock`.

**Switched off** — A part built on hardware that does nothing.
`hardware.switchedOff()` hands it motors and servos that take every write and
read back zero, an IMU and a voltage sensor the same, with the run's own
clock and dashboard. The part is built, held,
ticked and prints to its channel like any other, and every subsystem that
coordinates it keeps it; only what reaches the world is switched off. A part
is switched off by `@SwitchedOff` on its class, which has the robot hand it
`hardware.switchedOff()` instead of the hardware, so switching it off, or on
again, is that line, and the code that uses it compiles and runs either way.

It replaces commenting out the part, its devices and every reference to it in
the subsystems that coordinate it, after which turning one back on meant
finding them all again. The intake came back that way without its place in
the loop order, so it was built and told on and off by the driver and never
drove its motor. A switched-off part is exactly as quiet as a missing one: a
driver asking it for something sees nothing happen. Classes: `SwitchedOff`;
`Hardware.switchedOff`.


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

**Old code** — A part the robot no longer has, kept where a teammate can
still read it: an `oldcode` folder, now `reginald/oldcode`, which holds the
brain, the camera and its filter and webcam, the launcher and the turntable.
Android Studio shows it with the rest of the code, and nothing builds it: the
robot's build and the simulator's both skip every `oldcode` folder, so it
needs no upkeep and need not compile. Bringing a part back means moving it
out and making it build again. The entries for the turntable, the sighting
and the agreement below describe code that is there now.

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

## The robot gateway

**Robot gateway** — The host process that lets teammates on the LAN reach
the robot: the login page, the admin's approvals, the **robot relay** and the
**field models**. It reads and writes no file of the project: teammates edit,
build and deploy from their own clones in Android Studio, and simulate with
their own **local simulator**. One machine runs it with
`./gradlew :TeamCode:robotGateway`, or on a Mac with `./start`, which runs
that task on Android Studio's own Java (`ANDROID_STUDIO_APP` names the app
when it is not in `/Applications`). It has two listeners: the **admin
listener** on loopback only (port 21987, `ROBOT_GATEWAY_ADMIN_PORT`), and the
**user listener** on every interface (port 21986, `ROBOT_GATEWAY_USER_PORT`).
It replaces the coding server, whose `CODING_ADMIN_PORT` and
`CODING_USER_PORT` it refuses to start under, naming the setting that
replaces each. Which ports it listens on is decided in the **core**. Classes:
`RobotGateway`; `GatewayPorts` and `PortSetting`, in the core.

**Routes** — What each listener answers, said once as a table of method,
path pattern and handler: the user listener's public routes (the login page,
`POST /login`, `GET /me`), then the **approved routes** behind one guard (403
for anything but an approved session), which is `GET /relay` alone, then the
**field offer** behind the relay's own guard (403 for an address no approved
login came from); the admin listener's routes. A path no route knows is a 404, a path known by another
method a 405. The bench's routes are one table too, **mounted** at the root
of the **local simulator**'s page. Class: `Router`.

**Admin** — The person at the host machine. Only they can reach the admin
listener, where they decide logins, delete users and get the **field
models**. There is no admin login; being on the machine is the credential.

**User listing** — `GET /admin/users` and the roster it draws on the admin
page: every user with a session, in the order they first logged in, one row
to a name, each with the **sessions** they have made, oldest first, each
saying where it came from and its state, whether a connection relayed for
them is open (**relayed**), and a Delete button. Under `relay` it gives the relay's port,
the hub, the **hub state** and the line approved teammates run. Logging in
again adds a session to the user, never a second row; a **delete** takes the
row away.

**Fold** — A user's sessions on the admin page, folded away under their
row and opened by the caret. A user the admin has not folded by hand is
open exactly while a session of theirs is pending, so a login waiting to
be decided is never hidden; folded shut, the row still carries the
pending count.

**User** — A teammate on the LAN who has logged in with a username alone
(`POST /login?username=<name>`); a login is for no robot. Users
reach only the user listener. A user is their username, so one user is one
row of the listing however many times they log in.

**Session** — One login by one user, identified by a small integer **id**
the admin sees, and proven by a **token** the browser holds in the
`session` cookie as `<id>.<secret>`. A session moves through four
**states**: *pending* (asked, not yet decided), *approved* (its address may
reach the robot through the relay), *denied* (the admin said no), and
*revoked* (was approved, no longer is). A session carries where it logged in
from, how long ago, the robot it asked for, and its state, which the admin
decides per session at `POST /admin/logins/<id>/approve|deny|revoke`. An
approved login's page shows the `adb connect` line and nothing else to do. A
revoked session is still a session; only a **delete** takes sessions away,
and it takes all of that user's at once.

**Robot relay** — How a teammate's own Android Studio reaches the robot: the
robot gateway listens on every interface (port 5555, adb's own, so `adb
connect <host>` needs no port; `ROBOT_RELAY_PORT`) and passes each connection
it admits on to the hub's adb port (`192.168.43.1:5555`, `ROBOT_HUB_ADDRESS`),
byte for byte in both directions, so the teammate's own adb server treats the
robot as an ordinary network device and Run, Logcat and the debugger work from
their machine. It admits a connection only from an address an approved
**session** came from, and closes any other before the hub hears a byte of
it; that an address can be spoofed is accepted, as a fence against accidents.
Revoking a login, or deleting its user, closes every connection relayed from
its address unless another approved login came from the same one. A hub that
does not answer within three seconds closes the client's connection rather
than leaving it hanging. Whom it admits, which connections a change closes,
who is listed as relaying and the line a teammate runs are decided in the
**core**; the relay decides nothing. An approved login reads its line at `GET
/relay`, which the login page shows once the admin approves; the **user
listing** says per user whether a connection relayed for them is open
(**relayed**), and under `relay` the port, the hub and the **hub state**. The
hub is assumed to take several adb clients at once, the host's own among
them, so nothing takes turns. `doc/client-setup.md` takes a new client from a
clone to a first deploy. Classes: `RobotRelay`; `RelayAdmission` and `Peer`,
in the core; `FakeHub`.

**Field offer** — The field models the robot gateway built, served on the
user listener to an address an approved login came from, the relay's own
admission, so a teammate's **local simulator** needs no cookie and no key:
`GET /field` names the offer's version, and per model built its resolution,
file, size and SHA-256, lowest first; `GET /field/<file>` serves one. Only a
built model is served: neither the export, the textures nor anything else in
the **state directory**. The offer is a wire format between a host and clients
built from other checkouts, so it carries a version, and a client refuses one
it does not read, saying to pull. Classes: `RobotGateway`; `FieldOffer` and
`FieldModel`, in the core.

**Hub state** — Whether the host could open a connection to the hub's adb port
when it last tried: *untried*, *reachable* or *unreachable*, with why and when.
It is tried for each client the relay admits, and by a probe that opens and
closes a connection while the admin page is open, no more often than every ten
seconds. Class: `HubState`, in the core.

**Delete** — `POST /admin/users/delete?username=<name>`, the Delete button
on the user's row: the user leaves the listing. Every session they have is
forgotten, so their browsers are logged out and their relayed connections
close. A user nobody knows is a 404.

**Develop branch** — The permanent branch the team's work lands on,
`develop`. Teammates push to it
from their own clones. Getting `develop` to `main` is CI's: a **run
for develop** whose every step is green moves `main` to the commit it judged,
unless `main` is already past it; a run that would move `main` past a change to
a workflow file, as when its commits since `main` change one, holds `main` red
instead, naming those files, until a maintainer pushes `main` to the judged
commit by hand and re-runs the failed jobs.

**Run for develop** — What CI does each time `develop` is pushed. Runs go one
at a time: a push that arrives during a run waits, and a run still waiting
when a newer push comes is cancelled rather than judged, so it is never red,
and the pushes that arrived meanwhile are judged together by the next run. A
run judges `develop`'s tip as it is when the run starts, not the commit whose
push started it: a tip job names that commit, and a job on each platform checks
out the commit it named, so the two never judge different tips. Each runs the
same commands as a local run. **Windows** judges what a teammate's computer
runs, the client: `./gradlew spotlessCheck`, `python3 tools/run_tests.py client`
and `./gradlew :TeamCode:testDebugUnitTest`. **Linux** judges the client with
the same three, and the gateway, which runs on the host alone, with
`python3 tools/run_tests.py gateway` and `./gradlew :TeamCode:gatewayTest`. The
judging is green only when both platforms are, and the issue a red run opens
names the platform of each check that failed. When every step of the judging is
green and `main` has no commit `develop` lacks, a `main` that already has
commits of `develop` the judged commit lacks stays where it is, whatever its
workflows, and the run is green but leaves the issue as it is, open or not,
since only a run that judges the commit `main` stands at can say whether that
commit is good. Otherwise, with the judging green and `main` at the judged
commit or behind it, the workflows decide. No step reads a secret, and the
built-in token may not move a branch past a change to a workflow file, so when
any commit from `main` to the judged commit changes a file under
`.github/workflows`, even one on a branch merged into `develop`, one a later
commit undoes or one a merge changes in resolving it, or the judged commit's
copy of one differs from `main`'s, the run pushes nothing and is red, naming
the files the commits change, a renamed one by its old name and its new, apart
from those whose copy alone differs, as after a merge of `main` that keeps
`develop`'s copy, of which it says that a push of the judged commit replaces
`main`'s copy with its own. No commit to `develop` and no session can clear
that: it waits on a maintainer, who fetches the judged commit by its hash, so
any clone of the repository has it, single-branch or shallow too, and pushes
`main` to it by hand, never forced, since a clone that has `main`'s tip but
not the judged commit is told, without the fetch, that the push needs force,
which it does not. The maintainer then re-runs the failed jobs of the run. The judged commit's own
core decides whether `main` moves, in a job whose token may only read and whose
checkout keeps no credential, and that job fails unless `main` moves or stays,
so re-running the failed jobs looks at `main` again. The job that pushes runs
none of the judged commit's code: when the judging and that look both passed,
the core said move and `main` still stands where that look found it, as the
job's own checkout fetched it, it pushes the commit the judging named to `main`
with the built-in token its checkout keeps, a plain push, never forced and never
a merge, and a red judging never reaches the push whatever the core said. A look
is stale once `main` has moved since it, other than to the judged commit by the
run's own push. The job that pushes also asks whether `develop` holds `main`'s
tip, as its checkout fetched them, an error in asking counting as no. When the
judging and the look passed and it finds `main` moved since the look, it pushes
nothing, and it ends in success when `develop` holds that tip, `main` having
moved along `develop`; otherwise, as when a commit reached `main` by itself
after the look, it ends red, saying in its log that it could not find `main`'s
tip in `develop`, and the job that writes the issue says on it only that the run
is red, linking it. The core, when the look found that `main` moves or stays and
`main` is neither where the look found it nor, after the run's push, at the
judged commit, says nothing for the issue; and the job that writes the issue,
about to close it, writes nothing once `main`'s tip is not the judged commit but
`develop` holds it. Each of these two, and the job that pushes when `main` moved
along `develop`, says in its log that `main` moved since the run looked and that
a run that looks again, for a new push to `develop` or re-running all of the
run's jobs, speaks for `develop`, so re-running the failed jobs of a run whose
push was refused once, after a later green run moved `main` along `develop` and
closed the issue, pushes nothing, writes nothing to the issue and ends green.
Whatever the core said, the job that writes the issue first reads `develop`'s
tip and `main`'s as they are now and asks GitHub how they compare, and the run
is red unless GitHub finds `develop` ahead of `main` or identical to it, an
error or any other answer counting as no: that job still opens the issue or
comments on it as the core said for a red run, but never closes it, says where
the tips stand or says that `develop` moved on, which the core hands it apart
from a red run's comment, and when neither the core's open nor its red comment
went through it says on the issue only that the run is red, linking it. So a
commit that reaches `main` by itself after the job that pushes read `main`,
before the job that says what to write reads it or between that and the job
that writes the issue, turns the run red, as does re-running some of the
run's jobs after it, the job that writes the issue alone included; one that
lands after that job's own read is found by the next run, which holds `main`
naming it. Anything else is a red run, and `main` stays where it was. A red run
opens the issue **IMPORTANT: develop CI Failures**, or comments on it when it is
already open, naming the commit judged, whether its judging was green, and what
held `main` — the step that failed, a setup step or the report upload as much as
a check; the job that looks at `main` or the one that moves it ending early; the
commits `main` has that `develop` lacks, with the merge that brings them in; the
workflow files that hold `main` for a maintainer; or git's own words for a
refused push — and the run; a commit that reached `main` by itself keeps every
run red until `develop` holds it. The core says what to write in a job whose
tokens may only read, and the job that writes the issue runs none of the judged
commit's code; a red run for which that job carried out neither an open nor a
red comment the core said still opens or comments on the issue, saying only that
the run is red and linking it. Only a run whose every job passed closes the
issue: one that moves `main`, or finds it already at the commit it judged, as
the re-run after a maintainer's push does, closes it, saying `main` now stands
there, only when that commit is still both `develop`'s tip and `main`'s, as the
job that says what to write reads them after the push and as the job that writes
the issue reads them again before writing anything to it. When only `develop`
has moved on, it says on the issue where `develop`'s tip stands, in a comment
the core hands apart from a red run's and the job that writes the issue carries
out only once its reads of both tips and its check that `develop` holds `main`'s
tip passed, never that the run moved `main` when the look found `main` already
at the judged commit, and does not close it; when `main` has moved off the
judged commit along `develop`, its look is stale, and when `develop` does not
hold `main`'s tip, as when a commit reached `main` by itself, the run is red. So
re-running only some of a run's jobs, which reuses what the jobs that passed
handed on, as after a push refused once or a close that failed, never closes the
issue once `develop` has moved on or `main` has moved off the judged commit, and
is red once `develop` does not hold `main`'s tip. A pull request is still
judged, in a queue of its own, and never moves `main` or touches the issue. The
judged commit's core decides whether `main` moves and what to write on the
issue, and the workflow adds guards that do not depend on that commit's code: a
red judging or a job that did not pass never pushes or closes the issue, a push
also needs `main` to stand where the look found it, as the job that pushes reads
it, a `main` that has moved since the look to a tip that job cannot find in
`develop` turns the run red, the job that writes the issue turns the run red
whenever GitHub does not find `develop` holding `main`'s tip as that job reads
them, a close also needs `develop`'s tip and `main`'s, as the job that writes
the issue reads them, to still be the judged commit, and a red run the core
could not speak for still writes to the issue. A test holds each guard by
evaluating the workflow's own conditions.
Files: `.github/workflows/tests.yml`; `tools/ci/develop_run.py`;
`tools/ci/develop_issue.py`; `tools/ci/test_workflow.py`;
`tools/ci/workflow_conditions.py`.


**State directory** — Where the robot gateway keeps what outlives the
process: `sessions.json` (each login) and the
**field models** it fetched. It is where the coding server kept its state and
stays there, so the logins the coding server approved stay approved; the
`users.json`, `settings.json`, `editable.json` and `worktrees.json` it left
are no longer read and are left as they are. The robot a session was once for,
its retired `robot` field, is read past and dropped the next time the
sessions are written, and that name is never written again. It follows the XDG Base
Directory convention: `$XDG_STATE_HOME/midnight-snackers/coding-server`, else
`~/.local/state/midnight-snackers/coding-server`. Owner-only. Session secrets
are stored there only as salted scrypt hashes. A store that exists but cannot
be read stops the gateway from starting. Class: `StateStore`.

**Worktrees left behind** — The teammates' worktrees the coding server kept
under the state directory's `worktrees/`. The robot gateway never writes or
deletes them: at startup it names each one and says it no longer manages
them, so whoever wants work left in one can take it, and says nothing when
there are none. Class: `LeftWorktrees`, in the core.

**Part** — One piece of the **local simulator**'s page that starts on its
own: the simulator, the launch sim, the spindexer sim, the tabs. A part that
throws is caught, named in a banner at the top of the page and recorded in
`window.simulatorPage.broke`, and the others still start — so a browser that
cannot run one part of the page still gives the rest, and never leaves the
shell it was served as, which is what "nothing to show you" looks like too.
The page asks for nothing an **older tablet's** browser has not got: an
iPadOS 13 `MediaQueryList` carries `addListener` and not `addEventListener`,
and the page feature-tests it. Held by `tools/browser/dashboard.mjs`, which
opens the page on an engine with those APIs taken away; see
`doc/browser-tests.md`.

**Build** — Compiling a project's main sources as they are on disk with
the JDK's own compiler against the **libraries**, together with that
project's **simulator** — everything under its `TeamCode/src/test/java`
that is not a test, with the resources next to it copied along — cached
by a fingerprint of all three trees. The simulator runs in the child against the sources it was
just built with, so a simulator that does not fit them fails the build
naming the seam, rather than the child failing at run time with a
linkage error nobody can read. A run that does not compile shows the
**problems** (file, line, message) on the Simulate tab; a problem in the
simulator has no file the user can open and says so in its message. Not the
Android build: no Kotlin, no desugaring. Class: `SimBuild`.

## The simulator

**Core** — `simcore`: the simulator's geometry and physics — the field, the
hives and flight, the robot's motors and the **drivebase** they turn its wheels
through, dead wheels, the
flowers' stacks and nests, and the **noise** a run's robot is drawn with — and
the bench's decisions about a run — how it ends, what the **watchdog** holds its
child to, and what Stop does — in a module whose build proves every function
total. What is left outside it is the rigid-body engine and what reads and
writes it. A project's own core is built with its simulator. Classes:
`GatesTest`; `SimBuild`.

**Drivebase** — How Reginald's drive motors turn the four wheels of the rigid
body the simulator pushes, said once in the core: **mecanum**, a motor to each
wheel, the back pair mounted mirror-image to the front. Which way each motor drives
its wheel on positive power is one of the **directions**, so nothing about it is left
in code. Its motors' powers are
recorded `LF`, `RF`, `LB`, `RB`, and each motor has the noise drawn for its
wheel. Classes: `Wheel`; `Drivetrain`.

**Turned around** — Reginald's drivetrain, dead wheels and all, now faces the
other way: what was its front is its back. Each drive motor's cable was moved
to the port named for the corner it now sits at, and each corner is mounted as
it was, so the drive motors are what they were. The dead wheels' cables stayed
in their ports, so the parallel wheel is still read on `rightBack`'s port and
the perpendicular one on `leftFront`'s. The dead wheels count ahead where they
counted back, and each sits as far off
the centre as it did, on the other side; a turn in place reads as it did.
Their offsets are the simulator's own, the robot's measured ones turned
around, not whatever the robot code believes, so code that has not caught up
with the turn is caught. Classes: `SimRobot`; `DeadWheels.reginalds`.

**Dead wheels** — Reginald's two unpowered odometry wheels, each read on a
drive motor's encoder port; not the drag wheels or drag wheel encoders. Each
is reported as the SDK reports a motor's count, the way that port's motor is set to turn, so Road Runner's raw encoder,
which undoes that, reads the same whichever way the drive motor is set.
Classes: `DeadWheels`; `TheHubReportsAReversedMotorsCountNegatedTest`.

**Bench** — The simulation core behind the **local simulator**'s page: the
catalog, the runs so far, and the routes that start a run and follow it. One
run at a time per bench. Its Simulate tab lists and runs Reginald's op modes.
Given a project it builds that project's robot and simulator before
every run, and refuses a project with no simulator of its own, since the
child would fall through to the server's; without one (the tests) it
runs on the current classpath. Class: `SimBench`.

**Local simulator** — A teammate's own bench, run from their own checkout by
`./gradlew :TeamCode:simulator` or Android Studio's **Simulator** run
configuration (`.run/Simulator.run.xml`), which prints the URL to open. It
serves one bench for Reginald over the sources on disk as they were last
saved, so the next Run runs the last save with no restart and a compile error
names its file and line. It listens on loopback alone (port 21988,
`SIMULATOR_PORT`), so nobody on the LAN reaches it, and it has no login: the
page at its root is the simulator's own, `dashboard.html`, with the Robot, Simulate,
Launch Sim and Spindexer Sim tabs, and opens on Simulate. It draws the **kept
field**. Stopping the task stops any run and its child. Classes:
`LocalSimulator`; `SimulatorRunConfigurationTest`;
`tools/browser/simulator.mjs`.

**Kept field** — The field models a **local simulator** fetched from the host's
**field offer** and keeps on its computer, so a client draws the detailed
field with no Onshape key, away from the host too. `ROBOT_GATEWAY` says where
the host is (`192.168.1.20`, or `192.168.1.20:<port>` when its user listener is
not on 21986). At each start, before it prints its URL, the simulator reads the
offer, with two seconds to connect, and fetches every model whose SHA-256
differs from the one it keeps: so a model the admin rebuilt is fetched again,
and one already kept is not. A model is kept only once all of it has arrived
and matches the size and digest offered, written whole and moved into place,
so a fetch cut off partway leaves the one kept before. The Run window says what
was fetched, or why nothing was: no `ROBOT_GATEWAY`, no host in reach, a
refusal (no approved login came from this address), or an offer in a version
this simulator does not read. The models are kept on Windows under
`%LOCALAPPDATA%\midnight-snackers\field-models`, on a Mac under
`~/Library/Application Support/midnight-snackers/field-models` (not
`Caches`, which macOS may empty), and elsewhere under `$XDG_DATA_HOME`, else
`~/.local/share`, then `midnight-snackers/field-models`. The simulator serves
what it keeps in place of the bench's own assets, and writes the page's
`field-default.js` itself: the most detailed model kept is what a page draws
when it asks for none, and `FIELD_SAID` gives a sentence for each model it can
draw, which the page shows under the field. With nothing kept, that is the low
field committed with the project, and the page says so. Classes: `KeptField`;
`WhereTheHostIs`, `KeptModels`, `DrawnField` and `FieldOffer`, in the core.

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
exit for any still running then; a server killed outright, as Windows'
`TerminateProcess` kills when Gradle's task is stopped, runs no exit of its
own, so a child watches the process that started it and, once it is gone,
kills what it started, deletes its scratch and halts; and no more run at once than the
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
controller lists them: every concrete op mode of Reginald's own kind,
`framework.OpMode`, under its own package, `reginald`, that carries the
`@Autonomous` or `@TeleOp` annotation, plus whatever a
**registrar** (a
static method annotated `@OpModeRegistrar`) registers, found by calling
the registrar itself. Reginald's TeleOps and autos are registered that way,
by `OpModes`: each class that extends `TeleOp` is one op mode, named after
its class, and each `@Auto` annotation on a method that makes a plan is
one, with no class of its own, handed the subsystems its parameters name. Entries are keyed by the op mode's **name**, which the robot
controller requires to be unique, and carry a **kind** (*auto* or
*teleop*), **where** a person finds the code (a class, or the plan
method), and the **alliance** it plays for (*Blue*, *Red*, or none for a plan
relative to wherever the robot starts). An op mode says its alliance only once
it is made, so the catalog makes one to ask; one that cannot be made is listed
all the same and says why when it is run. A listing from a child from before
alliances plays for none. Nested classes are never listed, so test op modes
stay out of the real bench, and the tuning op modes in `tools` are not ours
to simulate.
The child reports the catalog after each build, so a newly written op
mode appears without a restart. Class: `SimCatalog`.

**Child** — The fresh JVM each run executes in, launched on the newly
built classes — the project's robot sources and its simulator, built
together — and the libraries, nothing else. Every class identity is
consistent, static state starts clean, and a hung op mode is a process
that can be killed. It says its **protocol** first, builds the catalog,
waits to be **placed**, then prints the run stream; the op mode's own output
goes to stderr, after a line naming which of the robot's imperfect selves
the run is on. Its standard
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
by name, with the fix, when a seed is set. Version 5 named the robot a child
simulated, when there were two; it is retired and its number is never used
again, and a version 5 child is read as any other. From version 6 the start line may carry the **simulation
constants** that were changed, and only those; an older child runs on the
constants it was built with, so the bench lets it run when none were changed
and refuses it by name, with the fix, when any were: pull `develop`. From
version 7 it may carry the **robot description**'s hub and **configuration**
when they differ from Reginald's as built; an older child runs on the ones it
was built with, so the bench refuses it by name, with the fix, when either
differs. From version 8 it may carry the **directions** too, when any mechanism is
reversed; an older child runs every mechanism as built, so the bench refuses it by
name, with the fix, when one is reversed. From version 9 the spinner is among the
directions and the spindexer's constants among the constants; an older child runs the
spindexer it was built with, so the bench refuses it by name, with the fix, when the
spinner is reversed or a spindexer constant changed. From version 10 a device in the
start line's configuration may carry the name it is committed under, beside the name the
Robot tab gave it; an older child finds each device by the name it was built with, so the
bench refuses it by name, with the fix, when a device is renamed. From version 11 the
flywheels are among the directions and the launcher's constants among the constants; an
older child runs the flywheels it was built with, so the bench refuses it by name, with the
fix, when a flywheel is reversed or a launcher constant changed. From version 12 the
launch's constants are among them too; an older child drops each pollen it feeds, so the
bench refuses it by name, with the fix, when one changed. From version 13 the four drive
motors are among the directions; an older child drives each wheel the way it was built
to, so the bench refuses it by name, with the fix, when a drive motor is reversed. Only the bench and its child speak this protocol: the browser's page
is reloaded when it changes, so it is never versioned. A tick's
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
autonomous period (30 s of simulated time). An op mode **runs a plan** by saying
whether its plan is done and which step it is on, and the run records that step
every loop; Reginald's autos do. A **TeleOp run** has no plan: it is
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
drive motor, in the order the **drivebase** names them, `LF`, `RF`, `LB`,
`RB`, which is how the replay page labels them — the
dashboard packets drawn that loop, where the balls are (each as x, y, z), how
many each alliance has **scored**, for a TeleOp what each gamepad read, and
the robot's mechanisms: how many pollen it **holds**, which way its intake
turns, which way the **spinner** turns and at what fraction of full speed, and the
**ramp**'s angle and whether it is up, how many pollen have been **fed**, and whether
the spindexer is handing one over or why not. The replay page marks an intake pushing out, a
spinner turning counterclockwise and a ramp down while the spinner turns as the wrong
way; none stops the run. Class: `SimRecording`.

**Golden trace** — What the simulator's flight did, sample by sample, kept
in a file under `TeamCode/src/test/resources/.../sim/traces` and compared
against the next run. A run is the same every time it is made, so a change
in how the engine moves a ball arrives as a diff with a sample number on it.
A trace is a change detector, not a judgement: it cannot tell a regression
from a change somebody meant, so an intended diff is read and then
**regenerated**, and a regenerating run fails, because a run that wrote the
answer down has not checked it. A missing file fails too, rather than
passing for want of anything to compare. No trace holds what the robot code
commanded: that is the team's to change. Classes: `FlightTrace`,
`FlightTraceTest`; `SimTrace` names where traces live and the flag that
regenerates them.

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

**What the suite costs** — What a run of one of the two unit-test suites, the
client's (`testDebugUnitTest`) or the gateway's (`gatewayTest`, whose tests sit
under the `gateway` package), spends on the few
things that are dear: a **child JVM** started, a **compile** of a project's
sources, a session secret put through **scrypt**, a **node** process, and a
**project copied** onto disk. Each is
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
touched it — and `CostTest` holds the build to starting it that way. It holds
whether the tests pass or fail: what they cost is printed and judged by a task
that runs after them either way (`TeamCode/suite-cost.gradle`), so a run with a
failing test still fails on a budget it missed, and one whose test JVM wrote no
verdict still says so. `tools/suite_cost` holds that task to it on a build of
its own, and `CostTest` holds the build to giving the suite one.

The counts are pinned in a **suite budget** and the times are only ever
printed: a count is the suite's and a time is the machine's. The same deal as
the scene **budget** above — a change detector, not a ceiling, so fewer misses
it as surely as more, a missing budget fails rather than passing for want of
anything to compare, and a regenerating run fails because a run that wrote the
answer down has not checked it. A ledger is one JVM's, so a run of part of the
suite, or of the suite split across JVMs, says it **could not judge** rather
than passing. Every kind is pinned, in a budget for each suite: the client's
`sim/suite-budget.json`, judged on Linux and on Windows against the same file,
and the gateway's `gateway/suite-budget.json`, judged on Linux. A run prints how
many tests it ran on which platform and how many of them were **passed over**: a
client test that cannot mean anything on one platform skips itself there only
if `sim/passed-over.json` names it, with why, under that platform, and the judge
fails on a skip it does not name and on a named test that ran.
`ThePassedOverListNamesTestsThatExistTest` holds every name there to a test that
exists. Files: `Cost`; `suite-budget.json`; `passed-over.json`.

**A file of the repository** — A **suite budget**, a **golden trace**: written
where somebody reads it in a diff, and holding nothing secret. None of them goes
through the **state directory**'s store, which makes the directory it writes
into owner-only because session secrets live there — right there, and wrong on a
source tree, where it leaves a checked-out directory only the person who last
regenerated something can read. Class: `RepoFile`.

**Refresh assets** — The admin page's button, and what the robot gateway does
for itself at startup when it has fetched none: ask Onshape for the field
assembly, build the field as it looks, fetch the five images the CAD cannot
carry, and write them under the server's state directory. Nothing is written
until every one has arrived and been checked, so a rename upstream stops a
refresh rather than leaving half of one behind. The admin page says what was fetched
and which resolutions are held, and the **field offer** serves the models it
built to teammates' simulators.
The fetch is `main`'s to start rather than the gateway's, so no test that
builds a robot gateway reaches the network. Classes: `Onshape`; `Gltf`;
`FieldGlb`; `FieldAssets`; `RobotGateway.Assets`.

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
page. A build asked for no resolution in particular builds **high**, the one a
page draws when it asks for none, and so does the startup fetch.

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

**Which one a page draws when it asks for none** is **high** — a field that
does not look like the field is the thing worth avoiding, and whoever cannot
afford it is the one who knows that — and a query string says otherwise for one
page with `?resolution=low`, `medium` or `high`. It reaches the pages as the one
line they import for it, `field-default.js`, which a bench serves as committed
beside the models and the **local simulator** writes from its **kept field**,
and the line is `SimAssets`' to write, so what a page draws and what a server
holds cannot drift apart by being said twice. A page
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

**Robot description** — The simulated robot as data: its **configuration**, the
physical truth of its mechanisms (the drive's true feedforward, its geometry and
mass, where the dead wheels and the hub sit, which way positive power turns each
mechanism, how each of the launcher's flywheels comes up to speed and where they
are mounted) and the **noise** a seed
draws from. Its defaults are a file in the repository,
`TeamCode/robot-description.json`, which names every value: the local simulator
reads it when it starts and checks it whole, and a file it cannot take refuses
the start, naming the file and the rule. A teammate's edits on the Robot tab are
their own, over those defaults, and each run is made on the description as it
stood when the run started: its child is told what differs from Reginald as
built on its start line, and its log names each difference against the built
value, or says "as built". The robot code's own tuning is only
what the code believes, never the description. The fake IMU reads the robot's
turn through the hub's mounting as the SDK's own helper would read it, so code that
takes the hub to sit otherwise reads what that robot would: Reginald's code takes
the hub to sit logo down, USB right, where the description sits it logo right, USB
up, and reads its yaw half a turn off and its turn rate about the wrong axis. Its
dead wheels' offsets differ from the description's too, so it believes the wrong
place after a turn. Classes: `RobotDescription`, `HubMounting` and `Directions`, in the **core**;
`FakeImu`; `SimDescription`, which reads the file and the start line.

**Robot tab** — The dashboard's first tab, before Simulate (`#robot`): the
**robot description** a teammate's runs are made on. It lists the
**configuration** by device (its kind, the name it is configured under, and for a
motor its SDK motor type and what its port's encoder port reads), which way the hub
is mounted, the **directions** as what positive power does each way, and every **simulation constant** by group with its label, unit, range,
sentence and committed value. A value that differs from the committed default is
marked, with a reset beside it, and Reset all puts every value back. Each change
saves the whole set as it stands: a set the core refuses names the rule beside the
value changed and nothing is saved, and the tab says what the next run is made on, as
`/status`, the run's log, its replay and its row on the Simulate tab say it: each
difference against the built value, or "as built". Answered by `GET /robot` and saved by `PUT /robot` with the
**robot overrides** asked. Classes: `SimOverrides`; `tools/browser/robot.mjs`.

**Robot overrides** — A teammate's edits to the **robot description**, kept by the
bench in `robot-overrides.json` beside `start-poses.json` and
`launch-settings.json` under its output directory, never in the repository: only
what differs from the committed defaults, as `{"constants": {name: value}, "hub":
{"logo", "usb"}, "configuration": {device: {"name", "type", "encoder"}},
"directions": {mechanism: way}}`, a device
named by its committed name. A save is checked whole and refused naming the rule,
an unknown name included. Recalled at the start, the file is taken as far as it
can be: a value past a range narrowed since goes back to its default, and one for
a value the description no longer has is kept in the file until the next save
drops it, and the tab says which. A removed value's name is never reused, since
an older file would be read with the new meaning: `Constant.retired()` lists them
(`launch_throw`, `turntable_speed`) and a test holds that no constant takes one.
Classes: `Overrides`, in the **core**; `SimOverrides`.

**Configuration** — The hub's devices by name and kind (a motor's kind is the SDK
motor type it is configured as), and what each motor port's encoder port reads, as
the robot's active configuration and its wiring have them. The robot code finds
its devices through it as it would on the robot, so a device it asks for that is
not there, or not of that kind, fails the run at init as the SDK would fail it.
Each device is also committed under the name `TeamCode/robot-description.json` gives
it, which a rename on the Robot tab never changes: the simulator finds each
mechanism's device by that committed name, so a rename changes only what the robot
code must ask for. Classes: `Configuration`, in the **core**; `SimDevices`.

**Hub** — What the simulated hub does with what the code writes to a motor, said
once in the core. The direction the code set is turned into the SDK's operational
direction (a motor type declared counterclockwise flips it). Without the encoder
the written power reaches the motor; using the encoder, the power or velocity
written is a target the hub's velocity loop closes on whatever that port's
encoder port reads. A port reading its own motor settles at the target; one
reading a dead wheel or nothing runs away toward full power, as on the robot.

**Directions** — Which way positive power turns each mechanism's pieces, with the
SDK's motor direction left forward: each **drivebase** motor's wheel, `left_front`
and `right_front` `drives_forward` as built, or `drives_backward`, and `left_back` and
`right_back` `drives_backward` as built, mounted mirror-image, or `drives_forward`; the intake `takes_in` as built, or
`pushes_out`; the **spinner** `turns_clockwise` from above as built, or
`turns_counterclockwise`; each **flywheel**'s motor `turns_counterclockwise` seen
from its shaft as built, or `turns_clockwise`. A part of the **robot description** a teammate reverses on the Robot
tab, so code written for the other way round is seen doing the wrong thing.
Classes: `Directions` and `Mechanism`, in the **core**.

**Holds** — The pollen the simulated robot has taken in. Its intake turning
inward takes every pollen touching its front face, up to two; turning outward or
still, the robot pushes pollen as its chassis would. A held pollen is off the
field, so its place in a tick is empty. Which way the intake turns comes from its
motor's spin and the **directions**, and is shown with what it holds on every tick,
live and in the replay. Class: `Intake`, in the **core**.

**Spinner** — The spindexer's continuous servo, turning the pollen it holds about a
shaft up from below. What the code writes becomes a pulse as the SDK makes it (power
−1…1 onto positions 0…1, negated for `REVERSE`, across the port's `PwmControl` range,
600–2400 µs unless the code sets one, in whole microseconds), or is the pulse itself
when the code sets one with `PwmControl.setPulseWidth`. As on the hub, a position the
same as the last one written sends nothing, so a range changed since waits for the
next position that differs, and `getPulseWidth` reads back the last pulse sent. The
pulse becomes motion as REV's continuous mode makes it: 1500 µs is still, above it
clockwise from above up to full speed at 2500 µs, below it counterclockwise; the
**directions** may reverse that. So power 1 `FORWARD`, or a pulse of 2400 µs, is
clockwise at 90% of full speed. A servo the code has not driven, or whose pulse it
disabled, is still. Classes: `ServoPort`, `PulseRange`, `ServoSignal` and `Spinner`, in
the **core**.

**Ramp** — The spindexer's servo that lifts a pollen to the launcher. Its pulse, made as
the spinner's is, becomes an angle as REV's servo mode makes it, 500→2500 µs onto
−135°→+135°, so position 0 through the SDK's range is stowed and the angle above stowed
is position × 243°. It slews toward that angle at the description's rate (0.168 s per
60° as built) and is **up** at or above the description's angle (35° as built). Class:
`Ramp`, in the **core**.

**Fed** — A pollen the spindexer has handed to the launcher. With the **spinner**
turning clockwise and the **ramp** up, each of its two arms passing the ramp, once
every half turn, hands over one held pollen into the **launcher**, which holds one, and
which **launches** it at once when it can. When nothing can be handed over, each tick
says why: the spinner is not turning clockwise, the ramp is down, nothing is held, or
the launcher still holds the pollen it was handed last. Class: `Handover`, in the
**core**.

**Launch** — How a fed pollen leaves the **launcher** in a run. With both **flywheels**
launching, it leaves into a **flight** with the speed and spin the core's **wheel pair**
gives from the two wheels' rpm, as the **Launch Sim**'s launcher settings would on their
defaults: from the exit point on the robot (14 in up and 3.26 in behind its centre as
built), pointing ahead along the robot's heading at the description's pitch (70°) and
roll (0), carrying that point's velocity as the robot drives and turns. At roll 0 the back
wheel is the wheel pair's wheel one, on top, so a faster back wheel gives topspin and a
faster front wheel backspin. Otherwise it stays in the launcher, and every tick says why:
the front or back wheel is still or turns the wrong way, or the launch would be faster than
a flight takes a ball (1,000 in/s). Each tick counts the pollen launched, live and in the
replay, where one kept in the launcher is marked. Classes: `Launch`, with `Launch.Robot`
and `Launch.Decision`, in the **core**; `SimRobot.launches`.
Each port reports its count as the SDK does: negated for a reversed motor,
velocity in twenty-tick steps, zeroed while in `STOP_AND_RESET_ENCODER`.
`RUN_TO_POSITION` is refused. The loop's gains are the simulator's own; the hub
firmware's are not published. A motor that turns only itself, such as the intake,
spins as its type does free (the UltraPlanetary HD Hex's 2800
ticks/s at 12 V), reached over a tenth of a second; a **flywheel**'s motor spins
on its own true feedforward instead. The loop's integral winds up while a heavy
wheel comes up to speed, so a flywheel asked for 1200 ticks/s passes it by about a
fifth and settles within 2% after about six seconds. Classes: `HubPort`,
`VelocityLoop`, `Spin`.

**Devices** — A robot's devices as fakes, and a clock: everything the robot
code can reach through its **Hardware**, and nothing that moves on its own.
A motor reads back what was last written to it, and time passes only when
someone says `advance`. The fakes are the devices the **configuration** names,
in an SDK `HardwareMap` that finds them by name and kind as the hub's does; the
**Hardware** is what the robot's own `Hardware.fromHardwareMap` finds there, on
the simulator's clock and dashboard, and a device the code does not ask for sits
configured and idle, still there for the physics. A motor answers the SDK motor
type it is configured as, built from the SDK's own annotation for that motor, and
the continuous servo and the servo are the SDK's own over a fake controller. The
SDK's matrix arithmetic, which the hub's mounting is worked out with, runs on a
stand-in for Android's `android.opengl.Matrix`, held to Android's documented
behavior by its own tests. This is the second adapter at the hardware seam —
the **simulated robot** is the first, and holds one of these, writing its
sensors from where the physics put the robot. A test of one subsystem wants
neither the physics nor the field (an intake spins because its own motor
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
A ball the robot does not **hold** is a ball it pushes, and a ball is off the floor
only as a **flight**, which is how a **launch** leaves it.
The model is planar apart from flight and the flowers'
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
tens of inches a second). The world moves in steps of at
most 5 ms whatever the loop rate, so nothing is jumped over. Classes:
`SimRobot`, which holds the engine; `Drivetrain`, `DeadWheels` and `Chassis`,
in the **core**.

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
A flight may carry Reginald's **launcher**, made with the frame it stands at and
its wheel stopped; one with no robot has none, so it is never fed, stepped with a
frame or carried, and flies as it always has, its work included. The launcher is
a part of its own beside the hives and the bores: a rebuild for what stands in the
bores keeps it, and a tip, which replaces the hives alone, never touches it, so a
ball in it stays where it was through both. A flight that carries one is stepped
only with the frame the tick ends at and the wheel's angular speed, and each tick's
course begins exactly at the frame and time the last one ended at; a launcher that
stands as it stood, at the same frame and not moving with its wheel turning as it
did, is not replaced, so while its robot rests a flight comes out the same to the
bit however its time is cut. Its motion wakes nothing it does not reach and costs
nothing while nothing is awake, so a flight idles ahead past a robot driving where
no ball is near its launcher. A ball is fed into it only if it is a pollen of the
launcher's 1.39 in that would come into flight no faster than a flight takes a
ball, which the frame's velocity at the entry and the feed together can pass while
the launcher sweeps near its bound the way the feed goes, and only while no ball
lies within the two radii and the touching distance of the entry ("the launcher's
entry is not clear"); it goes in at the entry with the feed and no spin, so balls
fed as fast as that allows are thrown one at a time and never overlap. A teleport carries the launcher to its new
frame at once, with no sweep, and the balls it holds go with it, lying and moving
in it as they did; a tick that would sweep it there faster than a flight takes a
ball is refused. Classes: `Flight` and `Bores`, in the **core**; `SimRobot.standInTheBores`, which
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

**Simulation constants** — The numbers the simulator was built with that a
run can be started on other values of: the **noise** a seed draws a robot from (how far a
motor strays from its tuning, the battery, its sag and drain, traction, how a
hand sets the robot down, and the loop and its hiccups), and the physical truth of
the **robot description** (the drive's feedforward per inch, its track width,
lateral multiplier and mass, the dead wheels' tick and where they sit, and the
**spindexer**'s: the **spinner**'s full speed, and the **ramp**'s slew and the angle it is up at;
and the **launcher**'s: each **flywheel**'s motor's kS, kV and kA, their encoders' ticks
a turn and each wheel's diameter, and where on the robot it lets a pollen go, at what pitch
and roll). Each has a range it may go in, and the ones that come in
pairs — the flattest and freshest battery, the least and most traction, the
shortest loop, the loop, and the shortest and longest hiccup — must be the
right way round. No set may let a wheel turn faster than the chassis may move,
with the freshest battery and the drive's kS and kV at their weakest by the motor
spread. So a set is checked whole and is either taken whole or
refused, saying which and why. The world's physics — gravity, a ball's bounce
and grip — and the robot code's own tuning are not among them: the one is the
field's, the other only what the code believes. The **Launch Sim**'s bounce, grip, drag and
lift are something else again: **shot settings**, chosen per launch by whoever
launches, kept per user, and never reaching a run: a run's **flight** flies on
their defaults, which are the field's physics. Every run is made on the
constants as they were when it was started: its child is told the ones that were changed on
its start line, its log names them, and `/status` says which they were. The
core's calls that draw a seeded robot name the constants rather than defaulting them, so none of them is made on the
built ones by forgetting to say. Classes: `Constant` and `Constants`, in the **core**;
`SimConstants`, which reads and writes them; `SimBench.Run.description`.

**Seed** — Which robot an op mode's runs are made on, kept by the bench
beside the op mode's start pose (in `start-poses.json`): a whole number, drawn into a robot by the noise, or
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
`start-poses.json` under its output directory (`TeamCode/build/sim`) so it
outlives the simulator; the origin until someone places the robot.
`GET /start?opmode=<name>` answers it and `PUT /start` places it, where
the field allows, so a pose beyond a wall or in an obstacle lands
against it. Chosen on
the **placement page** (`GET /place?opmode=<name>`, the Place button
next to Run): the replay page's field with the robot where the run will
start, dragged to move it and its handle dragged to turn it, or typed
in inches and degrees; every move is saved. Class: `StartPoses`;
`SimReplayPage.placement`.

## The launch sim

**Launch Sim** — The dashboard's tab after Simulate: one pollen thrown by a
pair of flywheels, on the field, in as much detail as there is a measured basis
for, and replayed. Its settings stand on the right of its view, with its launches
under the view; under 760 px wide the page stacks them, the view first, then the
settings, then the launches. `#launch`, with the view's options after a `?` as on
Simulate. Like a run, a launch is a bench's, so it is for the robot the bench is
for. It runs no robot code, starts no child and changes nothing a run
does; a run's balls in **flight** move on its engine and its defaults, but nothing a
user sets here reaches a run. It runs in the simulator's own process, on its own
**core**, since it is the simulator's physics and not the robot's code. What the model is, where every number came from, what was left
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

**Flywheel** — One of the **launcher**'s two wheels, the front and the back, each a
bare REV HD Hex (`flywheelFront`, `flywheelBack`) turning its wheel directly. Each
motor comes up to speed on its own true feedforward from the **robot description**
(as built kS 0.5647 V from the HD Hex's stall current, kV 0.004084 V per tick/s
from its 6,000 rpm free, and kA 0.00713 V per tick/s² from the Launch Sim's wheel
inertia, 0.95 lb·in²), so a wheel asked for 1200 ticks/s takes most of a second
to reach it; with no power it coasts on its friction alone when floating, and
brakes on its back-EMF too otherwise. Its encoder counts the description's ticks a
turn (28) and reports as the **hub** does. The motors' bodies point left: the front
wheel launches turning clockwise seen from its motor's shaft, the back
counterclockwise, so with the **directions** as built the code sets the front
`REVERSE` and the back `FORWARD`. Each tick says, live and in the replay, which way
each wheel turns (launching, still within a twentieth of its motor's free speed, or
the wrong way, which is marked), its rpm the launching way, and their mismatch:
how much faster the front wheel's surface moves than the back's, in inches a second.
Classes: `Flywheels`, with `Flywheels.Side` and `Flywheels.Turning`, and `Spin`, in
the **core**. The core still turns one alone, as Reginald's flywheel once was: its motor,
a goBILDA 5203 at 6,000 rpm with no load and 28 ticks a turn, turns at most 2,800 ticks a second
either way: a command past that turns it at that, and one that is not a number does
not turn it. Its **ticks per turn** are how many of its motor's ticks turn the flywheel
once, gearing included, from 18 to 10,000; any other count is refused, saying so. It
turns at its clamped command times a whole turn over its ticks per turn, in radians a
second, signed as the command is: positive when the back of the wheel moves up, the
motor's direction, which in the **launcher**'s frame is about the way to its left. So
no command turns it faster than 2,800 ticks a second over 18, 977 radians a second.
A run's **launch** takes its speed from the two flywheels instead. Classes: `Flywheel`, with `Flywheel.TicksPerTurn` and
`Flywheel.AngularSpeed`, in the **core**.

**Launcher** — Reginald's two **flywheels**, each with a motor of its own, that throw
a pollen out of the **spindexer**. The core still has the single flywheel and hood it
replaced, as a part of the **surroundings**, its
faces **arcs** a ball meets by **compliant contact**, as stage 4 of
`doc/plans/runtime-flight.md` gives them; a **flight** may carry it and feed it,
though no run does until `SimRobot` gives its flight one. Its **frame** is the robot's pose turned by the
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
a step in which nothing is then awake, and new surroundings a **flight**'s launcher
gives it while nothing is then awake, are no work, so whether an engine idles
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
bounce of each surface, the contact and the simulation. The launcher's exit height,
pitch and roll, each wheel's diameter (wheel one is the back flywheel, wheel two the
front) and the encoder's ticks per turn have no default of their own: they default to
the teammate's **robot description** as it stands, their **robot overrides**
included, and take its ranges, and the tab names the Robot tab's value each comes
from. The launcher's place on the field, its yaw and the wheels' speeds are the
Launch Sim's own, since the description holds none of them. They are chosen per launch
on the tab's right, the last ones launched are remembered per user and robot in
`launch-settings.json` beside the start poses, with how many launches there have
been, so a launch's number is never used again for another. Only what a launch set
apart from its defaults is remembered, so a value it left at the description's follows
the description when it changes; a remembered one that no longer fits goes back to its
default and the tab says so. `GET /sim/launch/settings`,
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


## The spindexer sim

**Spindexer Sim** — The dashboard's last tab, after Launch Sim: Reginald's **spindexer**
turning pollen round to the **ramp**, which lifts each into the flywheel, and the
**launcher** throwing it, replayed with the **spinner**'s and ramp's loads traced
against time. It is laid out as Launch Sim is: settings on the right of the view,
the feeds under it. `#spindexer`, with the view's options after a `?`. Like a launch, a
feed is a bench's, so it is for the robot the bench is for. It runs no
robot code and nothing set here reaches a run; it runs on the simulator's own
**core**. What it simulates, where every number came from, what the physics showed
and how to calibrate it is `doc/spindexer-sim.md`. Classes: `Feed`, `FeedRun`, in the
**core**; `SimFeeds`; `SimSpindexerPage`. Files: `spindexer.html`.

**Spindexer** — The round bowl the pollen wait in, turned by its **spinner**, and
what was once called the feeder. As the engine meets it, it is its floor
with the ramp's slot, its wall with the front and back windows, its top plate with
the notch the flywheel reaches through, the **spinner** and the **ramp** bar, each a
named collider. Its measures are the CAD's except the spinner, which is not drawn;
the back window's sill is a setting, 30 mm by default where the CAD has 43.12. It is
placed in Reginald by the CAD's transform, tilted 16.46° and rising toward the back,
and a feed sends its axes and measures with it, so the view draws what the physics
meets. Class: `Spindexer`, in the **core**.

**Spinner** — The hub and two arms on a REV Smart Robot Servo in continuous mode
that turn the pollen round the **spindexer**, clockwise seen from above. Its speed is
its full speed times its power, less in proportion to the torque the pollen load it
with, and none at its stall torque: 1.0 rev/s and 1.1 N·m at the Hub's 5 V. Turning
under a tenth of its unloaded speed for a tenth of a second while powered is a
**jam**, reported once naming the pollen touching it. Class: `Spindexer.Spinner`, in
the **core**.

**Ramp** — The bar under the **spindexer**'s floor at the back, on a second Smart Robot
Servo in angular mode, that swings up through the slot and lifts a passing pollen
into the flywheel; it rises toward the end away from its servo. It slews to its
commanded angle at 0.168 s per 60° and holds it whatever the load, reporting the
torque it holds against. A **feed** raises it at one time and lowers it at another.
Class: `Spindexer.Ramp`, in the **core**.

**Feed** — One run of the **spindexer** from the pollen at rest against its wall,
with the spinner, ramp and flywheel commanded every 4 ms, until every pollen has
left, everything is at rest, its time runs out or it has done **too much work**,
held to the same three million units a simulated second a **shot** is. On the way it
reports each pollen *thrown* from the hood (where, how fast, at what angle, with
what spin), each that *fell out* of a window, and each **jam**. A feed is a value,
and the same settings give the same feed bit for bit. Its settings are chosen per
feed on the tab's right and refused naming the rule they break, the last ones fed
are remembered per user and robot in `spindexer-settings.json` with how many feeds there
have been, so a feed's number is never used again.
`GET /sim/feed/settings`, `POST /sim/feed`, `GET /sim/feeds`, `GET /sim/feeds/`
(the view, with the feed to show in its fragment, `#7`), `GET /sim/feeds/<id>/feed`
and `GET /sim/feeds/<id>/frames/<chunk>`. Classes: `Feed`, `FeedRun`, `FeedSetting`,
`FeedSettings`, in the **core**; `SimFeeds`.
