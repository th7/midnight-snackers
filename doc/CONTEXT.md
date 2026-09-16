# Glossary

The words this project uses for the coding server, the simulator and the
robot, as the code uses them. When a term here and a name in the code
disagree, fix one of them. Robot-side vocabulary is covered only where a
module has been deepened; plans, steps and the other subsystems are not yet.

## The robot

**Hardware** — Everything the op modes touch outside their own code: the
configured devices, the camera's detections, the dashboard, and the
**clock**, the time in nanoseconds that every timer in the robot code
reads and nothing else does: a step that waits, the trajectory followers,
the camera's check of a detection's age. On the robot the clock is the
system's; in the simulator it is the simulated clock. A timed step must
name its clock, so a forgotten one is a compile error. Class: `Hardware`;
`Robot.clock`.


**Drive** — The subsystem that moves the robot. Whoever is driving gives it
one **intent** per loop and it writes the wheel motors itself: **manual**
(straight, strafe and turn powers from the sticks), **toward** (a pose the
drive steers to on its own, answering whether the robot has arrived and come
to rest), or **follow** (a Road Runner action, run loop by loop until it is
done or cancelled). An action being followed owns the wheels: manual and
toward do nothing until it is done or cancelled, and a stick pushed past the
takeover deflection cancels it. Class: `Drive`.

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

**Nav** — Where the robot is on the field and how to get somewhere else.
`pose(x, y, heading)` makes a pose from coordinates given the blue way, and
the alliance's mirroring (y and heading negated for red) happens there and
nowhere else. It answers the **current pose** (where the localizer believes
the robot is), whether the robot is **near** a pose (within 3 inches and
6 degrees), the **launch pose**, and builds the strafing and backward paths
the plans follow. Class: `Nav`.

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
turntable's offset and, when playing for an alliance, hands it to Nav: the
first sighting places the robot, a later one nudges its position by at
most an inch per axis and never its heading. Class: `Camera.sighting()`.

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
listener, where they decide logins and pick the editable set. There is no
admin login; being on the machine is the credential.

**User listing** — `GET /admin/users` and the roster it draws on the admin
page: every user who has ever logged in, in the order they first did, each
with the **sessions** they have made under them, oldest first. What is the
user's rather than any one session's is said once, on the user — the
worktree, its branch, its **status** (the same changed, ahead and behind
that `GET /git/status` gives the user; null until the worktree exists, and
null with a **statusError** when git cannot read it, so one broken worktree
does not blank the list), how their last pull or push ended, and Pull and
Delete buttons. Logging in again adds a session to the user, never a
second row; a **delete** takes the row away, and a login after that is a
new row for the same branch.

**Fold** — A user's sessions on the admin page, folded away under their
row and opened by the caret. A user the admin has not folded by hand is
open exactly while a session of theirs is pending, so a login waiting to
be decided is never hidden; folded shut, the row still carries the
pending count, the branch, the status and what its sessions have open.

**User** — A teammate on the LAN who has logged in with a username. Users
reach only the user listener. A user is their username: the worktree, the
branch and the work are owned by it, so one user is one row of the
listing however many times they log in.

**Session** — One login by one user, identified by a small integer **id**
the admin sees, and proven by a **token** the browser holds in the
`session` cookie as `<id>.<secret>`. A session moves through four
**states**: *pending* (asked, not yet decided), *approved* (may edit and
simulate), *denied* (the admin said no), and *revoked* (was approved, no
longer is). Only approved sessions reach files, builds, and the simulator.
A session carries what is its own alone: where it logged in from, how long
ago, the file it has open, and its state, which the admin decides per
session at `POST /admin/logins/<id>/approve|deny|revoke`. A revoked
session is still a session; only a **delete** takes sessions away, and it
takes all of that user's at once.

**Editable set** — The files the admin has picked for users to edit, each
named by its **root-relative path** with `/` separators
(`TeamCode/src/main/java/.../Plans.java`). A user may only ever name a file
by exact match against this set; nothing a user sends is resolved against
the filesystem. The admin picks from the host checkout; the key means the
same path in every worktree.

**Project root** — The repository checkout the coding server serves: the
top of a git working tree with a `develop` branch. All paths users see are
relative to it, and the editable set and the worktrees are stored per
project root. A user's save never writes under it; only git does, under
`.git`.

**Develop branch** — `develop`, the permanent branch every user's work
starts from, pushes to, and pulls from. The server requires it to exist
and never deletes or rewrites it: it only adds merge commits to it.
Getting `develop` to `main` is the coach's job, by pull request.

**Worktree** — A git worktree of the project root's repository, one per
**username**, on its own **user branch**. All of that user's edits are
written there and every run they start is compiled from there. Owned by
the username, not the session, so logging in again after a restart or a
revoke-and-reapprove finds the same work. Made when the admin approves
the login, so git's refusal, if any, is the admin's to see. Class:
`Worktrees`.

**User branch** — `coding/<slug>`, created at the tip of `develop` when
the worktree is made. Saves are uncommitted changes in the worktree
until the user presses Commit.

**Delete** — `POST /admin/users/delete?username=<name>`, the Delete button
on the user's row: the user leaves the listing. Every session they have is
forgotten, so their browsers are logged out, their bench and its child
stop, and their worktree directory goes. Their **user branch** stays, and
so does the mapping to it, so logging in again and being approved rebuilds
the same worktree on the same branch with everything they committed, under
the same slug. Refused, changing nothing, while they have work `develop`
does not have — uncommitted files, or commits ahead — naming it, until the
admin asks again with `force`; the page's Delete anyway is that second ask,
so what the admin reads is what the server enforced and not a warning the
page worked out for itself. The refusal is decided before the bench is
stopped, so a user who keeps their work keeps their run.

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
and on any pull or push, whoever asked for it.

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
`POST /admin/logins/<id>/pull`: the same merge in that user's worktree,
with the same refusals, so a teammate who has walked away from a stale
branch is brought up to date without their browser. The pull is the
user's, and any session of theirs names them, so the button on their row
sends their newest. Their editor
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
coach. The admin page shows the coach the recipe, with the real path:

```
cd <the user's worktree>
git merge develop         # resolve the conflicts in an editor
git add -A && git commit  # then the user presses Push
```

The coach's merge *is* the pull, so Push is all that is left, and
`develop` is never checked out to resolve anything.

**Slug** — The username lowercased, every run of characters outside
`[a-z0-9]` replaced by one `-`, trimmed of leading and trailing `-`, at
most 32 characters, never empty. It names the branch and the worktree
directory. Two usernames that collapse to one slug get `-2`, `-3`, … so
the mapping is decided once and stored, never recomputed. It outlives the
worktree: a deleted user's mapping stays, which is what gives them their
own slug and their own branch back when they return.

**Worktrees directory** — Where the worktrees live: under the state
directory, at `worktrees/<root name>-<8 hex of SHA-256(root path)>/<slug>`,
so two checkouts on one machine keep their worktrees apart and the admin
can still read the directory name.

**Version** — The SHA-256 of a file's bytes, as hex. A read returns the
content and its version; a save carries the **base version** it was edited
from, and is refused as a **conflict** (409, with the current content) when
the file on disk has moved on, whether from another browser or from an IDE
on the host.

**State directory** — Where the coding server keeps what outlives the
process: `sessions.json`, `editable.json`, `worktrees.json` (username to
slug, path, and branch, per project root), and the worktrees themselves.
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

**Bench** — The simulation core shared by the bench page and the coding
server's Simulate tab: the catalog, the runs so far, and the routes that
start a run and follow it. One run at a time per bench. The bench page
has one; the coding server makes one per worktree through a **bench
factory**, so one run at a time per user and two users may run at once,
each in its own child JVM, and a user's status lists only their runs.
Given a project it builds that project's robot and simulator before
every run, and refuses a project with no simulator of its own, since the
child would fall through to the server's; without one (the tests) it
runs on the current classpath. Class: `SimBench`.

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
simulator and tests) and the jar its robot classes come from; the FTC
SDK, Road Runner, and the FtcRobotController module's jar, which no
project rebuilds, are libraries. So a class a project lacks is missing —
in its build, in its child, and to the navigator — rather than quietly
the server's. Method: `SimBuild.libraries()`.

**Bench page** — The standalone page for one developer at
`./gradlew :TeamCode:simDev` (http://localhost:8765/), with no login.
Class: `SimDevServer`.

**Catalog** — The op modes the simulator can run, exactly as the robot
controller lists them: every concrete `OpMode` of ours that carries the
`@Autonomous` or `@TeleOp` annotation, plus whatever a **registrar** (a
static method annotated `@OpModeRegistrar`) registers, found by calling
the registrar itself. The autos are registered that way: each `@Auto`
annotation on a plan method in `Plans` is one op mode, with no class of
its own. Entries are keyed by the op mode's **name**, which the robot
controller requires to be unique, and carry a **kind** (*auto* or
*teleop*) and **where** a person finds the code (a class, or the plan
method). Nested classes are never listed, so test op modes stay out of
the real bench, and Road Runner's vendored code is not ours to simulate.
The child reports the catalog after each build, so a newly written op
mode appears without a restart. Class: `SimCatalog`.

**Child** — The fresh JVM each run executes in, launched on the newly
built classes — the project's robot sources and its simulator, built
together — and the libraries, nothing else. Every class identity is
consistent, static state starts clean, and a hung op mode is a process
that can be killed. It says its **protocol** first, builds the catalog, waits to
be **placed**, then prints the run stream; the op mode's own output goes
to stderr, after a line naming the robot the run is on. Its standard
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
by name, with the fix, when a seed is set. A tick's
line is also the form the replay page reads, so a run the bench knows
only by its lines is the same page the child wrote from its own
recording: the page reads either **source**. Class: `SimRunStream`;
`SimReplayPage.Source`.

**Run** — One execution of one op mode on a fresh simulated robot, on the
simulation's own clock: the world moves one **loop period** between one op
mode loop and the next, whatever the machine is doing, so a run is the
same tick for tick every time it is made. The period is 20 ms exactly on
a robot without noise, and on a robot with **noise** it varies the way a
real loop does, the same sequence for the same seed. A run's **pace** says whether it is held to real time:
a run somebody drives or watches live is (`REAL_TIME`), a run nobody
watches, such as a test, goes as fast as the machine can (`FASTEST`).
A run has a **phase** (*building*, *starting* while the child JVM
loads the catalog, *running* from the moment the op mode's time begins,
*finished*), a
**message** (compile errors, or why it was killed), and when finished an
**outcome**: *done*, *stopped*, *timed out*, *build failed*, *wrong
protocol*, or *child exited with code N*. It starts from the op mode's
**start pose**, on the op mode's **seed**, as they were when the run was
started. An **auto run** is done when its plan is, and times
out when the plan is not done within the **run timeout** (60 s on the
bench, of simulated time). A **TeleOp run** has no plan: it is driven from the controller
until the driver presses Stop, or is done when its **period** is over
(120 s on the bench, a match's driver-controlled period). Either kind is
*stopped* when the driver presses Stop. Class: `SimRunner`; `SimRunner.Pace`.

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
pose, the plan's current step (empty for a TeleOp), the drive powers, the
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
the op mode under `TeamCode/build/sim`. The field is drawn in three
dimensions from a camera that orbits it (drag to turn, scroll to zoom,
double-click for the audience's view): the walls at their height, the
field elements, tape and game pieces of the field model, the hives where
each tick says they lean, the balls the simulator moves where each tick
puts them, the robot as a cube turned to its
heading, and the dashboard's field overlay projected onto the floor. The
obstacles are outlined on the floor. The model and the sizes are the
simulated robot's, so the page draws what the simulator collides. Class:
`SimReplayPage`.

**Live view** — The same page in live mode, following a run while it is
still adding ticks. Class: `SimLiveServer`.

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
drops into the flywheel and leaves at a speed set by the flywheel's, on
an arc under gravity. A ball that goes in through an upturned cell's
mouth rests in it, on the floor at the back; one that meets a wall or a
back bounces off, whichever side it comes from; one that comes down on the
floor rolls on; one that clears a
wall is out. The model is planar apart from that flight and the flowers'
stacks: only the robot's footprint collides, nothing goes over a wall or
under a hive by being low, and a flying ball meets only the hives, the floor
and the walls. What a body on the floor meets is what it can **reach**: a
ball rolls under an obstacle whose underside clears it — a flower's pipes
begin four inches up — and the robot, being eighteen inches tall, runs into
the same obstacle. No
sensor noise, ever: the sensors read exactly what the robot did, as the
hub would report it, which for an encoder's velocity is to the nearest
20 ticks per second (Road Runner's encoder wrapper reads the remainder
as a count of the hub's 16-bit overflows, so anything finer is read as
tens of inches a second). The
launcher's throw and the turntable's speed are guesses until measured on
the robot, calibrated so the code's close launch from its launch distance
drops into the middle of the upturned cell's mouth, which a hive holds
five feet up: the throw is a steep one. The world moves in steps of at
most 5 ms whatever the loop rate, so nothing is jumped over. Class:
`SimRobot`.

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
at 20 ms: what every run had before there was noise. Class: `SimNoise`;
`SimRobot.setDown`.

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
away from the audience, +y to the audience's left. Class: `SimField`.

**Flower** — One of the four towers at the walls, whose four pipes make the
**bore** a stack of pollen stands in: a circle on the floor, the **axis**
midway between the pipes and the radius the nearest of them leaves clear,
with the **gap** between two neighbouring pipes narrower than a pollen, so
what is in the bore stays in it. The bore's wall begins at the **lip**, the
height the pipes start at; below that it reaches nothing but the **nest**.
Class: `SimField.Flower`.

**Nest** — The ring in a flower's base plate that the ball at the bottom of
its bore sits in the middle of. To leave, that ball has to roll up over the
ring carrying whatever rests on it, so the nest pushes it back toward the
axis — hardest at the bore's rim, not at all in the middle — and the weight
on it drags it to a stop there rather than letting it roll about. A nest
holds one ball, the one nearest its middle; another in the bore is on the
plate around it and free. Method: `SimRobot.holdTheNests`.

**Stack** — The four pollen a flower is set up with, standing one on another
in its bore from the floor up. Each rests on what is under it and falls onto
it under gravity when there is nothing there, landing and settling; the stack
does not move until something takes the bottom pollen out of the nest. What
can is the robot's own push, directly or through the balls between: its drive
is behind that push and the nest's ring is no barrier to it, and once a push
has rolled a ball up off its seat the ball is out of the nest until it comes
to rest again. A loose ball rolling in is far too light — it knocks the
pollen a little way up the ring and the ring rolls it back. When the bore's
floor does come clear, what is left comes down one place and stands again; a
ball that comes to rest in a bore holds a stack up as well as a pollen of its
own does. Method: `SimRobot.fallInTheFlowers`.

**Hive** — An alliance's see-saw, hanging over the middle of the field
from the axle the frame's top bar holds, with a **cell** at each end.
Everything a hive is made of is given in the hive's own frame — the
origin on the axle, +x along the beam toward the scoring cell with the
beam level, +y the field's and +z up — and the **tilt** it leans at, in
degrees above level, is all that says where that is on the field. The
field is set up with the blue hive leaning 30 degrees toward its scoring
cell and the red hive 30 degrees the other way; a hive that **tips**
leans the same 30 degrees the other side of level. Class:
`SimField.Hive`.

**Cell** — The basket at one end of a hive, which a launched ball scores
in: the opening the CAD's goal ribs frame — the **mouth**, twenty inches
across and fourteen high — swept twelve inches to the **back** that
closes it, with a **wall** between every pair of the mouth's corners. A
ball that crosses the mouth going in is in the cell; one that meets a
wall or the back bounces off it, whichever side it comes from; nothing
else leaves. Each hive has an **audience** cell and a **scoring** cell,
named for the end of the field they face. Class: `SimField.Cell`.

**Upturned** — Of a cell: its mouth faces up, so it holds what goes in,
resting on the floor at the back. The cell at the other end of the same
hive is **downturned**, mouth facing down, and whatever is in it rolls
out of the mouth and falls to the floor — which is what a hive tipping
does to the cell that goes under. One cell of each hive is upturned at a
time, and that is the one an alliance can score in. Class:
`SimField.Cell.upturnedAt`; `SimRobot.upturnedCell`.

**Load** — How full a hive is, where one is full: a nectar is a fifth of
it and a pollen an eighth, so five nectar fill a hive, or eight pollen, or
a combination worth as much — the three nectar a hive is set up with are
three fifths of it, and four pollen finish the job. A hive that is full
tips, and tipping empties it. Method: `SimRobot.load`.

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
