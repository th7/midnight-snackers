# Glossary

The words this project uses for the coding server, the simulator and the
robot, as the code uses them. When a term here and a name in the code
disagree, fix one of them. Robot-side vocabulary is covered only where a
module has been deepened; plans, steps and the other subsystems are not yet.

## The robot

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
admin login; being on the machine is the credential. The admin page
lists every login with its worktree's **status** (the same changed,
ahead and behind that `GET /git/status` gives the user; null until the
worktree exists, and null with a **statusError** when git cannot read
it, so one broken worktree does not blank the list), how its last pull
or push ended, and a Pull button.

**User** — A teammate on the LAN who has logged in with a username. Users
reach only the user listener.

**Session** — One login by one user, identified by a small integer **id**
the admin sees, and proven by a **token** the browser holds in the
`session` cookie as `<id>.<secret>`. A session moves through four
**states**: *pending* (asked, not yet decided), *approved* (may edit and
simulate), *denied* (the admin said no), and *revoked* (was approved, no
longer is). Only approved sessions reach files, builds, and the simulator.

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

**Commit** — `POST /git/commit` with a message: every uncommitted change in
the user's worktree becomes one commit on the user branch, authored by
the username. Nothing to commit is a success that says so. An empty
message is refused. `GET /git/status` reports the uncommitted files
(**changed**), the commits the branch has that `develop` lacks
(**ahead**), the commits `develop` has that the branch lacks
(**behind**), and the branch's tip (**head**), which moves on a commit
and on any pull or push, whoever asked for it.

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
branch is brought up to date without their browser. Their editor
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
the mapping is decided once and stored, never recomputed.

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
that can be killed. It says its **protocol** first, builds the catalog, then prints
the run stream; the op mode's own output goes to stderr. Its standard
input is the driver station, one line at a time; when the input ends,
the run ends stopped. Class: `SimChild`.

**Run stream** — The lines the child prints, one JSON object per line, in
order: which protocol it speaks, one that the op mode's time has begun,
each tick as it happens, and finally the outcome. Written and read in one
place, and every way a run can end is named there. But the child runs the
project's simulator, which may be another version of this code than the
bench, so the lines carry a **protocol** version: the bench refuses, by
name and with whose the fix is, a child newer than itself or older than
the oldest it still reads, rather than misreading it; and a test pins
what a child of that oldest version printed. A child from before the
version line prints content first and is that oldest version. A tick's
line is also the form the replay page reads, so a run the bench knows
only by its lines is the same page the child wrote from its own
recording: the page reads either **source**. Class: `SimRunStream`;
`SimReplayPage.Source`.

**Run** — One execution of one op mode on a fresh simulated robot, in real
time. A run has a **phase** (*building*, *starting* while the child JVM
loads the catalog, *running* from the moment the op mode's time begins,
*finished*), a
**message** (compile errors, or why it was killed), and when finished an
**outcome**: *done*, *stopped*, *timed out*, *build failed*, *wrong
protocol*, or *child exited with code N*. An **auto run** is done when its plan is, and times
out when the plan is not done within the **run timeout** (60 s on the
bench). A **TeleOp run** has no plan: it is driven from the controller
until the driver presses Stop, or is done when its **period** is over
(120 s on the bench, a match's driver-controlled period). Either kind is
*stopped* when the driver presses Stop.

**Driver station** — What the driver does during a run: the state of the
two **gamepads** and the **Stop** button. The controller page sends it to
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
dashboard packets drawn that loop, where the loose game pieces are, and
for a TeleOp what each gamepad read. Class: `SimRecording`.

**Replay** — A run's ticks written as a single self-contained HTML page
with the field, the true pose, and play/pause/scrub controls, named after
the op mode under `TeamCode/build/sim`. The field is drawn in three
dimensions from a camera that orbits it (drag to turn, scroll to zoom,
double-click for the audience's view): the walls at their height, the
field elements, tape and game pieces of the field model, the loose
pieces where each tick puts them, the robot as a cube turned to its
heading, and the dashboard's field overlay projected onto the floor. The
obstacles are outlined on the floor. The model and the sizes are the
simulated robot's, so the page draws what the simulator collides. Class:
`SimReplayPage`.

**Live view** — The same page in live mode, following a run while it is
still adding ticks. Class: `SimLiveServer`.

**Simulated robot** — A kinematic model on the field: motor powers become
wheel velocities through the tuned drive model, the true pose is
integrated from those and kept inside the walls and out of the obstacles,
and the localizer's sensors are written back from the true pose, so the
dead wheels read nothing while the wheels spin against a wall. The robot
is an 18-inch cube; a wall or an obstacle stops it dead and lets it slide
along. The **loose pieces** are balls the robot pushes ahead of itself:
they roll on with the speed they were given, slow to a stop, and stop at
the walls, the obstacles and each other, with a little bounce; nothing
pushes the robot back. The model is planar: only the robot's footprint
collides, and nothing goes over a wall or under a hive by being low. No
inertia, slip, or noise. The world moves in steps of at most 5 ms whatever
the loop rate, so nothing is jumped over. Class: `SimRobot`.

**Field** — The season's field as the simulator has it, reduced from
FIRST's CAD by `tools/field/step_to_field.py` to `field.json`: the
**size** between the walls (141 inches) and the walls' height, each
**field element** as a low-poly convex shape with its colour, the **game
pieces** where a match starts, the gaffer **tape** on the floor, and the
**obstacles**: the convex footprint of every element that stands lower
than the robot is tall, which is what the robot runs into. For BIOBUZZ:
the frame in the middle of the field is an obstacle leg by leg and foot
by foot, so the robot drives through it; the flowers at the walls are
obstacles; the hives hang from the frame's top bar above the robot and
are only drawn, each cell as its six flat **panels** (two sides, a
bottom, two tops and a back), seen through and outlined in the alliance's
colour. The game pieces on the floor in the open are **loose**, the
simulator's to roll; the rest (the flowers' stacks, the rows outside the
walls, the nectar in the hives) stay put. Everything is in the field frame
Road Runner uses, in inches: the origin at the centre, +x away from the
audience, +y to the audience's left. Class: `SimField`.

**True pose** — Where the simulated robot actually is, as opposed to where
the localizer believes it is.
