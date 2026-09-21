# Glossary

The words this project uses for the coding server, the simulator and the
robot, as the code uses them. When a term here and a name in the code
disagree, fix one of them. Robot-side vocabulary is covered only where a
module has been deepened; plans, steps and the other subsystems are not yet.

## The robot

**Where things live** — `teamcode` holds the **Robot** and nothing but the
subsystems it is made of, so the list of files is the list of parts.
Underneath it: `base` (what a subsystem *is* — `Loopable` — where it prints
— `Prints`, `Channels` — and the `Alliance` a run plays for), `opmode` (what the
driver station lists and how a plan becomes one), `hardware` (the devices
and what is wired to what), `control` (the controllers and filters a
subsystem steers by), `planrunner` (plans and steps) and `roadrunner`
(trajectories, which we own rather than vendor).

**Subsystem** — One part of the robot, and now only that: something the
robot ticks, which is to say a `Loopable` that lives directly in
`teamcode`. There is no base class to extend. A subsystem is built with
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
remembered, and a test says so: it finds every `Loopable` in `teamcode` and
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
does not blank the list), whether they are **deletable** (whether a
**delete** would go through rather than be refused — said by the server,
since the server is what enforces it, and false for a user git cannot be
read for, because what a delete would throw away is exactly what could not
be counted), how their last pull or push ended, and Pull and
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
named by its **key**. A user may only ever name a file by exact match
against this set; nothing a user sends is resolved against the filesystem.
The admin picks from the host checkout; the key means the same path in
every worktree. Class: `EditableSet`.

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

**Develop branch** — `develop`, the permanent branch every user's work
starts from, pushes to, and pulls from. The server requires it to exist
and never deletes or rewrites it: it only adds merge commits to it.
Getting `develop` to `main` is the coach's job, by pull request.

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

**Part** — One piece of the dashboard page that starts on its own: the
editor, the branch panel, the simulator, the header, the file list, the
tabs. A part that throws is caught, named in a banner at the top of the
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

**Waits** — How long a bench waits for each thing it waits for, said once by
name: a run's **timeout** and a TeleOp's **period**, both in simulated seconds;
the **kill grace** a child has after Stop; the **startup** the op mode's time
has to begin, which a child JVM's loading is inside; and the **silence** the
child may say nothing at all for mid-run before it is killed for hanging. It is
made from what the bench waits for a person watching a run — a match's periods,
and room to load — and said differently where a test means something different.
Five bare seconds in a row at a call site was five chances to hand the wrong one
to the wrong wait, and the silence was the one nobody could say at all, so a
test of a hung op mode sat through the real five. Class: `SimBench.Waits`.

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
the flat drawing, `?cost` for **what a frame costs**, `?detail=full` for the
full **detail**. The dashboard passes them through, so `#simulate?detail=full`
reaches the view it embeds. Classes: `SimLiveServer`; `SimBench` serves the
same page per run. Files: `webcam.js`; `framecost.js`.

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

**Drawn floor** — The dark plane the field scene puts under everything, and
scenery rather than geometry: the field's own floor plane is z = 0, where the
tape lies and the game pieces rest. The drawn floor sits `FLOOR_DROP_IN`
below it, because **full detail** brings the CAD's soft tiles whose top
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
than guessed: `/sim/runs/<id>/?cost` takes the reading in whatever browser
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
`./gradlew :TeamCode:testDebugUnitTest --tests '*SimBenchTest*'`.

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
Changing the detail, or rebuilding after the pipeline changes, is a build and
not another download. **Refresh** is still there and is the two in one, which
is what a server with nothing fetched runs at startup. At
`POST /admin/assets/download`, `POST /admin/assets/build?detail=…` and
`POST /admin/assets/refresh?detail=…`, with a button apiece on the admin
page.

**Detail** — Which field a page draws, and it differs in two ways rather
than one. **Normal** is the field as it plays: 305 parts, snapped to a
twentieth of an inch, 316,000 triangles and four megabytes. **Full** is every
part the export holds at the points the export holds them: 1,029 parts,
1,474,000 triangles and twenty megabytes. The difference in parts is the
larger of the two — two thirds of the assembly is the hardware that holds the
field together, dropped by name for normal because nothing draws a washer once
the field is up, and kept for full because *full* has to mean what it says.
The **build** makes either or both from one **export**, and keeps them side by
side, so comparing the two on a tablet is a query string rather than another
fetch. A page asks with `?detail=full`, and one that asks for a model nobody
built draws the normal one **and says on the page that it fell back**. Full
detail costs about five times the triangles and the bytes, and almost no draw
calls, since **batching** is by material rather than by part. Which parts a
model is made of is `FieldGlb.Keep`, named at every call rather than defaulted,
so a model that quietly drops two thirds of the CAD cannot be built by
forgetting to say.

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
clamp one pose. Class: `SimPlacement`; `SimPlacementTest`.

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
