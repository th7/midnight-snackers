# Glossary

The words this project uses for the coding server and the simulator, as the
code uses them. When a term here and a name in the code disagree, fix one of
them. Robot-side vocabulary (plans, steps, subsystems) is not yet covered.

## The coding server

**Coding server** — The host process teammates reach over the LAN to edit
the robot code and run simulations. One machine runs it with
`./gradlew :TeamCode:codingServer`. It has two listeners: the **admin
listener** on loopback only (port 21987, `CODING_ADMIN_PORT`), and the
**user listener** on every interface (port 21986, `CODING_USER_PORT`).
Class: `CodingServer`.

**Admin** — The person at the host machine. Only they can reach the admin
listener, where they decide logins and pick the editable set. There is no
admin login; being on the machine is the credential.

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
(**ahead**), and the commits `develop` has that the branch lacks
(**behind**).

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
otherwise. `develop` does not change. It needs a clean worktree (commit
first) and refuses, changing nothing, on a merge conflict. The Pull
button pulses, and wears the count, while the branch is behind.

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

**Build** — Compiling the main sources as they are on disk with the JDK's
own compiler, cached by a fingerprint of the tree. The Edit tab asks for a
build after every save and shows the **problems** (file, line, message).
In the coding server the sources are the user's worktree, so one user's
broken edit breaks only their own build. Not the Android build: no
Kotlin, no desugaring. Class: `SimBuild`.

## The simulator

**Bench** — The simulation core shared by the bench page and the coding
server's Simulate tab: the catalog, the runs so far, and the routes that
start a run and follow it. One run at a time per bench. The bench page
has one; the coding server makes one per worktree through a **bench
factory**, so one run at a time per user and two users may run at once,
each in its own child JVM, and a user's status lists only their runs.
Given a source root it builds before every run; without one (the tests)
it runs on the current classpath. Class: `SimBench`.

**Bench page** — The standalone page for one developer at
`./gradlew :TeamCode:simDev` (http://localhost:8765/), with no login.
Class: `SimDevServer`.

**Catalog** — The autonomous op modes the simulator can run: every
concrete `AutoOp` in the auto package with the `@Autonomous` annotation.
The child reports it after each build, so a newly written auto appears
without a restart. Class: `SimCatalog`.

**Child** — The fresh JVM each run executes in, launched with the newly
built classes first on its classpath. Every class identity is consistent,
static state starts clean, and a hung op mode is a process that can be
killed. It prints each tick as one JSON line and finally the outcome; the
op mode's own output goes to stderr. Class: `SimChild`.

**Run** — One execution of one op mode on a fresh simulated robot, in real
time. A run has a **phase** (*building*, *running*, *finished*), a
**message** (compile errors, or why it was killed), and when finished an
**outcome**: *done*, *timed out*, *build failed*, or *child exited with
code N*.

**Tick** — One entry in a run's recording, one per op mode loop: the true
pose, the plan's current step, the drive powers, and the dashboard packets
drawn that loop. Class: `SimRecording`.

**Replay** — A run's ticks written as a single self-contained HTML page
with the field, the true pose, and play/pause/scrub controls, named after
the op mode under `TeamCode/build/sim`. Class: `SimReplayPage`.

**Live view** — The same page in live mode, following a run while it is
still adding ticks. Class: `SimLiveServer`.

**Simulated robot** — A kinematic model on a flat field: motor powers
become wheel velocities through the tuned drive model, the true pose is
integrated from those, and the localizer's sensors are written back from
the true pose. No inertia, slip, or noise. Class: `SimRobot`.

**True pose** — Where the simulated robot actually is, as opposed to where
the localizer believes it is.
