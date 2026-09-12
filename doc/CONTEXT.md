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
the filesystem.

**Project root** — The repository checkout the coding server serves. All
paths users see are relative to it, and the editable set is stored per
project root.

**Version** — The SHA-256 of a file's bytes, as hex. A read returns the
content and its version; a save carries the **base version** it was edited
from, and is refused as a **conflict** (409, with the current content) when
the file on disk has moved on, whether from another browser or from an IDE
on the host.

**State directory** — Where the coding server keeps what outlives the
process: `sessions.json` and `editable.json`. It follows the XDG Base
Directory convention: `$XDG_STATE_HOME/midnight-snackers/coding-server`,
else `~/.local/state/midnight-snackers/coding-server`. Owner-only. Session
secrets are stored there only as salted scrypt hashes.

**Build** — Compiling the main sources as they are on disk with the JDK's
own compiler, cached by a fingerprint of the tree. The Edit tab asks for a
build after every save and shows the **problems** (file, line, message).
Not the Android build: no Kotlin, no desugaring. Class: `SimBuild`.

## The simulator

**Bench** — The simulation core shared by the bench page and the coding
server's Simulate tab: the catalog, the runs so far, and the routes that
start a run and follow it. One run at a time, whoever asks. Given a source
root it builds before every run; without one (the tests) it runs on the
current classpath. Class: `SimBench`.

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
