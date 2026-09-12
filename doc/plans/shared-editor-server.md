# Coding server: a name, and a memory

## Batch 5 · rename, persistent sessions, persistent editable set

The shared editor is now the **coding server**: class `CodingServer`, task
`./gradlew :TeamCode:codingServer`, ports from `CODING_ADMIN_PORT` and
`CODING_USER_PORT`, and the pages say so.

Restarting the host used to log everyone out and forget which files were
editable. Both now outlive the process, in the XDG state directory on the
host: `$XDG_STATE_HOME/midnight-snackers/coding-server`, else
`~/.local/state/midnight-snackers/coding-server` (a relative `XDG_STATE_HOME`
is ignored, as the spec says). The directory and its files are readable by
this user only.

- `sessions.json` holds every session's id, username, address, state, and
  creation time. The cookie is `session=<id>.<secret>`; the store holds a
  **salted scrypt hash** of the secret (N=2^14, r=8, p=1, 16-byte salt,
  32-byte hash) with its parameters, never the secret and never a fast
  digest of it. A cookie is checked against the hash once per process, then
  the cookie value itself is remembered in memory. Ids keep counting after
  a restart.
- `editable.json` holds the editable set keyed by the project root's
  absolute path, so two checkouts on one machine keep separate sets.
- Each file is written whole and moved into place. A save that fails is a
  500 naming the file (and a login that could not be saved is not created).
  A store that exists but cannot be read stops the server from starting,
  naming the file, rather than silently starting over.

Tests first (`CodingServerTest`): approved, pending, and revoked sessions
survive a restart, and a pending one can still be decided; new logins get
fresh ids; the store never contains a secret and its hash is scrypt of the
secret under the stored salt, with a distinct salt per session and
owner-only permissions; a real id with the wrong secret is no session; an
unreadable store stops startup; the editable set survives a restart and is
kept per root; nothing is written under the project root; the XDG rules.

---

# Shared editor: compile on save, then saved edits in Simulate

## Batch 4 · compile on save

Every successful save in the Edit tab asks the server to compile
(`GET /build`, cached by the source fingerprint, so an unchanged tree costs
nothing). The reply is structured: `ok`, and `problems` as root-relative
file, line, and message. The tab shows "compiles" or the problems under the
editor; clicking one moves the cursor to that line. The server never
rebuilds while a run is in progress, since the child is executing from the
current classes; it answers with the last result instead. A bench with no
source root answers `available: false`.

Tests first: structured problems from the build step; the route after a good
save and after a broken one (file and line match what the editor holds); the
route during a run returns the previous result and the run is unharmed;
unapproved sessions get 403; the page markers.

---

# Batch 3 · saved edits take effect in Simulate

Batch 3. Batch 1 (login, approval, editing) shipped in PR #6; batch 2 (the
Simulate tab) in PR #7. This batch removes the restart: a run always executes
the main sources as they are on disk at the moment Run is pressed.

## The approach, and why

Each run recompiles the main sources with the JDK's own compiler (about a
second for the whole tree, measured) and then runs the op mode in a **fresh
child JVM** whose classpath starts with the new classes. One class loader per
process means every class identity is consistent, so every file under
`TeamCode/src/main/java` takes effect, not only the autos. Each run starts
from clean static state. A hung op mode is a process the server can kill.
Compile errors come back as text and are shown in the tab.

Rejected: reloading in the same JVM (the simulator names main classes by type
in the base, plan runner, and road runner packages, so only the auto
subclasses would really reload and edits elsewhere would be silently
ignored); compiling with Gradle (works concurrently, but ten seconds plus the
jar bundling, for nothing the simulator needs); restarting the server (drops
every session).

## Guards

- **No Kotlin.** If a `.kt` file ever appears under `TeamCode/src/main`, the
  build step fails with a message naming it. It never skips the file.
- **A JDK, not a JRE.** If the running JVM has no compiler, the build step
  fails saying so. (Android Studio's bundled runtime has one.)
- **Build output lives under `TeamCode/build/sim/classes`**, never in a tree
  Gradle owns. Only the latest build is kept.
- **A child that dies without reporting an outcome is an outcome**: the run
  ends "child exited with code N" and its stderr tail is the message.
- **Student `System.out` cannot corrupt the stream.** The child redirects
  `System.out` to stderr at startup and keeps the real stdout for the protocol.

## Design

`SimBuild` compiles a source root against the running JVM's classpath into a
fresh directory, caching by a fingerprint of the sources (path, size, mtime).
It returns the classes directory, or null plus the diagnostics.

`SimChild` is the child JVM's main. `--list` prints the catalog as JSON.
`--run <class> <timeout> <replayDir>` runs the op mode through the existing
`SimRunner.record`, prints each tick as one JSON line as it happens, then one
`{"outcome": ...}` line. The parent launches it with
`java -cp <newClasses>:<its own classpath>`.

`SimBench` keeps a run's ticks as JSON and serves the same routes as today
plus `/runs/<id>/log` (the child's stderr). Status gains `phase`
(`building`, `running`, `finished`) and `message` (compile errors, kill
reason). The catalog is asked of the child after each new build, so a newly
written auto appears without a restart. A bench without a source root (the
tests) skips the build and runs the child on the parent's classpath.

`SimCatalog.Entry` gains `className`; `type` is only set when the class is
loadable here. Discovery scans every classpath entry for the auto package,
first wins, the way class loading does.

## Phases

Gate: `./gradlew :TeamCode:testDebugUnitTest`, locally and in CI.

### Phase 0 · The build step

Tests first (`SimBuildTest`):
- a temp source root with one class compiles; the class file is in the
  returned directory;
- a syntax error returns no directory and diagnostics naming the file and line;
- an unchanged tree returns the same directory without recompiling; an edited
  file yields a new directory;
- a `.kt` file under `src/main` fails naming the file;
- the real `src/main/java` compiles.

### Phase 1 · The child

Tests first (`SimChildTest`, `SimCatalogTest`):
- `--list` prints the real autos with name, group, and class;
- `--run` of "Count to three" prints three tick lines and an outcome line, and
  writes the replay page;
- an auto that prints to `System.out` does not break the stream;
- a catalog entry parsed from JSON has a class name and no type.

### Phase 2 · The bench over the child

Tests first (`SimBenchTest`, `SharedEditorServerTest`):
- a run through the child ends "done" with three ticks and the replay file;
- an auto whose `loop()` never returns is killed after timeout plus grace and
  the outcome says so;
- a source root with a compile error ends the run "build failed" with the
  diagnostics as the message, and the catalog route reports the error;
- **the contract:** write an auto into a temp source root, run it, edit its
  source, run again; the second run reflects the edit, no restart;
- **through the editor:** the admin marks the auto editable, the user saves a
  new body with `PUT /files/…`, presses Run, and the status shows the edit;
- `/sim/runs/<id>/log` returns the child's stderr; `/sim/status` carries
  `phase` and `message`.

Then: both servers take a `SimBench`; `main` builds one over
`TeamCode/src/main/java`.

### Phase 3 · The pages

Tests first: the dashboard and the bench page show the building state and
the message, and the dashboard refreshes the catalog when the tab opens and
when a run ends.

Manual acceptance on two machines: edit an auto's plan from a teammate's
machine, press Run, watch the new plan drive; introduce a syntax error, press
Run, read the error in the tab; fix it, run again.

## Definition of done

Branch, pull request, green `tests` workflow, landed.
