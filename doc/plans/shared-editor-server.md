# Coding server: a worktree per user, and Commit, Push, Pull buttons

## Batch 7 · every user edits and runs in their own git worktree

Today every approved user edits the host's working tree directly. Two users
on one file trade 409 conflicts, one user's half-typed syntax error breaks
everyone's build and catalog, and the admin's own checkout changes under
them while they work. This batch gives each user a **git worktree** of the
project on a branch of their own, started from the permanent **develop
branch**. Their saves land there, their runs are built from there, and the
host checkout is never written by a user's save. A **Commit** button turns
their edits into commits on their branch, a **Push** button lands those
commits on `develop`, and a **Pull** button brings `develop` into their
worktree, animating while there is something to pull. A push or pull that
conflicts changes nothing and sends the user to their coach.

## Terms (to add to the glossary)

**Develop branch** — `develop`, the permanent branch every user's work
starts from, pushes to, and pulls from. The server requires it to exist
and never deletes or rewrites it: it only adds merge commits to it.
Getting `develop` to `main` is the coach's job, by pull request as today.

**Worktree** — A git worktree of the project root's repository, one per
**username**, on its own **user branch**. All of that user's edits are
written there and every run they start is compiled from there. Owned by
the username, not the session, so logging in again after a restart or a
revoke-and-reapprove finds the same work.

**User branch** — `coding/<slug>`, created at the tip of `develop` when
the worktree is made. Saves are uncommitted changes in the worktree until
the user presses Commit.

**Slug** — The username lowercased, every run of characters outside
`[a-z0-9]` replaced by one `-`, trimmed of leading and trailing `-`, at
most 32 characters. It names the branch and the worktree directory. Two
usernames that collapse to one slug get `-2`, `-3`, … so the mapping is
decided once and stored, never recomputed.

**Worktrees directory** — Where the worktrees live: under the state
directory, at `worktrees/<root name>-<8 hex of SHA-256(root path)>/<slug>`,
so two checkouts on one machine keep their worktrees apart and the admin
can still read the directory name. Nothing is written under the project
root except what git itself records under `.git`.

**Worktrees store** — `worktrees.json` in the state directory, keyed by the
project root's absolute path like `editable.json`: username to slug, path,
and branch. Read at startup; an unreadable store stops the server, naming
the file, like the other two stores.

**Commit** — `POST /git/commit` with a message: every uncommitted change in
the user's worktree becomes one commit on the user branch, authored by
the username. Nothing to commit is a success that says so. An empty
message is refused.

**Push** — `POST /git/push`: the user branch is merged into `develop` with
a merge commit, and then the user branch and worktree are fast-forwarded
to the new `develop`, so the user carries everyone's pushed work from
then on. It needs a clean worktree (commit first) and refuses, changing
nothing, on a **merge conflict**. The name is the user's view of it: the
server never talks to a remote.

**Pull** — `POST /git/pull`: `develop` is merged into the user branch, in
the user's worktree, a fast-forward when it can be and a merge commit
otherwise. Nothing on `develop` and on the user branch changes in
history. It needs a clean worktree (commit first) and refuses, changing
nothing, on a merge conflict. The Pull button animates while `develop`
has commits the user branch does not.

**Merge conflict** — `develop` and the user branch changed the same lines
since they diverged. Not the save **conflict** (a stale base version on
`PUT /files`), which keeps its name. Push and Pull hit the same
conflict, since both merge the same two branches; either changes nothing
anywhere, the Edit tab names the files and says to ask the coach, and the
admin page shows the coach the recipe.

**Bench (revised)** — One per worktree, made on first use by a
**bench factory** the server is given at start. One run at a time *per
user*; two users' runs may be in progress at once, each in its own child
JVM. `/sim/status` and the run list are the user's own.

## Design

### Worktrees

- **Creation.** The admin's approve request creates the worktree (an
  idempotent `ensure(username)`: `git worktree add -b coding/<slug> <path>
  develop`). If git fails, the approval is refused with a 500 carrying
  git's message and the session stays pending, so the failure shows in
  front of the admin, not at a user's first save. Every file, build, git,
  and sim route also calls `ensure`, which is an existence check when the
  worktree is already there.
- **Starting point.** The tip of `develop` at creation. The host
  checkout's own state, committed or not, is irrelevant to a new worktree.
- **Reuse.** A second login with an existing username, once approved, gets
  the existing worktree. Revoking keeps the worktree. Nothing in this
  batch deletes one; the admin can `git worktree remove` by hand.
- **A missing directory** (someone deleted it) is recreated by `ensure` on
  the existing branch after `git worktree prune`, and a line saying so is
  printed. Uncommitted work in a deleted directory was already gone;
  committed work is on the branch and comes back.
- **Editable set.** Unchanged. Keys are root-relative and mean the same
  path in every worktree. The admin still browses and picks from the host
  checkout. A picked file a user's branch does not have yet reads as 404
  naming it, as a missing file does today; it appears after their next
  Pull or Push.
- **Versions and save conflicts.** Unchanged. A save conflict now means
  the same user's other browser, the admin editing that worktree on the
  host, or a Pull or Push that just moved the file under the open editor,
  which the tab handles by reloading, as it does today.
- **Who has a file open** stays across all users, by key: it now says who
  else is working on the same file in their own copy.

### Commit

`git -C <worktree> add -A` then `git commit -m <message>` with
`user.name` the username and `user.email` `<slug>@coding-server.invalid`
(`.invalid` is reserved for exactly this). Build output is under `build/`,
which is ignored, and the editor's `.editing` temp files never outlive a
save, so `add -A` picks up only edits. One lock per worktree serializes
commits, pushes, and pulls with saves, so a keystroke during a commit
lands in the next one.

### Pull

1. **Clean worktree or 409** "commit first", naming the changed files.
2. **Nothing to pull or 200** when `develop` is already an ancestor of the
   user branch (`git merge-base --is-ancestor`).
3. **Detect conflicts without touching the working tree**:
   `git merge-tree --write-tree --name-only coding/<slug> develop`. Exit 1
   lists the conflicting files: reply 409 with them, record the attempt
   for the admin page, change nothing.
4. **Merge in the worktree**: `git -C <worktree> merge -m "Pull develop"
   develop`, a fast-forward when the user branch is an ancestor of
   `develop` and a merge commit otherwise. Known clean, so it can only
   refuse when a save landed after step 1 in a file the merge changes;
   git then changes nothing, and the reply is 409 "commit first" again.
5. The Edit tab reloads the file list and the open file.

**Animation.** `GET /git/status` reports `behind`, the commits on
`develop` that the user branch lacks. The Edit tab already polls the
server; it polls status too, and while `behind` is above zero the Pull
button pulses and carries the count as a badge. Under
`prefers-reduced-motion` the badge alone marks it. A pull or push clears
it on the next poll.

### Push

The coach's laptop usually runs the server *and* deploys to the robot from
`develop`, so `develop` is normally checked out in the host checkout, and
a push has to leave that working tree at the new commit. Sometimes it is
checked out nowhere. The push is the same in both cases; only the landing
step differs.

1. **Clean worktree or 409** "commit first", naming the changed files.
2. **Nothing to push or 200** when the user branch is already an ancestor
   of `develop`.
3. **Detect conflicts without touching any working tree**:
   `git merge-tree --write-tree --name-only develop coding/<slug>`. Exit 1
   lists the conflicting files: reply 409 with them, record the attempt
   for the admin page, change nothing.
4. **Land it.** If `develop` is checked out somewhere (`git worktree list
   --porcelain` says where), run `git -C <there> merge --no-ff -m
   "Push <username>'s work" coding/<slug>`, which is known clean, so the
   only way it can refuse is uncommitted host changes in a file the merge
   touches, or a host checkout mid-rebase or mid-merge. Git then changes
   nothing; reply 409 "ask your coach" and put git's message on the admin
   page. If `develop` is checked out nowhere, `git commit-tree` on the
   tree from step 3 with both parents, then `git update-ref
   refs/heads/develop <new> <old>`, which fails rather than clobbers if
   someone moved the branch meanwhile. One server-wide lock serializes
   pushes.
5. **Pull.** The same Pull as above, which is now a fast-forward by
   construction since `develop` contains the user branch. If it refuses
   (a save landed since step 1), the reply is 200 "pushed, but your
   worktree is not up to date" with git's message, and the Pull button
   animates until the user commits and pulls.

**The coach's recipe** for a merge conflict, from Push or Pull alike,
shown on the admin page next to the user's login with the real path
filled in:

```
cd <worktree path>
git merge develop         # resolve the conflicts in an editor
git add -A && git commit  # then the student presses Push
```

The coach's merge *is* the pull, so Push is all that is left. `develop`
is never checked out by the coach to resolve anything, so nothing the
coach does blocks other users' pushes.

### Requirements, checked at startup, each stopping the server naming what is wrong

- `git` on `PATH`, at least 2.38 (`merge-tree --write-tree`).
- the root is the top of a git working tree (`git rev-parse
  --show-toplevel` equals the root).
- `refs/heads/develop` exists. The message says how to make it:
  `git branch develop`.

There is no mode that edits the host checkout directly.

### Code

A new `Worktrees` class owns every git call (`ProcessBuilder` with a
timeout and captured stderr, the executable a constructor parameter so a
test can hand it a stub), the slug rule, the store, `ensure`, `status`,
`commit`, `push`, and `pull`; each returns a small result type, never a
response. Push's landing and Pull's merge share one "merge these two,
clean or conflicts" step. `CodingServer.start` takes a `SimBench.Factory`
(`Path worktree -> SimBench`) in place of a bench; `main` gives one over
`<worktree>/TeamCode/src/main/java` with output under
`<worktree>/TeamCode/build/sim`. Every route that used `root` for a
user's file uses the session's worktree; the admin's tree and picker keep
`root`. `/build` relativizes problems against the worktree. `stop` stops
every bench. New user routes, approved sessions only: `GET /git/status`
(`changed` files, `ahead` commits not on `develop`, `behind` commits not
on the user branch), `POST /git/commit` (JSON body with `message`), `POST
/git/push`, `POST /git/pull`. `/me` gains `branch`; the admin logins list
gains `worktree`, `branch`, and `lastMerge` (push or pull, outcome,
files, git's message); `/admin/info` gains `worktreesDir` and `develop`'s
commit.

The Edit tab gets a git line above the editor: the branch, "N files
changed since your last commit", Commit (asks for a message), Push, and
Pull buttons, and the last result: "committed 3 files", "pushed to
develop", "pulled develop", "nothing to pull", "your changes conflict
with develop in TeamCode/…/Plans.java; ask your coach for help", or
"commit first".

## Phases

Gate: `./gradlew :TeamCode:testDebugUnitTest`, locally and in CI (CI's
runner has git 2.38 or later; the tests never touch this repository,
only temp ones). Two pull requests: phases 0 to 2 are the worktrees,
phases 3 to 5 are the buttons.

### Phase 0 · Worktrees

Tests first (`WorktreesTest`, over a temp repo with one commit on
`develop`):
- `ensure("Ada Lovelace")` makes `<dir>/ada-lovelace` on branch
  `coding/ada-lovelace` at `develop`'s tip, not at the host's `HEAD` when
  the host is on another branch; `git worktree list` in the repo shows
  it; a committed file reads the same in both.
- `ensure` twice is the same worktree; a second username with the same
  slug gets `ada-lovelace-2` and its own branch.
- nasty usernames (`..`, `a/b`, `über`, 32 characters of punctuation) give
  valid ref names and directories; none escapes the worktrees directory.
- the store outlives a new `Worktrees` over the same state directory; an
  unreadable store throws naming the file; the file is owner-only.
- a worktree whose directory was deleted is recreated on the same branch
  with its committed content.
- a root that is not a repository, a repository with no `develop`, a git
  executable that does not exist, and a stub git that reports 2.30, each
  fail at construction naming it.

Then: `Worktrees`.

### Phase 1 · The server over worktrees

Tests first (`CodingServerTest`; the fixture's root becomes a repo with one
commit on `develop`, checked out, a `commitAll()` helper commits files a
test adds under it, and `serverWith` takes a factory):
- approving a login creates the user's worktree; a user's save changes the
  worktree's file and leaves the host checkout's bytes as they were.
- two users save the same file with different content, both 200, and each
  reads back their own.
- the same username approved again in a new session sees the earlier save.
- a picked file the user's branch lacks is a 404 naming it.
- `/me` carries the branch and the dashboard shows it; the admin logins
  list carries worktree and branch and the admin page shows them.
- a root that is not a repository stops the server from starting; nothing
  outside `.git` is written under the root (the existing test, refined).
- approved worktrees and their uncommitted edits survive a restart.
- when git refuses, approval is a 500 with git's message and the session
  is still pending.

Then: the server routes, pages, and `main`.

### Phase 2 · Builds and runs per worktree

Tests first (`CodingServerTest`, `SimBenchTest` where the factory lives):
- an edit a user saves drives that user's next run, built from their
  worktree (the existing test, on worktrees).
- one user's broken edit does not break another's: the second user's
  catalog lists and their run ends `done` while the first's ends
  `build failed`.
- two users' runs are in progress at once; a user's second run while
  their first is running is 409 naming them.
- a user's `/sim/status` shows their runs only.
- `/build` problems name root-relative files from the worktree.
- stopping the server stops every user's bench and child.

Then: the factory, one bench per worktree, `stop`.

### Phase 3 · Commit

Tests first (`WorktreesTest`, `CodingServerTest`):
- after a save, Commit with a message makes one commit on the user branch
  whose author name is the username, whose tree has the edit, and after
  which the worktree is clean; `develop` and the host checkout are
  untouched.
- Commit with nothing changed is 200 saying so and makes no commit.
- Commit without a message is 400 and makes no commit.
- `/git/status` lists the changed file before the commit and none after,
  and `ahead` goes from 0 to 1.
- unapproved sessions get 403 on every `/git` route.
- the Edit tab has the git line, the Commit button, and asks for a message
  (page markers).

Then: `Worktrees.status`, `Worktrees.commit`, the routes, the page.

### Phase 4 · Pull

Tests first (`WorktreesTest`, `CodingServerTest`):
- the fixture commits a change to `develop`; `/git/status` reports
  `behind` 1; Pull fast-forwards the user branch to `develop` and the
  worktree file reads the change; `behind` is 0; `develop` is unchanged.
- with a commit on the user branch as well, Pull makes a merge commit on
  the user branch with both parents and both edits; `develop` is
  unchanged.
- Pull with uncommitted changes is 409 "commit first" naming the file and
  nothing changes.
- Pull with nothing new is 200 saying so and makes no commit.
- a merge conflict (a different change to the same line on `develop`) is
  409 naming the file; the user branch, the worktree, and `develop` are
  unchanged; no merge is in progress in the worktree; the admin logins
  list carries the files and the recipe with the user's path.
- the Edit tab has the Pull button, animates it only while `behind` is
  above zero, shows the badge under reduced motion, shows the coach
  message on a conflict, and reloads the open file after a pull (page
  markers; the jsdom harness for the flow).

Then: `Worktrees.pull`, `behind` in status, the route, the page.

### Phase 5 · Push

Tests first (`WorktreesTest`, `CodingServerTest`):
- after a commit, Push moves `develop` to a merge commit whose parents are
  the old `develop` and the user branch, with the edit in its tree; the
  user branch now equals `develop`; the worktree file reads the merged
  content.
- with `develop` checked out in the host checkout (the fixture's default),
  the host's working tree shows the pushed content afterwards and `git
  status` there is clean.
- with `develop` checked out nowhere (the fixture switches the host to
  `main`), the push still lands and no working tree but the user's
  changes.
- two users push in turn: after the second push, `develop` has both edits
  and the second user's worktree has the first user's edit.
- Push with uncommitted changes is 409 "commit first" naming the file,
  and nothing changes.
- Push with nothing new is 200 saying so and makes no commit.
- a merge conflict is 409 naming the file; `develop`, the user branch,
  and the user's worktree are unchanged; no merge is in progress in any
  worktree; the admin logins list carries the files and the recipe.
- the host checkout, on `develop`, has an uncommitted change in the file
  the push touches: Push is 409 "ask your coach", the host's edit is
  intact, `develop` is unchanged, and the admin sees git's message.
- the Edit tab has the Push button, shows each outcome including the
  coach message, and reloads the open file after a push.

Then: `Worktrees.push`, the route, the page, the admin page.

### Phase 6 · Words

The glossary entries above; the `CodingServer` javadoc; `CONTEXT.md`'s
bench entry revised to "one run at a time per user"; a short note for the
coach: the `develop` rule, the recipe, and how `develop` reaches `main`.

Manual acceptance on two machines: both log in, both edit the same auto
differently, both press Run and each watches their own plan drive; the
first commits and pushes, the second's Pull button starts pulsing, they
pull and see the change, then commit and push their own; on the host,
`git log --graph develop` shows both pushes and the checkout has them.
Then both edit the same line, the second push is refused, the coach
follows the recipe, and the second Push lands.

## Left out, on purpose

- Deleting worktrees from the admin page.
- Showing a worktree's diff on the admin page.
- A cap on how many runs the host executes at once. One child JVM per
  user in real time is expected to be fine for a team's laptop; measure
  before adding one.

## Definition of done

Two branches, two pull requests, green `tests` workflow on each, landed.

---

# Coding server: a real editor

## Batch 6 · CodeMirror in the Edit tab

The Edit tab's textarea is replaced by CodeMirror 6: Java syntax colouring,
line numbers, bracket matching, search, multiple cursors, and a lint gutter.
The compiler problems the server already returns after each save become
diagnostics in the gutter and under the text, for the open file; the list
under the editor stays and still jumps to the line. Tab indents, Ctrl-S or
Cmd-S saves at once instead of after the pause, undo is per file, and the
editor follows the system's light or dark scheme.

**Served from the host.** Teammates are on the robot's wifi, which has no
internet, so the editor is one prebuilt bundle at `/static/codemirror.js`,
served by the user listener to anyone, session or not. `/static` serves
nothing else: an allowlist of names, so no page or class file is reachable
through it. The bundle is built by `tools/codemirror/build.sh` (npm and
esbuild, versions pinned in `package.json` and the lock file) and committed
under the test resources, so the Gradle build and CI never need node.

Tests first (`CodingServerTest`): the bundle is served with a JavaScript
content type and is not a stub; `/static` refuses other names, pages,
traversal, and the admin port; the page loads the bundle, builds a
`CM.EditorView` with `CM.java()`, `CM.lintGutter()`, `CM.setDiagnostics`,
`CM.indentWithTab`, and a `Mod-s` binding, and has no textarea; and every
`CM.<name>` the page uses is a name the bundle exports. The page's script
was also driven in jsdom against the real bundle and a mocked server
(load, autosave, build check, diagnostics, jump to line, Ctrl-S, conflict
and reload); that harness is not in CI, since CI has no node.

---

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
