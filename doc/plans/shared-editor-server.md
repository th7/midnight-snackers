# Shared editor: the Simulate tab

Batch 2. Batch 1 (login, approval, real-time editing) shipped in PR #6.

## What it does

The dashboard gets two tabs. **Edit** is the batch 1 dashboard. **Simulate**
lists the autonomous op modes, runs the chosen one on the simulated robot,
shows the live view as it drives, and keeps the history of runs with outcomes
and replays. One run at a time for everyone; the status says who started it.
Only autonomous op modes (`AutoOp` subclasses with the `Autonomous`
annotation) can be run, exactly the set the bench lists today.

## Assumptions

- **Simulate runs the code the server was started with.** Saving a file in
  the Edit tab does not change what a run executes until the server is
  restarted. The Simulate tab says this in its header. Rebuilding and
  reloading op modes without a restart is a follow-up batch.
- **The same bench core as `./gradlew :TeamCode:simDev`.** The run
  management in `SimDevServer` moves into a class both servers use, so the
  two cannot drift. The bench keeps its own page and task.
- **Runs write replays to `TeamCode/build/sim`** as the bench does.

## Design

Routes on the user listener, approved sessions only:

| Route | Does |
|---|---|
| `GET /sim/catalog` | the runnable op modes: name, group, class |
| `GET /sim/status` | running flag and the runs, newest first, each with who started it |
| `POST /sim/run?opmode=<class>` | starts a run; 409 while one is in progress |
| `GET /sim/runs/<id>/` | the live view page (polls `ticks` relatively, so it works under the prefix) |
| `GET /sim/runs/<id>/ticks?from=<n>` | ticks from `n` onward and the outcome once there is one |

The dashboard keeps the active tab in the URL hash (`#edit`, `#simulate`) so a
reload lands on the same tab. The Simulate tab's markup is its own; it is not
an iframe of the bench page, whose script uses absolute paths.

## Phases

Gate, locally and in CI: `./gradlew :TeamCode:testDebugUnitTest`.

### Phase 0 · Extract the bench core

A refactor: `SimBench` holds the catalog, the runs, `start`, `find`, and
`status`, and takes the username that started a run. `SimDevServer` becomes a
thin page-and-routes layer over it. The existing `SimDevServerTest` is the
test; it must stay green with no edits.

### Phase 1 · Sim routes on the user listener

Tests first (`SharedEditorServerTest`, catalog of the two test autos, short
run timeout):
- `GET /sim/catalog` lists "Count to three" and "Never done" with their class
  names;
- `POST /sim/run` starts a run and `GET /sim/status` follows it to
  `"outcome":"done"`, carrying `"startedBy":"ada"`;
- a second `POST /sim/run` while "Never done" is running gets 409;
- a pending session gets 403 on every `/sim` route;
- `GET /sim/runs/<id>/` serves the live page (`"live":true`) and
  `GET /sim/runs/<id>/ticks?from=0` the ticks;
- an unknown op mode is 404, `GET /sim/run` is 405.

Then build: `SharedEditorServer.start` takes a `SimCatalog`, an output
directory, and a run timeout; the routes delegate to the bench core.

### Phase 2 · Tabs in the dashboard

Tests first: the dashboard page has the tab controls, the Simulate panel with
the op mode list, the run history, and the live iframe, and its script
references `/sim/catalog`, `/sim/run`, `/sim/status`, and `/sim/runs/`.

Then build: the tab bar, the Simulate panel adapted from the bench page with
the `/sim` prefix and the "started by" column, the hash routing, and the
"runs the code the server was started with" note.

Manual acceptance on two machines before the PR is marked ready:
1. From a teammate's machine, open Simulate, run an auto, watch it drive.
2. Two teammates: the second Run while one is in progress says who is running.
3. Switch tabs, reload; the same tab comes back.

## Definition of done

Branch, pull request, green `tests` workflow, landed.
