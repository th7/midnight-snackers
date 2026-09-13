package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * The bench core shared by the simulation bench and the coding server's Simulate tab: the
 * catalog of runnable autonomous op modes, the runs so far, and the routes that start a run and
 * follow it. One run at a time, whoever asks.
 * <p>
 * Given a source root, every run first recompiles the main sources ({@link SimBuild}) and then
 * runs the op mode in a child JVM ({@link SimChild}) with the new classes first on its classpath,
 * so a run always executes the sources as last saved, and a hung op mode is a process that gets
 * killed. Without a source root (the tests), the child runs on this JVM's classpath as is.
 */
public final class SimBench {
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final int LOG_LINES = 200;
    /** How long a child may take to load its catalog and say the op mode has started. */
    private static final double STARTUP_SECONDS = 60;

    /** The sources do not compile; the message is the compiler's diagnostics. */
    public static final class BuildFailed extends RuntimeException {
        BuildFailed(String diagnostics) {
            super(diagnostics);
        }
    }

    public final class Run implements SimReplayPage.Source {
        public final int id;
        public final SimCatalog.Entry entry;
        public final long startedAtMillis = System.currentTimeMillis();
        /** Who started it, or null when the bench's own page did. */
        public final String startedBy;
        private final JsonArray ticks = new JsonArray();
        private final Deque<String> log = new ArrayDeque<>();
        private String phase = "building";
        private String outcome;
        private String message;
        private Process child;

        Run(int id, SimCatalog.Entry entry, String startedBy) {
            this.id = id;
            this.entry = entry;
            this.startedBy = startedBy;
        }

        public synchronized boolean running() {
            return outcome == null;
        }

        /** {@code building}, {@code starting} (the child JVM is up, the op mode not yet), {@code running}, or {@code finished}. */
        public synchronized String phase() {
            return phase;
        }

        @Override
        public synchronized String outcome() {
            return outcome;
        }

        /** Compiler errors or the kill reason, when there is something to say. */
        public synchronized String message() {
            return message;
        }

        public synchronized JsonArray ticks() {
            return ticksJson(0);
        }

        @Override
        public synchronized JsonArray ticksJson(int from) {
            JsonArray rest = new JsonArray();
            for (int i = Math.min(from, ticks.size()); i < ticks.size(); i++) {
                rest.add(ticks.get(i));
            }
            return rest;
        }

        /** What the child wrote to stderr: the runner's own notes and anything the op mode printed. */
        public synchronized String log() {
            return String.join("\n", log);
        }

        synchronized void addTick(JsonObject tick) {
            ticks.add(tick);
        }

        /** How far into its time the run is: the last tick's seconds, or none yet. */
        synchronized double seconds() {
            return ticks.size() == 0 ? 0 : SimRunStream.seconds(ticks.get(ticks.size() - 1).getAsJsonObject());
        }

        synchronized void addLog(String line) {
            if (log.size() == LOG_LINES) {
                log.removeFirst();
            }
            log.addLast(line);
        }

        synchronized void launched(Process process) {
            child = process;
            phase = "starting";
        }

        /** The child said the op mode's time has begun. */
        synchronized void started() {
            phase = "running";
        }

        synchronized void finish(String outcome, String message) {
            if (this.outcome == null) {
                this.outcome = outcome;
                this.message = message;
            }
            phase = "finished";
        }

        synchronized Process child() {
            return child;
        }

        /** How long this run may take: a TeleOp its period, an auto its timeout. */
        double budgetSeconds() {
            return entry.kind.equals(SimCatalog.TELEOP) ? teleOpSeconds : runTimeoutSeconds;
        }

        /**
         * One driver station line to the child, on its standard input.
         *
         * @return false when the child is not there to read it
         */
        synchronized boolean send(JsonObject line) {
            if (child == null || outcome != null) {
                return false;
            }
            try {
                OutputStream in = child.getOutputStream();
                in.write((GSON.toJson(line) + "\n").getBytes(StandardCharsets.UTF_8));
                in.flush();
                return true;
            } catch (IOException e) {
                return false; // the child went away; the exit code says how
            }
        }

        /**
         * The driver pressed Stop. A child that is running is told and ends the run itself, with
         * its replay written; one still building never starts; one that ignores the request is
         * killed after the grace period.
         */
        void stop() {
            boolean told;
            synchronized (this) {
                if (outcome != null) {
                    return;
                }
                if (child == null) {
                    finish(SimRunStream.Outcome.stopped(), "stopped before the build finished");
                    return;
                }
                JsonObject line = new JsonObject();
                line.addProperty("stop", true);
                told = send(line);
            }
            Process process = child();
            if (!told) {
                process.destroyForcibly();
                return;
            }
            Thread grace = new Thread(() -> {
                try {
                    if (!process.waitFor((long) (killGraceSeconds * 1000), TimeUnit.MILLISECONDS)) {
                        finish(SimRunStream.Outcome.killedAfterStop(killGraceSeconds),
                                "loop() never came back after Stop, so nothing in the child could end the run; the child JVM was killed");
                        process.destroyForcibly();
                    }
                } catch (InterruptedException ignored) {
                    // stopping
                }
            }, "sim-run-" + id + "-stop");
            grace.setDaemon(true);
            grace.start();
        }

        @Override
        public String name() {
            return entry.name;
        }

        @Override
        public String kind() {
            return entry.kind;
        }
    }

    /** Makes the bench for one worktree: the coding server gives each user's worktree its own. */
    public interface Factory {
        SimBench create(Path worktree);
    }

    private final SimCatalog fixedCatalog;
    private final SimBuild build;
    private final Path outputDir;
    private final double runTimeoutSeconds;
    private final double teleOpSeconds;
    private final double killGraceSeconds;
    private final List<Run> runs = new ArrayList<>();
    private SimCatalog listed;
    private Path listedFrom;
    private SimBuild.Result lastCheck;

    /**
     * @param fixedCatalog      the op modes to offer, or null to compile and list them from {@code sourceRoot}
     * @param sourceRoot        the main sources to compile before each run, or null to run this JVM's classes
     * @param runTimeoutSeconds how long an auto may take to finish its plan before the run times out
     * @param teleOpSeconds     how long a TeleOp runs when the driver never presses Stop
     * @param killGraceSeconds  how long past its time the child may live before it is killed
     */
    public SimBench(SimCatalog fixedCatalog, Path sourceRoot, Path outputDir, double runTimeoutSeconds, double teleOpSeconds,
                    double killGraceSeconds) {
        if ((fixedCatalog == null) == (sourceRoot == null)) {
            throw new IllegalArgumentException("give either a fixed catalog or a source root");
        }
        this.fixedCatalog = fixedCatalog;
        this.build = sourceRoot == null ? null : new SimBuild(sourceRoot, outputDir.resolve("classes"));
        this.outputDir = outputDir;
        this.runTimeoutSeconds = runTimeoutSeconds;
        this.teleOpSeconds = teleOpSeconds;
        this.killGraceSeconds = killGraceSeconds;
    }

    /**
     * The runnable op modes as of the sources on disk. While a run is in progress the last
     * listing is returned, since a rebuild would pull the classes out from under the child.
     *
     * @throws BuildFailed when the sources do not compile
     */
    public SimCatalog catalog() {
        if (fixedCatalog != null) {
            return fixedCatalog;
        }
        synchronized (this) {
            if (current() != null && listed != null) {
                return listed;
            }
            SimBuild.Result result = build.build();
            if (result.classes == null) {
                throw new BuildFailed(result.diagnostics);
            }
            if (listed != null && result.classes.equals(listedFrom)) {
                return listed;
            }
            listed = list(result.classes);
            listedFrom = result.classes;
            return listed;
        }
    }

    /**
     * Compiles the sources as saved, for the editor to show problems as they are made. Cached by
     * the source fingerprint, so an unchanged tree costs nothing. While a run is in progress the
     * previous result is returned, since a rebuild would pull the classes out from under the child.
     *
     * @return null when there are no sources to build
     */
    public synchronized SimBuild.Result check() {
        if (build == null) {
            return null;
        }
        if (current() != null && lastCheck != null) {
            return lastCheck;
        }
        lastCheck = build.build();
        return lastCheck;
    }

    public Path sourceRoot() {
        return build == null ? null : build.sourceRoot();
    }

    /** The sources the child is told to build its catalog from: a fixed catalog's, else none, so it discovers. */
    private List<String> sources() {
        return fixedCatalog == null ? List.of() : fixedCatalog.sources();
    }

    private static SimCatalog list(Path classes) {
        Process child = SimChild.launch(List.of(classes), "--list");
        try (BufferedReader out = new BufferedReader(new InputStreamReader(child.getInputStream(), StandardCharsets.UTF_8))) {
            String line = out.readLine();
            child.getErrorStream().transferTo(java.io.OutputStream.nullOutputStream());
            if (!child.waitFor(60, TimeUnit.SECONDS) || line == null) {
                child.destroyForcibly();
                throw new IllegalStateException("the simulation child did not list the op modes");
            }
            return SimCatalog.fromJson(GSON.fromJson(line, JsonArray.class));
        } catch (IOException e) {
            throw new java.io.UncheckedIOException(e);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException(e);
        }
    }

    /**
     * The routes, to mount wherever the caller likes: {@code /catalog}, {@code /status},
     * {@code POST /run?opmode=<name>}, {@code /runs/<id>/}, {@code /runs/<id>/ticks?from=<n>},
     * {@code /runs/<id>/log}, {@code POST /runs/<id>/gamepad}, {@code POST /runs/<id>/stop}.
     *
     * @param startedBy the name to record on a run started through these routes, or null
     */
    public Router routes(String startedBy) {
        return new Router()
                .route("GET", "/catalog", (request, params) -> catalogJson())
                .route("GET", "/status", (request, params) -> Response.json(status()))
                .route("POST", "/run", (request, params) -> run(request.query("opmode"), startedBy))
                .route("GET", "/runs/{id}", (request, params) -> withRun(params, request, this::page))
                .route("GET", "/runs/{id}/", (request, params) -> withRun(params, request, this::page))
                .route("GET", "/runs/{id}/ticks", (request, params) -> withRun(params, request, (run, r) ->
                        Response.json(SimReplayPage.update(run, r.queryInt("from", 0)))))
                .route("GET", "/runs/{id}/log", (request, params) -> withRun(params, request, (run, r) ->
                        new Response(200, "text/plain; charset=utf-8", run.log())))
                .route("POST", "/runs/{id}/gamepad", (request, params) -> withRun(params, request, this::gamepad))
                .route("POST", "/runs/{id}/stop", (request, params) -> withRun(params, request, (run, r) -> {
                    if (!run.running()) {
                        return Response.error(409, "the run is over: " + run.outcome());
                    }
                    run.stop();
                    return Response.json("{}");
                }));
    }

    private interface RunRoute {
        Response handle(Run run, Request request);
    }

    private Response withRun(java.util.Map<String, String> params, Request request, RunRoute route) {
        Run run = find(params.get("id"));
        if (run == null) {
            return Response.error(404, "no such run: " + request.path);
        }
        return route.handle(run, request);
    }

    private Response catalogJson() {
        try {
            return Response.json(GSON.toJson(catalog().toJson()));
        } catch (BuildFailed e) {
            return Response.error(500, "the sources do not compile:\n" + e.getMessage());
        }
    }

    private Response page(Run run, Request request) {
        return Response.html(SimReplayPage.page(run, true));
    }

    /** {@code POST /run?opmode=<name>}: starts the run and answers its id. */
    private Response run(String opMode, String startedBy) {
        Optional<SimCatalog.Entry> entry = Optional.empty();
        if (opMode != null) {
            try {
                entry = catalog().find(opMode);
            } catch (BuildFailed e) {
                // let the run itself report the build failure, where the tab shows it
                entry = listed == null ? Optional.empty() : listed.find(opMode);
                if (entry.isEmpty()) {
                    entry = Optional.of(new SimCatalog.Entry(opMode, "", SimCatalog.AUTO, "", null));
                }
            }
        }
        if (entry.isEmpty()) {
            return Response.error(404, "no runnable op mode named " + opMode);
        }
        Run run = start(entry.get(), startedBy);
        if (run == null) {
            Run current = current();
            return Response.error(409, "a run is already in progress"
                    + (current != null && current.startedBy != null ? " (started by " + current.startedBy + ")" : ""));
        }
        JsonObject body = new JsonObject();
        body.addProperty("id", run.id);
        return Response.json(GSON.toJson(body));
    }

    /**
     * {@code POST /runs/<id>/gamepad} with a driver station line, {@code {"gamepad": 1, "state":
     * {...}}}, relayed to the child as it is once it has been checked, so a typo in a page is a 400
     * here and never reaches the run.
     */
    private Response gamepad(Run run, Request request) {
        JsonObject line;
        try {
            line = GSON.fromJson(request.body, JsonObject.class);
            if (line == null || !line.has("gamepad") || !line.has("state")) {
                throw new IllegalArgumentException("expected {\"gamepad\": 1, \"state\": {...}}");
            }
            new SimDriverStation().accept(line);
        } catch (RuntimeException e) {
            return Response.error(400, "not a gamepad state: " + e.getMessage());
        }
        if (!run.running()) {
            return Response.error(409, "the run is over: " + run.outcome());
        }
        if (!run.send(line)) {
            return Response.error(409, "the run is not taking input" + (run.phase().equals("building") ? " while building" : ""));
        }
        return Response.json("{}");
    }

    /** Starts a run, or returns null while another is in progress. */
    public synchronized Run start(SimCatalog.Entry entry, String startedBy) {
        if (current() != null) {
            return null;
        }
        Run run = new Run(runs.size() + 1, entry, startedBy);
        runs.add(run);
        Thread thread = new Thread(() -> perform(run), "sim-run-" + run.id);
        thread.setDaemon(true);
        thread.start();
        return run;
    }

    private void perform(Run run) {
        List<Path> classpathFirst = List.of();
        if (build != null) {
            SimBuild.Result result;
            try {
                result = build.build();
            } catch (RuntimeException e) {
                run.finish(SimRunStream.Outcome.buildFailed(), e.getMessage());
                return;
            }
            if (result.classes == null) {
                run.finish(SimRunStream.Outcome.buildFailed(), result.diagnostics);
                return;
            }
            classpathFirst = List.of(result.classes);
        }
        if (!run.running()) {
            return; // stopped while building
        }
        Process child;
        try {
            List<String> args = new ArrayList<>(List.of("--run", run.entry.name, String.valueOf(run.budgetSeconds()),
                    outputDir.toAbsolutePath().toString()));
            args.addAll(sources());
            child = SimChild.launch(classpathFirst, args.toArray(new String[0]));
        } catch (RuntimeException e) {
            run.finish(SimRunStream.Outcome.couldNotStartChild(), e.getMessage());
            return;
        }
        run.launched(child);
        Thread stderr = new Thread(() -> {
            try (BufferedReader err = new BufferedReader(new InputStreamReader(child.getErrorStream(), StandardCharsets.UTF_8))) {
                for (String line = err.readLine(); line != null; line = err.readLine()) {
                    run.addLog(line);
                }
            } catch (IOException ignored) {
                // the child went away
            }
        }, "sim-run-" + run.id + "-log");
        stderr.setDaemon(true);
        stderr.start();
        double maxSeconds = run.budgetSeconds() + killGraceSeconds;
        CountDownLatch started = new CountDownLatch(1);
        Thread watchdog = new Thread(() -> {
            try {
                // The run's time starts when the op mode does, not when the JVM does: loading the
                // catalog is the child's business, and it can be slow.
                if (!started.await((long) (STARTUP_SECONDS * 1000), TimeUnit.MILLISECONDS)) {
                    if (child.isAlive()) {
                        run.finish(SimRunStream.Outcome.killed(STARTUP_SECONDS, "the op mode never started"),
                                "the child JVM never said the op mode had started; it was killed");
                        child.destroyForcibly();
                    }
                    return;
                }
                if (!child.waitFor((long) (maxSeconds * 1000), TimeUnit.MILLISECONDS)) {
                    run.finish(SimRunStream.Outcome.killed(maxSeconds, "the op mode did not return"),
                            "loop() never came back, so nothing in the child could end the run; the child JVM was killed");
                    child.destroyForcibly();
                }
            } catch (InterruptedException ignored) {
                // stopping
            }
        }, "sim-run-" + run.id + "-watchdog");
        watchdog.setDaemon(true);
        watchdog.start();
        String[] outcome = {null};
        SimRunStream.Listener listener = new SimRunStream.Listener() {
            @Override
            public void started() {
                run.started();
                started.countDown();
            }

            @Override
            public void tick(JsonObject tick) {
                run.addTick(tick);
            }

            @Override
            public void finished(String how) {
                outcome[0] = how;
            }
        };
        try (BufferedReader out = new BufferedReader(new InputStreamReader(child.getInputStream(), StandardCharsets.UTF_8))) {
            for (String line = out.readLine(); line != null; line = out.readLine()) {
                if (line.isBlank()) {
                    continue;
                }
                try {
                    SimRunStream.accept(line, listener);
                } catch (IllegalArgumentException e) {
                    run.addLog("unreadable line from the child: " + line);
                }
            }
        } catch (IOException ignored) {
            // the child went away; the exit code says how
        }
        try {
            child.waitFor();
            stderr.join(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        watchdog.interrupt();
        if (outcome[0] != null) {
            run.finish(outcome[0], null);
        } else {
            run.finish(SimRunStream.Outcome.childExited(child.exitValue()), run.log());
        }
    }

    /** The run in progress, or null. */
    public synchronized Run current() {
        Run last = runs.isEmpty() ? null : runs.get(runs.size() - 1);
        return last != null && last.running() ? last : null;
    }

    public synchronized Run find(String id) {
        for (Run run : runs) {
            if (String.valueOf(run.id).equals(id)) {
                return run;
            }
        }
        return null;
    }

    /** Kills any run in progress. */
    public void stop() {
        Run current = current();
        if (current != null) {
            current.finish(SimRunStream.Outcome.stopped(), "the bench was stopped");
            Process child = current.child();
            if (child != null) {
                child.destroyForcibly();
            }
        }
    }

    public synchronized String status() {
        JsonObject root = new JsonObject();
        root.addProperty("running", current() != null);
        JsonArray list = new JsonArray();
        for (int i = runs.size() - 1; i >= 0; i--) {
            Run run = runs.get(i);
            JsonObject item = new JsonObject();
            item.addProperty("id", run.id);
            item.addProperty("name", run.entry.name);
            item.addProperty("where", run.entry.where);
            item.addProperty("kind", run.entry.kind);
            item.addProperty("startedAt", run.startedAtMillis);
            item.addProperty("startedBy", run.startedBy);
            item.addProperty("phase", run.phase());
            item.addProperty("loops", run.ticks().size());
            item.addProperty("seconds", run.seconds());
            item.addProperty("outcome", run.outcome());
            item.addProperty("message", run.message());
            list.add(item);
        }
        root.add("runs", list);
        return GSON.toJson(root);
    }
}
