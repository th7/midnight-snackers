package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

/**
 * The bench core shared by the simulation bench and the coding server's Simulate tab: the
 * catalog of runnable autonomous op modes, the runs so far, and the routes that start a run and
 * follow it. One run at a time, whoever asks.
 * <p>
 * Given a project, every run first rebuilds its robot sources and its simulator ({@link SimBuild}) and then
 * runs the op mode in a child JVM ({@link SimChild}) with the new classes first on its classpath,
 * so a run always executes the sources as last saved, and a hung op mode is a process that gets
 * killed. Without a project (the tests), the child runs on this JVM's classpath as is.
 */
public final class SimBench {
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final int LOG_LINES = 200;
    /** How long a child may take to load its catalog and say the op mode has started. */
    private static final double STARTUP_SECONDS = 60;
    /** Where the robot is placed for each op mode, remembered under the output directory. */
    static final String START_POSES_FILE = "start-poses.json";

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
        /** Where the robot is placed as the run starts: the op mode's start pose when the run was started. */
        public final Pose2d start;
        /** Which robot the run is on: the op mode's seed when the run was started, or null for the exact robot. */
        public final Long seed;

        private final JsonArray ticks = new JsonArray();
        private final Deque<String> log = new ArrayDeque<>();
        private String phase = "building";
        private String outcome;
        private String message;
        private Process child;

        Run(int id, SimCatalog.Entry entry, String startedBy, Pose2d start, Long seed) {
            this.id = id;
            this.entry = entry;
            this.startedBy = startedBy;
            this.start = start;
            this.seed = seed;
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
            return ticks.size() == 0
                    ? 0
                    : SimRunStream.seconds(ticks.get(ticks.size() - 1).getAsJsonObject());
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

        /**
         * This run as the status lists it, taken in one hold of the run's own lock: whoever reads
         * it sees the phase, the loops and the outcome of one moment, never of two.
         */
        synchronized JsonObject json() {
            JsonObject item = new JsonObject();
            item.addProperty("id", id);
            item.addProperty("name", entry.name);
            item.addProperty("where", entry.where);
            item.addProperty("kind", entry.kind);
            item.addProperty("startedAt", startedAtMillis);
            item.addProperty("startedBy", startedBy);
            item.add("seed", StartPoses.seedToJson(seed));
            item.addProperty("phase", phase);
            item.addProperty("loops", ticks().size());
            item.addProperty("seconds", seconds());
            item.addProperty("outcome", outcome);
            item.addProperty("message", message);
            return item;
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
            Thread grace = new Thread(
                    () -> {
                        try {
                            if (!process.waitFor((long) (killGraceSeconds * 1000), TimeUnit.MILLISECONDS)) {
                                finish(
                                        SimRunStream.Outcome.killedAfterStop(killGraceSeconds),
                                        "loop() never came back after Stop, so nothing in the child could end the run; the child JVM was killed");
                                process.destroyForcibly();
                            }
                        } catch (InterruptedException ignored) {
                            // stopping
                        }
                    },
                    "sim-run-" + id + "-stop");
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
    private final StartPoses startPoses;
    private final List<Run> runs = new ArrayList<>();
    private SimCatalog listed;
    private Path listedFrom;
    private SimBuild.Result lastCheck;

    /**
     * @param fixedCatalog      the op modes to offer, or null to build and list them from {@code project}
     * @param project           the project (a checkout or worktree) whose robot sources and simulator are built before each run, or null to run this JVM's classes
     * @param runTimeoutSeconds how long an auto may take to finish its plan before the run times out
     * @param teleOpSeconds     how long a TeleOp runs when the driver never presses Stop
     * @param killGraceSeconds  how long past its time the child may live before it is killed
     * @throws IllegalStateException when the start poses remembered under {@code outputDir} cannot be read
     */
    public SimBench(
            SimCatalog fixedCatalog,
            Path project,
            Path outputDir,
            double runTimeoutSeconds,
            double teleOpSeconds,
            double killGraceSeconds) {
        if ((fixedCatalog == null) == (project == null)) {
            throw new IllegalArgumentException("give either a fixed catalog or a project");
        }
        this.fixedCatalog = fixedCatalog;
        this.build = project == null ? null : buildOf(project, outputDir);
        this.outputDir = outputDir;
        this.runTimeoutSeconds = runTimeoutSeconds;
        this.teleOpSeconds = teleOpSeconds;
        this.killGraceSeconds = killGraceSeconds;
        this.startPoses = new StartPoses(outputDir.resolve(START_POSES_FILE));
    }

    /**
     * The build of a project's robot sources and its own simulator, which the child runs. A project
     * without a simulator is refused: the child would fall through to this server's, built for
     * other robot sources.
     */
    private static SimBuild buildOf(Path project, Path outputDir) {
        Path harnessRoot = project.resolve("TeamCode/src/test/java");
        Path child = harnessRoot.resolve(SimChild.class.getName().replace('.', '/') + ".java");
        if (!Files.isRegularFile(child)) {
            throw new IllegalArgumentException("no simulator in " + project + ": " + child
                    + " is missing, and the child would run this server's simulator instead of the project's own");
        }
        return new SimBuild(project.resolve("TeamCode/src/main/java"), harnessRoot, outputDir.resolve("classes"));
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

    /**
     * @throws SimRunStream.WrongProtocol when the project's simulator speaks a protocol this bench cannot read
     */
    private static SimCatalog list(Path classes) {
        Process child = SimChild.launch(classes, "--list");
        try (BufferedReader out =
                new BufferedReader(new InputStreamReader(child.getInputStream(), StandardCharsets.UTF_8))) {
            String first = out.readLine();
            String line;
            try {
                line = first == null ? null : SimRunStream.afterHello(first);
            } catch (SimRunStream.WrongProtocol e) {
                child.destroyForcibly();
                throw e;
            }
            if (line == null && first != null) {
                line = out.readLine();
            }
            String said = new String(child.getErrorStream().readAllBytes(), StandardCharsets.UTF_8);
            if (!child.waitFor(60, TimeUnit.SECONDS) || line == null) {
                child.destroyForcibly();
                throw new IllegalStateException("the simulation child did not list the op modes"
                        + (child.isAlive() ? " within 60s" : " (exit " + child.exitValue() + ")")
                        + (said.isBlank() ? " and printed nothing" : "; it printed:\n" + said.strip()));
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
     * {@code /start?opmode=<name>} ({@code GET} the start pose, {@code PUT} one),
     * {@code /place?opmode=<name>} (the placement page), {@code POST /run?opmode=<name>},
     * {@code /runs/<id>/}, {@code /runs/<id>/ticks?from=<n>}, {@code /runs/<id>/log},
     * {@code POST /runs/<id>/gamepad}, {@code POST /runs/<id>/stop}.
     *
     * @param startedBy the name to record on a run started through these routes, or null
     */
    public Router routes(String startedBy) {
        return new Router()
                .route("GET", "/catalog", (request, params) -> catalogJson())
                .route("GET", "/status", (request, params) -> Response.json(status()))
                .route(
                        "GET",
                        "/start",
                        (request, params) -> withOpMode(
                                request, name -> Response.json(GSON.toJson(StartPoses.toJson(startPoses.get(name))))))
                .route("PUT", "/start", (request, params) -> withOpMode(request, name -> place(name, request.body)))
                .route(
                        "GET",
                        "/seed",
                        (request, params) ->
                                withOpMode(request, name -> Response.json(seedJson(startPoses.seed(name)))))
                .route("PUT", "/seed", (request, params) -> withOpMode(request, name -> seed(name, request.body)))
                .route(
                        "GET",
                        "/place",
                        (request, params) -> withOpMode(
                                request,
                                name -> Response.html(
                                        SimReplayPage.placement(name, kindOf(name), startPoses.get(name)))))
                .route("POST", "/run", (request, params) -> run(request.query("opmode"), startedBy))
                .route("GET", "/runs/{id}", (request, params) -> withRun(params, request, this::page))
                .route("GET", "/runs/{id}/", (request, params) -> withRun(params, request, this::page))
                .route(
                        "GET",
                        "/runs/{id}/ticks",
                        (request, params) -> withRun(
                                params,
                                request,
                                (run, r) -> Response.json(SimReplayPage.update(run, r.queryInt("from", 0)))))
                .route(
                        "GET",
                        "/runs/{id}/log",
                        (request, params) -> withRun(
                                params, request, (run, r) -> new Response(200, "text/plain; charset=utf-8", run.log())))
                .route("POST", "/runs/{id}/gamepad", (request, params) -> withRun(params, request, this::gamepad))
                .route(
                        "POST",
                        "/runs/{id}/stop",
                        (request, params) -> withRun(params, request, (run, r) -> {
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

    private interface OpModeRoute {
        Response handle(String opMode);
    }

    private static Response withOpMode(Request request, OpModeRoute route) {
        String name = request.query("opmode");
        if (name == null || name.isBlank()) {
            return Response.error(400, "which op mode? " + request.path + "?opmode=<name>");
        }
        return route.handle(name);
    }

    /**
     * {@code PUT /start?opmode=<name>} with {@code {"x": .., "y": .., "heading": ..}}: places the
     * robot for that op mode's runs, against a wall or an obstacle when the pose is beyond one, and
     * answers the pose as placed. A body that is not a pose is a 400 naming what is wrong.
     */
    private Response place(String opMode, String body) {
        Pose2d pose;
        try {
            JsonObject json = GSON.fromJson(body, JsonObject.class);
            if (json == null) {
                throw new IllegalArgumentException("expected {\"x\": .., \"y\": .., \"heading\": ..}");
            }
            pose = StartPoses.fromJson(json);
        } catch (RuntimeException e) {
            return Response.error(400, "not a start pose: " + e.getMessage());
        }
        return Response.json(GSON.toJson(StartPoses.toJson(startPoses.put(opMode, pose))));
    }

    /**
     * {@code PUT /seed?opmode=<name>} with {@code {"seed": <whole number or null>}}: sets which
     * robot that op mode's runs are made on, null for the exact robot, and answers it. A body
     * that is not that is a 400 naming what is wrong.
     */
    private Response seed(String opMode, String body) {
        Long seed;
        try {
            JsonObject json = GSON.fromJson(body, JsonObject.class);
            if (json == null || !json.has("seed")) {
                throw new IllegalArgumentException("expected {\"seed\": <whole number, or null for the exact robot>}");
            }
            seed = StartPoses.seedFromJson(json.get("seed"));
        } catch (RuntimeException e) {
            return Response.error(400, "not a seed: " + e.getMessage());
        }
        return Response.json(seedJson(startPoses.putSeed(opMode, seed)));
    }

    /** {@code {"seed": ..}}, null for the exact robot. */
    private static String seedJson(Long seed) {
        JsonObject json = new JsonObject();
        json.add("seed", StartPoses.seedToJson(seed));
        return GSON.toJson(json);
    }

    /** The op mode's kind as last listed, or auto when it has not been. */
    private synchronized String kindOf(String opMode) {
        SimCatalog known = fixedCatalog != null ? fixedCatalog : listed;
        return known == null
                ? SimCatalog.AUTO
                : known.find(opMode).map(entry -> entry.kind).orElse(SimCatalog.AUTO);
    }

    private Response withRun(java.util.Map<String, String> params, Request request, RunRoute route) {
        Run run = find(params.get("id"));
        if (run == null) {
            return Response.error(404, "no such run: " + request.path);
        }
        return route.handle(run, request);
    }

    /** The catalog as JSON, each op mode with the {@code seed} its runs are made on. */
    private Response catalogJson() {
        try {
            return Response.json(GSON.toJson(withSeeds(catalog().toJson())));
        } catch (BuildFailed e) {
            return Response.error(500, "the sources do not compile:\n" + e.getMessage());
        } catch (SimRunStream.WrongProtocol e) {
            return Response.error(500, e.getMessage());
        }
    }

    /** Each catalog entry with its op mode's seed added, null for the exact robot. */
    public JsonArray withSeeds(JsonArray catalog) {
        for (JsonElement entry : catalog) {
            JsonObject item = entry.getAsJsonObject();
            item.add(
                    "seed",
                    StartPoses.seedToJson(startPoses.seed(item.get("name").getAsString())));
        }
        return catalog;
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
            } catch (BuildFailed | SimRunStream.WrongProtocol e) {
                // let the run itself report the failure, where the tab shows it
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
            return Response.error(
                    409,
                    "a run is already in progress"
                            + (current != null && current.startedBy != null
                                    ? " (started by " + current.startedBy + ")"
                                    : ""));
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
            return Response.error(
                    409, "the run is not taking input" + (run.phase().equals("building") ? " while building" : ""));
        }
        return Response.json("{}");
    }

    /** Starts a run from the op mode's start pose, or returns null while another is in progress. */
    public synchronized Run start(SimCatalog.Entry entry, String startedBy) {
        if (current() != null) {
            return null;
        }
        Run run = new Run(runs.size() + 1, entry, startedBy, startPoses.get(entry.name), startPoses.seed(entry.name));
        runs.add(run);
        Thread thread = new Thread(() -> perform(run), "sim-run-" + run.id);
        thread.setDaemon(true);
        thread.start();
        return run;
    }

    private void perform(Run run) {
        Path classes = null;
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
            classes = result.classes;
        }
        if (!run.running()) {
            return; // stopped while building
        }
        Process child;
        try {
            List<String> args = new ArrayList<>(List.of(
                    "--run",
                    run.entry.name,
                    String.valueOf(run.budgetSeconds()),
                    outputDir.toAbsolutePath().toString()));
            args.addAll(sources());
            child = classes == null
                    ? SimChild.launchOnThisClasspath(args.toArray(new String[0]))
                    : SimChild.launch(classes, args.toArray(new String[0]));
        } catch (RuntimeException e) {
            run.finish(SimRunStream.Outcome.couldNotStartChild(), e.getMessage());
            return;
        }
        run.launched(child);
        Thread stderr = new Thread(
                () -> {
                    try (BufferedReader err =
                            new BufferedReader(new InputStreamReader(child.getErrorStream(), StandardCharsets.UTF_8))) {
                        for (String line = err.readLine(); line != null; line = err.readLine()) {
                            run.addLog(line);
                        }
                    } catch (IOException ignored) {
                        // the child went away
                    }
                },
                "sim-run-" + run.id + "-log");
        stderr.setDaemon(true);
        stderr.start();
        double maxSeconds = run.budgetSeconds() + killGraceSeconds;
        CountDownLatch started = new CountDownLatch(1);
        Thread watchdog = new Thread(
                () -> {
                    try {
                        // The run's time starts when the op mode does, not when the JVM does: loading the
                        // catalog is the child's business, and it can be slow.
                        if (!started.await((long) (STARTUP_SECONDS * 1000), TimeUnit.MILLISECONDS)) {
                            if (child.isAlive()) {
                                run.finish(
                                        SimRunStream.Outcome.killed(STARTUP_SECONDS, "the op mode never started"),
                                        "the child JVM never said the op mode had started; it was killed");
                                child.destroyForcibly();
                            }
                            return;
                        }
                        if (!child.waitFor((long) (maxSeconds * 1000), TimeUnit.MILLISECONDS)) {
                            run.finish(
                                    SimRunStream.Outcome.killed(maxSeconds, "the op mode did not return"),
                                    "loop() never came back, so nothing in the child could end the run; the child JVM was killed");
                            child.destroyForcibly();
                        }
                    } catch (InterruptedException ignored) {
                        // stopping
                    }
                },
                "sim-run-" + run.id + "-watchdog");
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
        try (BufferedReader out =
                new BufferedReader(new InputStreamReader(child.getInputStream(), StandardCharsets.UTF_8))) {
            boolean first = true;
            for (String line = out.readLine(); line != null; line = out.readLine()) {
                if (line.isBlank()) {
                    continue;
                }
                if (first) {
                    first = false;
                    int protocol;
                    try {
                        protocol = SimRunStream.protocolOf(line);
                    } catch (SimRunStream.WrongProtocol e) {
                        run.finish(SimRunStream.Outcome.wrongProtocol(e.childProtocol), e.getMessage());
                        child.destroyForcibly();
                        break;
                    }
                    if (!place(run, protocol)) {
                        child.destroyForcibly();
                        break;
                    }
                    line = SimRunStream.afterHello(line);
                    if (line == null) {
                        continue; // the hello, consumed
                    }
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

    /**
     * Places the robot for the run, and says which robot. A child that waits to be placed is told
     * where and, when the op mode runs on a seed, which robot; one from before the seed runs the
     * exact robot, so it may run when that is the op mode's robot and not otherwise; one from
     * before placing places itself at the origin, so it may run when that is the start pose and
     * not otherwise.
     *
     * @return false when the run cannot start where the robot is placed, with the run finished saying why
     */
    private static boolean place(Run run, int childProtocol) {
        if (run.seed != null && childProtocol < SimRunStream.SEEDED_PROTOCOL) {
            run.finish(
                    SimRunStream.Outcome.cannotSeed(childProtocol),
                    "the simulator in these sources speaks protocol "
                            + childProtocol
                            + " and runs the exact robot on its own; a seed needs protocol "
                            + SimRunStream.SEEDED_PROTOCOL
                            + ": the sources are older than the server. Pull develop, or clear the seed.");
            return false;
        }
        if (childProtocol >= SimRunStream.PLACED_PROTOCOL) {
            run.send(SimDriverStation.startLine(
                    run.start, run.seed)); // a child already gone ends the run by its exit code
            return true;
        }
        Twist2d fromOrigin = run.start.minus(StartPoses.ORIGIN);
        if (Math.hypot(fromOrigin.line.x, fromOrigin.line.y) < 1e-9 && Math.abs(fromOrigin.angle) < 1e-9) {
            return true;
        }
        run.finish(
                SimRunStream.Outcome.cannotPlace(childProtocol),
                "the simulator in these sources speaks protocol "
                        + childProtocol
                        + " and starts the robot at the origin on its own; placing it elsewhere needs protocol "
                        + SimRunStream.PLACED_PROTOCOL
                        + ": the sources are older than the server. Pull develop, or place the robot"
                        + " back at the origin.");
        return false;
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
        List<JsonObject> newestFirst = new ArrayList<>();
        for (int i = runs.size() - 1; i >= 0; i--) {
            newestFirst.add(runs.get(i).json());
        }
        return statusOf(newestFirst);
    }

    /**
     * The status from the runs, newest first: the bench is running exactly when the newest run
     * has no outcome in the very snapshot the status shows. Asking the run a second time would
     * let one that finished in between be reported as running and done at once — a moment that
     * never was, and what the Simulate tab would then draw.
     */
    static String statusOf(List<JsonObject> newestFirst) {
        JsonObject root = new JsonObject();
        root.addProperty(
                "running",
                !newestFirst.isEmpty() && newestFirst.get(0).get("outcome").isJsonNull());
        JsonArray list = new JsonArray();
        for (JsonObject item : newestFirst) {
            list.add(item);
        }
        root.add("runs", list);
        return GSON.toJson(root);
    }
}
