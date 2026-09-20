package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

public final class SimBench {
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final int LOG_LINES = 200;

    private static final double STARTUP_SECONDS = 60;
    private final double startupSeconds;
    private final Clock clock;
    private static final double SILENCE_SECONDS = 5;
    private static final double CATALOG_SECONDS = 60;
    private static final long SILENCE_POLL_MILLIS = 100;

    public static final double DEFAULT_RUN_TIMEOUT_SECONDS = 60;

    public static final double DEFAULT_TELEOP_SECONDS = 120;

    public static final double DEFAULT_KILL_GRACE_SECONDS = 5;

    static final String START_POSES_FILE = "start-poses.json";

    public static final class BuildFailed extends RuntimeException {
        BuildFailed(String diagnostics) {
            super(diagnostics);
        }
    }

    public final class Run implements SimReplayPage.Source {
        public final int id;
        public final SimCatalog.Entry entry;
        public final long startedAtMillis = clock.millisSinceEpoch();

        public final String startedBy;

        public final Pose2d start;

        public final Long seed;

        private final JsonArray ticks = new JsonArray();
        private final Deque<String> log = new ArrayDeque<>();
        private String phase = "building";
        private String outcome;
        private String message;
        private Child.Running child;

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

        public synchronized String phase() {
            return phase;
        }

        @Override
        public synchronized String outcome() {
            return outcome;
        }

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

        public synchronized String log() {
            return String.join("\n", log);
        }

        synchronized void addTick(JsonObject tick) {
            ticks.add(tick);
        }

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

        synchronized void launched(Child.Running process) {
            child = process;
            phase = "starting";
        }

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

        synchronized Child.Running child() {
            return child;
        }

        double budgetSeconds() {
            return entry.kind.equals(SimCatalog.TELEOP) ? teleOpSeconds : runTimeoutSeconds;
        }

        synchronized boolean send(JsonObject line) {
            if (child == null || outcome != null) {
                return false;
            }
            return child.say(GSON.toJson(line));
        }

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
            Child.Running process = child();
            if (!told) {
                process.kill();
                return;
            }
            Thread grace = new Thread(
                    () -> {
                        if (!process.endedWithin(killGraceSeconds)) {
                            finish(
                                    SimRunStream.Outcome.killedAfterStop(killGraceSeconds),
                                    "loop() never came back after Stop, so nothing in the child could end the run; the child JVM was killed");
                            process.kill();
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

    public interface Factory {
        SimBench create(Path worktree);
    }

    private final SimCatalog fixedCatalog;
    private final SimBuild build;
    private final Path outputDir;
    private final double runTimeoutSeconds;
    private final double teleOpSeconds;
    private final double killGraceSeconds;
    private final Child children;
    private final StartPoses startPoses;
    private final List<Run> runs = new ArrayList<>();
    private SimCatalog listed;
    private Path listedFrom;
    private SimBuild.Result lastCheck;

    public SimBench(
            SimCatalog fixedCatalog,
            Path project,
            Path outputDir,
            double runTimeoutSeconds,
            double teleOpSeconds,
            double killGraceSeconds) {
        this(fixedCatalog, project, outputDir, runTimeoutSeconds, teleOpSeconds, killGraceSeconds, new JvmChild());
    }

    public SimBench(
            SimCatalog fixedCatalog,
            Path project,
            Path outputDir,
            double runTimeoutSeconds,
            double teleOpSeconds,
            double killGraceSeconds,
            Child children) {
        this(
                fixedCatalog,
                project,
                outputDir,
                runTimeoutSeconds,
                teleOpSeconds,
                killGraceSeconds,
                children,
                STARTUP_SECONDS);
    }

    public SimBench(
            SimCatalog fixedCatalog,
            Path project,
            Path outputDir,
            double runTimeoutSeconds,
            double teleOpSeconds,
            double killGraceSeconds,
            Child children,
            double startupSeconds) {
        this(
                fixedCatalog,
                project,
                outputDir,
                runTimeoutSeconds,
                teleOpSeconds,
                killGraceSeconds,
                children,
                startupSeconds,
                new SystemClock());
    }

    public SimBench(
            SimCatalog fixedCatalog,
            Path project,
            Path outputDir,
            double runTimeoutSeconds,
            double teleOpSeconds,
            double killGraceSeconds,
            Child children,
            double startupSeconds,
            Clock clock) {
        if ((fixedCatalog == null) == (project == null)) {
            throw new IllegalArgumentException("give either a fixed catalog or a project");
        }
        this.fixedCatalog = fixedCatalog;
        this.build = project == null ? null : buildOf(project, outputDir);
        this.outputDir = outputDir;
        this.runTimeoutSeconds = runTimeoutSeconds;
        this.teleOpSeconds = teleOpSeconds;
        this.killGraceSeconds = killGraceSeconds;
        this.children = children;
        this.startupSeconds = startupSeconds;
        this.clock = clock;
        this.startPoses = new StartPoses(outputDir.resolve(START_POSES_FILE));
    }

    private static SimBuild buildOf(Path project, Path outputDir) {
        Path harnessRoot = project.resolve("TeamCode/src/test/java");
        Path child = harnessRoot.resolve(SimChild.class.getName().replace('.', '/') + ".java");
        if (!Files.isRegularFile(child)) {
            throw new IllegalArgumentException("no simulator in " + project + ": " + child
                    + " is missing, and the child would run this server's simulator instead of the project's own");
        }
        return new SimBuild(project.resolve("TeamCode/src/main/java"), harnessRoot, outputDir.resolve("classes"));
    }

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

    private List<String> sources() {
        return fixedCatalog == null ? List.of() : fixedCatalog.sources();
    }

    private SimCatalog list(Path classes) {
        StringBuilder said = new StringBuilder();
        try (Child.Running child =
                children.onTheClassesAt(classes, line -> said.append(line).append('\n'), "--list")) {
            String first = child.hear();
            String line;
            try {
                line = first == null ? null : SimRunStream.afterHello(first);
            } catch (SimRunStream.WrongProtocol e) {
                child.kill();
                throw e;
            }
            if (line == null && first != null) {
                line = child.hear();
            }
            if (!child.endedWithin(CATALOG_SECONDS) || line == null) {
                child.kill();
                throw new IllegalStateException("the simulation child did not list the op modes"
                        + (child.alive()
                                ? " within " + CATALOG_SECONDS + "s"
                                : " (exit " + child.exitCode().orElse(-1) + ")")
                        + (said.toString().isBlank()
                                ? " and printed nothing"
                                : "; it printed:\n" + said.toString().strip()));
            }
            return SimCatalog.fromJson(GSON.fromJson(line, JsonArray.class));
        }
    }

    private static JsonObject model() {
        JsonObject model = GSON.fromJson(GSON.toJson(SimPlacement.FIELD.json()), JsonObject.class);
        model.addProperty("robotIn", SimPlacement.ROBOT_SIZE_IN);
        return model;
    }

    public Router routes(String startedBy) {
        return new Router()
                .route("GET", "/catalog", (request, params) -> catalogJson())
                .route("GET", "/status", (request, params) -> Response.json(status()))
                .route("GET", "/assets/{name*}", (request, params) -> SimAssets.serve(params.get("name")))
                .route("GET", "/field", (request, params) -> Response.html(SimAssets.page("field.html")))
                .route("GET", "/model", (request, params) -> Response.json(GSON.toJson(model())))
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
                .redirect("GET", "/runs/{id}", params -> "/runs/" + params.get("id") + "/")
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

    private static String seedJson(Long seed) {
        JsonObject json = new JsonObject();
        json.add("seed", StartPoses.seedToJson(seed));
        return GSON.toJson(json);
    }

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

    private Response catalogJson() {
        try {
            return Response.json(GSON.toJson(withSeeds(catalog().toJson())));
        } catch (BuildFailed e) {
            return Response.error(500, "the sources do not compile:\n" + e.getMessage());
        } catch (SimRunStream.WrongProtocol e) {
            return Response.error(500, e.getMessage());
        }
    }

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

    private Response run(String opMode, String startedBy) {
        Optional<SimCatalog.Entry> entry = Optional.empty();
        if (opMode != null) {
            try {
                entry = catalog().find(opMode);
            } catch (BuildFailed | SimRunStream.WrongProtocol e) {
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
            return;
        }
        Child.Running child;
        try {
            List<String> args = new ArrayList<>(List.of(
                    "--run",
                    run.entry.name,
                    String.valueOf(run.budgetSeconds()),
                    outputDir.toAbsolutePath().toString()));
            args.addAll(sources());
            child = classes == null
                    ? children.onThisClasspath(run::addLog, args.toArray(new String[0]))
                    : children.onTheClassesAt(classes, run::addLog, args.toArray(new String[0]));
        } catch (RuntimeException e) {
            run.finish(SimRunStream.Outcome.couldNotStartChild(), e.getMessage());
            return;
        }
        run.launched(child);
        CountDownLatch started = new CountDownLatch(1);
        AtomicLong lastHeardNanos = new AtomicLong(clock.nanos());
        Thread watchdog = new Thread(
                () -> {
                    try {
                        if (!started.await((long) (startupSeconds * 1000), TimeUnit.MILLISECONDS)) {
                            if (child.alive()) {
                                run.finish(
                                        SimRunStream.Outcome.killed(startupSeconds, "the op mode never started"),
                                        "the child JVM never said the op mode had started; it was killed");
                                child.kill();
                            }
                            return;
                        }
                        while (child.alive() && run.outcome() == null) {
                            double silentSeconds = (clock.nanos() - lastHeardNanos.get()) / 1e9;
                            if (silentSeconds > SILENCE_SECONDS) {
                                run.finish(
                                        SimRunStream.Outcome.killed(SILENCE_SECONDS, "the op mode did not return"),
                                        "loop() never came back, so nothing in the child could end the run; the child JVM was killed");
                                child.kill();
                                return;
                            }
                            clock.sleep(SILENCE_POLL_MILLIS / 1000.0);
                        }
                    } catch (InterruptedException ignored) {
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
        try (Child.Running talking = child) {
            boolean first = true;
            for (String line = talking.hear(); line != null; line = talking.hear()) {
                lastHeardNanos.set(clock.nanos());
                if (line.isBlank()) {
                    continue;
                }
                if (first) {
                    first = false;
                    SimRunStream.Handshake handshake;
                    try {
                        handshake = SimRunStream.handshake(line, run.start, run.seed);
                    } catch (SimRunStream.WrongProtocol e) {
                        run.finish(SimRunStream.Outcome.wrongProtocol(e.childProtocol), e.getMessage());
                        child.kill();
                        break;
                    }
                    if (handshake.refused()) {
                        run.finish(handshake.outcome, handshake.message);
                        child.kill();
                        break;
                    }
                    if (handshake.startLine != null) {
                        run.send(handshake.startLine);
                    }
                    line = handshake.firstContentLine;
                    if (line == null) {
                        continue;
                    }
                }
                try {
                    SimRunStream.accept(line, listener);
                } catch (IllegalArgumentException e) {
                    run.addLog("unreadable line from the child: " + line);
                }
            }
        }
        child.endedWithin(CATALOG_SECONDS);
        watchdog.interrupt();
        if (outcome[0] != null) {
            run.finish(outcome[0], null);
        } else {
            run.finish(SimRunStream.Outcome.childExited(child.exitCode().orElse(-1)), run.log());
        }
    }

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

    public void stop() {
        Run current = current();
        if (current != null) {
            current.finish(SimRunStream.Outcome.stopped(), "the bench was stopped");
            Child.Running child = current.child();
            if (child != null) {
                child.kill();
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
