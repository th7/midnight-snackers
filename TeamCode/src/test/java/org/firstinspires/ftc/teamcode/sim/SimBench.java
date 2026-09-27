package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
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
    private static final String ASSETS_FROM_A_RUN = "../../assets/";
    private static final int LOG_LINES = 200;

    private final Clock clock;
    private static final double CATALOG_SECONDS = 60;
    private static final long SILENCE_POLL_MILLIS = 100;

    /**
     * How long a bench waits for each thing it waits for. Said once, by name, because five bare
     * seconds in a row at a call site is five chances to give the wrong one to the wrong wait.
     */
    public static final class Waits {
        /** An auto's time in a game, in simulated seconds: a match's autonomous period. */
        public final double autonomousPeriod;

        /** A TeleOp's time in a game, in simulated seconds: a match's driver-controlled period. */
        public final double teleOpPeriod;

        /** How long after Stop the child has to end before it is killed. */
        public final double killGrace;

        /** How long the op mode's time has to begin, which a child JVM's loading is inside. */
        public final double startup;

        /** How long the child may say nothing at all mid-run before it is killed for hanging. */
        public final double silence;

        /** How long a run may go with nobody looking at it before it is stopped. */
        public final double unwatched;

        private Waits(
                double autonomousPeriod,
                double teleOpPeriod,
                double killGrace,
                double startup,
                double silence,
                double unwatched) {
            this.autonomousPeriod = autonomousPeriod;
            this.teleOpPeriod = teleOpPeriod;
            this.killGrace = killGrace;
            this.startup = startup;
            this.silence = silence;
            this.unwatched = unwatched;
        }

        /**
         * What the bench waits for a person watching a run: a match's periods, room to load, and
         * five minutes for somebody to come back to a run they left.
         */
        public static Waits ofTheBench() {
            return new Waits(30, 120, 5, 60, 5, 300);
        }

        public Waits autonomousPeriod(double seconds) {
            return new Waits(seconds, teleOpPeriod, killGrace, startup, silence, unwatched);
        }

        public Waits teleOpPeriod(double seconds) {
            return new Waits(autonomousPeriod, seconds, killGrace, startup, silence, unwatched);
        }

        public Waits killGrace(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, seconds, startup, silence, unwatched);
        }

        public Waits startup(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, killGrace, seconds, silence, unwatched);
        }

        public Waits silence(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, killGrace, startup, seconds, unwatched);
        }

        public Waits unwatched(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, killGrace, startup, silence, seconds);
        }
    }

    /** How a run is made: as a match is played, or for as long as whoever is driving wants. */
    public enum Mode {
        FREE_PLAY("free"),
        GAME("game");

        /** What the run route and the status call it. */
        public final String word;

        Mode(String word) {
            this.word = word;
        }

        static Optional<Mode> called(String word) {
            for (Mode mode : values()) {
                if (mode.word.equals(word)) {
                    return Optional.of(mode);
                }
            }
            return Optional.empty();
        }
    }

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

        public final Mode mode;

        private final AtomicLong lookedAtNanos = new AtomicLong(clock.nanos());

        private final JsonArray ticks = new JsonArray();
        private final Deque<String> log = new ArrayDeque<>();
        private String phase = "building";
        private String outcome;
        private String message;
        private Child.Running child;

        Run(int id, SimCatalog.Entry entry, String startedBy, Pose2d start, Long seed, Mode mode) {
            this.id = id;
            this.entry = entry;
            this.startedBy = startedBy;
            this.start = start;
            this.seed = seed;
            this.mode = mode;
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

        /** Somebody asked about this run: its page, its ticks, its log, or a press of the controller. */
        void lookedAt() {
            lookedAtNanos.set(clock.nanos());
        }

        double secondsUnwatched() {
            return (clock.nanos() - lookedAtNanos.get()) / 1e9;
        }

        synchronized JsonObject json() {
            JsonObject item = new JsonObject();
            item.addProperty("id", id);
            item.addProperty("name", entry.name);
            item.addProperty("where", entry.where);
            item.addProperty("kind", entry.kind);
            item.addProperty("mode", mode.word);
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

        /** A game's period for the op mode's kind, and in free play no limit at all. */
        double budgetSeconds() {
            if (mode == Mode.FREE_PLAY) {
                return Double.POSITIVE_INFINITY;
            }
            return entry.kind.equals(SimCatalog.TELEOP) ? waits.teleOpPeriod : waits.autonomousPeriod;
        }

        /**
         * What the live view is told of the match a game is: how long it lasts, and where its drivers
         * stand to watch it. Free play is no match, and is watched from wherever the viewer likes.
         */
        @Override
        public JsonElement match() {
            if (mode == Mode.FREE_PLAY) {
                return JsonNull.INSTANCE;
            }
            SimField.AllianceArea area = SimPlacement.FIELD.allianceArea(entry.drivenFrom());
            JsonObject match = new JsonObject();
            match.addProperty("period", budgetSeconds());
            match.addProperty("alliance", area.alliance);
            match.add("eye", GSON.toJsonTree(area.eye()));
            match.add("lookingAt", GSON.toJsonTree(area.lookingAt()));
            return match;
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
                        if (!process.endedWithin(waits.killGrace)) {
                            finish(
                                    SimRunStream.Outcome.killedAfterStop(waits.killGrace),
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

    private final SimSources sources;
    private final Path outputDir;
    private final Waits waits;
    private final Child children;
    private final StartPoses startPoses;
    private final List<Run> runs = new ArrayList<>();
    private SimBuild.Result lastCheck;

    public SimBench(SimCatalog fixedCatalog, Path project, Path outputDir, Waits waits) {
        this(fixedCatalog, project, outputDir, waits, new JvmChild());
    }

    public SimBench(SimCatalog fixedCatalog, Path project, Path outputDir, Waits waits, Child children) {
        this(sourcesOf(fixedCatalog, project, outputDir), outputDir, waits, children, new SystemClock());
    }

    private static SimSources sourcesOf(SimCatalog fixedCatalog, Path project, Path outputDir) {
        if ((fixedCatalog == null) == (project == null)) {
            throw new IllegalArgumentException("give either a fixed catalog or a project");
        }
        return fixedCatalog != null
                ? SimSources.ofThisClasspath(fixedCatalog)
                : SimSources.ofTheProjectAt(project, outputDir);
    }

    /**
     * A bench over whatever it is that builds and starts: the classpath this JVM runs on, a project
     * on disk, or -- in a test of the bench itself -- something that need do neither.
     */
    public SimBench(SimSources sources, Path outputDir, Waits waits, Child children, Clock clock) {
        this.sources = sources;
        this.outputDir = outputDir;
        this.waits = waits;
        this.children = children;
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
        synchronized (this) {
            if (current() != null) {
                Optional<SimCatalog> already = sources.known();
                if (already.isPresent()) {
                    return already.get();
                }
            }
            return sources.catalog(children);
        }
    }

    public synchronized SimBuild.Result check() {
        if (current() != null && lastCheck != null) {
            return lastCheck;
        }
        lastCheck = sources.check().orElse(null);
        return lastCheck;
    }

    public Path sourceRoot() {
        return sources.sourceRoot().orElse(null);
    }

    static SimCatalog listOn(Child children, Path classes) {
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

    public Router routes(String startedBy) {
        return new Router()
                .route("GET", "/catalog", (request, params) -> catalogJson())
                .route("GET", "/status", (request, params) -> Response.json(status()))
                .route("GET", "/assets/{name*}", (request, params) -> SimAssets.serve(params.get("name")))
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
                .route(
                        "POST",
                        "/run",
                        (request, params) -> run(request.query("opmode"), request.query("mode"), startedBy))
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
        return sources.known()
                .flatMap(known -> known.find(opMode))
                .map(entry -> entry.kind)
                .orElse(SimCatalog.AUTO);
    }

    private Response withRun(java.util.Map<String, String> params, Request request, RunRoute route) {
        Run run = find(params.get("id"));
        if (run == null) {
            return Response.error(404, "no such run: " + request.path);
        }
        run.lookedAt();
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
        return Response.html(SimReplayPage.live(run, ASSETS_FROM_A_RUN));
    }

    private Response run(String opMode, String modeWord, String startedBy) {
        Optional<Mode> mode = modeWord == null ? Optional.of(Mode.FREE_PLAY) : Mode.called(modeWord);
        if (mode.isEmpty()) {
            return Response.error(
                    400,
                    "a run is a game or free play: mode=" + Mode.GAME.word + " or mode=" + Mode.FREE_PLAY.word
                            + ", not '" + modeWord + "'");
        }
        Optional<SimCatalog.Entry> entry = Optional.empty();
        if (opMode != null) {
            try {
                entry = catalog().find(opMode);
            } catch (BuildFailed | SimRunStream.WrongProtocol e) {
                entry = sources.known().flatMap(known -> known.find(opMode));
                if (entry.isEmpty()) {
                    entry = Optional.of(new SimCatalog.Entry(opMode, "", SimCatalog.AUTO, "", null, null));
                }
            }
        }
        if (entry.isEmpty()) {
            return Response.error(404, "no runnable op mode named " + opMode);
        }
        Run run = start(entry.get(), startedBy, mode.get());
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

    public synchronized Run start(SimCatalog.Entry entry, String startedBy, Mode mode) {
        if (current() != null) {
            return null;
        }
        Run run = new Run(
                runs.size() + 1, entry, startedBy, startPoses.get(entry.name), startPoses.seed(entry.name), mode);
        runs.add(run);
        Thread thread = new Thread(() -> perform(run), "sim-run-" + run.id);
        thread.setDaemon(true);
        thread.start();
        return run;
    }

    private void perform(Run run) {
        if (!run.running()) {
            return;
        }
        Child.Running child;
        List<String> args = new ArrayList<>(List.of(
                "--run",
                run.entry.name,
                String.valueOf(run.budgetSeconds()),
                outputDir.toAbsolutePath().toString()));
        args.addAll(sources.classNames());
        try {
            child = sources.start(children, run::addLog, args.toArray(new String[0]));
        } catch (BuildFailed e) {
            run.finish(SimRunStream.Outcome.buildFailed(), e.getMessage());
            return;
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
                        if (!started.await((long) (waits.startup * 1000), TimeUnit.MILLISECONDS)) {
                            if (child.alive()) {
                                run.finish(
                                        SimRunStream.Outcome.killed(waits.startup, "the op mode never started"),
                                        "the child JVM never said the op mode had started; it was killed");
                                child.kill();
                            }
                            return;
                        }
                        while (child.alive() && run.outcome() == null) {
                            double silentSeconds = (clock.nanos() - lastHeardNanos.get()) / 1e9;
                            if (silentSeconds > waits.silence) {
                                run.finish(
                                        SimRunStream.Outcome.killed(waits.silence, "the op mode did not return"),
                                        "loop() never came back, so nothing in the child could end the run; the child JVM was killed");
                                child.kill();
                                return;
                            }
                            // Free play has no end of its own, so a run somebody walked away from
                            // would go on holding a child and growing its ticks until the server
                            // ran out of memory. Nobody is watching once no page is asking about it.
                            if (run.secondsUnwatched() > waits.unwatched) {
                                run.finish(
                                        SimRunStream.Outcome.stopped(),
                                        String.format(
                                                "nobody had looked at the run for %.1fs, so the bench stopped it and"
                                                        + " the child JVM with it",
                                                waits.unwatched));
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
