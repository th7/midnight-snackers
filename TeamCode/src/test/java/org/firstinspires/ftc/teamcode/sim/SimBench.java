package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Moment;
import org.firstinspires.ftc.teamcode.simcore.RunState;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.firstinspires.ftc.teamcode.simcore.View;
import org.firstinspires.ftc.teamcode.simcore.Watchdog;

public final class SimBench {
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final String ASSETS_FROM_A_RUN = "../../assets/";
    private static final int LOG_LINES = 200;

    private final Clock clock;
    private static final double CATALOG_SECONDS = 60;
    private static final double WATCH_POLL_SECONDS = 0.1;

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

        public final double view;

        /** The waits the watchdog holds a run's child to, each checked here to be a time. */
        final Watchdog.Waits watched;

        private Waits(
                double autonomousPeriod,
                double teleOpPeriod,
                double killGrace,
                double startup,
                double silence,
                double unwatched,
                double view) {
            this.autonomousPeriod = autonomousPeriod;
            this.teleOpPeriod = teleOpPeriod;
            this.killGrace = killGrace;
            this.startup = startup;
            this.silence = silence;
            this.unwatched = unwatched;
            this.view = view;
            this.watched =
                    new Watchdog.Waits(time(startup), time(silence), time(unwatched), time(killGrace), time(view));
        }

        private static Seconds time(double seconds) {
            return Valid.value(Seconds.of(seconds));
        }

        /**
         * What the bench waits for a person watching a run: a match's periods, room to load, and
         * five minutes for somebody to come back to a run they left.
         */
        public static Waits ofTheBench() {
            return new Waits(30, 120, 5, 60, 5, 300, 30);
        }

        public Waits autonomousPeriod(double seconds) {
            return new Waits(seconds, teleOpPeriod, killGrace, startup, silence, unwatched, view);
        }

        public Waits teleOpPeriod(double seconds) {
            return new Waits(autonomousPeriod, seconds, killGrace, startup, silence, unwatched, view);
        }

        public Waits killGrace(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, seconds, startup, silence, unwatched, view);
        }

        public Waits startup(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, killGrace, seconds, silence, unwatched, view);
        }

        public Waits silence(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, killGrace, startup, seconds, unwatched, view);
        }

        public Waits unwatched(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, killGrace, startup, silence, seconds, view);
        }

        public Waits view(double seconds) {
            return new Waits(autonomousPeriod, teleOpPeriod, killGrace, startup, silence, unwatched, seconds);
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

    public enum Begin {
        AT_ONCE("now"),
        WHEN_ITS_VIEW_IS_READY("ready");

        public final String word;

        Begin(String word) {
            this.word = word;
        }

        static Optional<Begin> called(String word) {
            for (Begin begin : values()) {
                if (begin.word.equals(word)) {
                    return Optional.of(begin);
                }
            }
            return Optional.empty();
        }

        View view(Moment asked) {
            return this == WHEN_ITS_VIEW_IS_READY ? new View.Awaited(asked) : new View.NotAwaited();
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

        private final Moment asked = now();

        private volatile Moment looked = asked;

        private final List<SimRunStream.TickLine> ticks = new ArrayList<>();
        private final Deque<String> log = new ArrayDeque<>();
        private String phase = "building";
        private String outcome;
        private String message;
        private Child.Running child;
        private View view;
        private JsonObject heldStart;
        private Moment startupSince;
        private Moment heard;
        private Moment toldToStop;

        Run(int id, SimCatalog.Entry entry, String startedBy, Pose2d start, Long seed, Mode mode, Begin begin) {
            this.id = id;
            this.entry = entry;
            this.startedBy = startedBy;
            this.start = start;
            this.seed = seed;
            this.mode = mode;
            this.view = begin.view(asked);
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

        public synchronized List<SimRunStream.TickLine> ticks() {
            return List.copyOf(ticks);
        }

        @Override
        public synchronized String ticksJson(int from) {
            return SimRunStream.TickLine.array(ticks.subList(Math.min(from, ticks.size()), ticks.size()));
        }

        public synchronized String log() {
            return String.join("\n", log);
        }

        synchronized void addTick(SimRunStream.TickLine tick) {
            ticks.add(tick);
        }

        synchronized double seconds() {
            return ticks.isEmpty() ? 0 : ticks.get(ticks.size() - 1).seconds();
        }

        synchronized void addLog(String line) {
            if (log.size() == LOG_LINES) {
                log.removeFirst();
            }
            log.addLast(line);
        }

        /** Whether the run takes the child built for it: not once Stop has ended it while it built. */
        synchronized boolean launched(Child.Running process, Moment at) {
            if (!state().takesAChild()) {
                return false;
            }
            child = process;
            phase = "starting";
            startupSince = at;
            heard = at;
            return true;
        }

        synchronized void place(JsonObject startLine, Moment at) {
            if (view.holdsTheStart(at, waits.watched.view())) {
                heldStart = startLine;
                return;
            }
            send(startLine);
        }

        synchronized void viewReady(Moment at) {
            view = new View.Ready();
            placeHeld(at);
        }

        synchronized void placeHeld(Moment at) {
            if (heldStart == null) {
                return;
            }
            JsonObject startLine = heldStart;
            heldStart = null;
            startupSince = at;
            send(startLine);
        }

        synchronized boolean viewWaitedFor() {
            return view.waitedFor();
        }

        /** The child said something, which is all a watchdog listening for a hang needs to know. */
        synchronized void heard(Moment at) {
            heard = at;
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
            looked = now();
        }

        /** How far the run has got, as the watchdog and Stop decide from it, at one moment. */
        synchronized RunState state() {
            if (outcome != null || (child != null && !child.alive())) {
                return new RunState.Over();
            }
            if (child == null) {
                return new RunState.Building();
            }
            if (toldToStop != null) {
                return new RunState.Stopping(toldToStop);
            }
            if (heldStart != null) {
                return new RunState.Held(asked);
            }
            if (phase.equals("starting")) {
                return new RunState.Starting(startupSince);
            }
            return new RunState.Running(heard, looked);
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
            item.addProperty("loops", ticks.size());
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
            Field.AllianceArea area = Valid.value(SimPlacement.FIELD.allianceArea(entry.drivenFrom()));
            Vec3 eye = area.eye(), lookingAt = area.lookingAt();
            JsonObject match = new JsonObject();
            match.addProperty("period", budgetSeconds());
            match.addProperty("alliance", area.alliance());
            match.add("eye", GSON.toJsonTree(new double[] {eye.x(), eye.y(), eye.z()}));
            match.add("lookingAt", GSON.toJsonTree(new double[] {lookingAt.x(), lookingAt.y(), lookingAt.z()}));
            return match;
        }

        synchronized boolean send(JsonObject line) {
            if (child == null || outcome != null) {
                return false;
            }
            return child.say(GSON.toJson(line));
        }

        /** What pressing Stop does is {@link org.firstinspires.ftc.teamcode.simcore.Stop}'s table. */
        void stop() {
            Child.Running unheard;
            synchronized (this) {
                boolean tell =
                        switch (state().onStop()) {
                            case NOTHING -> false;
                            case END_UNBUILT -> {
                                finish(SimRunStream.Outcome.stopped(), "stopped before the build finished");
                                yield false;
                            }
                            case TELL -> true;
                        };
                if (!tell) {
                    return;
                }
                // The grace runs from now, and the watchdog holds the child to it.
                toldToStop = now();
                heldStart = null;
                JsonObject line = new JsonObject();
                line.addProperty("stop", true);
                if (send(line)) {
                    return;
                }
                unheard = child;
            }
            unheard.kill();
        }

        @Override
        public String name() {
            return entry.name;
        }

        @Override
        public String kind() {
            return entry.kind;
        }

        @Override
        public TeamRobot robot() {
            return robot;
        }
    }

    public interface Factory {
        SimBench create(Path worktree, TeamRobot robot);
    }

    private final TeamRobot robot;
    private final SimSources sources;
    private final Path outputDir;
    private final Waits waits;
    private final Child children;
    private final StartPoses startPoses;
    private final List<Run> runs = new ArrayList<>();
    private SimBuild.Result lastCheck;

    public SimBench(TeamRobot robot, SimCatalog fixedCatalog, Path project, Path outputDir, Waits waits) {
        this(robot, fixedCatalog, project, outputDir, waits, new JvmChild());
    }

    public SimBench(
            TeamRobot robot, SimCatalog fixedCatalog, Path project, Path outputDir, Waits waits, Child children) {
        this(robot, sourcesOf(fixedCatalog, project, outputDir), outputDir, waits, children, new SystemClock());
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
    public SimBench(TeamRobot robot, SimSources sources, Path outputDir, Waits waits, Child children, Clock clock) {
        this.robot = robot;
        this.sources = sources;
        this.outputDir = outputDir;
        this.waits = waits;
        this.children = children;
        this.clock = clock;
        this.startPoses = new StartPoses(outputDir.resolve(START_POSES_FILE));
    }

    public SimCatalog catalog() {
        synchronized (this) {
            if (current() != null) {
                Optional<SimCatalog> already = sources.known();
                if (already.isPresent()) {
                    return already.get();
                }
            }
            return sources.catalog(children, robot);
        }
    }

    public TeamRobot robot() {
        return robot;
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

    static SimCatalog listOn(Child children, Path classes, TeamRobot robot) {
        StringBuilder said = new StringBuilder();
        List<String> args = new ArrayList<>(List.of("--list"));
        args.addAll(SimRunStream.robotArguments(robot));
        try (Child.Running child =
                children.onTheClassesAt(classes, line -> said.append(line).append('\n'), args.toArray(new String[0]))) {
            String first = child.hear();
            String line;
            try {
                line = first == null ? null : SimRunStream.afterHello(first, robot);
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
                                        SimReplayPage.placement(name, kindOf(name), robot, startPoses.get(name)))))
                .route(
                        "POST",
                        "/run",
                        (request, params) ->
                                run(request.query("opmode"), request.query("mode"), request.query("begin"), startedBy))
                .redirect("GET", "/runs/{id}", params -> "/runs/" + params.get("id") + "/")
                .route("GET", "/runs/{id}/", (request, params) -> withRun(params, request, this::page))
                .route(
                        "POST",
                        "/runs/{id}/ready",
                        (request, params) -> withRun(params, request, (run, r) -> {
                            run.viewReady(now());
                            return Response.json("{}");
                        }))
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
        return Response.html(SimReplayPage.live(run, ASSETS_FROM_A_RUN, run.viewWaitedFor()));
    }

    private Response run(String opMode, String modeWord, String beginWord, String startedBy) {
        Optional<Mode> mode = modeWord == null ? Optional.of(Mode.FREE_PLAY) : Mode.called(modeWord);
        if (mode.isEmpty()) {
            return Response.error(
                    400,
                    "a run is a game or free play: mode=" + Mode.GAME.word + " or mode=" + Mode.FREE_PLAY.word
                            + ", not '" + modeWord + "'");
        }
        Optional<Begin> begin = beginWord == null ? Optional.of(Begin.AT_ONCE) : Begin.called(beginWord);
        if (begin.isEmpty()) {
            return Response.error(
                    400,
                    "a run begins at once or once its live view is ready: begin=" + Begin.AT_ONCE.word + " or begin="
                            + Begin.WHEN_ITS_VIEW_IS_READY.word + ", not '" + beginWord + "'");
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
        Run run = start(entry.get(), startedBy, mode.get(), begin.get());
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

    public Run start(SimCatalog.Entry entry, String startedBy, Mode mode) {
        return start(entry, startedBy, mode, Begin.AT_ONCE);
    }

    public synchronized Run start(SimCatalog.Entry entry, String startedBy, Mode mode, Begin begin) {
        if (current() != null) {
            return null;
        }
        Run run = new Run(
                runs.size() + 1,
                entry,
                startedBy,
                startPoses.get(entry.name),
                startPoses.seed(entry.name),
                mode,
                begin);
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
        args.addAll(SimRunStream.robotArguments(robot));
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
        if (!run.launched(child, now())) {
            // Stop ended the run while its sources built, and a child nobody will place or stop
            // would wait to be placed for as long as the server runs.
            child.kill();
            child.close();
            return;
        }
        Thread watchdog = new Thread(() -> watch(run, child), "sim-run-" + run.id + "-watchdog");
        watchdog.setDaemon(true);
        watchdog.start();
        String[] outcome = {null};
        SimRunStream.Listener listener = new SimRunStream.Listener() {
            @Override
            public void started() {
                run.started();
            }

            @Override
            public void tick(SimRunStream.TickLine tick) {
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
                run.heard(now());
                if (line.isBlank()) {
                    continue;
                }
                if (first) {
                    first = false;
                    SimRunStream.Handshake handshake;
                    try {
                        handshake = SimRunStream.handshake(line, run.start, run.seed, robot);
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
                        run.place(handshake.startLine, now());
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
        if (outcome[0] != null) {
            run.finish(outcome[0], null);
        } else {
            run.finish(SimRunStream.Outcome.childExited(child.exitCode().orElse(-1)), run.log());
        }
    }

    /** How a run the watchdog gave up on ends: its outcome, and the message that says why. */
    private record Ended(String outcome, String message) {}

    /**
     * Holds a run's child to the waits, on the bench's clock, until the run is over; what each
     * verdict is, is {@link Watchdog}'s. It ends when the run does, so nothing need stop it.
     */
    private void watch(Run run, Child.Running child) {
        Watchdog.Verdict verdict = Watchdog.verdict(run.state(), now(), waits.watched);
        while (verdict.keepsWatching()) {
            if (verdict == Watchdog.Verdict.VIEW_LATE) {
                run.placeHeld(now());
            }
            clock.sleep(WATCH_POLL_SECONDS);
            verdict = Watchdog.verdict(run.state(), now(), waits.watched);
        }
        Optional<Ended> ended =
                switch (verdict) {
                    case WATCHING, OVER, VIEW_LATE -> Optional.empty();
                    case NEVER_STARTED ->
                        Optional.of(new Ended(
                                SimRunStream.Outcome.killed(waits.startup, "the op mode never started"),
                                "the child JVM never said the op mode had started; it was killed"));
                    case SILENT ->
                        Optional.of(
                                new Ended(
                                        SimRunStream.Outcome.killed(waits.silence, "the op mode did not return"),
                                        "loop() never came back, so nothing in the child could end the run; the child JVM was killed"));
                    // Free play has no end of its own, so a run somebody walked away from would go on
                    // holding a child and growing its ticks until the server ran out of memory.
                    // Nobody is watching once no page is asking about it.
                    case UNWATCHED ->
                        Optional.of(new Ended(
                                SimRunStream.Outcome.stopped(),
                                String.format(
                                        "nobody had looked at the run for %.1fs, so the bench stopped it and the child JVM"
                                                + " with it",
                                        waits.unwatched)));
                    case IGNORED_STOP ->
                        Optional.of(
                                new Ended(
                                        SimRunStream.Outcome.killedAfterStop(waits.killGrace),
                                        "loop() never came back after Stop, so nothing in the child could end the run; the child JVM was killed"));
                };
        ended.ifPresent(end -> {
            run.finish(end.outcome, end.message);
            child.kill();
        });
    }

    private Moment now() {
        return new Moment(clock.nanos());
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
