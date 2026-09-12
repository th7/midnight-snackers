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
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
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

    /** The sources do not compile; the message is the compiler's diagnostics. */
    public static final class BuildFailed extends RuntimeException {
        BuildFailed(String diagnostics) {
            super(diagnostics);
        }
    }

    public final class Run {
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

        /** {@code building}, {@code running}, or {@code finished}. */
        public synchronized String phase() {
            return phase;
        }

        public synchronized String outcome() {
            return outcome;
        }

        /** Compiler errors or the kill reason, when there is something to say. */
        public synchronized String message() {
            return message;
        }

        public synchronized JsonArray ticks() {
            return ticksFrom(0);
        }

        public synchronized JsonArray ticksFrom(int from) {
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

        synchronized void addLog(String line) {
            if (log.size() == LOG_LINES) {
                log.removeFirst();
            }
            log.addLast(line);
        }

        synchronized void running(Process process) {
            child = process;
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

        String name() {
            String name = entry.className.substring(entry.className.lastIndexOf('.') + 1);
            return name.substring(name.lastIndexOf('$') + 1);
        }
    }

    private final SimCatalog fixedCatalog;
    private final SimBuild build;
    private final Path outputDir;
    private final double runTimeoutSeconds;
    private final double killGraceSeconds;
    private final List<Run> runs = new ArrayList<>();
    private SimCatalog listed;
    private Path listedFrom;
    private SimBuild.Result lastCheck;

    /**
     * @param fixedCatalog     the op modes to offer, or null to compile and list them from {@code sourceRoot}
     * @param sourceRoot       the main sources to compile before each run, or null to run this JVM's classes
     * @param killGraceSeconds how long past the run timeout the child may live before it is killed
     */
    public SimBench(SimCatalog fixedCatalog, Path sourceRoot, Path outputDir, double runTimeoutSeconds, double killGraceSeconds) {
        if ((fixedCatalog == null) == (sourceRoot == null)) {
            throw new IllegalArgumentException("give either a fixed catalog or a source root");
        }
        this.fixedCatalog = fixedCatalog;
        this.build = sourceRoot == null ? null : new SimBuild(sourceRoot, outputDir.resolve("classes"));
        this.outputDir = outputDir;
        this.runTimeoutSeconds = runTimeoutSeconds;
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
     * The routes, with {@code path} relative to wherever the caller mounted them: {@code /catalog},
     * {@code /status}, {@code POST /run?opmode=}, {@code /runs/<id>/}, {@code /runs/<id>/ticks},
     * {@code /runs/<id>/log}.
     *
     * @param startedBy the name to record on a run started by this request, or null
     */
    public Response handle(String path, Request request, String startedBy) {
        if (path.equals("/catalog")) {
            try {
                return Response.json(GSON.toJson(catalog().toJson()));
            } catch (BuildFailed e) {
                return Response.error(500, "the sources do not compile:\n" + e.getMessage());
            }
        }
        if (path.equals("/status")) {
            return Response.json(status());
        }
        if (path.equals("/run")) {
            if (!request.method.equals("POST")) {
                return Response.error(405, "POST /run?opmode=<class name> to start a run");
            }
            String opMode = request.query("opmode");
            Optional<SimCatalog.Entry> entry = Optional.empty();
            if (opMode != null) {
                try {
                    entry = catalog().find(opMode);
                } catch (BuildFailed e) {
                    // let the run itself report the build failure, where the tab shows it
                    entry = listed == null ? Optional.empty() : listed.find(opMode);
                    if (entry.isEmpty()) {
                        entry = Optional.of(new SimCatalog.Entry(opMode, "", opMode, null));
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
        if (path.startsWith("/runs/")) {
            String[] parts = path.split("/");
            Run run = parts.length >= 3 ? find(parts[2]) : null;
            if (run == null) {
                return Response.error(404, "no such run: " + path);
            }
            String rest = parts.length >= 4 ? parts[3] : "";
            if (rest.isEmpty()) {
                return Response.html(SimReplayPage.page(run.name(), true, run.ticks(), run.outcome()));
            }
            if (rest.equals("ticks")) {
                return Response.json(SimReplayPage.update(run.ticksFrom(SimLiveServer.from(request)), run.outcome()));
            }
            if (rest.equals("log")) {
                return new Response(200, "text/plain; charset=utf-8", run.log());
            }
        }
        return Response.error(404, "not found: " + path);
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
                run.finish("build failed", e.getMessage());
                return;
            }
            if (result.classes == null) {
                run.finish("build failed", result.diagnostics);
                return;
            }
            classpathFirst = List.of(result.classes);
        }
        Process child;
        try {
            child = SimChild.launch(classpathFirst, "--run", run.entry.className, String.valueOf(runTimeoutSeconds),
                    outputDir.toAbsolutePath().toString());
        } catch (RuntimeException e) {
            run.finish("could not start the child JVM", e.getMessage());
            return;
        }
        run.running(child);
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
        double maxSeconds = runTimeoutSeconds + killGraceSeconds;
        Thread watchdog = new Thread(() -> {
            try {
                if (!child.waitFor((long) (maxSeconds * 1000), TimeUnit.MILLISECONDS)) {
                    run.finish(String.format("killed after %.1fs: the op mode did not return", maxSeconds),
                            "loop() never came back, so nothing in the child could end the run; the child JVM was killed");
                    child.destroyForcibly();
                }
            } catch (InterruptedException ignored) {
                // stopping
            }
        }, "sim-run-" + run.id + "-watchdog");
        watchdog.setDaemon(true);
        watchdog.start();
        String outcome = null;
        try (BufferedReader out = new BufferedReader(new InputStreamReader(child.getInputStream(), StandardCharsets.UTF_8))) {
            for (String line = out.readLine(); line != null; line = out.readLine()) {
                JsonObject json;
                try {
                    json = GSON.fromJson(line, JsonObject.class);
                } catch (RuntimeException e) {
                    run.addLog("unreadable line from the child: " + line);
                    continue;
                }
                if (json == null) {
                    continue;
                }
                if (json.has("outcome")) {
                    outcome = json.get("outcome").getAsString();
                } else {
                    run.addTick(json);
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
        if (outcome != null) {
            run.finish(outcome, null);
        } else {
            run.finish("child exited with code " + child.exitValue(), run.log());
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
            current.finish("stopped", "the bench was stopped");
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
            JsonArray ticks = run.ticks();
            JsonObject item = new JsonObject();
            item.addProperty("id", run.id);
            item.addProperty("opMode", run.entry.className);
            item.addProperty("name", run.entry.name);
            item.addProperty("startedAt", run.startedAtMillis);
            item.addProperty("startedBy", run.startedBy);
            item.addProperty("phase", run.phase());
            item.addProperty("loops", ticks.size());
            item.addProperty("seconds", ticks.size() == 0 ? 0 : ticks.get(ticks.size() - 1).getAsJsonObject().get("t").getAsDouble());
            item.addProperty("outcome", run.outcome());
            item.addProperty("message", run.message());
            list.add(item);
        }
        root.add("runs", list);
        return GSON.toJson(root);
    }
}
