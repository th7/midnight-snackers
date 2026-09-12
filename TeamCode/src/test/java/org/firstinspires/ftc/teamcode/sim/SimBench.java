package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * The bench core shared by the simulation bench and the shared editor's Simulate tab: the
 * catalog of runnable autonomous op modes, the runs so far, and the routes that start a run and
 * follow it. One run at a time, in real time, whoever asks. Runs execute the op mode classes as
 * loaded when the process started; edits to their source take effect after a restart.
 */
public final class SimBench {
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    public static final class Run {
        public final int id;
        public final SimCatalog.Entry entry;
        public final SimRecording recording;
        public final long startedAtMillis = System.currentTimeMillis();
        /** Who started it, or null when the bench's own page did. */
        public final String startedBy;
        Thread thread;

        Run(int id, SimCatalog.Entry entry, String startedBy) {
            this.id = id;
            this.entry = entry;
            this.startedBy = startedBy;
            this.recording = new SimRecording(entry.type.getSimpleName());
        }

        /**
         * In progress until the recording has its outcome; the thread may still be writing the
         * replay file for a moment after that, which nobody need wait for.
         */
        public boolean running() {
            return thread != null && thread.isAlive() && !recording.finished();
        }
    }

    private final SimCatalog catalog;
    private final Path outputDir;
    private final double runTimeoutSeconds;
    private final List<Run> runs = new ArrayList<>();

    public SimBench(SimCatalog catalog, Path outputDir, double runTimeoutSeconds) {
        this.catalog = catalog;
        this.outputDir = outputDir;
        this.runTimeoutSeconds = runTimeoutSeconds;
    }

    public SimCatalog catalog() {
        return catalog;
    }

    public JsonArray catalogJson() {
        JsonArray entries = new JsonArray();
        for (SimCatalog.Entry entry : catalog.entries()) {
            JsonObject item = new JsonObject();
            item.addProperty("name", entry.name);
            item.addProperty("group", entry.group);
            item.addProperty("opMode", entry.type.getName());
            entries.add(item);
        }
        return entries;
    }

    /**
     * The routes, with {@code path} relative to wherever the caller mounted them: {@code /catalog},
     * {@code /status}, {@code POST /run?opmode=}, {@code /runs/<id>/}, {@code /runs/<id>/ticks}.
     *
     * @param startedBy the name to record on a run started by this request, or null
     */
    public Response handle(String path, Request request, String startedBy) {
        if (path.equals("/catalog")) {
            return Response.json(GSON.toJson(catalogJson()));
        }
        if (path.equals("/status")) {
            return Response.json(status());
        }
        if (path.equals("/run")) {
            if (!request.method.equals("POST")) {
                return Response.error(405, "POST /run?opmode=<class name> to start a run");
            }
            String opMode = request.query("opmode");
            Optional<SimCatalog.Entry> entry = opMode == null ? Optional.empty() : catalog.find(opMode);
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
                return Response.html(SimReplayPage.page(run.recording, true));
            }
            if (rest.equals("ticks")) {
                return Response.json(SimReplayPage.update(run.recording, SimLiveServer.from(request)));
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
        run.thread = new Thread(() -> {
            SimRobot sim = new SimRobot();
            AutoOp opMode = entry.create();
            try {
                SimRunner.record(run.recording, opMode, sim, runTimeoutSeconds, outputDir);
            } catch (RuntimeException | Error e) {
                // the outcome is on the recording; the pages show it
            }
        }, "sim-run-" + run.id);
        run.thread.setDaemon(true);
        run.thread.start();
        return run;
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

    public synchronized String status() {
        JsonObject root = new JsonObject();
        root.addProperty("running", current() != null);
        JsonArray list = new JsonArray();
        for (int i = runs.size() - 1; i >= 0; i--) {
            Run run = runs.get(i);
            List<SimRecording.Tick> ticks = run.recording.ticks();
            JsonObject item = new JsonObject();
            item.addProperty("id", run.id);
            item.addProperty("opMode", run.entry.type.getName());
            item.addProperty("name", run.entry.name);
            item.addProperty("startedAt", run.startedAtMillis);
            item.addProperty("startedBy", run.startedBy);
            item.addProperty("loops", ticks.size());
            item.addProperty("seconds", ticks.isEmpty() ? 0 : ticks.get(ticks.size() - 1).seconds);
            item.addProperty("outcome", run.recording.outcome());
            list.add(item);
        }
        root.add("runs", list);
        return GSON.toJson(root);
    }
}
