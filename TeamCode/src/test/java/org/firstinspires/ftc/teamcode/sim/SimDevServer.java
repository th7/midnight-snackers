package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * The simulation bench: a page listing every runnable autonomous op mode, a Run control that
 * starts it on a fresh simulated robot, the live view of the run in progress, and the history of
 * runs with their outcomes and replays. One run at a time, in real time.
 * <pre>
 * ./gradlew :TeamCode:simDev      then open http://localhost:8765/
 * </pre>
 */
public final class SimDevServer {
    public static final String PORT_ENV = "SIM_DEV_PORT";
    public static final int DEFAULT_PORT = 8765;
    public static final double DEFAULT_RUN_TIMEOUT_SECONDS = 60;
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private static final class Run {
        final int id;
        final SimCatalog.Entry entry;
        final SimRecording recording;
        final long startedAtMillis = System.currentTimeMillis();
        Thread thread;

        Run(int id, SimCatalog.Entry entry) {
            this.id = id;
            this.entry = entry;
            this.recording = new SimRecording(entry.type.getSimpleName());
        }

        /**
         * In progress until the recording has its outcome; the thread may still be writing the
         * replay file for a moment after that, which the bench need not wait for.
         */
        boolean running() {
            return thread != null && thread.isAlive() && !recording.finished();
        }
    }

    private final SimCatalog catalog;
    private final Path outputDir;
    private final double runTimeoutSeconds;
    private final TinyHttpServer http;
    private final List<Run> runs = new ArrayList<>();

    private SimDevServer(SimCatalog catalog, int port, Path outputDir, double runTimeoutSeconds) {
        this.catalog = catalog;
        this.outputDir = outputDir;
        this.runTimeoutSeconds = runTimeoutSeconds;
        this.http = TinyHttpServer.start(port, "sim-bench", this::handle);
    }

    public static SimDevServer start(SimCatalog catalog, int port, Path outputDir, double runTimeoutSeconds) {
        return new SimDevServer(catalog, port, outputDir, runTimeoutSeconds);
    }

    public static void main(String[] args) throws InterruptedException {
        String portValue = System.getenv(PORT_ENV);
        int port = portValue == null || portValue.isBlank() ? DEFAULT_PORT : Integer.parseInt(portValue.trim());
        SimCatalog catalog = SimCatalog.discover();
        SimDevServer server = start(catalog, port, SimRunner.DEFAULT_OUTPUT_DIR, DEFAULT_RUN_TIMEOUT_SECONDS);
        System.out.println("Simulation bench: " + server.url() + "  (" + catalog.entries().size() + " op modes; Ctrl-C to stop)");
        Thread.currentThread().join();
    }

    public int port() {
        return http.port();
    }

    public String url() {
        return http.url();
    }

    public void stop() {
        http.stop();
    }

    private Response handle(Request request) {
        if (request.path.equals("/")) {
            return Response.html(page());
        }
        if (request.path.equals("/status")) {
            return Response.json(status());
        }
        if (request.path.equals("/run")) {
            if (!request.method.equals("POST")) {
                return Response.error(405, "POST /run?opmode=<class name> to start a run");
            }
            String opMode = request.query("opmode");
            Optional<SimCatalog.Entry> entry = opMode == null ? Optional.empty() : catalog.find(opMode);
            if (entry.isEmpty()) {
                return Response.error(404, "no runnable op mode named " + opMode);
            }
            Run run = startRun(entry.get());
            if (run == null) {
                return Response.error(409, "a run is already in progress");
            }
            JsonObject body = new JsonObject();
            body.addProperty("id", run.id);
            return Response.json(GSON.toJson(body));
        }
        if (request.path.startsWith("/runs/")) {
            String[] parts = request.path.split("/");
            Run run = parts.length >= 3 ? find(parts[2]) : null;
            if (run == null) {
                return Response.error(404, "no such run: " + request.path);
            }
            String rest = parts.length >= 4 ? parts[3] : "";
            if (rest.isEmpty()) {
                return Response.html(SimReplayPage.page(run.recording, true));
            }
            if (rest.equals("ticks")) {
                return Response.json(SimReplayPage.update(run.recording, SimLiveServer.from(request)));
            }
        }
        return Response.error(404, "not found: " + request.path);
    }

    private synchronized Run startRun(SimCatalog.Entry entry) {
        if (!runs.isEmpty() && runs.get(runs.size() - 1).running()) {
            return null;
        }
        Run run = new Run(runs.size() + 1, entry);
        runs.add(run);
        run.thread = new Thread(() -> {
            SimRobot sim = new SimRobot();
            AutoOp opMode = entry.create();
            try {
                SimRunner.record(run.recording, opMode, sim, runTimeoutSeconds, outputDir);
            } catch (RuntimeException | Error e) {
                // the outcome is on the recording; the bench shows it
            }
        }, "sim-run-" + run.id);
        run.thread.setDaemon(true);
        run.thread.start();
        return run;
    }

    private synchronized Run find(String id) {
        for (Run run : runs) {
            if (String.valueOf(run.id).equals(id)) {
                return run;
            }
        }
        return null;
    }

    private synchronized String status() {
        JsonObject root = new JsonObject();
        boolean running = !runs.isEmpty() && runs.get(runs.size() - 1).running();
        root.addProperty("running", running);
        JsonArray list = new JsonArray();
        for (int i = runs.size() - 1; i >= 0; i--) {
            Run run = runs.get(i);
            List<SimRecording.Tick> ticks = run.recording.ticks();
            JsonObject item = new JsonObject();
            item.addProperty("id", run.id);
            item.addProperty("opMode", run.entry.type.getName());
            item.addProperty("name", run.entry.name);
            item.addProperty("startedAt", run.startedAtMillis);
            item.addProperty("loops", ticks.size());
            item.addProperty("seconds", ticks.isEmpty() ? 0 : ticks.get(ticks.size() - 1).seconds);
            item.addProperty("outcome", run.recording.outcome());
            list.add(item);
        }
        root.add("runs", list);
        return GSON.toJson(root);
    }

    private String page() {
        JsonArray entries = new JsonArray();
        for (SimCatalog.Entry entry : catalog.entries()) {
            JsonObject item = new JsonObject();
            item.addProperty("name", entry.name);
            item.addProperty("group", entry.group);
            item.addProperty("opMode", entry.type.getName());
            entries.add(item);
        }
        return template().replace("__CATALOG__", GSON.toJson(entries));
    }

    private static String template() {
        try (InputStream in = SimDevServer.class.getResourceAsStream("bench.html")) {
            if (in == null) {
                throw new IllegalStateException("missing resource bench.html next to " + SimDevServer.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
