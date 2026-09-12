package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;

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

    private final SimBench bench;
    private final TinyHttpServer http;

    private SimDevServer(SimCatalog catalog, int port, Path outputDir, double runTimeoutSeconds) {
        this.bench = new SimBench(catalog, outputDir, runTimeoutSeconds);
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
        return bench.handle(request.path, request, null);
    }

    private String page() {
        return template().replace("__CATALOG__", GSON.toJson(bench.catalogJson()));
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
