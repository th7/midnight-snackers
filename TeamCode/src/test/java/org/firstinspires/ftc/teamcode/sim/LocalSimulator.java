package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.PortSetting;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;

/**
 * The simulator a teammate runs from their own checkout: one bench over the sources on disk, for
 * Reginald, served on loopback alone to the browser on the same machine. It serves the dashboard's
 * Simulate, Launch Sim and Feeder Sim tabs.
 * <pre>
 * ./gradlew :TeamCode:simulator      (or the Simulator run configuration in Android Studio)
 * </pre>
 */
public final class LocalSimulator {
    public static final String PORT_ENV = "SIMULATOR_PORT";
    public static final int DEFAULT_PORT = 21988;

    static final String PAGE = "dashboard.html";

    private final SimBench bench;
    private final TinyHttpServer http;

    private LocalSimulator(SimBench bench, TinyHttpServer http) {
        this.bench = bench;
        this.http = http;
    }

    public static LocalSimulator start(SimBench bench, int port, String whoRunsIt) {
        String page = resource(PAGE);
        Router routes = new Router()
                .route("GET", "/", (request, params) -> Response.html(page))
                .mount("/sim", bench.routes(whoRunsIt));
        return new LocalSimulator(
                bench, TinyHttpServer.start(InetAddress.getLoopbackAddress(), port, "simulator", routes));
    }

    public static void main(String[] args) throws InterruptedException {
        Path checkout = Path.of("").toAbsolutePath();
        SimBench bench = new SimBench(
                TeamRobot.REGINALD,
                null,
                checkout,
                checkout.resolve("TeamCode").resolve(SimRunner.DEFAULT_OUTPUT_DIR),
                SimBench.Waits.ofTheBench(),
                new JvmChild());
        LocalSimulator simulator = start(
                bench,
                Valid.value(PortSetting.of(System.getenv(), PORT_ENV, DEFAULT_PORT)),
                System.getProperty("user.name"));
        Runtime.getRuntime().addShutdownHook(new Thread(simulator::stop, "simulator-stop"));
        System.out.println("Simulator: " + simulator.url());
        System.out.println(
                "  runs the sources in " + checkout + " as saved; this machine only; stop the task to stop it");
        Thread.currentThread().join();
    }

    public String url() {
        return "http://" + bindAddress().getHostAddress() + ":" + port() + "/";
    }

    public int port() {
        return http.port();
    }

    public InetAddress bindAddress() {
        return http.bindAddress();
    }

    public void stop() {
        http.stop();
        bench.stop();
    }

    private static String resource(String name) {
        try (InputStream in = LocalSimulator.class.getResourceAsStream(name)) {
            if (in == null) {
                throw new IllegalStateException(
                        "missing resource " + name + " next to " + LocalSimulator.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
