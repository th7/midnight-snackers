package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.KeptModels;
import org.firstinspires.ftc.teamcode.simcore.PortSetting;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.firstinspires.ftc.teamcode.simcore.WhereTheHostIs;

/**
 * The simulator a teammate runs from their own checkout: one bench over the sources on disk, for
 * Reginald, served on loopback alone to the browser on the same machine. It serves the dashboard's
 * Simulate, Launch Sim and Spindexer Sim tabs.
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

    public static LocalSimulator start(SimBench bench, int port, String whoRunsIt, KeptField field) {
        String page = resource(PAGE);
        Router routes = new Router()
                .route("GET", "/", (request, params) -> Response.html(page))
                .route("GET", "/sim/assets/{name*}", (request, params) -> field.serve(params.get("name")))
                .mount("/sim", bench.routes(whoRunsIt));
        return new LocalSimulator(
                bench, TinyHttpServer.start(InetAddress.getLoopbackAddress(), port, "simulator", routes));
    }

    static Checked<RobotDescription> descriptionIn(Path checkout) {
        return SimDescription.read(checkout.resolve("TeamCode").resolve(SimDescription.FILE));
    }

    public static void main(String[] args) throws InterruptedException {
        Path checkout = Path.of("").toAbsolutePath();
        Checked<RobotDescription> described = descriptionIn(checkout);
        if (described instanceof Checked.Rejected<RobotDescription> refused) {
            System.err.println("The simulator did not start: " + refused.rule());
            System.exit(2);
            return;
        }
        RobotDescription description = Valid.value(described);
        SimBench bench = new SimBench(
                null,
                checkout,
                checkout.resolve("TeamCode").resolve(SimRunner.DEFAULT_OUTPUT_DIR),
                SimBench.Waits.ofTheBench(),
                new JvmChild(),
                description);
        Map<String, String> env = System.getenv();
        int port = Valid.value(PortSetting.of(env, PORT_ENV, DEFAULT_PORT));
        WhereTheHostIs host = Valid.value(WhereTheHostIs.from(env));
        KeptModels where =
                Valid.value(KeptModels.of(System.getProperty("os.name"), env, System.getProperty("user.home")));
        KeptField field = new KeptField(Path.of(where.root(), where.under().toArray(new String[0])), new OnDiskStore());
        List<String> fetched = field.fetchFrom(host);
        LocalSimulator simulator = start(bench, port, System.getProperty("user.name"), field);
        Runtime.getRuntime().addShutdownHook(new Thread(simulator::stop, "simulator-stop"));
        System.out.println("Simulator: " + simulator.url());
        System.out.println(
                "  runs the sources in " + checkout + " as saved; this machine only; stop the task to stop it");
        System.out.println("  robot description: " + SimDescription.described(bench.description()));
        fetched.forEach(System.out::println);
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
