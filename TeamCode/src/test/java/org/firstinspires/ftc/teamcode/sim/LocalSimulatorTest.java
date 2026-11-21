package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.net.ConnectException;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class LocalSimulatorTest {
    private static final String OP_MODE = "Count to three";

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private LocalSimulator simulator;

    @After
    public void stopTheSimulator() {
        if (simulator != null) {
            simulator.stop();
        }
    }

    private LocalSimulator over(FakeChild child) {
        SimBench bench = new SimBench(
                TeamRobot.REGINALD,
                FakeSources.listing(SimCatalog.of(ThreeLoopAuto.class)),
                folder.getRoot().toPath().resolve("sim"),
                SimBench.Waits.ofTheBench(),
                child,
                new SystemClock());
        simulator = LocalSimulator.start(bench, 0, "mia");
        return simulator;
    }

    private static FakeChild aChildThatRunsUntilStopped() {
        return FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started()).thatStaysAliveSayingNothingMore();
    }

    @Test
    public void thePageAtItsRootIsTheDashboard() throws IOException {
        over(aChildThatRunsUntilStopped());

        Reply page = send("GET", "/");

        assertEquals(200, page.status);
        assertEquals(SimAssets.page("dashboard.html"), page.body);
        assertTrue(page.body, page.body.contains("<title>Simulator</title>"));
    }

    @Test
    public void itsPageAsksForNoEditorSinceItServesNone() throws IOException {
        over(aChildThatRunsUntilStopped());

        Reply page = send("GET", "/");

        assertFalse(page.body, page.body.contains("/static/"));
        assertEquals(404, send("GET", "/static/codemirror.js").status);
    }

    @Test
    public void theBenchIsMountedWhereTheDashboardAsksForIt() throws IOException {
        over(aChildThatRunsUntilStopped());

        Reply catalog = send("GET", "/sim/catalog");

        assertEquals(catalog.body, 200, catalog.status);
        assertTrue(catalog.body, catalog.body.contains("\"name\":\"" + OP_MODE + "\""));
    }

    @Test
    public void itListensOnLoopbackAloneSoTheLanIsRefused() throws IOException {
        over(aChildThatRunsUntilStopped());
        assertTrue(simulator.bindAddress().isLoopbackAddress());
        InetAddress lan = LanAddress.first();

        try (Socket socket = new Socket()) {
            socket.connect(new InetSocketAddress(lan, simulator.port()), 2000);
            fail("connected to the simulator via " + lan);
        } catch (ConnectException | SocketTimeoutException refused) {
        }
        assertEquals(200, send("GET", "/sim/status").status);
    }

    @Test
    public void itsUrlIsTheLoopbackAddressItListensOn() {
        over(aChildThatRunsUntilStopped());

        assertEquals(
                "http://" + simulator.bindAddress().getHostAddress() + ":" + simulator.port() + "/", simulator.url());
    }

    @Test
    public void stoppingItStopsTheRunItStartedAndTheChildWithIt() throws Exception {
        FakeChild child = aChildThatRunsUntilStopped();
        over(child);
        Reply started = send("POST", "/sim/run?opmode=" + URLEncoder.encode(OP_MODE, StandardCharsets.UTF_8));
        assertEquals(started.body, 200, started.status);
        JsonObject run = awaitRunning();
        assertEquals("mia", run.get("startedBy").getAsString());

        simulator.stop();

        assertFalse("the child outlived the simulator", child.running().alive());
        simulator = null;
    }

    private JsonObject awaitRunning() throws Exception {
        long deadline = System.nanoTime() + 10_000_000_000L;
        JsonObject run = null;
        while (System.nanoTime() < deadline) {
            JsonObject status = new Gson().fromJson(send("GET", "/sim/status").body, JsonObject.class);
            run = status.getAsJsonArray("runs").get(0).getAsJsonObject();
            if (run.get("phase").getAsString().equals("running")) {
                return run;
            }
            Thread.sleep(20);
        }
        assertNotNull("no run was ever listed", run);
        throw new AssertionError("the run never began running: " + run);
    }

    private record Reply(int status, String body) {}

    private Reply send(String method, String path) throws IOException {
        HttpURLConnection connection =
                (HttpURLConnection) new URL(simulator.url() + path.substring(1)).openConnection();
        connection.setRequestMethod(method);
        if (method.equals("POST")) {
            connection.setDoOutput(true);
            connection.getOutputStream().close();
        }
        int status = connection.getResponseCode();
        try (InputStream in = status < 400 ? connection.getInputStream() : connection.getErrorStream()) {
            return new Reply(status, in == null ? "" : new String(in.readAllBytes(), StandardCharsets.UTF_8));
        }
    }
}
