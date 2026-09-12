package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.sim.TestAutos.NeverDoneAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

public class SimDevServerTest {
    private static final double RUN_TIMEOUT_SECONDS = 0.3;

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private SimDevServer server;

    private SimDevServer server() {
        if (server == null) {
            server = SimDevServer.start(new SimBench(SimCatalog.of(ThreeLoopAuto.class, NeverDoneAuto.class), null,
                    folder.getRoot().toPath(), RUN_TIMEOUT_SECONDS, 1), 0);
        }
        return server;
    }

    @After
    public void stopServer() {
        if (server != null) {
            server.stop();
        }
    }

    @Test
    public void theBenchPageListsTheRunnableOpModes() throws IOException {
        Response page = get("/");

        assertEquals(200, page.status);
        assertTrue(page.body.contains("Count to three"));
        assertTrue(page.body.contains("Never done"));
        assertTrue(page.body.contains(ThreeLoopAuto.class.getName()));
        assertTrue(page.body, page.body.contains("run.phase === 'building'"));
        assertTrue(page.body, page.body.contains("run.message"));
        assertTrue(page.body, page.body.contains("id=\"problem\""));
        assertFalse(page.body, page.body.contains("__PROBLEM__"));
    }

    @Test
    public void runStartsAnOpModeAndTheStatusFollowsItToItsOutcome() throws Exception {
        Response started = post("/run?opmode=" + ThreeLoopAuto.class.getName());
        assertEquals(200, started.status);
        String id = started.json("id");

        String status = awaitStatus("\"outcome\":\"done\"");

        assertTrue(status, status.contains("\"id\":" + id));
        assertTrue(status, status.contains("\"name\":\"Count to three\""));
        assertTrue(status, status.contains("\"running\":false"));
        Response ticks = get("/runs/" + id + "/ticks?from=0");
        assertEquals(200, ticks.status);
        assertEquals(3, ticks.body.split("\"step\"").length - 1);
        Response live = get("/runs/" + id + "/");
        assertTrue(live.body.contains("<canvas"));
        assertTrue(live.body.contains("\"live\":true"));
        assertTrue(folder.getRoot().toPath().resolve("ThreeLoopAuto.html").toFile().exists());
    }

    @Test
    public void onlyOneRunAtATimeAndATimedOutRunReportsIt() throws Exception {
        Response first = post("/run?opmode=" + NeverDoneAuto.class.getName());
        assertEquals(200, first.status);
        assertTrue(get("/status").body.contains("\"running\":true"));

        Response second = post("/run?opmode=" + ThreeLoopAuto.class.getName());

        assertEquals(409, second.status);
        String status = awaitStatus("\"outcome\":\"timed out");
        assertTrue(status, status.contains("\"running\":false"));
    }

    @Test
    public void unknownOpModesAndWrongMethodsAreRejected() throws IOException {
        assertEquals(404, post("/run?opmode=org.example.Nope").status);
        assertEquals(405, get("/run?opmode=" + ThreeLoopAuto.class.getName()).status);
        assertEquals(404, get("/runs/999/ticks?from=0").status);
    }

    private String awaitStatus(String marker) throws Exception {
        long deadline = System.nanoTime() + 10_000_000_000L;
        String status = "";
        while (System.nanoTime() < deadline) {
            status = get("/status").body;
            if (status.contains(marker)) {
                return status;
            }
            Thread.sleep(20);
        }
        throw new AssertionError("status never contained " + marker + "; last: " + status);
    }

    private Response get(String path) throws IOException {
        return request("GET", path);
    }

    private Response post(String path) throws IOException {
        return request("POST", path);
    }

    private Response request(String method, String path) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(server().url() + path.substring(1)).openConnection();
        connection.setRequestMethod(method);
        int status = connection.getResponseCode();
        try (InputStream in = status < 400 ? connection.getInputStream() : connection.getErrorStream()) {
            return new Response(status, in == null ? "" : new String(in.readAllBytes(), StandardCharsets.UTF_8));
        } finally {
            connection.disconnect();
        }
    }

    private static final class Response {
        final int status;
        final String body;

        Response(int status, String body) {
            this.status = status;
            this.body = body;
        }

        String json(String key) {
            String needle = "\"" + key + "\":";
            int at = body.indexOf(needle) + needle.length();
            int end = at;
            while (end < body.length() && Character.isDigit(body.charAt(end))) {
                end++;
            }
            return body.substring(at, end);
        }
    }
}
