package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.sim.TestAutos.NeverDoneAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

public class SimRunnerTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private final SimRobot sim = new SimRobot();

    @Test
    public void recordsEveryLoopUntilThePlanIsDoneAndWritesTheReplay() {
        Path out = folder.getRoot().toPath();

        SimRecording recording = SimRunner.run(new ThreeLoopAuto(), sim, 5, out);

        assertEquals(3, recording.ticks().size());
        assertEquals("count to three", recording.ticks().get(0).step);
        assertEquals("done", recording.outcome());
        assertTrue(Files.exists(out.resolve("ThreeLoopAuto.html")));
    }

    @Test
    public void keepsEveryLoopsPoseButThinsDrawingsToTwentyPerSecond() {

        SimRecording recording = SimRunner.run(new ThreeLoopAuto(), sim, 5, folder.getRoot().toPath());

        // Three loops a few milliseconds apart: the first carries the dashboard drawing, the rest don't.
        assertEquals(3, recording.poses().size());
        assertFalse(recording.ticks().get(0).packets.isEmpty());
        assertTrue(recording.ticks().get(1).packets.isEmpty());
        assertTrue(recording.ticks().get(2).packets.isEmpty());
    }

    @Test
    public void anAnonymousSubclassIsNamedAfterItsNearestNamedClass() {
        Path out = folder.getRoot().toPath();

        SimRunner.run(new ThreeLoopAuto() {
        }, sim, 5, out);

        assertTrue(Files.exists(out.resolve("ThreeLoopAuto.html")));
    }

    @Test
    public void withALivePortTheRunCanBeWatchedWhileItRunsAndUntilTheViewerHasSeenTheEnd() throws Exception {
        int port = freePort();
        Thread runner = new Thread(() -> {
            try {
                SimRunner.run(new NeverDoneAuto(), sim, 0.5, folder.getRoot().toPath(), port);
            } catch (AssertionError expected) {
                // the plan never finishes; the run times out by design
            }
        });
        runner.start();

        String duringRun = null;
        String afterRun = null;
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (System.nanoTime() < deadline) {
            String response = tryGet("http://localhost:" + port + "/ticks?from=0");
            if (response == null) {
                Thread.sleep(20);
                continue;
            }
            if (response.contains("\"outcome\":\"timed out")) {
                afterRun = response;
                break;
            }
            if (response.contains("\"forever\"")) {
                duringRun = response;
            }
            Thread.sleep(20);
        }
        runner.join(10_000);

        assertTrue("saw ticks while the run was in progress", duringRun != null);
        assertTrue("saw the outcome after the run", afterRun != null);
        assertFalse("the runner returned once the viewer had seen the end", runner.isAlive());
    }

    @Test
    public void aTimedOutRunStillWritesTheReplayBeforeFailing() throws Exception {
        Path out = folder.getRoot().toPath();

        AssertionError error = assertThrows(AssertionError.class,
                () -> SimRunner.run(new NeverDoneAuto(), sim, 0.1, out));

        assertTrue(error.getMessage(), error.getMessage().contains("forever"));
        Path page = out.resolve("NeverDoneAuto.html");
        assertTrue(Files.exists(page));
        String html = new String(Files.readAllBytes(page), StandardCharsets.UTF_8);
        assertTrue(html.contains("timed out"));
        assertFalse(html.contains("\"done\""));
    }

    private static int freePort() throws java.io.IOException {
        try (java.net.ServerSocket socket = new java.net.ServerSocket(0)) {
            return socket.getLocalPort();
        }
    }

    private static String tryGet(String url) {
        try {
            java.net.HttpURLConnection connection = (java.net.HttpURLConnection) new java.net.URL(url).openConnection();
            try (java.io.InputStream in = connection.getInputStream()) {
                return new String(in.readAllBytes(), StandardCharsets.UTF_8);
            } finally {
                connection.disconnect();
            }
        } catch (java.io.IOException notUpYet) {
            return null;
        }
    }
}
