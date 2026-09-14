package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.firstinspires.ftc.teamcode.sim.SimDriverStation.State;
import org.firstinspires.ftc.teamcode.sim.TestAutos.GatedAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.NeverDoneAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.WaitingAuto;
import org.firstinspires.ftc.teamcode.sim.TestTeleOps.StickTeleOp;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

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
        assertNull("an auto's ticks carry no driver inputs", recording.ticks().get(0).gamepad1);
        assertEquals("auto", recording.kind());
        assertTrue(Files.exists(out.resolve("ThreeLoopAuto.html")));
    }

    // --- time: a run is a fixed number of loops, the same every time, paced to real time only when watched ---

    @Test
    public void theWorldMovesOneLoopPeriodPerLoopOnTheSimulatedClock() {
        SimRecording recording =
                SimRunner.run(new WaitingAuto(), sim, 10, folder.getRoot().toPath());

        java.util.List<SimRecording.Tick> ticks = recording.ticks();
        for (int i = 0; i < ticks.size(); i++) {
            assertEquals("tick " + i, i * SimRunner.LOOP_SECONDS, ticks.get(i).seconds, 1e-9);
        }
        assertEquals(
                "the world moved once per loop", ticks.size() * SimRunner.LOOP_SECONDS, sim.nanoTime() / 1e9, 1e-9);
        // The loops at 0, 0.02, ... 2.00 s are still waiting; the one at 2.02 s finds the wait over.
        assertEquals((int) Math.ceil(WaitingAuto.SECONDS / SimRunner.LOOP_SECONDS) + 2, ticks.size());
    }

    @Test
    public void everyRunOfAnAutoIsTheSameTickForTick() {
        SimRecording first = SimRunner.run(
                new WaitingAuto(), new SimRobot(), 10, folder.getRoot().toPath());
        SimRecording second = SimRunner.run(
                new WaitingAuto(), new SimRobot(), 10, folder.getRoot().toPath());

        assertEquals(first.ticks().size(), second.ticks().size());
        for (int i = 0; i < first.ticks().size(); i++) {
            assertEquals(first.ticks().get(i).seconds, second.ticks().get(i).seconds, 0);
            assertEquals(first.ticks().get(i).truePose, second.ticks().get(i).truePose);
        }
    }

    @Test
    public void anAutoRunsAsFastAsItCanUnlessItIsWatched() {
        long before = System.nanoTime();

        SimRunner.run(new WaitingAuto(), sim, 10, folder.getRoot().toPath());

        double wallSeconds = (System.nanoTime() - before) / 1e9;
        assertTrue(
                "two simulated seconds took " + wallSeconds + "s of wall clock", wallSeconds < WaitingAuto.SECONDS / 2);
    }

    @Test
    public void aRunFromADriverStationKeepsToRealTimeSoTheDriverCanDriveIt() {
        long before = System.nanoTime();

        SimRunner.record(
                new SimRecording("StickTeleOp", "teleop"),
                new StickTeleOp(),
                sim,
                0.3,
                folder.getRoot().toPath(),
                new SimDriverStation());

        double wallSeconds = (System.nanoTime() - before) / 1e9;
        assertTrue("0.3 simulated seconds took " + wallSeconds + "s of wall clock", wallSeconds >= 0.25);
    }

    @Test
    public void anAutosTimeoutIsOnTheSimulatedClock() {
        AssertionError error = assertThrows(
                AssertionError.class,
                () -> SimRunner.run(new WaitingAuto(), sim, 1, folder.getRoot().toPath()));

        assertTrue(error.getMessage(), error.getMessage().contains("1.0s"));
        assertEquals("the world stopped when the timeout fell", 1, sim.nanoTime() / 1e9, SimRunner.LOOP_SECONDS + 1e-9);
    }

    @Test
    public void aTeleOpRunsOnTheDriverStationsInputsUntilTheDriverPressesStop() throws Exception {
        SimDriverStation station = new SimDriverStation();
        StickTeleOp teleOp = new StickTeleOp();
        SimRecording recording = new SimRecording("StickTeleOp", "teleop");
        Path out = folder.getRoot().toPath();
        Thread runner = new Thread(() -> SimRunner.record(recording, teleOp, sim, 30, out, station));
        runner.start();

        station.set(1, state("{\"left_stick_y\": -1, \"cross\": true}"));
        await("drove forward", () -> sim.pose().position.x > 6);
        await(
                "recorded the press",
                () -> lastGamepad1(recording) != null
                        && lastGamepad1(recording).pressed.contains("cross"));
        station.set(1, State.NEUTRAL);
        await(
                "recorded the release",
                () -> lastGamepad1(recording) != null && lastGamepad1(recording).neutral());
        station.set(1, state("{\"cross\": true}"));
        await("recorded the second press", () -> lastGamepad1(recording).pressed.contains("cross"));

        station.stop();
        runner.join(10_000);

        assertFalse(runner.isAlive());
        assertEquals("stopped", recording.outcome());
        assertEquals("holding a button is one press", 2, teleOp.presses);
        assertEquals("a TeleOp has no plan step", "", recording.ticks().get(0).step);
        assertTrue(recording.ticks().stream().anyMatch(t -> t.gamepad1 != null && t.gamepad1.leftStickY == -1));
        assertTrue(Files.exists(out.resolve("StickTeleOp.html")));
    }

    @Test
    public void aTeleOpEndsDoneWhenItsTimeIsUp() {
        SimRecording recording = SimRunner.record(
                new SimRecording("StickTeleOp", "teleop"),
                new StickTeleOp(),
                sim,
                0.2,
                folder.getRoot().toPath(),
                new SimDriverStation());

        assertEquals("done", recording.outcome());
        assertTrue(recording.ticks().size() > 1);
    }

    @Test
    public void stopEndsAnAutoBeforeItsPlanIsDone() {
        SimDriverStation station = new SimDriverStation();
        station.stop();

        SimRecording recording = SimRunner.record(
                new SimRecording("NeverDoneAuto"),
                new NeverDoneAuto(),
                sim,
                5,
                folder.getRoot().toPath(),
                station);

        assertEquals("stopped", recording.outcome());
    }

    private static SimDriverStation.State lastGamepad1(SimRecording recording) {
        java.util.List<SimRecording.Tick> ticks = recording.ticks();
        return ticks.isEmpty() ? null : ticks.get(ticks.size() - 1).gamepad1;
    }

    private static State state(String json) {
        return State.fromJson(new com.google.gson.Gson().fromJson(json, com.google.gson.JsonObject.class));
    }

    private static void await(String what, java.util.function.BooleanSupplier condition) throws InterruptedException {
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (!condition.getAsBoolean()) {
            if (System.nanoTime() > deadline) {
                throw new AssertionError("never " + what);
            }
            Thread.sleep(20);
        }
    }

    @Test
    public void keepsEveryLoopsPoseButThinsDrawingsToTwentyPerSecond() {

        SimRecording recording =
                SimRunner.run(new ThreeLoopAuto(), sim, 5, folder.getRoot().toPath());

        // Three loops a few milliseconds apart: the first carries the dashboard drawing, the rest don't.
        assertEquals(3, recording.poses().size());
        assertFalse(recording.ticks().get(0).packets.isEmpty());
        assertTrue(recording.ticks().get(1).packets.isEmpty());
        assertTrue(recording.ticks().get(2).packets.isEmpty());
    }

    @Test
    public void aRunFromACatalogEntryIsNamedAsTheDriverStationNamesIt() {
        Path out = folder.getRoot().toPath();

        SimRecording recording = SimRunner.run(
                SimCatalog.of(ThreeLoopAuto.class).find("Count to three").get(), sim, 5, out);

        assertEquals("Count to three", recording.name());
        assertEquals("auto", recording.kind());
        assertEquals(3, recording.ticks().size());
        assertTrue(Files.exists(out.resolve("Count to three.html")));
    }

    @Test
    public void aNameNoFileCanHaveStillGetsAReplayFile() {
        Path out = folder.getRoot().toPath();

        SimRunner.record(new SimRecording("odd/name"), new ThreeLoopAuto(), sim, 5, out, new SimDriverStation());

        assertTrue(Files.exists(out.resolve("odd_name.html")));
    }

    @Test
    public void anAnonymousSubclassIsNamedAfterItsNearestNamedClass() {
        Path out = folder.getRoot().toPath();

        SimRunner.run(new ThreeLoopAuto() {}, sim, 5, out);

        assertTrue(Files.exists(out.resolve("ThreeLoopAuto.html")));
    }

    @Test
    public void withALivePortTheRunCanBeWatchedWhileItRunsAndUntilTheViewerHasSeenTheEnd() throws Exception {
        int port = freePort();
        // The auto runs until this test releases it, so the run is in progress for as long as the
        // first fetch takes, however loaded the machine is. The timeout is only a safety net.
        GatedAuto auto = new GatedAuto();
        Thread runner =
                new Thread(() -> SimRunner.run(auto, sim, 60, folder.getRoot().toPath(), port));
        runner.start();

        String duringRun = null;
        String afterRun = null;
        long deadline = System.nanoTime() + 30_000_000_000L;
        while (System.nanoTime() < deadline) {
            String response = tryGet("http://localhost:" + port + "/ticks?from=0");
            if (response == null) {
                Thread.sleep(20);
                continue;
            }
            if (response.contains("\"outcome\":\"done\"")) {
                afterRun = response;
                break;
            }
            if (duringRun == null && response.contains("\"" + GatedAuto.STEP + "\"")) {
                duringRun = response;
                auto.release();
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

        AssertionError error =
                assertThrows(AssertionError.class, () -> SimRunner.run(new NeverDoneAuto(), sim, 0.1, out));

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
