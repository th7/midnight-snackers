package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.base.Hardware;
import org.firstinspires.ftc.teamcode.base.RelativeAutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
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

    /**
     * An auto whose single step finishes on its third loop.
     */
    public static class ThreeLoopAuto extends RelativeAutoOp {
        private int loops = 0;

        @Override
        public PlanPart getPlan() {
            return new Step("count to three", () -> {
            }, () -> ++loops >= 3);
        }

        @Override
        protected Hardware hardware() {
            return SimRunnerTest.currentSim.hardware();
        }
    }

    public static class NeverDoneAuto extends RelativeAutoOp {
        @Override
        public PlanPart getPlan() {
            return new Step("forever", () -> {
            }, () -> false);
        }

        @Override
        protected Hardware hardware() {
            return SimRunnerTest.currentSim.hardware();
        }
    }

    private static SimRobot currentSim;

    @Test
    public void recordsEveryLoopUntilThePlanIsDoneAndWritesTheReplay() {
        currentSim = sim;
        Path out = folder.getRoot().toPath();

        SimRecording recording = SimRunner.run(new ThreeLoopAuto(), sim, 5, out);

        assertEquals(3, recording.ticks().size());
        assertEquals("count to three", recording.ticks().get(0).step);
        assertEquals("done", recording.outcome());
        assertTrue(Files.exists(out.resolve("ThreeLoopAuto.html")));
    }

    @Test
    public void keepsEveryLoopsPoseButThinsDrawingsToTwentyPerSecond() {
        currentSim = sim;

        SimRecording recording = SimRunner.run(new ThreeLoopAuto(), sim, 5, folder.getRoot().toPath());

        // Three loops a few milliseconds apart: the first carries the dashboard drawing, the rest don't.
        assertEquals(3, recording.poses().size());
        assertFalse(recording.ticks().get(0).packets.isEmpty());
        assertTrue(recording.ticks().get(1).packets.isEmpty());
        assertTrue(recording.ticks().get(2).packets.isEmpty());
    }

    @Test
    public void anAnonymousSubclassIsNamedAfterItsNearestNamedClass() {
        currentSim = sim;
        Path out = folder.getRoot().toPath();

        SimRunner.run(new ThreeLoopAuto() {
        }, sim, 5, out);

        assertTrue(Files.exists(out.resolve("ThreeLoopAuto.html")));
    }

    @Test
    public void aTimedOutRunStillWritesTheReplayBeforeFailing() throws Exception {
        currentSim = sim;
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
}
