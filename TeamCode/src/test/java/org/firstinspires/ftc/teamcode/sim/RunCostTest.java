package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.Map;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class RunCostTest {
    @Rule
    public final TemporaryFolder folder = new TemporaryFolder();

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "No plan", group = "Test")
    public static class NoPlanAuto extends TestAutos.TestAuto {
        @Override
        public PlanPart getPlan() {
            throw new IllegalStateException("no plan");
        }
    }

    private RunCost of(Class<?> opMode, String name) {
        Path output = folder.getRoot().toPath();
        return RunCost.of(SimCatalog.of(opMode).find(name).get(), 30, output);
    }

    @Test
    public void aRunIsTimedStageByStage() {
        RunCost cost = of(TestAutos.ThreeLoopAuto.class, "Count to three");

        assertEquals(3, cost.recording().ticks().size());
        String report = cost.report();
        for (RunCost.Stage stage : RunCost.Stage.values()) {
            assertTrue(report, report.contains(stage.named));
        }
        assertTrue(report, report.contains("faster than real time"));
    }

    @Test
    public void whatARunCarriesIsCountedAsTheBenchSendsIt() {
        RunCost cost = of(TestAutos.ThreeLoopAuto.class, "Count to three");
        String whole = SimReplayPage.update(cost.recording(), 0);
        long lines = 0;
        for (SimRecording.Tick tick : cost.recording().ticks()) {
            lines += SimRunStream.tick(tick).getBytes(StandardCharsets.UTF_8).length;
        }

        assertEquals(lines, cost.lineBytes);
        assertEquals(whole.getBytes(StandardCharsets.UTF_8).length, cost.runBytes);
        assertEquals(
                TinyHttpServer.sent(Response.json(whole), Map.of("accept-encoding", RunCost.A_BROWSER_TAKES))
                        .body
                        .length,
                cost.runBytesSent);
        assertEquals(SimReplayPage.written(cost.recording()).getBytes(StandardCharsets.UTF_8).length, cost.pageBytes);
    }

    @Test(timeout = 60_000)
    public void aLongRunIsCarriedOnePollPerPeriodOfTheLiveView() {
        RunCost cost = RunCost.of(
                SimCatalog.of(TestTeleOps.StickTeleOp.class).find("Stick").get(),
                10,
                folder.getRoot().toPath());

        assertTrue(String.valueOf(cost.polls), cost.polls >= 200 && cost.polls <= 201);
    }

    @Test
    public void aRunWithNoTicksSaysItCouldNotJudgeRatherThanDividingByNone() {
        RunCost cost = of(NoPlanAuto.class, "No plan");

        String report = cost.report();

        assertEquals(0, cost.recording().ticks().size());
        assertTrue(report, report.contains("could not judge"));
        assertFalse(report, report.contains("NaN") || report.contains("Infinity"));
    }

    @Test
    public void theLiveViewPollsAsOftenAsTheCostOfServingItAssumes() {
        String page = SimAssets.page("replay.html");

        assertTrue(page.contains("setTimeout(poll, teleop ? "
                + Math.round(RunCost.TELEOP_POLL_SECONDS * 1000)
                + " : "
                + Math.round(RunCost.AUTO_POLL_SECONDS * 1000)
                + ")"));
    }
}
