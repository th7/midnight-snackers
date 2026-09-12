package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;

public class SimBenchTest {
    private static final double TIMEOUT_SECONDS = 2;
    private static final double GRACE_SECONDS = 1;

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private SimBench bench;

    @After
    public void stopBench() {
        if (bench != null) {
            bench.stop();
        }
    }

    private Path outputDir() {
        return folder.getRoot().toPath().resolve("sim");
    }

    static final String TEMP_AUTO_CLASS = "org.firstinspires.ftc.teamcode.auto.TempAuto";

    static String tempAuto(int loops) {
        return "package org.firstinspires.ftc.teamcode.auto;\n"
                + "import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n"
                + "import org.firstinspires.ftc.teamcode.base.RelativeAutoOp;\n"
                + "import org.firstinspires.ftc.teamcode.planrunner.PlanPart;\n"
                + "import org.firstinspires.ftc.teamcode.planrunner.Step;\n"
                + "@Autonomous(name = \"Temp\", group = \"Test\")\n"
                + "public class TempAuto extends RelativeAutoOp {\n"
                + "    private int loops = 0;\n"
                + "    @Override public PlanPart getPlan() { return new Step(\"count\", () -> { }, () -> ++loops >= " + loops + "); }\n"
                + "}\n";
    }

    /** Writes the auto's source under a temp project and returns the package root. */
    static Path sourceRootWith(Path project, String source) throws IOException {
        Path sourceRoot = project.resolve("TeamCode/src/main/java");
        Path file = sourceRoot.resolve("org/firstinspires/ftc/teamcode/auto/TempAuto.java");
        Files.createDirectories(file.getParent());
        Files.write(file, source.getBytes(StandardCharsets.UTF_8));
        // a save within the same second as the previous one must still be noticed
        Files.setLastModifiedTime(file, FileTime.fromMillis(System.currentTimeMillis() + 2000));
        return sourceRoot;
    }

    private static SimBench.Run await(SimBench.Run run) throws InterruptedException {
        long deadline = System.nanoTime() + 30_000_000_000L;
        while (run.outcome() == null && System.nanoTime() < deadline) {
            Thread.sleep(20);
        }
        assertNotNull("run never finished; phase " + run.phase(), run.outcome());
        return run;
    }

    @Test
    public void aRunThroughTheChildEndsDoneWithItsTicksAndReplay() throws Exception {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find(ThreeLoopAuto.class.getName()).get(), "ada"));

        assertEquals("done", run.outcome());
        assertEquals("finished", run.phase());
        assertEquals(3, run.ticks().size());
        assertEquals("ada", run.startedBy);
        assertTrue(Files.isRegularFile(outputDir().resolve("ThreeLoopAuto.html")));
        assertNull(bench.current());
    }

    @Test
    public void anOpModeWhoseLoopNeverReturnsIsKilled() throws Exception {
        bench = new SimBench(SimCatalog.of(HangingAuto.class), null, outputDir(), 0.3, GRACE_SECONDS);
        long startedAt = System.nanoTime();

        SimBench.Run run = await(bench.start(bench.catalog().find(HangingAuto.class.getName()).get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("1.3"));
        double seconds = (System.nanoTime() - startedAt) / 1e9;
        assertTrue("took " + seconds + "s", seconds < 10);
        assertNull(bench.current());
    }

    @Test
    public void aCompileErrorIsTheRunsOutcomeAndTheCatalogsToo() throws Exception {
        Path sourceRoot = sourceRootWith(folder.getRoot().toPath(), tempAuto(2).replace("private int loops = 0;", "private int loops = ;"));
        bench = new SimBench(null, sourceRoot, outputDir(), TIMEOUT_SECONDS, GRACE_SECONDS);

        try {
            bench.catalog();
            fail("a catalog cannot be listed from sources that do not compile");
        } catch (SimBench.BuildFailed e) {
            assertTrue(e.getMessage(), e.getMessage().contains("TempAuto.java:8"));
        }
        SimBench.Run run = await(bench.start(new SimCatalog.Entry("Temp", "Test", TEMP_AUTO_CLASS, null), "ada"));
        assertEquals("build failed", run.outcome());
        assertTrue(run.message(), run.message().contains("TempAuto.java:8"));
        assertEquals(0, run.ticks().size());
    }

    @Test
    public void savedEditsTakeEffectOnTheNextRunWithoutARestart() throws Exception {
        Path sourceRoot = sourceRootWith(folder.getRoot().toPath(), tempAuto(2));
        bench = new SimBench(null, sourceRoot, outputDir(), TIMEOUT_SECONDS, GRACE_SECONDS);
        SimCatalog.Entry temp = bench.catalog().find(TEMP_AUTO_CLASS).get();
        assertEquals("Temp", temp.name);
        assertEquals(2, await(bench.start(temp, "ada")).ticks().size());

        sourceRootWith(folder.getRoot().toPath(), tempAuto(4).replace("name = \"Temp\"", "name = \"Temp v2\""));

        SimBench.Run second = await(bench.start(temp, "ada"));
        assertEquals("done", second.outcome());
        assertEquals(4, second.ticks().size());
        assertEquals("Temp v2", bench.catalog().find(TEMP_AUTO_CLASS).get().name);
    }

    @Test
    public void checkCompilesWithoutRunningAndNeverUnderARun() throws Exception {
        Path sourceRoot = sourceRootWith(folder.getRoot().toPath(), tempAuto(100000));
        bench = new SimBench(null, sourceRoot, outputDir(), 1.0, GRACE_SECONDS);
        assertTrue(bench.check().problems.isEmpty());
        assertNull("nothing ran", bench.current());
        SimBench.Run run = bench.start(new SimCatalog.Entry("Temp", "Test", TEMP_AUTO_CLASS, null), "ada");
        while (!"running".equals(run.phase()) && run.outcome() == null) {
            Thread.sleep(10);
        }

        sourceRootWith(folder.getRoot().toPath(), tempAuto(2).replace("loops = 0", "loops = "));
        SimBuild.Result during = bench.check();

        assertTrue("the last result, since a rebuild would pull the classes from under the child", during.problems.isEmpty());
        await(run);
        assertTrue(run.outcome(), run.outcome().startsWith("timed out"));
        assertEquals(1, bench.check().problems.size());
    }

    @Test
    public void aBenchWithoutSourcesHasNothingToCheck() {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, GRACE_SECONDS);

        assertNull(bench.check());
    }

    @Test
    public void theLogKeepsWhatTheChildWroteToStderr() throws Exception {
        bench = new SimBench(SimCatalog.of(TestAutos.ChattyAuto.class), null, outputDir(), TIMEOUT_SECONDS, GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find(TestAutos.ChattyAuto.class.getName()).get(), "ada"));

        assertTrue(run.log(), run.log().contains("hello from the op mode"));
    }
}
