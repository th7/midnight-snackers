package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.base.PlanOp;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.firstinspires.ftc.teamcode.sim.TestTeleOps.StickTeleOp;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
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
    private static final double TELEOP_SECONDS = 30;
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

    static final String TEMP_NAME = "Temp";
    /** The line of {@link #tempPlans} that holds the loop counter, where a compile error is planted. */
    static final int TEMP_LOOPS_LINE = 8;

    /**
     * A stand-in for the team's Plans with one {@code @Auto} plan, the way a student's edit adds
     * one: compiled first on the child's classpath, it is the Plans the registrar finds.
     */
    static String tempPlans(int loops, String group) {
        return "package org.firstinspires.ftc.teamcode;\n"
                + "import org.firstinspires.ftc.teamcode.base.Auto;\n"
                + "import org.firstinspires.ftc.teamcode.base.SuperSystem;\n"
                + "import org.firstinspires.ftc.teamcode.planrunner.PlanPart;\n"
                + "import org.firstinspires.ftc.teamcode.planrunner.Step;\n"
                + "\n"
                + "public class Plans extends SuperSystem {\n"
                + "    private int loops = 0;\n"
                + "    @Auto(name = \"" + TEMP_NAME + "\", group = \"" + group + "\", alliance = Alliance.RELATIVE)\n"
                + "    public PlanPart temp() { return new Step(\"count\", () -> { }, () -> ++loops >= " + loops + "); }\n"
                + "}\n";
    }

    static String tempPlans(int loops) {
        return tempPlans(loops, "Test");
    }

    /** Writes the stand-in Plans under a temp project and returns the package root. */
    static Path sourceRootWith(Path project, String source) throws IOException {
        Path sourceRoot = project.resolve("TeamCode/src/main/java");
        Path file = sourceRoot.resolve("org/firstinspires/ftc/teamcode/Plans.java");
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
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Count to three").get(), "ada"));

        assertEquals("done", run.outcome());
        assertEquals("finished", run.phase());
        assertEquals(3, run.ticks().size());
        assertEquals("ada", run.startedBy);
        assertTrue(Files.isRegularFile(outputDir().resolve("Count to three.html")));
        assertNull(bench.current());
    }

    @Test
    public void anOpModeWhoseLoopNeverReturnsIsKilled() throws Exception {
        bench = new SimBench(SimCatalog.of(HangingAuto.class), null, outputDir(), 0.3, TELEOP_SECONDS, GRACE_SECONDS);
        long startedAt = System.nanoTime();

        SimBench.Run run = await(bench.start(bench.catalog().find("Hangs").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("1.3"));
        double seconds = (System.nanoTime() - startedAt) / 1e9;
        assertTrue("took " + seconds + "s", seconds < 10);
        assertNull(bench.current());
    }

    /** Registers an auto that never finishes, slowly, the way a big catalog loads slowly. */
    public static class SlowRegistrar {
        public static final double SECONDS = 1.5;

        @OpModeRegistrar
        public static void register(OpModeManager manager) {
            try {
                Thread.sleep((long) (SECONDS * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            manager.register(new OpModeMeta.Builder().setFlavor(OpModeMeta.Flavor.AUTONOMOUS).setName("Slow to start").build(),
                    new PlanOp(Alliance.RELATIVE, "SlowRegistrar", plans -> new Step("forever", () -> {
                    }, () -> false)));
        }
    }

    @Test
    public void theRunsTimeStartsWhenTheOpModeDoesNotWhenTheChildJvmDoes() throws Exception {
        bench = new SimBench(SimCatalog.of(SlowRegistrar.class), null, outputDir(), 0.3, TELEOP_SECONDS, GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Slow to start").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("timed out after 0.3s"));
    }

    @Test
    public void aCompileErrorIsTheRunsOutcomeAndTheCatalogsToo() throws Exception {
        Path sourceRoot = sourceRootWith(folder.getRoot().toPath(), tempPlans(2).replace("private int loops = 0;", "private int loops = ;"));
        bench = new SimBench(null, sourceRoot, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

        try {
            bench.catalog();
            fail("a catalog cannot be listed from sources that do not compile");
        } catch (SimBench.BuildFailed e) {
            assertTrue(e.getMessage(), e.getMessage().contains("Plans.java:" + TEMP_LOOPS_LINE));
        }
        SimBench.Run run = await(bench.start(new SimCatalog.Entry(TEMP_NAME, "Test", SimCatalog.AUTO, "", null), "ada"));
        assertEquals("build failed", run.outcome());
        assertTrue(run.message(), run.message().contains("Plans.java:" + TEMP_LOOPS_LINE));
        assertEquals(0, run.ticks().size());
    }

    @Test
    public void savedEditsTakeEffectOnTheNextRunWithoutARestart() throws Exception {
        Path sourceRoot = sourceRootWith(folder.getRoot().toPath(), tempPlans(2));
        bench = new SimBench(null, sourceRoot, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
        SimCatalog.Entry temp = bench.catalog().find(TEMP_NAME).get();
        assertEquals("Test", temp.group);
        assertEquals("Plans.temp()", temp.where);
        assertEquals(2, await(bench.start(temp, "ada")).ticks().size());

        sourceRootWith(folder.getRoot().toPath(), tempPlans(4, "Test v2"));

        SimBench.Run second = await(bench.start(temp, "ada"));
        assertEquals("done", second.outcome());
        assertEquals(4, second.ticks().size());
        assertEquals("Test v2", bench.catalog().find(TEMP_NAME).get().group);
    }

    @Test
    public void checkCompilesWithoutRunningAndNeverUnderARun() throws Exception {
        Path sourceRoot = sourceRootWith(folder.getRoot().toPath(), tempPlans(100000));
        bench = new SimBench(null, sourceRoot, outputDir(), 1.0, TELEOP_SECONDS, GRACE_SECONDS);
        assertTrue(bench.check().problems.isEmpty());
        assertNull("nothing ran", bench.current());
        SimBench.Run run = bench.start(new SimCatalog.Entry(TEMP_NAME, "Test", SimCatalog.AUTO, "", null), "ada");
        while (!"running".equals(run.phase()) && run.outcome() == null) {
            Thread.sleep(10);
        }

        sourceRootWith(folder.getRoot().toPath(), tempPlans(2).replace("loops = 0", "loops = "));
        SimBuild.Result during = bench.check();

        assertTrue("the last result, since a rebuild would pull the classes from under the child", during.problems.isEmpty());
        await(run);
        assertTrue(run.outcome(), run.outcome().startsWith("timed out"));
        assertEquals(1, bench.check().problems.size());
    }

    @Test
    public void aBenchWithoutSourcesHasNothingToCheck() {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

        assertNull(bench.check());
    }

    @Test
    public void aTeleOpRunDrivesFromGamepadPostsAndStopEndsIt() throws Exception {
        bench = new SimBench(SimCatalog.of(StickTeleOp.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada");
        awaitRunning(run);

        Response pushed = routes().handle(post("/runs/" + run.id + "/gamepad", "{\"gamepad\": 1, \"state\": {\"left_stick_y\": -1}}"));
        assertEquals(pushed.body, 200, pushed.status);
        awaitTicks(run, tick -> tick.get("x").getAsDouble() > 6);
        assertTrue(bench.status(), bench.status().contains("\"kind\":\"teleop\""));
        assertTrue("the ticks carry the driver's inputs", anyTick(run, tick -> tick.has("gamepads")
                && tick.getAsJsonObject("gamepads").getAsJsonObject("1").get("left_stick_y").getAsDouble() == -1));

        Response typo = routes().handle(post("/runs/" + run.id + "/gamepad", "{\"gamepad\": 1, \"state\": {\"corss\": true}}"));
        assertEquals(400, typo.status);
        assertTrue(typo.body, typo.body.contains("corss"));
        assertEquals(405, routes().handle(get("/runs/" + run.id + "/gamepad")).status);
        assertTrue("a bad post changes nothing", run.running());

        Response stopped = routes().handle(post("/runs/" + run.id + "/stop", ""));
        assertEquals(stopped.body, 200, stopped.status);
        await(run);
        assertEquals("stopped", run.outcome());
        assertNull(bench.current());
        Response late = routes().handle(post("/runs/" + run.id + "/gamepad", "{\"gamepad\": 1, \"state\": {}}"));
        assertEquals(409, late.status);
    }

    @Test
    public void aTeleOpRunEndsDoneWhenItsPeriodIsOver() throws Exception {
        bench = new SimBench(SimCatalog.of(StickTeleOp.class), null, outputDir(), TIMEOUT_SECONDS, 0.3, GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada"));

        assertEquals("done", run.outcome());
        assertTrue(run.ticks().size() > 1);
    }

    @Test
    public void stopEndsAnAutoRunToo() throws Exception {
        bench = new SimBench(SimCatalog.of(TestAutos.NeverDoneAuto.class), null, outputDir(), 30, TELEOP_SECONDS, GRACE_SECONDS);
        SimBench.Run run = bench.start(bench.catalog().find("Never done").get(), "ada");
        awaitRunning(run);
        long startedAt = System.nanoTime();

        assertEquals(200, routes().handle(post("/runs/" + run.id + "/stop", "")).status);
        await(run);

        assertEquals("stopped", run.outcome());
        assertTrue("took " + (System.nanoTime() - startedAt) / 1e9 + "s", (System.nanoTime() - startedAt) / 1e9 < 10);
        assertEquals(404, routes().handle(post("/runs/999/stop", "")).status);
    }

    private static void awaitRunning(SimBench.Run run) throws InterruptedException {
        long deadline = System.nanoTime() + 30_000_000_000L;
        while (!"running".equals(run.phase()) && run.outcome() == null && System.nanoTime() < deadline) {
            Thread.sleep(10);
        }
        assertEquals(run.outcome(), "running", run.phase());
    }

    private static void awaitTicks(SimBench.Run run, java.util.function.Predicate<com.google.gson.JsonObject> condition) throws InterruptedException {
        long deadline = System.nanoTime() + 20_000_000_000L;
        while (!anyTick(run, condition)) {
            assertTrue("the run ended: " + run.outcome(), run.running());
            assertTrue("no tick ever matched", System.nanoTime() < deadline);
            Thread.sleep(20);
        }
    }

    private static boolean anyTick(SimBench.Run run, java.util.function.Predicate<com.google.gson.JsonObject> condition) {
        for (com.google.gson.JsonElement tick : run.ticks()) {
            if (condition.test(tick.getAsJsonObject())) {
                return true;
            }
        }
        return false;
    }

    /** The bench's routes as the coding server mounts them for ada. */
    private Router routes() {
        return bench.routes("ada");
    }

    private static TinyHttpServer.Request post(String path, String body) {
        return TinyHttpServer.Request.of("POST", path, body);
    }

    private static TinyHttpServer.Request get(String path) {
        return TinyHttpServer.Request.of("GET", path, "");
    }

    @Test
    public void theLogKeepsWhatTheChildWroteToStderr() throws Exception {
        bench = new SimBench(SimCatalog.of(TestAutos.ChattyAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Chatty").get(), "ada"));

        assertTrue(run.log(), run.log().contains("hello from the op mode"));
    }
}
