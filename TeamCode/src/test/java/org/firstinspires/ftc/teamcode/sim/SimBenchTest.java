package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.opmode.PlanOp;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.firstinspires.ftc.teamcode.sim.TestTeleOps.StickTeleOp;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimBenchTest {
    private static final double TIMEOUT_SECONDS = 2;
    private static final double TELEOP_SECONDS = 30;
    private static final double GRACE_SECONDS = 1;

    /**
     * What a bench waits in a test: a run's budget and a match's period cut to something a test can
     * sit through. The silence stays what the bench waits in earnest, because a child that is
     * talking never reaches it and a child that is not is only ever one test's business -- cut
     * here, it would be a busy machine's chance to have a healthy run killed for pausing.
     */
    private static final SimBench.Waits WAITS = SimBench.Waits.ofTheBench()
            .runTimeout(TIMEOUT_SECONDS)
            .teleOpPeriod(TELEOP_SECONDS)
            .killGrace(GRACE_SECONDS);

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

    static void copyTree(Path from, Path to) {
        SimProject.copyTree(from, to);
    }

    static Path simulatorInto(Path project) {
        return SimProject.simulatorOnlyIn(project).root();
    }

    static Path projectWith(Path project, String plans) {
        return SimProject.copiedTo(project).withPlans(plans).root();
    }

    static Path realProjectCopiedUnder(Path project) {
        return SimProject.copiedTo(project).root();
    }

    static void edit(Path project, String relative, String from, String to) {
        SimProject.at(project).edited(relative, from, to);
    }

    static final String HARDWARE = SimProject.HARDWARE;
    static final String SIM_ROBOT = SimProject.SIM_ROBOT;
    static final String SIM_DEVICES = SimProject.SIM_DEVICES;
    static final String SIM_CHILD = SimProject.SIM_CHILD;
    static final String SIM_RUN_STREAM = SimProject.SIM_RUN_STREAM;
    static final SimCatalog.Entry BLUE_TELEOP =
            new SimCatalog.Entry("BlueTeleOp", "TeleOp", SimCatalog.TELEOP, "", null);

    static final String TEMP_NAME = SimProject.TEMP_NAME;

    static final int TEMP_LOOPS_LINE = SimProject.TEMP_LOOPS_LINE;

    static String tempPlans(int loops, String group) {
        return SimProject.tempPlans(loops, group);
    }

    static String tempPlans(int loops) {
        return SimProject.tempPlans(loops);
    }

    static Path sourceRootWith(Path project, String source) {
        return SimProject.at(project).withPlans(source).sourceRoot();
    }

    private static SimBench.Run await(SimBench.Run run) throws InterruptedException {
        long deadline = System.nanoTime() + 30_000_000_000L;
        while (run.outcome() == null && System.nanoTime() < deadline) {
            Thread.sleep(20);
        }
        assertNotNull("run never finished; phase " + run.phase(), run.outcome());
        return run;
    }

    private static JsonObject item(int id, String outcome) {
        JsonObject item = new JsonObject();
        item.addProperty("id", id);
        item.addProperty("outcome", outcome);
        return item;
    }

    @Test
    public void theStatusIsRunningExactlyWhenTheNewestRunHasNoOutcomeYet() {
        JsonObject going = item(2, null);
        JsonObject done = item(1, "done");

        assertEquals("{\"running\":false,\"runs\":[]}", SimBench.statusOf(List.of()));
        assertEquals(
                "{\"running\":true,\"runs\":[{\"id\":2,\"outcome\":null},{\"id\":1,\"outcome\":\"done\"}]}",
                SimBench.statusOf(List.of(going, done)));
        assertEquals(
                "an older run that never reached an outcome is not what the bench is doing now",
                "{\"running\":false,\"runs\":[{\"id\":1,\"outcome\":\"done\"},{\"id\":2,\"outcome\":null}]}",
                SimBench.statusOf(List.of(done, going)));
    }

    @Test
    public void aRunsLineWaitsForWhoeverIsChangingTheRun() throws Exception {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);
        SimBench.Run run =
                bench.new Run(1, bench.catalog().find("Count to three").get(), "ada", null, null);
        CountDownLatch asking = new CountDownLatch(1);
        CountDownLatch answered = new CountDownLatch(1);
        AtomicReference<JsonObject> line = new AtomicReference<>();
        Thread reader = new Thread(() -> {
            asking.countDown();
            line.set(run.json());
            answered.countDown();
        });

        synchronized (run) {
            reader.start();
            assertTrue("the reader never got as far as asking", asking.await(10, TimeUnit.SECONDS));
            assertFalse(
                    "a line was taken while the run was being changed: " + line.get(),
                    answered.await(200, TimeUnit.MILLISECONDS));
            run.finish("done", "as it happens");
        }

        assertTrue("the reader never got its line", answered.await(10, TimeUnit.SECONDS));
        JsonObject one = line.get();
        String half = "the line carries half of the finish: " + one;
        assertEquals(half, "finished", one.get("phase").getAsString());
        assertEquals(half, "done", one.get("outcome").getAsString());
        assertEquals(half, "as it happens", one.get("message").getAsString());
    }

    @Test
    public void aRunThroughTheChildEndsDoneWithItsTicksAndReplay() throws Exception {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada"));

        assertEquals("done", run.outcome());
        assertEquals("finished", run.phase());
        assertEquals(3, run.ticks().size());
        assertEquals("ada", run.startedBy);
        assertTrue(Files.isRegularFile(outputDir().resolve("Count to three.html")));
        assertNull(bench.current());
    }

    @Test
    public void anOpModeWhoseLoopNeverReturnsIsKilled() throws Exception {
        // The one test that waits out a silence, so the one that shortens it: the op mode never
        // comes back, so there is nothing here for a short wait to kill early.
        bench = new SimBench(
                SimCatalog.of(HangingAuto.class),
                null,
                outputDir(),
                WAITS.runTimeout(0.3).silence(0.5));
        long startedAt = System.nanoTime();

        SimBench.Run run = await(bench.start(bench.catalog().find("Hangs").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("the op mode did not return"));
        double seconds = (System.nanoTime() - startedAt) / 1e9;
        assertTrue("took " + seconds + "s", seconds < 30);
        assertNull(bench.current());
    }

    /** An op mode that takes longer to register than the run it registers is given to finish. */
    public static class SlowRegistrar {
        public static final double SECONDS = 0.6;

        @OpModeRegistrar
        public static void register(OpModeManager manager) {
            try {
                Thread.sleep((long) (SECONDS * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            manager.register(
                    new OpModeMeta.Builder()
                            .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                            .setName("Slow to start")
                            .build(),
                    new PlanOp(
                            Alliance.RELATIVE, "SlowRegistrar", plans -> new Step("forever", () -> {}, () -> false)));
        }
    }

    @Test
    public void theRunsTimeStartsWhenTheOpModeDoesNotWhenTheChildJvmDoes() throws Exception {
        double budget = 0.3;
        assertTrue(
                "the registrar must take longer than the run is given, or a run whose clock started at"
                        + " the child JVM would time out for the right answer by accident",
                SlowRegistrar.SECONDS > budget);
        bench = new SimBench(SimCatalog.of(SlowRegistrar.class), null, outputDir(), WAITS.runTimeout(budget));

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Slow to start").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("timed out after " + budget + "s"));
    }

    @Test
    public void aBenchOverAProjectNeedsTheProjectsSimulator() throws Exception {
        Path project = folder.getRoot().toPath();
        sourceRootWith(project, tempPlans(2));
        try {
            bench = new SimBench(null, project, outputDir(), WAITS);
            fail("without a simulator of its own the child would run this server's, built for other robot sources");
        } catch (IllegalArgumentException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("simulator"));
            assertTrue(e.getMessage(), e.getMessage().contains("SimChild.java"));
        }
    }

    @Test
    public void theChildRunsTheProjectsOwnSimulatorNotThisServers() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());

        edit(project, HARDWARE, "public static Builder builder()", "public static Builder wiring()");
        edit(project, HARDWARE, "return builder()", "return wiring()");
        edit(project, SIM_DEVICES, "Hardware.builder()", "Hardware.wiring()");
        edit(
                project,
                SIM_CHILD,
                "System.setOut(System.err);",
                "System.setOut(System.err);\n        System.err.println(\"this project's simulator\");");
        bench = new SimBench(null, project, outputDir(), WAITS.teleOpPeriod(0.3));

        assertTrue(bench.catalog().find("BlueTeleOp").isPresent());
        SimBench.Run run = await(bench.start(BLUE_TELEOP, "ada"));

        assertEquals(run.message() + "\n" + run.log(), "done", run.outcome());
        assertTrue(run.log(), run.log().contains("this project's simulator"));
    }

    @Test
    public void aClassTheProjectLacksIsMissingNotThisServers() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        Files.delete(project.resolve("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/BlueTeleOp.java"));
        bench = new SimBench(null, project, outputDir(), WAITS);

        SimCatalog catalog = bench.catalog();

        assertTrue(catalog.find("RedTeleOp").isPresent());
        assertFalse(
                "this server has a BlueTeleOp; the project does not",
                catalog.find("BlueTeleOp").isPresent());
    }

    @Test
    public void eachOpModeHasASeedTheCatalogShowsAndARouteSets() throws Exception {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);
        String opMode = "opmode=" + encode("Count to three");

        assertEquals("{\"seed\":1}", routes().handle(get("/seed?" + opMode)).body);
        assertTrue(
                routes().handle(get("/catalog")).body,
                routes().handle(get("/catalog")).body.contains("\"seed\":1"));

        Response set = routes().handle(put("/seed?" + opMode, "{\"seed\": 5}"));
        assertEquals(set.body, 200, set.status);
        assertEquals("{\"seed\":5}", set.body);
        assertEquals("{\"seed\":5}", routes().handle(get("/seed?" + opMode)).body);
        assertTrue(routes().handle(get("/catalog")).body.contains("\"seed\":5"));

        Response exact = routes().handle(put("/seed?" + opMode, "{\"seed\": null}"));
        assertEquals(exact.body, 200, exact.status);
        assertEquals("{\"seed\":null}", exact.body);
        assertTrue(routes().handle(get("/catalog")).body.contains("\"seed\":null"));

        assertEquals(400, routes().handle(put("/seed?" + opMode, "{\"seed\": \"lucky\"}")).status);
        assertEquals(400, routes().handle(put("/seed?" + opMode, "{}")).status);
        assertEquals(400, routes().handle(put("/seed?" + opMode, "not json")).status);
        assertEquals(400, routes().handle(put("/seed", "{\"seed\": 1}")).status);
    }

    @Test
    public void aRunIsMadeOnTheOpModesSeedAndSaysSo() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.PROTOCOL);
        bench = benchOverAChild(child);
        SimCatalog.Entry entry = bench.catalog().find("Count to three").get();
        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Count to three"), "{\"seed\": 5}")).status);

        SimBench.Run seeded = await(bench.start(entry, "ada"));
        assertEquals(Long.valueOf(5), seeded.seed);
        assertEquals("done", seeded.outcome());
        assertEquals(
                "the seed is on the start line",
                5,
                theStartLineIn(child).get("seed").getAsLong());
        assertTrue(bench.status(), bench.status().contains("\"seed\":5"));

        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Count to three"), "{\"seed\": null}")).status);
        SimBench.Run exact = await(bench.start(entry, "ada"));
        assertNull(exact.seed);
        assertFalse(
                "the exact robot is a start line with no seed at all",
                theStartLineIn(child).has("seed"));
        assertTrue(bench.status(), bench.status().contains("\"seed\":null"));
    }

    @Test
    public void savedEditsTakeEffectOnTheNextRunWithoutARestart() throws Exception {
        Path project = projectWith(folder.getRoot().toPath(), tempPlans(2));
        bench = new SimBench(null, project, outputDir(), WAITS);
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
        Path project = projectWith(folder.getRoot().toPath(), tempPlans(100000));
        bench = new SimBench(null, project, outputDir(), WAITS.runTimeout(1.0));
        assertTrue(bench.check().problems.isEmpty());
        assertNull("nothing ran", bench.current());
        SimBench.Run run = bench.start(new SimCatalog.Entry(TEMP_NAME, "Test", SimCatalog.AUTO, "", null), "ada");
        while (!"running".equals(run.phase()) && run.outcome() == null) {
            Thread.sleep(10);
        }

        sourceRootWith(folder.getRoot().toPath(), tempPlans(2).replace("loops = 0", "loops = "));
        SimBuild.Result during = bench.check();

        assertTrue(
                "the last result, since a rebuild would pull the classes from under the child",
                during.problems.isEmpty());
        await(run);
        assertTrue(run.outcome(), run.outcome().startsWith("timed out"));
        assertEquals(1, bench.check().problems.size());
    }

    @Test
    public void theLiveViewIsOnlyEverServedWhereItsOwnRelativeFetchesResolve() throws Exception {
        bench = benchOverAChild(aChildSpeaking(SimRunStream.PROTOCOL));
        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada"));

        Response slashless = routes().handle(get("/runs/" + run.id));
        Response mounted = routes().handle(get("/runs/" + run.id + "/"));
        Response ticks = routes().handle(get("/runs/" + run.id + "/ticks?from=0"));

        assertEquals(308, slashless.status);
        assertEquals("/runs/" + run.id + "/", slashless.headers.get("Location"));
        assertEquals(200, mounted.status);
        assertEquals(200, ticks.status);

        String base = SimLiveServerTest.assetsBaseIn(mounted.body);
        for (String asset : List.of(
                "field.glb", "fieldscene.js", "vendor/three.module.min.js", "vendor/jsm/loaders/GLTFLoader.js")) {
            String where =
                    URI.create("/runs/" + run.id + "/").resolve(base + asset).getPath();
            assertEquals(where + ", which the live view will ask for", 200, routes().handle(get(where)).status);
        }
    }

    private SimBench benchWith(FakeChild child, SimBench.Waits waits) {
        return new SimBench(SimCatalog.of(TestTeleOps.StickTeleOp.class), null, outputDir(), waits, child);
    }

    @Test
    public void aChildThatGoesSilentWhileItsRunIsUnfinishedIsKilledForNotReturning() throws Exception {
        FakeChild child =
                FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started()).thatStaysAliveSayingNothingMore();
        bench = new SimBench(
                SimSources.ofThisClasspath(SimCatalog.of(TestTeleOps.StickTeleOp.class)),
                outputDir(),
                WAITS,
                child,
                new FakeClock());

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("the op mode did not return"));
    }

    @Test
    public void aChildThatCannotBeStartedEndsTheRunSayingSo() throws Exception {
        bench = benchWith(new FakeChild().thatWillNotStart("no java on this machine"), WAITS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada"));

        assertEquals(SimRunStream.Outcome.couldNotStartChild(), run.outcome());
        assertTrue(run.message(), run.message().contains("no java on this machine"));
    }

    @Test
    public void aChildThatNeverSaysTheOpModeStartedIsKilledForNeverStarting() throws Exception {
        bench = benchWith(
                FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore(), WAITS.startup(0.3));

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("never started"));
    }

    @Test
    public void aChildThatIgnoresStopIsKilledAfterItsGrace() throws Exception {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started())
                .thatStaysAliveSayingNothingMore()
                .thatIgnoresStop();
        bench = benchWith(child, WAITS.killGrace(0.2));
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada");
        awaitRunning(run);

        run.stop();
        await(run);

        assertEquals(SimRunStream.Outcome.killedAfterStop(0.2), run.outcome());
        assertTrue(
                child.whatItWasTold().toString(), child.whatItWasTold().stream().anyMatch(t -> t.contains("stop")));
    }

    /**
     * A bench over sources that need neither a project on disk nor a compile, and a child that need
     * not be a JVM. What a real build does with real sources is SimBuildTest's and what a real child
     * prints is SimChildTest's; what this holds is what the bench does with either.
     */
    private SimBench benchOver(SimSources sources, Child child) {
        return new SimBench(sources, outputDir(), WAITS, child, new SystemClock());
    }

    private SimBench benchOverAChild(FakeChild child) {
        return benchOver(FakeSources.listing(SimCatalog.of(ThreeLoopAuto.class)), child);
    }

    /** A child of that protocol, printing its hello and then a whole run: started, a tick, done. */
    private static FakeChild aChildSpeaking(int protocol) {
        return FakeChild.thatSays(
                SimRunStream.helloOf(protocol),
                SimRunStream.started(),
                aTickLine(),
                SimRunStream.finished(SimRunStream.Outcome.done()));
    }

    /** A tick as a child writes one, through the same writer a child writes it with. */
    private static String aTickLine() {
        return SimRunStream.tick(SimRecording.Tick.at(0.02, StartPoses.ORIGIN, "", new double[] {0, 0, 0, 0}, List.of())
                .tick());
    }

    private static boolean aStartLineIsIn(FakeChild child) {
        return child.whatItWasTold().stream().anyMatch(told -> told.contains("\"start\""));
    }

    /** The newest line placing the child, which is how the bench starts every run. */
    private static com.google.gson.JsonObject theStartLineIn(FakeChild child) {
        com.google.gson.JsonObject newest = null;
        for (String told : child.whatItWasTold()) {
            com.google.gson.JsonObject line = json(told);
            if (line.has("start")) {
                newest = line;
            }
        }
        if (newest == null) {
            throw new AssertionError("the child was never placed; it was told " + child.whatItWasTold());
        }
        return newest;
    }

    @Test
    public void aBuildThatFailsIsTheRunsOutcomeAndTheCatalogsToo() throws Exception {
        for (String diagnostics : List.of(
                "Plans.java:" + TEMP_LOOPS_LINE + ": illegal start of expression",
                SimBuild.SIMULATOR_DOES_NOT_FIT + "\nsimulator sim/SimDevices.java:41: builder() is not public")) {
            FakeChild child = new FakeChild();
            bench = benchOver(
                    FakeSources.listing(SimCatalog.of(ThreeLoopAuto.class)).thatWillNotBuild(diagnostics), child);

            try {
                bench.catalog();
                fail("a catalog cannot be listed from sources that do not build");
            } catch (SimBench.BuildFailed e) {
                assertEquals(diagnostics, e.getMessage());
            }
            SimBench.Run run =
                    await(bench.start(new SimCatalog.Entry(TEMP_NAME, "Test", SimCatalog.AUTO, "", null), "ada"));

            assertEquals("build failed", run.outcome());
            assertEquals(diagnostics, run.message());
            assertEquals(0, run.ticks().size());
            assertEquals(
                    "no child is started for sources that will not build", List.of(), child.whatItWasStartedWith());
            bench.stop();
        }
    }

    @Test
    public void aChildThatCannotListTheOpModesSaysWhatItPrinted() {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello())
                .thatPrints("no catalog today")
                .thatExitsWith(1);
        bench = benchOver(FakeSources.askingTheChild(), child);

        try {
            bench.catalog();
            fail("a catalog cannot be listed by a child that dies first");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("did not list the op modes"));
            assertTrue(e.getMessage(), e.getMessage().contains("no catalog today"));
            assertTrue(e.getMessage(), e.getMessage().contains("exit"));
        }
    }

    @Test
    public void aChildThatPrintsNoHelloIsAVersionOneChildAndStillRuns() throws Exception {
        FakeChild child = FakeChild.thatSays(aTickLine(), SimRunStream.finished(SimRunStream.Outcome.done()));
        bench = benchOverAChild(child);
        exactRobot("Count to three");

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada"));

        assertEquals(run.message(), "done", run.outcome());
        assertEquals("its first line was content, not a hello", 1, run.ticks().size());
        assertFalse("a version-one child places itself, so it is told nothing", aStartLineIsIn(child));
    }

    @Test
    public void aChildFromBeforePlacementRunsFromTheOriginAndIsRefusedAnywhereElse() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.PLACED_PROTOCOL - 1);
        bench = benchOverAChild(child);
        exactRobot("Count to three");
        SimCatalog.Entry entry = bench.catalog().find("Count to three").get();

        SimBench.Run atTheOrigin = await(bench.start(entry, "ada"));
        assertEquals(atTheOrigin.message(), "done", atTheOrigin.outcome());
        assertTrue(atTheOrigin.ticks().size() > 0);
        assertFalse("it places itself, so it is told nothing", aStartLineIsIn(child));

        assertEquals(
                200,
                routes().handle(put(
                                "/start?opmode=" + encode("Count to three"), "{\"x\": 24, \"y\": 0, \"heading\": 0}"))
                        .status);
        SimBench.Run elsewhere = await(bench.start(entry, "ada"));

        assertEquals(SimRunStream.Outcome.cannotPlace(SimRunStream.PLACED_PROTOCOL - 1), elsewhere.outcome());
        assertTrue(elsewhere.message(), elsewhere.message().toLowerCase().contains("pull"));
        assertTrue(elsewhere.message(), elsewhere.message().contains("protocol " + (SimRunStream.PLACED_PROTOCOL - 1)));
        assertEquals(0, elsewhere.ticks().size());
        assertNull(bench.current());
    }

    @Test
    public void aChildFromBeforeTheSeedRunsTheExactRobotAndIsRefusedASeed() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.SEEDED_PROTOCOL - 1);
        bench = benchOverAChild(child);
        SimCatalog.Entry entry = bench.catalog().find("Count to three").get();
        exactRobot("Count to three");

        SimBench.Run exact = await(bench.start(entry, "ada"));
        assertEquals(exact.message(), "done", exact.outcome());
        assertTrue(exact.ticks().size() > 0);

        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Count to three"), "{\"seed\": 3}")).status);
        SimBench.Run seeded = await(bench.start(entry, "ada"));

        assertEquals(SimRunStream.Outcome.cannotSeed(SimRunStream.SEEDED_PROTOCOL - 1), seeded.outcome());
        assertTrue(seeded.message(), seeded.message().toLowerCase().contains("pull"));
        assertTrue(seeded.message(), seeded.message().contains("protocol " + (SimRunStream.SEEDED_PROTOCOL - 1)));
        assertEquals(0, seeded.ticks().size());
        assertNull(bench.current());
    }

    @Test
    public void aChildOfANewerProtocolIsRefusedByNameNotMisread() throws Exception {
        int newer = SimRunStream.PROTOCOL + 1;
        bench = benchOver(FakeSources.askingTheChild(), aChildSpeaking(newer));

        try {
            bench.catalog();
            fail("a catalog printed in a protocol this server cannot read must not be listed");
        } catch (SimRunStream.WrongProtocol e) {
            assertEquals(newer, e.childProtocol);
        }
        SimBench.Run run = await(bench.start(BLUE_TELEOP, "ada"));

        assertEquals(run.message(), SimRunStream.Outcome.wrongProtocol(newer), run.outcome());
        assertTrue(run.message(), run.message().contains("protocol " + newer));
        assertTrue(run.message(), run.message().contains("server"));
        assertEquals(0, run.ticks().size());
    }

    @Test
    public void aRunIsGivenTheBudgetOfSimulatedTimeItsKindHas() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.PROTOCOL);
        bench = benchOver(FakeSources.listing(SimCatalog.of(ThreeLoopAuto.class, StickTeleOp.class)), child);

        await(bench.start(bench.catalog().find("Count to three").get(), "ada"));
        assertEquals(
                "an auto is given the run timeout",
                List.of("--run", "Count to three", String.valueOf(TIMEOUT_SECONDS)),
                child.theLastStart().subList(0, 3));

        await(bench.start(bench.catalog().find("Stick").get(), "ada"));
        assertEquals(
                "a TeleOp is given a match's driver-controlled period",
                List.of("--run", "Stick", String.valueOf(TELEOP_SECONDS)),
                child.theLastStart().subList(0, 3));
    }

    @Test
    public void aBenchWithoutSourcesHasNothingToCheck() {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);

        assertNull(bench.check());
    }

    @Test
    public void aTeleOpRunDrivesFromGamepadPostsAndStopEndsIt() throws Exception {
        bench = new SimBench(SimCatalog.of(StickTeleOp.class), null, outputDir(), WAITS);
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada");
        awaitRunning(run);

        Response pushed = routes().handle(
                        post("/runs/" + run.id + "/gamepad", "{\"gamepad\": 1, \"state\": {\"left_stick_y\": -1}}"));
        assertEquals(pushed.body, 200, pushed.status);
        awaitTicks(run, tick -> tick.get("x").getAsDouble() > 6);
        assertTrue(bench.status(), bench.status().contains("\"kind\":\"teleop\""));
        assertTrue(
                "the ticks carry the driver's inputs",
                anyTick(
                        run,
                        tick -> tick.has("gamepads")
                                && tick.getAsJsonObject("gamepads")
                                                .getAsJsonObject("1")
                                                .get("left_stick_y")
                                                .getAsDouble()
                                        == -1));

        Response typo =
                routes().handle(post("/runs/" + run.id + "/gamepad", "{\"gamepad\": 1, \"state\": {\"corss\": true}}"));
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
    public void aRunIsNotKilledForTakingLongerInRealTimeThanItsPeriodLastsInSimulatedTime() throws Exception {
        bench = new SimBench(
                SimCatalog.of(TestTeleOps.SlowMachineTeleOp.class), null, outputDir(), WAITS.teleOpPeriod(0.3));

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Slow machine").get(), "ada"));

        assertEquals(run.message() + "\n" + run.log(), "done", run.outcome());
        assertTrue(run.ticks().size() > 1);
    }

    @Test
    public void stopEndsAnAutoRunToo() throws Exception {
        bench = new SimBench(SimCatalog.of(TestAutos.NeverDoneAuto.class), null, outputDir(), WAITS.runTimeout(30));
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

    private static void awaitTicks(SimBench.Run run, java.util.function.Predicate<com.google.gson.JsonObject> condition)
            throws InterruptedException {
        long deadline = System.nanoTime() + 20_000_000_000L;
        while (!anyTick(run, condition)) {
            assertTrue("the run ended: " + run.outcome(), run.running());
            assertTrue("no tick ever matched", System.nanoTime() < deadline);
            Thread.sleep(20);
        }
    }

    private static boolean anyTick(
            SimBench.Run run, java.util.function.Predicate<com.google.gson.JsonObject> condition) {
        for (com.google.gson.JsonElement tick : run.ticks()) {
            if (condition.test(tick.getAsJsonObject())) {
                return true;
            }
        }
        return false;
    }

    @Test
    public void anOpModeNobodyHasAndAMethodTheRouteDoesNotTakeAreRefused() {
        bench = new SimBench(SimCatalog.of(HangingAuto.class), null, outputDir(), WAITS);

        assertEquals(404, routes().handle(post("/run?opmode=org.example.Nope", "")).status);
        assertEquals(405, routes().handle(get("/run?opmode=" + TEMP_NAME)).status);
        assertEquals(404, routes().handle(get("/runs/999/ticks?from=0")).status);
    }

    private Router routes() {
        return bench.routes("ada");
    }

    private void exactRobot(String opMode) throws Exception {
        assertEquals(200, routes().handle(put("/seed?opmode=" + encode(opMode), "{\"seed\": null}")).status);
    }

    private static TinyHttpServer.Request post(String path, String body) {
        return TinyHttpServer.Request.of("POST", path, body);
    }

    private static TinyHttpServer.Request get(String path) {
        return TinyHttpServer.Request.of("GET", path, "");
    }

    private static TinyHttpServer.Request put(String path, String body) {
        return TinyHttpServer.Request.of("PUT", path, body);
    }

    private static String encode(String name) throws java.io.UnsupportedEncodingException {
        return java.net.URLEncoder.encode(name, "UTF-8");
    }

    private static com.google.gson.JsonObject json(String body) {
        return new com.google.gson.Gson().fromJson(body, com.google.gson.JsonObject.class);
    }

    private static final String ORIGIN = "{\"x\":0.0,\"y\":0.0,\"heading\":0.0}";

    private static double limitAt(double heading) {
        return SimPlacement.FIELD_SIZE_IN / 2
                - SimPlacement.ROBOT_SIZE_IN / 2 * (Math.abs(Math.cos(heading)) + Math.abs(Math.sin(heading)));
    }

    @Test
    public void theStartPoseIsRememberedPerOpModeAndTheRunIsPlacedThere() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.PROTOCOL);
        bench = benchOver(
                FakeSources.listing(SimCatalog.of(ThreeLoopAuto.class, TestAutos.NeverDoneAuto.class)), child);
        Response before = routes().handle(get("/start?opmode=" + encode("Count to three")));
        assertEquals(before.body, 200, before.status);
        assertEquals(ORIGIN, before.body);

        Response placed = routes().handle(put(
                "/start?opmode=" + encode("Count to three"), "{\"x\": -60, \"y\": 1000, \"heading\": 1.5}"));

        assertEquals(placed.body, 200, placed.status);
        com.google.gson.JsonObject stored = json(placed.body);
        assertEquals(-60, stored.get("x").getAsDouble(), 0);
        assertEquals("kept inside the walls", limitAt(1.5), stored.get("y").getAsDouble(), 0.001);
        assertEquals(1.5, stored.get("heading").getAsDouble(), 0);
        assertEquals(placed.body, routes().handle(get("/start?opmode=" + encode("Count to three"))).body);
        assertEquals(
                "another op mode has its own",
                ORIGIN,
                routes().handle(get("/start?opmode=" + encode("Never done"))).body);

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada"));
        assertEquals(run.message(), "done", run.outcome());
        com.google.gson.JsonObject start = theStartLineIn(child).getAsJsonObject("start");
        assertEquals(
                "the child is placed where the op mode was", -60, start.get("x").getAsDouble(), 0.001);
        assertEquals(limitAt(1.5), start.get("y").getAsDouble(), 0.001);
        assertEquals(1.5, start.get("heading").getAsDouble(), 0.001);

        await(bench.start(bench.catalog().find("Never done").get(), "ada"));
        com.google.gson.JsonObject elsewhere = theStartLineIn(child).getAsJsonObject("start");
        assertEquals(
                "another op mode starts at its own pose", 0, elsewhere.get("x").getAsDouble(), 0.001);

        bench.stop();
        bench = benchOverAChild(aChildSpeaking(SimRunStream.PROTOCOL));
        assertEquals(
                "remembered across a restart",
                placed.body,
                routes().handle(get("/start?opmode=" + encode("Count to three"))).body);
    }

    @Test
    public void aStartThatIsNotAPoseIsRefusedAndNothingIsRemembered() throws Exception {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);
        String start = "/start?opmode=" + encode("Count to three");

        assertEquals(400, routes().handle(put("/start", ORIGIN)).status);
        Response missing = routes().handle(put(start, "{\"x\": 1, \"y\": 2}"));
        assertEquals(400, missing.status);
        assertTrue(missing.body, missing.body.contains("heading"));
        assertEquals(400, routes().handle(put(start, "{\"x\": \"far\", \"y\": 2, \"heading\": 0}")).status);
        assertEquals(400, routes().handle(put(start, "nope")).status);
        assertEquals(405, routes().handle(post(start, ORIGIN)).status);
        assertEquals(400, routes().handle(get("/start")).status);

        assertEquals(ORIGIN, routes().handle(get(start)).body);
    }

    @Test
    public void thePlacementPageShowsTheRobotWhereTheRunWillStart() throws Exception {
        bench = new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);
        routes().handle(put("/start?opmode=" + encode("Count to three"), "{\"x\": 12, \"y\": -6, \"heading\": 0.5}"));

        Response page = routes().handle(get("/place?opmode=" + encode("Count to three")));

        assertEquals(200, page.status);
        assertTrue(page.body, page.body.contains("<canvas"));
        assertTrue(page.body, page.body.contains("\"name\":\"Count to three\""));
        assertTrue(page.body, page.body.contains("\"placing\":{\"x\":12.0,\"y\":-6.0,\"heading\":0.5}"));
        assertEquals(400, routes().handle(get("/place")).status);
    }

    @Test
    public void theLogKeepsWhatTheChildWroteToStderr() throws Exception {
        bench = new SimBench(SimCatalog.of(TestAutos.ChattyAuto.class), null, outputDir(), WAITS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Chatty").get(), "ada"));

        assertTrue(run.log(), run.log().contains("hello from the op mode"));
    }
}
