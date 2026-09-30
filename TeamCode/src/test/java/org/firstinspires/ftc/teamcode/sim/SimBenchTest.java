package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
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
import org.firstinspires.ftc.nugget.NuggetTeleOp;
import org.firstinspires.ftc.reginald.opmode.PlanOp;
import org.firstinspires.ftc.reginald.opmode.RedTeleOp;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.firstinspires.ftc.teamcode.sim.TestTeleOps.StickTeleOp;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.RunState;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimBenchTest {
    private static final double AUTONOMOUS_SECONDS = 2;
    private static final double TELEOP_SECONDS = 30;
    private static final double GRACE_SECONDS = 1;

    /**
     * What a bench waits in a test: a match's two periods cut to something a test can sit through. The silence stays what the bench waits in earnest, because a child that is
     * talking never reaches it and a child that is not is only ever one test's business -- cut
     * here, it would be a busy machine's chance to have a healthy run killed for pausing.
     */
    private static final SimBench.Waits WAITS = SimBench.Waits.ofTheBench()
            .autonomousPeriod(AUTONOMOUS_SECONDS)
            .teleOpPeriod(TELEOP_SECONDS)
            .killGrace(GRACE_SECONDS);

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private SimBench bench;

    @After
    public void stopBench() {
        SimBench.Run left = bench == null ? null : bench.current();
        if (bench != null) {
            bench.stop();
        }
        // See CodingServerTest: a run still going when a test ends leaves the suite spending into the
        // shutdown that takes its ledger, and the count wobbles by whatever did or did not get away.
        assertNull(
                "this test ended with a simulation still running",
                left == null ? null : left.entry.name + " (" + left.phase() + ")");
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
            new SimCatalog.Entry("BlueTeleOp", "TeleOp", SimCatalog.TELEOP, "", "Blue", null);

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
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);
        SimBench.Run run = bench
        .new Run(
                1,
                bench.catalog().find("Count to three").get(),
                "ada",
                null,
                null,
                SimBench.Mode.FREE_PLAY,
                SimBench.Begin.AT_ONCE);
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
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada", SimBench.Mode.FREE_PLAY));

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
                TeamRobot.REGINALD,
                SimCatalog.of(HangingAuto.class),
                null,
                outputDir(),
                WAITS.autonomousPeriod(0.3).silence(0.5));
        long startedAt = System.nanoTime();

        SimBench.Run run = await(bench.start(bench.catalog().find("Hangs").get(), "ada", SimBench.Mode.GAME));

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
        bench = new SimBench(
                TeamRobot.REGINALD,
                SimCatalog.of(SlowRegistrar.class),
                null,
                outputDir(),
                WAITS.autonomousPeriod(budget));

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Slow to start").get(), "ada", SimBench.Mode.GAME));

        assertTrue(run.outcome(), run.outcome().startsWith("timed out after " + budget + "s"));
    }

    @Test
    public void aBenchOverAProjectNeedsTheProjectsSimulator() throws Exception {
        Path project = folder.getRoot().toPath();
        sourceRootWith(project, tempPlans(2));
        try {
            bench = new SimBench(TeamRobot.REGINALD, null, project, outputDir(), WAITS);
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
        bench = new SimBench(TeamRobot.REGINALD, null, project, outputDir(), WAITS.teleOpPeriod(0.3));

        assertTrue(bench.catalog().find("BlueTeleOp").isPresent());
        SimBench.Run run = await(bench.start(BLUE_TELEOP, "ada", SimBench.Mode.GAME));

        assertEquals(run.message() + "\n" + run.log(), "done", run.outcome());
        assertTrue(run.log(), run.log().contains("this project's simulator"));
    }

    @Test
    public void aNuggetBenchOverAProjectListsNuggetsOpModesAloneAndRunsThemOnTheSimulatedNugget() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        bench = new SimBench(TeamRobot.NUGGET, null, project, outputDir(), WAITS.teleOpPeriod(0.3));

        SimCatalog listed = bench.catalog();
        assertTrue(listed.find("Nugget TeleOp").isPresent());
        assertFalse(
                "Reginald's op modes are not Nugget's",
                listed.find("BlueTeleOp").isPresent());
        assertFalse(
                "Reginald's op modes are not Nugget's",
                listed.find("driveForward").isPresent());
        SimBench.Run run = await(bench.start(listed.find("Nugget TeleOp").get(), "ada", SimBench.Mode.GAME));

        assertEquals(run.message() + "\n" + run.log(), "done", run.outcome());
        assertTrue(run.log(), run.log().contains("Robot: Nugget"));
        assertFalse(run.ticks().isEmpty());
        for (SimRunStream.TickLine tick : run.ticks()) {
            assertEquals(
                    tick.text(), 2, json(tick.text()).getAsJsonArray("powers").size());
        }
    }

    @Test
    public void aClassTheProjectLacksIsMissingNotThisServers() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        Files.delete(project.resolve("TeamCode/src/main/java/org/firstinspires/ftc/reginald/opmode/BlueTeleOp.java"));
        bench = new SimBench(TeamRobot.REGINALD, null, project, outputDir(), WAITS);

        SimCatalog catalog = bench.catalog();

        assertTrue(catalog.find("RedTeleOp").isPresent());
        assertFalse(
                "this server has a BlueTeleOp; the project does not",
                catalog.find("BlueTeleOp").isPresent());
    }

    @Test
    public void eachOpModeHasASeedTheCatalogShowsAndARouteSets() throws Exception {
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);
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

        SimBench.Run seeded = await(bench.start(entry, "ada", SimBench.Mode.FREE_PLAY));
        assertEquals(Long.valueOf(5), seeded.seed);
        assertEquals("done", seeded.outcome());
        assertEquals(
                "the seed is on the start line",
                5,
                theStartLineIn(child).get("seed").getAsLong());
        assertTrue(bench.status(), bench.status().contains("\"seed\":5"));

        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Count to three"), "{\"seed\": null}")).status);
        SimBench.Run exact = await(bench.start(entry, "ada", SimBench.Mode.FREE_PLAY));
        assertNull(exact.seed);
        assertFalse(
                "the exact robot is a start line with no seed at all",
                theStartLineIn(child).has("seed"));
        assertTrue(bench.status(), bench.status().contains("\"seed\":null"));
    }

    @Test
    public void savedEditsTakeEffectOnTheNextRunWithoutARestart() throws Exception {
        Path project = projectWith(folder.getRoot().toPath(), tempPlans(2));
        bench = new SimBench(TeamRobot.REGINALD, null, project, outputDir(), WAITS);
        SimCatalog.Entry temp = bench.catalog().find(TEMP_NAME).get();
        assertEquals("Test", temp.group);
        assertEquals("Plans.temp()", temp.where);
        assertEquals(
                2,
                await(bench.start(temp, "ada", SimBench.Mode.FREE_PLAY)).ticks().size());

        sourceRootWith(folder.getRoot().toPath(), tempPlans(4, "Test v2"));

        SimBench.Run second = await(bench.start(temp, "ada", SimBench.Mode.FREE_PLAY));
        assertEquals("done", second.outcome());
        assertEquals(4, second.ticks().size());
        assertEquals("Test v2", bench.catalog().find(TEMP_NAME).get().group);
    }

    @Test
    public void checkCompilesWithoutRunningAndNeverUnderARun() throws Exception {
        Path project = projectWith(folder.getRoot().toPath(), tempPlans(100000));
        bench = new SimBench(TeamRobot.REGINALD, null, project, outputDir(), WAITS.autonomousPeriod(1.0));
        assertTrue(bench.check().problems.isEmpty());
        assertNull("nothing ran", bench.current());
        SimBench.Run run = bench.start(
                new SimCatalog.Entry(TEMP_NAME, "Test", SimCatalog.AUTO, "", null, null), "ada", SimBench.Mode.GAME);
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
    public void aLiveViewIsSentEachTickAsTheChildPrintedIt() throws Exception {
        String asPrinted = SimRunStream.tick(SimRecording.Tick.at(
                        0.02, StartPoses.ORIGIN, "1. <aim> & 'launch'", new double[] {0.5, 0, 0, 0}, List.of())
                .tick());
        bench = benchOverAChild(FakeChild.thatSays(
                SimRunStream.hello(),
                SimRunStream.started(),
                asPrinted,
                SimRunStream.finished(SimRunStream.Outcome.done())));
        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada", SimBench.Mode.FREE_PLAY));

        Response ticks = routes().handle(get("/runs/" + run.id + "/ticks?from=0"));

        assertEquals(200, ticks.status);
        assertEquals("{\"outcome\":\"done\",\"ticks\":[" + asPrinted + "]}", ticks.body);
        assertEquals(
                "a run's status counts its ticks",
                1,
                json(routes().handle(get("/status")).body)
                        .getAsJsonArray("runs")
                        .get(0)
                        .getAsJsonObject()
                        .get("loops")
                        .getAsInt());
    }

    @Test
    public void theLiveViewIsOnlyEverServedWhereItsOwnRelativeFetchesResolve() throws Exception {
        bench = benchOverAChild(aChildSpeaking(SimRunStream.PROTOCOL));
        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada", SimBench.Mode.FREE_PLAY));

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
        return new SimBench(
                TeamRobot.REGINALD, SimCatalog.of(TestTeleOps.StickTeleOp.class), null, outputDir(), waits, child);
    }

    /** A bench whose every wait is judged on a clock that moves only when the test moves it. */
    private SimBench benchOn(FakeClock clock, FakeChild child, SimBench.Waits waits) {
        return new SimBench(
                TeamRobot.REGINALD,
                SimSources.ofThisClasspath(SimCatalog.of(TestTeleOps.StickTeleOp.class)),
                outputDir(),
                waits,
                child,
                clock);
    }

    /**
     * Waits for a run the test's clock has just ended, in far less real time than any wait it was
     * given: a wait that ran on real time rather than the bench's clock has not ended it by then.
     */
    private static SimBench.Run awaitPromptly(SimBench.Run run) throws InterruptedException {
        long deadline = System.nanoTime() + 3_000_000_000L;
        while (run.outcome() == null && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertNotNull("the bench's clock ran out a wait and nothing came of it; phase " + run.phase(), run.outcome());
        return run;
    }

    private static void awaitPhase(SimBench.Run run, String phase) throws InterruptedException {
        long deadline = System.nanoTime() + 30_000_000_000L;
        while (!phase.equals(run.phase()) && run.outcome() == null && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertEquals(run.outcome(), phase, run.phase());
    }

    @Test
    public void aChildThatGoesSilentWhileItsRunIsUnfinishedIsKilledForNotReturning() throws Exception {
        FakeClock clock = new FakeClock();
        bench = benchOn(
                clock,
                FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started()).thatStaysAliveSayingNothingMore(),
                WAITS);
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY);
        awaitRunning(run);

        clock.advance(WAITS.silence + 0.1);

        assertTrue(run.outcome(), awaitPromptly(run).outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("the op mode did not return"));
    }

    @Test
    public void theStartupIsWaitedOnTheBenchsClock() throws Exception {
        FakeClock clock = new FakeClock();
        FakeChild child = FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore();
        bench = benchOn(clock, child, WAITS);
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY);
        awaitPhase(run, "starting");

        clock.advance(WAITS.startup + 0.1);

        assertEquals(
                SimRunStream.Outcome.killed(WAITS.startup, "the op mode never started"),
                awaitPromptly(run).outcome());
        assertFalse("the child is killed with it", child.running().alive());
    }

    @Test
    public void aChildThatIgnoresStopHasItsGraceOnTheBenchsClock() throws Exception {
        FakeClock clock = new FakeClock();
        double grace = 10;
        FakeChild child = FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started())
                .thatStaysAliveSayingNothingMore()
                .thatIgnoresStop();
        bench = benchOn(clock, child, WAITS.killGrace(grace));
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY);
        awaitRunning(run);

        run.stop();
        Thread.sleep(200);
        assertTrue("killed before the bench's clock said its grace was over", run.running());
        clock.advance(grace + 0.1);

        assertEquals(
                SimRunStream.Outcome.killedAfterStop(grace), awaitPromptly(run).outcome());
        assertFalse("the child is killed", child.running().alive());
    }

    @Test
    public void aRunStoppedWhileItsSourcesBuildEndsThereAndLeavesNoChildRunning() throws Exception {
        CountDownLatch built = new CountDownLatch(1);
        FakeChild child = FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore();
        FakeSources sources =
                FakeSources.listing(SimCatalog.of(StickTeleOp.class)).thatBuildForARunUntil(built);
        bench = benchOver(sources, child);
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY);
        sources.awaitARunsBuild();
        assertEquals("building", run.phase());

        assertEquals(200, routes().handle(post("/runs/" + run.id + "/stop", "")).status);
        assertEquals(SimRunStream.Outcome.stopped(), run.outcome());
        assertTrue(run.message(), run.message().contains("before the build finished"));

        built.countDown();
        long deadline = System.nanoTime() + 10_000_000_000L;
        while ((child.running() == null || child.running().alive()) && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertNotNull("the build was let finish, and a child started from it", child.running());
        assertFalse(
                "the child built for a run that was stopped is still running, and nobody will stop it",
                child.running().alive());
        assertEquals("finished", run.phase());
        assertTrue(
                "nothing was sent to a child nobody wanted: " + child.whatItWasTold(),
                child.whatItWasTold().isEmpty());
    }

    @Test
    public void aChildThatCannotBeStartedEndsTheRunSayingSo() throws Exception {
        bench = benchWith(new FakeChild().thatWillNotStart("no java on this machine"), WAITS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY));

        assertEquals(SimRunStream.Outcome.couldNotStartChild(), run.outcome());
        assertTrue(run.message(), run.message().contains("no java on this machine"));
    }

    @Test
    public void aChildThatNeverSaysTheOpModeStartedIsKilledForNeverStarting() throws Exception {
        bench = benchWith(
                FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore(), WAITS.startup(0.3));

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("never started"));
    }

    @Test
    public void aChildThatIgnoresStopIsKilledAfterItsGrace() throws Exception {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started())
                .thatStaysAliveSayingNothingMore()
                .thatIgnoresStop();
        bench = benchWith(child, WAITS.killGrace(0.2));
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY);
        awaitRunning(run);

        run.stop();
        await(run);

        assertEquals(SimRunStream.Outcome.killedAfterStop(0.2), run.outcome());
        assertTrue(
                child.whatItWasTold().toString(), child.whatItWasTold().stream().anyMatch(t -> t.contains("stop")));
    }

    private SimBench.Run aRunThatWaitsForItsView() {
        return bench.start(
                bench.catalog().find("Stick").get(),
                "ada",
                SimBench.Mode.FREE_PLAY,
                SimBench.Begin.WHEN_ITS_VIEW_IS_READY);
    }

    private static void awaitHeld(SimBench.Run run) throws InterruptedException {
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (!(run.state() instanceof RunState.Held) && run.outcome() == null && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertTrue(
                "the child was not held for its view: " + run.state() + ", " + run.outcome(),
                run.state() instanceof RunState.Held);
    }

    private static void awaitPlaced(FakeChild child) throws InterruptedException {
        long deadline = System.nanoTime() + 3_000_000_000L;
        while (!aStartLineIsIn(child) && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertTrue("the child was never placed; it was told " + child.whatItWasTold(), aStartLineIsIn(child));
    }

    private Response ready(SimBench.Run run) {
        return routes().handle(post("/runs/" + run.id + "/ready", ""));
    }

    private static void endIt(SimBench.Run run) throws InterruptedException {
        run.stop();
        awaitPromptly(run);
    }

    @Test
    public void aRunThatWaitsForItsViewIsPlacedOnlyOnceItsViewIsReady() throws Exception {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore();
        bench = benchOn(new FakeClock(), child, WAITS);
        SimBench.Run run = aRunThatWaitsForItsView();
        awaitHeld(run);

        assertFalse("placed before its view was ready: " + child.whatItWasTold(), aStartLineIsIn(child));
        assertEquals("starting", run.phase());

        Response ready = ready(run);

        assertEquals(ready.body, 200, ready.status);
        assertTrue("its view is ready, so its op mode's time may begin", aStartLineIsIn(child));
        assertTrue(run.state().toString(), run.state() instanceof RunState.Starting);
        assertEquals("placed once", 1, startLinesIn(child));
        assertEquals(200, ready(run).status);
        assertEquals("a view that says so twice places nothing twice", 1, startLinesIn(child));
        endIt(run);
    }

    @Test
    public void aViewReadyBeforeTheChildIsHasItsRunPlacedTheMomentTheChildSaysHello() throws Exception {
        CountDownLatch built = new CountDownLatch(1);
        FakeChild child = FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore();
        FakeSources sources =
                FakeSources.listing(SimCatalog.of(StickTeleOp.class)).thatBuildForARunUntil(built);
        bench = new SimBench(TeamRobot.REGINALD, sources, outputDir(), WAITS, child, new FakeClock());
        SimBench.Run run = aRunThatWaitsForItsView();
        sources.awaitARunsBuild();

        assertEquals(200, ready(run).status);
        built.countDown();

        awaitPlaced(child);
        assertFalse(run.state() instanceof RunState.Held);
        endIt(run);
    }

    @Test
    public void aViewNotReadyWithinTheViewWaitIsNotWaitedForLonger() throws Exception {
        FakeClock clock = new FakeClock();
        FakeChild child = FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore();
        bench = benchOn(clock, child, WAITS);
        SimBench.Run run = aRunThatWaitsForItsView();
        awaitHeld(run);

        clock.advance(WAITS.view);
        Thread.sleep(200);
        assertFalse("placed before the view wait was over", aStartLineIsIn(child));
        clock.advance(0.2);

        awaitPlaced(child);
        assertTrue("a run begun without its view goes on", run.running());
        endIt(run);
    }

    @Test
    public void theStartupOfAChildHeldForItsViewRunsFromWhenItWasPlaced() throws Exception {
        FakeClock clock = new FakeClock();
        double startup = 10;
        FakeChild child = FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore();
        bench = benchOn(clock, child, WAITS.startup(startup));
        SimBench.Run run = aRunThatWaitsForItsView();
        awaitHeld(run);

        clock.advance(startup * 2);
        Thread.sleep(200);
        assertTrue("killed for a startup it was held through: " + run.outcome(), run.running());

        assertEquals(200, ready(run).status);
        clock.advance(startup - 1);
        Thread.sleep(200);
        assertTrue("killed before its own startup was over: " + run.outcome(), run.running());
        clock.advance(1.1);

        assertEquals(
                SimRunStream.Outcome.killed(startup, "the op mode never started"),
                awaitPromptly(run).outcome());
    }

    @Test
    public void stopWhileAChildIsHeldForItsViewEndsTheRunUnplaced() throws Exception {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore();
        bench = benchOn(new FakeClock(), child, WAITS);
        SimBench.Run run = aRunThatWaitsForItsView();
        awaitHeld(run);

        assertEquals(200, routes().handle(post("/runs/" + run.id + "/stop", "")).status);
        ready(run);
        awaitPromptly(run);

        assertTrue(
                "told to stop, and nothing else: " + child.whatItWasTold(),
                child.whatItWasTold().stream().anyMatch(t -> t.contains("\"stop\"")));
        assertFalse("placed after Stop: " + child.whatItWasTold(), aStartLineIsIn(child));
    }

    @Test
    public void theRunRouteTakesWhenARunBeginsAndBeginsAtOnceOtherwise() throws Exception {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore();
        bench = benchOn(new FakeClock(), child, WAITS);
        String run = "/run?opmode=Stick&mode=free";

        Response waiting = routes().handle(post(run + "&begin=ready", ""));
        assertEquals(waiting.body, 200, waiting.status);
        SimBench.Run held = bench.find(json(waiting.body).get("id").getAsString());
        awaitHeld(held);
        assertTrue("its page is told to say when it is ready", awaitedOn(held));
        assertEquals(200, ready(held).status);
        assertEquals(1, startLinesIn(child));
        assertFalse("a page served once the view is ready has nothing to say", awaitedOn(held));
        endIt(held);

        Response atOnce = routes().handle(post(run + "&begin=now", ""));
        assertEquals(atOnce.body, 200, atOnce.status);
        SimBench.Run placed = bench.find(json(atOnce.body).get("id").getAsString());
        awaitStartLines(child, 2);
        assertFalse("nobody asked its page to say anything", awaitedOn(placed));
        endIt(placed);

        Response unsaid = routes().handle(post(run, ""));
        assertEquals(unsaid.body, 200, unsaid.status);
        SimBench.Run unsaidRun = bench.find(json(unsaid.body).get("id").getAsString());
        awaitStartLines(child, 3);
        assertFalse(awaitedOn(unsaidRun));
        endIt(unsaidRun);

        Response neither = routes().handle(post(run + "&begin=later", ""));
        assertEquals(neither.body, 400, neither.status);
        assertTrue(neither.body, neither.body.contains("now") && neither.body.contains("ready"));
        assertEquals(
                "nothing was started",
                3,
                json(bench.status()).getAsJsonArray("runs").size());
    }

    private static long startLinesIn(FakeChild child) {
        return child.whatItWasTold().stream()
                .filter(t -> t.contains("\"start\""))
                .count();
    }

    private static void awaitStartLines(FakeChild child, long count) throws InterruptedException {
        long deadline = System.nanoTime() + 3_000_000_000L;
        while (startLinesIn(child) < count && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertEquals("placed as soon as the child could be: " + child.whatItWasTold(), count, startLinesIn(child));
    }

    @Test
    public void aViewSayingItIsReadyForARunThatIsNotWaitingChangesNothing() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.PROTOCOL);
        bench = benchOverAChild(child);
        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada", SimBench.Mode.FREE_PLAY));

        Response ready = ready(run);

        assertEquals(ready.body, 200, ready.status);
        assertEquals(run.message(), "done", run.outcome());
        assertEquals(404, routes().handle(post("/runs/999/ready", "")).status);
        assertEquals(405, routes().handle(get("/runs/" + run.id + "/ready")).status);
    }

    private boolean awaitedOn(SimBench.Run run) {
        Response page = routes().handle(get("/runs/" + run.id + "/"));
        assertEquals(200, page.status);
        String opening = "<script id=\"recording\" type=\"application/json\">";
        int from = page.body.indexOf(opening) + opening.length();
        com.google.gson.JsonElement awaited = json(page.body.substring(from, page.body.indexOf("</script>", from)))
                .get("awaited");
        assertNotNull("the page is told nothing either way about whether its run waits for it", awaited);
        return awaited.getAsBoolean();
    }

    /**
     * A bench over sources that need neither a project on disk nor a compile, and a child that need
     * not be a JVM. What a real build does with real sources is SimBuildTest's and what a real child
     * prints is SimChildTest's; what this holds is what the bench does with either.
     */
    private SimBench benchOver(SimSources sources, Child child) {
        return new SimBench(TeamRobot.REGINALD, sources, outputDir(), WAITS, child, new SystemClock());
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
            SimBench.Run run = await(bench.start(
                    new SimCatalog.Entry(TEMP_NAME, "Test", SimCatalog.AUTO, "", null, null),
                    "ada",
                    SimBench.Mode.FREE_PLAY));

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
                await(bench.start(bench.catalog().find("Count to three").get(), "ada", SimBench.Mode.FREE_PLAY));

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

        SimBench.Run atTheOrigin = await(bench.start(entry, "ada", SimBench.Mode.FREE_PLAY));
        assertEquals(atTheOrigin.message(), "done", atTheOrigin.outcome());
        assertTrue(atTheOrigin.ticks().size() > 0);
        assertFalse("it places itself, so it is told nothing", aStartLineIsIn(child));

        assertEquals(
                200,
                routes().handle(put(
                                "/start?opmode=" + encode("Count to three"), "{\"x\": 24, \"y\": 0, \"heading\": 0}"))
                        .status);
        SimBench.Run elsewhere = await(bench.start(entry, "ada", SimBench.Mode.FREE_PLAY));

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

        SimBench.Run exact = await(bench.start(entry, "ada", SimBench.Mode.FREE_PLAY));
        assertEquals(exact.message(), "done", exact.outcome());
        assertTrue(exact.ticks().size() > 0);

        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Count to three"), "{\"seed\": 3}")).status);
        SimBench.Run seeded = await(bench.start(entry, "ada", SimBench.Mode.FREE_PLAY));

        assertEquals(SimRunStream.Outcome.cannotSeed(SimRunStream.SEEDED_PROTOCOL - 1), seeded.outcome());
        assertTrue(seeded.message(), seeded.message().toLowerCase().contains("pull"));
        assertTrue(seeded.message(), seeded.message().contains("protocol " + (SimRunStream.SEEDED_PROTOCOL - 1)));
        assertEquals(0, seeded.ticks().size());
        assertNull(bench.current());
    }

    private SimBench nuggetBenchOver(SimSources sources, Child child) {
        return new SimBench(TeamRobot.NUGGET, sources, outputDir(), WAITS, child, new SystemClock());
    }

    @Test
    public void aNuggetBenchTellsTheChildItStartsThatItIsNugget() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.PROTOCOL);
        bench = nuggetBenchOver(FakeSources.listing(SimCatalog.of(NuggetTeleOp.class)), child);

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Nugget TeleOp").get(), "ada", SimBench.Mode.FREE_PLAY));

        assertEquals(run.message(), "done", run.outcome());
        assertTrue(child.theLastStart().toString(), child.theLastStart().contains("--robot=nugget"));
        assertEquals(TeamRobot.NUGGET, run.robot());
    }

    @Test
    public void aReginaldBenchTellsTheChildNothingOfRobotsSoAChildFromBeforeThereWereTwoStillRuns() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.ROBOTS_PROTOCOL - 1);
        bench = benchOverAChild(child);

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada", SimBench.Mode.FREE_PLAY));

        assertEquals(run.message(), "done", run.outcome());
        assertTrue(
                child.theLastStart().toString(),
                child.theLastStart().stream().noneMatch(argument -> argument.startsWith("--robot")));
        assertEquals(TeamRobot.REGINALD, run.robot());
    }

    @Test
    public void aChildFromBeforeThereWereTwoRobotsIsRefusedANuggetRunNamingNuggetsLine() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.ROBOTS_PROTOCOL - 1);
        bench = nuggetBenchOver(FakeSources.listing(SimCatalog.of(NuggetTeleOp.class)), child);

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Nugget TeleOp").get(), "ada", SimBench.Mode.FREE_PLAY));

        assertEquals(
                run.message(),
                SimRunStream.Outcome.cannotSimulate(SimRunStream.ROBOTS_PROTOCOL - 1, TeamRobot.NUGGET),
                run.outcome());
        assertTrue(run.message(), run.message().contains("Pull nugget-develop"));
        assertEquals(0, run.ticks().size());
        assertFalse("a refused run is not placed", aStartLineIsIn(child));
    }

    @Test
    public void aNuggetBenchAsksTheChildForNuggetsOpModesAndRefusesAListingFromBeforeThereWereTwoRobots() {
        FakeChild older = FakeChild.thatSays(SimRunStream.helloOf(SimRunStream.ROBOTS_PROTOCOL - 1), "[]");
        bench = nuggetBenchOver(FakeSources.askingTheChild(), older);

        try {
            bench.catalog();
            fail("a listing of Reginald's op modes must not be shown to Nugget's users");
        } catch (SimRunStream.WrongProtocol e) {
            assertEquals(SimRunStream.ROBOTS_PROTOCOL - 1, e.childProtocol);
            assertTrue(e.getMessage(), e.getMessage().contains("Pull nugget-develop"));
        }
        assertEquals(List.of("--list", "--robot=nugget"), older.theLastStart());
        Response listed = routes().handle(get("/catalog"));
        assertEquals(500, listed.status);
        assertTrue(listed.body, listed.body.contains("nugget-develop"));
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
        SimBench.Run run = await(bench.start(BLUE_TELEOP, "ada", SimBench.Mode.FREE_PLAY));

        assertEquals(run.message(), SimRunStream.Outcome.wrongProtocol(newer), run.outcome());
        assertTrue(run.message(), run.message().contains("protocol " + newer));
        assertTrue(run.message(), run.message().contains("server"));
        assertEquals(0, run.ticks().size());
    }

    @Test
    public void aGameIsGivenTheMatchsPeriodForItsKindAndFreePlayIsGivenNoLimitAtAll() throws Exception {
        FakeChild child = aChildSpeaking(SimRunStream.PROTOCOL);
        bench = benchOver(FakeSources.listing(SimCatalog.of(ThreeLoopAuto.class, StickTeleOp.class)), child);
        SimCatalog.Entry auto = bench.catalog().find("Count to three").get();
        SimCatalog.Entry teleOp = bench.catalog().find("Stick").get();

        await(bench.start(auto, "ada", SimBench.Mode.GAME));
        assertEquals(
                "an auto in a game has the autonomous period",
                List.of("--run", "Count to three", String.valueOf(AUTONOMOUS_SECONDS)),
                child.theLastStart().subList(0, 3));

        await(bench.start(teleOp, "ada", SimBench.Mode.GAME));
        assertEquals(
                "a TeleOp in a game has the driver-controlled period",
                List.of("--run", "Stick", String.valueOf(TELEOP_SECONDS)),
                child.theLastStart().subList(0, 3));

        for (SimCatalog.Entry either : List.of(auto, teleOp)) {
            await(bench.start(either, "ada", SimBench.Mode.FREE_PLAY));
            assertEquals(
                    "free play goes on until the plan is done or somebody presses Stop",
                    List.of("--run", either.name, String.valueOf(Double.POSITIVE_INFINITY)),
                    child.theLastStart().subList(0, 3));
        }
    }

    @Test
    public void theRunRouteTakesAGameByNameAndIsFreePlayOtherwiseAndTheStatusSaysWhich() throws Exception {
        bench = benchOverAChild(aChildSpeaking(SimRunStream.PROTOCOL));
        String run = "/run?opmode=" + encode("Count to three");

        Response game = routes().handle(post(run + "&mode=game", ""));
        assertEquals(game.body, 200, game.status);
        await(bench.find(json(game.body).get("id").getAsString()));
        Response free = routes().handle(post(run + "&mode=free", ""));
        assertEquals(free.body, 200, free.status);
        await(bench.find(json(free.body).get("id").getAsString()));
        Response unsaid = routes().handle(post(run, ""));
        assertEquals(unsaid.body, 200, unsaid.status);
        await(bench.find(json(unsaid.body).get("id").getAsString()));

        com.google.gson.JsonArray runs = json(bench.status()).getAsJsonArray("runs");
        assertEquals(
                "newest first",
                "free",
                runs.get(0).getAsJsonObject().get("mode").getAsString());
        assertEquals("free", runs.get(1).getAsJsonObject().get("mode").getAsString());
        assertEquals("game", runs.get(2).getAsJsonObject().get("mode").getAsString());

        Response neither = routes().handle(post(run + "&mode=match", ""));
        assertEquals(neither.body, 400, neither.status);
        assertTrue(neither.body, neither.body.contains("game") && neither.body.contains("free"));
        assertEquals(
                "nothing was started",
                3,
                json(bench.status()).getAsJsonArray("runs").size());
    }

    @Test
    public void aGameIsWatchedFromTheAlliancesOwnAreaAndFreePlayFromWhereverYouLike() throws Exception {
        bench = benchOver(
                FakeSources.listing(SimCatalog.of(ThreeLoopAuto.class, RedTeleOp.class)),
                aChildSpeaking(SimRunStream.PROTOCOL));
        SimCatalog.Entry red = bench.catalog().find("RedTeleOp").get();
        SimCatalog.Entry nobodys = bench.catalog().find("Count to three").get();

        com.google.gson.JsonObject redGame = matchOn(await(bench.start(red, "ada", SimBench.Mode.GAME)));
        assertEquals(TELEOP_SECONDS, redGame.get("period").getAsDouble(), 0);
        assertEquals("Red", redGame.get("alliance").getAsString());
        assertArrayEquals(
                Points.array(Valid.value(SimPlacement.FIELD.allianceArea("Red")).eye()),
                new com.google.gson.Gson().fromJson(redGame.get("eye"), double[].class),
                1e-9);
        assertArrayEquals(
                Points.array(Valid.value(SimPlacement.FIELD.allianceArea("Red")).lookingAt()),
                new com.google.gson.Gson().fromJson(redGame.get("lookingAt"), double[].class),
                1e-9);

        com.google.gson.JsonObject nobodysGame = matchOn(await(bench.start(nobodys, "ada", SimBench.Mode.GAME)));
        assertEquals(AUTONOMOUS_SECONDS, nobodysGame.get("period").getAsDouble(), 0);
        assertEquals(
                "an op mode that plays for no alliance plays the blue way, so it is driven from there",
                "Blue",
                nobodysGame.get("alliance").getAsString());

        assertNull("free play is not a match", matchOn(await(bench.start(red, "ada", SimBench.Mode.FREE_PLAY))));
    }

    /** What the live view of a run is told about the match it is, read off the page as served. */
    private com.google.gson.JsonObject matchOn(SimBench.Run run) {
        Response page = routes().handle(get("/runs/" + run.id + "/"));
        assertEquals(200, page.status);
        String opening = "<script id=\"recording\" type=\"application/json\">";
        int from = page.body.indexOf(opening) + opening.length();
        com.google.gson.JsonElement match = json(page.body.substring(from, page.body.indexOf("</script>", from)))
                .get("match");
        assertNotNull("the page says nothing either way about a match", match);
        return match.isJsonNull() ? null : match.getAsJsonObject();
    }

    @Test
    public void aRunNobodyIsWatchingIsStoppedAndSaysWhy() throws Exception {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started(), aTickLine())
                .thatStaysAliveSayingNothingMore();
        bench = benchWith(child, WAITS.unwatched(0.3));

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY));

        assertEquals(SimRunStream.Outcome.stopped(), run.outcome());
        assertTrue(run.message(), run.message().contains("nobody"));
        assertFalse("the child is ended with it", child.running().alive());
    }

    @Test
    public void aRunSomebodyIsWatchingGoesOnUntilTheyStopIt() throws Exception {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started(), aTickLine())
                .thatStaysAliveSayingNothingMore();
        bench = benchWith(child, WAITS.unwatched(0.5));
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY);
        awaitRunning(run);

        long until = System.nanoTime() + 1_500_000_000L;
        while (System.nanoTime() < until) {
            assertEquals(200, routes().handle(get("/runs/" + run.id + "/ticks?from=0")).status);
            assertTrue("a run somebody is following was ended: " + run.message(), run.running());
            Thread.sleep(50);
        }

        assertEquals(200, routes().handle(post("/runs/" + run.id + "/stop", "")).status);
        await(run);
    }

    @Test
    public void aBenchWithoutSourcesHasNothingToCheck() {
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);

        assertNull(bench.check());
    }

    @Test
    public void aTeleOpRunDrivesFromGamepadPostsAndStopEndsIt() throws Exception {
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(StickTeleOp.class), null, outputDir(), WAITS);
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada", SimBench.Mode.FREE_PLAY);
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
                TeamRobot.REGINALD,
                SimCatalog.of(TestTeleOps.SlowMachineTeleOp.class),
                null,
                outputDir(),
                WAITS.teleOpPeriod(0.3));

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Slow machine").get(), "ada", SimBench.Mode.GAME));

        assertEquals(run.message() + "\n" + run.log(), "done", run.outcome());
        assertTrue(run.ticks().size() > 1);
    }

    @Test
    public void stopEndsAnAutoRunToo() throws Exception {
        bench = new SimBench(
                TeamRobot.REGINALD, SimCatalog.of(TestAutos.NeverDoneAuto.class), null, outputDir(), WAITS);
        SimBench.Run run = bench.start(bench.catalog().find("Never done").get(), "ada", SimBench.Mode.FREE_PLAY);
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
        for (SimRunStream.TickLine tick : run.ticks()) {
            if (condition.test(tick.json())) {
                return true;
            }
        }
        return false;
    }

    @Test
    public void anOpModeNobodyHasAndAMethodTheRouteDoesNotTakeAreRefused() {
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(HangingAuto.class), null, outputDir(), WAITS);

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
                await(bench.start(bench.catalog().find("Count to three").get(), "ada", SimBench.Mode.FREE_PLAY));
        assertEquals(run.message(), "done", run.outcome());
        com.google.gson.JsonObject start = theStartLineIn(child).getAsJsonObject("start");
        assertEquals(
                "the child is placed where the op mode was", -60, start.get("x").getAsDouble(), 0.001);
        assertEquals(limitAt(1.5), start.get("y").getAsDouble(), 0.001);
        assertEquals(1.5, start.get("heading").getAsDouble(), 0.001);

        await(bench.start(bench.catalog().find("Never done").get(), "ada", SimBench.Mode.FREE_PLAY));
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
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);
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
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), WAITS);
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
        bench = new SimBench(TeamRobot.REGINALD, SimCatalog.of(TestAutos.ChattyAuto.class), null, outputDir(), WAITS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Chatty").get(), "ada", SimBench.Mode.FREE_PLAY));

        assertTrue(run.log(), run.log().contains("hello from the op mode"));
    }
}
