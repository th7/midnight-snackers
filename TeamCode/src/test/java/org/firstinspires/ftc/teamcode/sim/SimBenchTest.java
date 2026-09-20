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
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.FileTime;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Stream;
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

    private static Path real(String relative) {
        Path path = Paths.get(relative).toAbsolutePath();
        assertTrue("tests run from the TeamCode module directory: " + path, Files.isDirectory(path));
        return path;
    }

    static void copyTree(Path from, Path to) throws IOException {
        try (Stream<Path> files = Files.walk(from)) {
            for (Path file : (Iterable<Path>) files::iterator) {
                Path target = to.resolve(from.relativize(file).toString());
                if (Files.isDirectory(file)) {
                    Files.createDirectories(target);
                } else {
                    Files.copy(file, target, StandardCopyOption.REPLACE_EXISTING);
                }
            }
        }
    }

    static Path simulatorInto(Path project) throws IOException {
        copyTree(real("src/test/java"), project.resolve("TeamCode/src/test/java"));
        copyTree(real("src/test/resources"), project.resolve("TeamCode/src/test/resources"));
        return project;
    }

    static Path projectWith(Path project, String plans) throws IOException {
        realProjectCopiedUnder(project);
        sourceRootWith(project, plans);
        return project;
    }

    static Path realProjectCopiedUnder(Path project) throws IOException {
        copyTree(real("src/main/java"), project.resolve("TeamCode/src/main/java"));
        return simulatorInto(project);
    }

    static void edit(Path project, String relative, String from, String to) throws IOException {
        Path file = project.resolve(relative);
        String source = new String(Files.readAllBytes(file), StandardCharsets.UTF_8);
        assertTrue(relative + " has no " + from, source.contains(from));
        Files.write(file, source.replace(from, to).getBytes(StandardCharsets.UTF_8));
    }

    static final String HARDWARE = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/Hardware.java";
    static final String SIM_ROBOT = "TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimRobot.java";
    static final String SIM_DEVICES = "TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimDevices.java";
    static final String SIM_CHILD = "TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimChild.java";
    static final String SIM_RUN_STREAM = "TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimRunStream.java";
    static final SimCatalog.Entry BLUE_TELEOP =
            new SimCatalog.Entry("BlueTeleOp", "TeleOp", SimCatalog.TELEOP, "", null);

    static final String TEMP_NAME = "Temp";

    static final int TEMP_LOOPS_LINE = 8;

    static String tempPlans(int loops, String group) {
        return "package org.firstinspires.ftc.teamcode;\n"
                + "import org.firstinspires.ftc.teamcode.opmode.Auto;\n"
                + "import org.firstinspires.ftc.teamcode.base.Loopable;\n"
                + "import org.firstinspires.ftc.teamcode.planrunner.PlanPart;\n"
                + "import org.firstinspires.ftc.teamcode.planrunner.Step;\n"
                + "\n"
                + "public class Plans implements Loopable {\n"
                + "    private int loops = 0;\n"
                + "    public Plans(org.firstinspires.ftc.teamcode.Drive drive,"
                + " org.firstinspires.ftc.teamcode.Nav nav,"
                + " org.firstinspires.ftc.teamcode.Launcher launcher,"
                + " java.util.function.LongSupplier nanoClock) { }\n"
                + "    @Auto(name = \"" + TEMP_NAME + "\", group = \"" + group
                + "\", alliance = org.firstinspires.ftc.teamcode.base.Alliance.RELATIVE)\n"
                + "    public PlanPart temp() { return new Step(\"count\", () -> { }, () -> ++loops >= " + loops
                + "); }\n"
                + "    public void loop() { }\n"
                + "}\n";
    }

    static String tempPlans(int loops) {
        return tempPlans(loops, "Test");
    }

    static Path sourceRootWith(Path project, String source) throws IOException {
        Path sourceRoot = project.resolve("TeamCode/src/main/java");
        Path file = sourceRoot.resolve("org/firstinspires/ftc/teamcode/Plans.java");
        Files.createDirectories(file.getParent());
        Files.write(file, source.getBytes(StandardCharsets.UTF_8));

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
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
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
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

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
        bench = new SimBench(SimCatalog.of(HangingAuto.class), null, outputDir(), 0.3, TELEOP_SECONDS, GRACE_SECONDS);
        long startedAt = System.nanoTime();

        SimBench.Run run = await(bench.start(bench.catalog().find("Hangs").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("the op mode did not return"));
        double seconds = (System.nanoTime() - startedAt) / 1e9;
        assertTrue("took " + seconds + "s", seconds < 30);
        assertNull(bench.current());
    }

    public static class SlowRegistrar {
        public static final double SECONDS = 1.5;

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
        bench = new SimBench(SimCatalog.of(SlowRegistrar.class), null, outputDir(), 0.3, TELEOP_SECONDS, GRACE_SECONDS);

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Slow to start").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("timed out after 0.3s"));
    }

    @Test
    public void aCompileErrorIsTheRunsOutcomeAndTheCatalogsToo() throws Exception {
        Path project = projectWith(
                folder.getRoot().toPath(), tempPlans(2).replace("private int loops = 0;", "private int loops = ;"));
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

        try {
            bench.catalog();
            fail("a catalog cannot be listed from sources that do not compile");
        } catch (SimBench.BuildFailed e) {
            assertTrue(e.getMessage(), e.getMessage().contains("Plans.java:" + TEMP_LOOPS_LINE));
        }
        SimBench.Run run =
                await(bench.start(new SimCatalog.Entry(TEMP_NAME, "Test", SimCatalog.AUTO, "", null), "ada"));
        assertEquals("build failed", run.outcome());
        assertTrue(run.message(), run.message().contains("Plans.java:" + TEMP_LOOPS_LINE));
        assertEquals(0, run.ticks().size());
    }

    @Test
    public void aBenchOverAProjectNeedsTheProjectsSimulator() throws Exception {
        Path project = folder.getRoot().toPath();
        sourceRootWith(project, tempPlans(2));
        try {
            bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
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
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, 0.3, GRACE_SECONDS);

        assertTrue(bench.catalog().find("BlueTeleOp").isPresent());
        SimBench.Run run = await(bench.start(BLUE_TELEOP, "ada"));

        assertEquals(run.message() + "\n" + run.log(), "done", run.outcome());
        assertTrue(run.log(), run.log().contains("this project's simulator"));
    }

    @Test
    public void aProjectWhoseSimulatorDoesNotFitItsRobotFailsTheBuildNamingTheSeam() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());

        edit(project, HARDWARE, "public static Builder builder()", "private static Builder builder()");
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

        try {
            bench.catalog();
            fail("a catalog cannot be listed for a simulator that does not fit the sources");
        } catch (SimBench.BuildFailed e) {
            assertTrue(e.getMessage(), e.getMessage().contains("does not fit"));
            assertTrue(e.getMessage(), e.getMessage().contains("sim/SimDevices.java"));
        }
        SimBench.Run run = await(bench.start(BLUE_TELEOP, "ada"));
        assertEquals(run.message(), "build failed", run.outcome());
        assertTrue(run.message(), run.message().contains("sim/SimDevices.java"));
        assertEquals(0, run.ticks().size());
    }

    @Test
    public void aChildThatCannotListTheOpModesSaysWhatItPrinted() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        edit(
                project,
                SIM_CHILD,
                "            protocol.println(GSON.toJson(catalog(args, 1).toJson()));",
                "            if (args.length > 0) throw new IllegalStateException(\"no catalog today\");");
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

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
    public void aClassTheProjectLacksIsMissingNotThisServers() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        Files.delete(project.resolve("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/BlueTeleOp.java"));
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

        SimCatalog catalog = bench.catalog();

        assertTrue(catalog.find("RedTeleOp").isPresent());
        assertFalse(
                "this server has a BlueTeleOp; the project does not",
                catalog.find("BlueTeleOp").isPresent());
    }

    static void placesItself(Path project) throws IOException {
        edit(
                project,
                SIM_CHILD,
                "driverStation.awaitPlacement()",
                "java.util.Optional.of(new com.acmerobotics.roadrunner.Pose2d(0, 0, 0))");
    }

    @Test
    public void aChildThatPrintsNoHelloIsAVersionOneChildAndStillRuns() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        edit(project, SIM_CHILD, "protocol.println(SimRunStream.hello());", "");
        placesItself(project);
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, 0.3, GRACE_SECONDS);
        exactRobot("BlueTeleOp");

        assertTrue(bench.catalog().find("BlueTeleOp").isPresent());
        SimBench.Run run = await(bench.start(BLUE_TELEOP, "ada"));

        assertEquals(run.message() + "\n" + run.log(), "done", run.outcome());
        assertTrue(run.ticks().size() > 0);
        assertEquals(0, run.ticks().get(0).getAsJsonObject().get("x").getAsDouble(), 0.001);
    }

    @Test
    public void aChildFromBeforePlacementRunsFromTheOriginAndIsRefusedAnywhereElse() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        int before = SimRunStream.PLACED_PROTOCOL - 1;
        edit(project, SIM_RUN_STREAM, "PROTOCOL = " + SimRunStream.PROTOCOL + ";", "PROTOCOL = " + before + ";");
        placesItself(project);
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, 0.3, GRACE_SECONDS);
        exactRobot("BlueTeleOp");

        SimBench.Run atTheOrigin = await(bench.start(BLUE_TELEOP, "ada"));
        assertEquals(atTheOrigin.message() + "\n" + atTheOrigin.log(), "done", atTheOrigin.outcome());
        assertTrue(atTheOrigin.ticks().size() > 0);

        assertEquals(
                200, routes().handle(put("/start?opmode=BlueTeleOp", "{\"x\": 24, \"y\": 0, \"heading\": 0}")).status);
        SimBench.Run elsewhere = await(bench.start(BLUE_TELEOP, "ada"));

        assertEquals(SimRunStream.Outcome.cannotPlace(before), elsewhere.outcome());
        assertTrue(elsewhere.message(), elsewhere.message().toLowerCase().contains("pull"));
        assertTrue(elsewhere.message(), elsewhere.message().contains("protocol " + before));
        assertEquals(0, elsewhere.ticks().size());
        assertNull(bench.current());
    }

    @Test
    public void eachOpModeHasASeedTheCatalogShowsAndARouteSets() throws Exception {
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
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
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
        SimCatalog.Entry entry = bench.catalog().find("Count to three").get();
        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Count to three"), "{\"seed\": 5}")).status);

        SimBench.Run seeded = await(bench.start(entry, "ada"));
        assertEquals(Long.valueOf(5), seeded.seed);
        assertEquals("done", seeded.outcome());
        assertTrue(seeded.log(), seeded.log().contains(SimNoise.seeded(5).toString()));
        assertTrue(bench.status(), bench.status().contains("\"seed\":5"));

        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Count to three"), "{\"seed\": null}")).status);
        SimBench.Run exact = await(bench.start(entry, "ada"));
        assertNull(exact.seed);
        assertTrue(exact.log(), exact.log().contains(SimNoise.NONE.toString()));
        assertTrue(bench.status(), bench.status().contains("\"seed\":null"));
    }

    @Test
    public void aChildFromBeforeTheSeedRunsTheExactRobotAndIsRefusedASeed() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        int before = SimRunStream.SEEDED_PROTOCOL - 1;
        edit(project, SIM_RUN_STREAM, "PROTOCOL = " + SimRunStream.PROTOCOL + ";", "PROTOCOL = " + before + ";");
        edit(project, SIM_CHILD, "driverStation.seed()", "null");
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, 0.3, GRACE_SECONDS);
        assertEquals(200, routes().handle(put("/seed?opmode=BlueTeleOp", "{\"seed\": null}")).status);

        SimBench.Run exact = await(bench.start(BLUE_TELEOP, "ada"));
        assertEquals(exact.message() + "\n" + exact.log(), "done", exact.outcome());
        assertTrue(exact.ticks().size() > 0);

        assertEquals(200, routes().handle(put("/seed?opmode=BlueTeleOp", "{\"seed\": 3}")).status);
        SimBench.Run seeded = await(bench.start(BLUE_TELEOP, "ada"));

        assertEquals(SimRunStream.Outcome.cannotSeed(before), seeded.outcome());
        assertTrue(seeded.message(), seeded.message().toLowerCase().contains("pull"));
        assertTrue(seeded.message(), seeded.message().contains("protocol " + before));
        assertEquals(0, seeded.ticks().size());
        assertNull(bench.current());
    }

    @Test
    public void aChildOfAnotherProtocolIsRefusedByNameNotMisread() throws Exception {
        Path project = realProjectCopiedUnder(folder.getRoot().toPath());
        int newer = SimRunStream.PROTOCOL + 1;
        edit(project, SIM_RUN_STREAM, "PROTOCOL = " + SimRunStream.PROTOCOL + ";", "PROTOCOL = " + newer + ";");
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, 0.3, GRACE_SECONDS);

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
    public void savedEditsTakeEffectOnTheNextRunWithoutARestart() throws Exception {
        Path project = projectWith(folder.getRoot().toPath(), tempPlans(2));
        bench = new SimBench(null, project, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
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
        bench = new SimBench(null, project, outputDir(), 1.0, TELEOP_SECONDS, GRACE_SECONDS);
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
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada"));

        Response slashless = routes().handle(get("/runs/" + run.id));
        Response mounted = routes().handle(get("/runs/" + run.id + "/"));
        Response ticks = routes().handle(get("/runs/" + run.id + "/ticks?from=0"));

        assertEquals(308, slashless.status);
        assertEquals("/runs/" + run.id + "/", slashless.headers.get("Location"));
        assertEquals(200, mounted.status);
        assertEquals(200, ticks.status);
    }

    private SimBench benchWith(FakeChild child, double graceSeconds) {
        return benchWith(child, graceSeconds, 60);
    }

    private SimBench benchWith(FakeChild child, double graceSeconds, double startupSeconds) {
        return new SimBench(
                SimCatalog.of(TestTeleOps.StickTeleOp.class),
                null,
                outputDir(),
                TIMEOUT_SECONDS,
                TELEOP_SECONDS,
                graceSeconds,
                child,
                startupSeconds);
    }

    @Test
    public void aChildThatGoesSilentWhileItsRunIsUnfinishedIsKilledForNotReturning() throws Exception {
        FakeChild child =
                FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started()).thatStaysAliveSayingNothingMore();
        bench = new SimBench(
                SimCatalog.of(TestTeleOps.StickTeleOp.class),
                null,
                outputDir(),
                TIMEOUT_SECONDS,
                TELEOP_SECONDS,
                GRACE_SECONDS,
                child,
                60,
                new FakeClock());

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("the op mode did not return"));
    }

    @Test
    public void aChildThatCannotBeStartedEndsTheRunSayingSo() throws Exception {
        bench = benchWith(new FakeChild().thatWillNotStart("no java on this machine"), GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada"));

        assertEquals(SimRunStream.Outcome.couldNotStartChild(), run.outcome());
        assertTrue(run.message(), run.message().contains("no java on this machine"));
    }

    @Test
    public void aChildThatNeverSaysTheOpModeStartedIsKilledForNeverStarting() throws Exception {
        bench = benchWith(
                FakeChild.thatSays(SimRunStream.hello()).thatStaysAliveSayingNothingMore(), GRACE_SECONDS, 0.3);

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada"));

        assertTrue(run.outcome(), run.outcome().startsWith("killed"));
        assertTrue(run.outcome(), run.outcome().contains("never started"));
    }

    @Test
    public void aChildThatIgnoresStopIsKilledAfterItsGrace() throws Exception {
        FakeChild child = FakeChild.thatSays(SimRunStream.hello(), SimRunStream.started())
                .thatStaysAliveSayingNothingMore()
                .thatIgnoresStop();
        bench = benchWith(child, 0.2);
        SimBench.Run run = bench.start(bench.catalog().find("Stick").get(), "ada");
        awaitRunning(run);

        run.stop();
        await(run);

        assertEquals(SimRunStream.Outcome.killedAfterStop(0.2), run.outcome());
        assertTrue(
                child.whatItWasTold().toString(), child.whatItWasTold().stream().anyMatch(t -> t.contains("stop")));
    }

    @Test
    public void aBenchWithoutSourcesHasNothingToCheck() {
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

        assertNull(bench.check());
    }

    @Test
    public void aTeleOpRunDrivesFromGamepadPostsAndStopEndsIt() throws Exception {
        bench = new SimBench(
                SimCatalog.of(StickTeleOp.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
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
    public void aTeleOpRunEndsDoneWhenItsPeriodIsOver() throws Exception {
        bench = new SimBench(SimCatalog.of(StickTeleOp.class), null, outputDir(), TIMEOUT_SECONDS, 0.3, GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Stick").get(), "ada"));

        assertEquals("done", run.outcome());
        assertTrue(run.ticks().size() > 1);
    }

    @Test
    public void aRunIsNotKilledForTakingLongerInRealTimeThanItsPeriodLastsInSimulatedTime() throws Exception {
        bench = new SimBench(
                SimCatalog.of(TestTeleOps.SlowMachineTeleOp.class),
                null,
                outputDir(),
                TIMEOUT_SECONDS,
                0.3,
                GRACE_SECONDS);

        SimBench.Run run =
                await(bench.start(bench.catalog().find("Slow machine").get(), "ada"));

        assertEquals(run.message() + "\n" + run.log(), "done", run.outcome());
        assertTrue(run.ticks().size() > 1);
    }

    @Test
    public void stopEndsAnAutoRunToo() throws Exception {
        bench = new SimBench(
                SimCatalog.of(TestAutos.NeverDoneAuto.class), null, outputDir(), 30, TELEOP_SECONDS, GRACE_SECONDS);
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
        bench = new SimBench(
                SimCatalog.of(HangingAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);

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
    public void theStartPoseIsRememberedPerOpModeAndTheRunStartsThere() throws Exception {
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class, TestAutos.NeverDoneAuto.class),
                null,
                outputDir(),
                TIMEOUT_SECONDS,
                TELEOP_SECONDS,
                GRACE_SECONDS);
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

        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Count to three"), "{\"seed\": null}")).status);
        assertEquals(200, routes().handle(put("/seed?opmode=" + encode("Never done"), "{\"seed\": null}")).status);
        SimBench.Run run =
                await(bench.start(bench.catalog().find("Count to three").get(), "ada"));
        assertEquals(run.message(), "done", run.outcome());
        com.google.gson.JsonObject first = run.ticks().get(0).getAsJsonObject();
        assertEquals(-60, first.get("x").getAsDouble(), 0.001);
        assertEquals(limitAt(1.5), first.get("y").getAsDouble(), 0.001);
        assertEquals(1.5, first.get("heading").getAsDouble(), 0.001);
        com.google.gson.JsonObject origin = await(
                        bench.start(bench.catalog().find("Never done").get(), "ada"))
                .ticks()
                .get(0)
                .getAsJsonObject();
        assertEquals(0, origin.get("x").getAsDouble(), 0.001);

        bench.stop();
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
        assertEquals(
                "remembered across a restart",
                placed.body,
                routes().handle(get("/start?opmode=" + encode("Count to three"))).body);
    }

    @Test
    public void aStartThatIsNotAPoseIsRefusedAndNothingIsRemembered() throws Exception {
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
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
        bench = new SimBench(
                SimCatalog.of(ThreeLoopAuto.class), null, outputDir(), TIMEOUT_SECONDS, TELEOP_SECONDS, GRACE_SECONDS);
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
        bench = new SimBench(
                SimCatalog.of(TestAutos.ChattyAuto.class),
                null,
                outputDir(),
                TIMEOUT_SECONDS,
                TELEOP_SECONDS,
                GRACE_SECONDS);

        SimBench.Run run = await(bench.start(bench.catalog().find("Chatty").get(), "ada"));

        assertTrue(run.log(), run.log().contains("hello from the op mode"));
    }
}
