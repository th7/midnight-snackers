package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.FileTime;
import java.util.List;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimBuildTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private Path sourceRoot;
    private Path harnessRoot;
    private Path buildRoot;

    private SimBuild build() throws IOException {
        sourceRoot = folder.getRoot().toPath().resolve("src/main/java");
        Files.createDirectories(sourceRoot);
        harnessRoot = folder.getRoot().toPath().resolve("src/test/java");
        Files.createDirectories(harnessRoot);
        buildRoot = folder.getRoot().toPath().resolve("build/sim/classes");
        return new SimBuild(sourceRoot, harnessRoot, buildRoot);
    }

    private Path write(String relative, String source) throws IOException {
        return writeUnder(sourceRoot, relative, source);
    }

    private Path writeHarness(String relative, String source) throws IOException {
        return writeUnder(harnessRoot, relative, source);
    }

    private static Path writeUnder(Path root, String relative, String source) throws IOException {
        Path file = root.resolve(relative);
        Files.createDirectories(file.getParent());
        Files.write(file, source.getBytes(StandardCharsets.UTF_8));
        return file;
    }

    /** A stand-in for the simulator's own sources: something that uses the robot's. */
    private static final String BENCH =
            "package demo;\n" + "public class Bench { String said = new Greeter().greet(); }\n";

    private static final String GREETER =
            "package demo;\n" + "public class Greeter { public String greet() { return \"hi\"; } }\n";

    @Test
    public void compilesASourceTreeIntoAFreshDirectory() throws IOException {
        SimBuild build = build();
        write("demo/Greeter.java", GREETER);

        SimBuild.Result result = build.build();

        assertNotNull(result.diagnostics, result.classes);
        assertTrue(result.rebuilt);
        assertTrue(Files.isRegularFile(result.classes.resolve("demo/Greeter.class")));
        assertTrue(result.classes.startsWith(buildRoot));
    }

    @Test
    public void aCompileErrorReportsTheFileAndLine() throws IOException {
        SimBuild build = build();
        write("demo/Broken.java", "package demo;\n\npublic class Broken { int x = ; }\n");

        SimBuild.Result result = build.build();

        assertNull(result.classes);
        assertTrue(result.diagnostics, result.diagnostics.contains("Broken.java:3"));
        assertTrue(result.diagnostics, result.diagnostics.contains("illegal start of expression"));
        assertEquals(1, result.problems.size());
        assertEquals("demo/Broken.java", result.problems.get(0).file);
        assertEquals(3, result.problems.get(0).line);
        assertTrue(
                result.problems.get(0).message, result.problems.get(0).message.contains("illegal start of expression"));
    }

    @Test
    public void aGoodBuildHasNoProblems() throws IOException {
        SimBuild build = build();
        write("demo/Greeter.java", GREETER);

        assertTrue(build.build().problems.isEmpty());
    }

    @Test
    public void anUnchangedTreeIsNotRecompiledAndAnEditedOneIs() throws IOException {
        SimBuild build = build();
        Path greeter = write("demo/Greeter.java", GREETER);
        Path first = build.build().classes;

        SimBuild.Result again = build.build();
        assertFalse(again.rebuilt);
        assertEquals(first, again.classes);

        Files.write(greeter, GREETER.replace("hi", "hello").getBytes(StandardCharsets.UTF_8));
        Files.setLastModifiedTime(greeter, FileTime.fromMillis(System.currentTimeMillis() + 2000));
        SimBuild.Result edited = build.build();
        assertTrue(edited.rebuilt);
        assertNotEquals(first, edited.classes);
        assertFalse("only the latest build is kept", Files.exists(first));
    }

    @Test
    public void aKotlinFileFailsTheBuildByName() throws IOException {
        SimBuild build = build();
        write("demo/Greeter.java", GREETER);
        Path kotlin = folder.getRoot().toPath().resolve("src/main/kotlin/demo/Extra.kt");
        Files.createDirectories(kotlin.getParent());
        Files.write(kotlin, "package demo\nclass Extra\n".getBytes(StandardCharsets.UTF_8));

        try {
            build.build();
            fail("a Kotlin file must fail the build, never be skipped");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("Extra.kt"));
        }
    }

    @Test
    public void theSimulatorsOwnSourcesAreCompiledWithTheRobotsButNotItsTests() throws IOException {
        SimBuild build = build();
        write("demo/Greeter.java", GREETER);
        writeHarness("demo/Bench.java", BENCH);
        writeHarness("demo/BenchTest.java", "package demo;\n\npublic class BenchTest { int x = ; }\n");

        SimBuild.Result result = build.build();

        assertNotNull(result.diagnostics, result.classes);
        assertTrue(Files.isRegularFile(result.classes.resolve("demo/Greeter.class")));
        assertTrue(
                "the simulator runs in the child, so it is built with the robot",
                Files.isRegularFile(result.classes.resolve("demo/Bench.class")));
        assertFalse("its tests are not the simulator", Files.exists(result.classes.resolve("demo/BenchTest.class")));
    }

    @Test
    public void aSimulatorThatDoesNotFitTheRobotSourcesFailsTheBuildNamingTheSeam() throws IOException {
        SimBuild build = build();
        write("demo/Greeter.java", GREETER.replace("greet", "hello"));
        writeHarness("demo/Bench.java", BENCH);

        SimBuild.Result result = build.build();

        assertNull("the child would fail at run time in a way no one could read", result.classes);
        assertEquals(result.diagnostics, 2, result.problems.size());
        SimBuild.Problem why = result.problems.get(0);
        assertEquals("", why.file);
        assertTrue(why.message, why.message.contains("simulator") && why.message.contains("does not fit"));
        SimBuild.Problem seam = result.problems.get(1);
        assertEquals("not a file the user can open", "", seam.file);
        assertTrue(seam.message, seam.message.contains("simulator demo/Bench.java:2"));
        assertTrue(seam.message, seam.message.contains("cannot find symbol"));
        assertTrue(result.diagnostics, result.diagnostics.contains("does not fit"));
        assertTrue(result.diagnostics, result.diagnostics.contains("simulator demo/Bench.java:2"));
    }

    @Test
    public void anEditedSimulatorSourceIsRecompiled() throws IOException {
        SimBuild build = build();
        write("demo/Greeter.java", GREETER);
        Path bench = writeHarness("demo/Bench.java", BENCH);
        Path first = build.build().classes;
        assertFalse(build.build().rebuilt);

        Files.write(bench, BENCH.replace("said", "heard").getBytes(StandardCharsets.UTF_8));
        Files.setLastModifiedTime(bench, FileTime.fromMillis(System.currentTimeMillis() + 2000));
        SimBuild.Result edited = build.build();

        assertTrue(edited.rebuilt);
        assertNotEquals(first, edited.classes);
    }

    @Test
    public void theSimulatorsResourcesRideAlongWithItsClasses() throws IOException {
        SimBuild build = build();
        write("demo/Greeter.java", GREETER);
        Path page = folder.getRoot().toPath().resolve("src/test/resources/demo/page.html");
        Files.createDirectories(page.getParent());
        Files.write(page, "<p>one</p>".getBytes(StandardCharsets.UTF_8));

        SimBuild.Result result = build.build();

        assertNotNull(result.diagnostics, result.classes);
        assertEquals(
                "<p>one</p>",
                new String(Files.readAllBytes(result.classes.resolve("demo/page.html")), StandardCharsets.UTF_8));
        assertFalse(build.build().rebuilt);

        Files.write(page, "<p>two</p>".getBytes(StandardCharsets.UTF_8));
        Files.setLastModifiedTime(page, FileTime.fromMillis(System.currentTimeMillis() + 2000));
        SimBuild.Result edited = build.build();
        assertTrue("an edited resource is a change to the simulator", edited.rebuilt);
        assertEquals(
                "<p>two</p>",
                new String(Files.readAllBytes(edited.classes.resolve("demo/page.html")), StandardCharsets.UTF_8));
    }

    /** What a project is built against and run with: this JVM's jars, none of this server's own code. */
    @Test
    public void theLibrariesAreThisJvmsJarsWithoutThisServersOwnCode() {
        List<String> libraries = SimBuild.libraries();
        String gson = locationOf(com.google.gson.Gson.class);
        String serverRobot = locationOf(org.firstinspires.ftc.teamcode.base.OpMode.class);
        String serverSimulator = locationOf(SimBuild.class);

        assertTrue(libraries.toString(), libraries.contains(gson));
        assertFalse("this server's robot classes: " + serverRobot, libraries.contains(serverRobot));
        assertFalse("this server's simulator: " + serverSimulator, libraries.contains(serverSimulator));
        for (String library : libraries) {
            assertTrue(library, Files.isRegularFile(Paths.get(library)));
        }
    }

    @Test
    public void librariesOfKeepsOnlyExistingFilesThatAreNotThisServersRobotClasses() throws IOException {
        Path dir = Files.createDirectories(folder.getRoot().toPath().resolve("classes"));
        Path library = Files.createFile(folder.getRoot().toPath().resolve("lib.jar"));
        Path robot = Files.createFile(folder.getRoot().toPath().resolve("robot.jar"));
        Path gone = folder.getRoot().toPath().resolve("gone");

        List<String> libraries = SimBuild.librariesOf(
                List.of(dir.toString(), library.toString(), robot.toString(), gone.toString()), robot);

        assertEquals(List.of(library.toString()), libraries);
    }

    private static String locationOf(Class<?> type) {
        try {
            return Paths.get(type.getProtectionDomain()
                            .getCodeSource()
                            .getLocation()
                            .toURI())
                    .toString();
        } catch (java.net.URISyntaxException e) {
            throw new IllegalStateException(e);
        }
    }

    @Test
    public void aMissingSimulatorSourceRootIsRefusedByName() throws IOException {
        build();
        Path missing = folder.getRoot().toPath().resolve("nowhere");
        try {
            new SimBuild(sourceRoot, missing, buildRoot);
            fail("a build that silently ran without the simulator's sources would run a stale simulator");
        } catch (IllegalArgumentException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("nowhere"));
        }
    }

    @Test
    public void theRealMainSourcesCompile() throws IOException {
        Path real = Paths.get("src/main/java").toAbsolutePath();
        assertTrue("tests run from the TeamCode module directory: " + real, Files.isDirectory(real));
        SimBuild build =
                new SimBuild(real, emptyHarness(), folder.getRoot().toPath().resolve("classes"));

        SimBuild.Result result = build.build();

        assertNotNull(result.diagnostics, result.classes);
        assertTrue(Files.isRegularFile(result.classes.resolve("org/firstinspires/ftc/teamcode/base/AutoOp.class")));
        assertTrue(Files.isRegularFile(result.classes.resolve("org/firstinspires/ftc/teamcode/PlanOpModes.class")));
    }

    @Test
    public void theRealSimulatorCompilesWithTheRealMainSources() throws IOException {
        Path real = Paths.get("src/main/java").toAbsolutePath();
        Path harness = Paths.get("src/test/java").toAbsolutePath();
        assertTrue("tests run from the TeamCode module directory: " + harness, Files.isDirectory(harness));
        SimBuild build = new SimBuild(real, harness, folder.getRoot().toPath().resolve("classes"));

        SimBuild.Result result = build.build();

        assertNotNull(result.diagnostics, result.classes);
        Path teamCode = result.classes.resolve("org/firstinspires/ftc/teamcode");
        assertTrue(Files.isRegularFile(teamCode.resolve("sim/SimChild.class")));
        assertTrue(Files.isRegularFile(teamCode.resolve("sim/SimRobot.class")));
        assertTrue(Files.isRegularFile(teamCode.resolve("fakes/FakeDcMotorEx.class")));
        assertFalse(Files.exists(teamCode.resolve("sim/SimBuildTest.class")));
    }

    private Path emptyHarness() throws IOException {
        return Files.createDirectories(folder.getRoot().toPath().resolve("src/test/java"));
    }
}
