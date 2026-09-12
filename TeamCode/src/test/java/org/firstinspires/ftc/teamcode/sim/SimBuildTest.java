package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.FileTime;

public class SimBuildTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private Path sourceRoot;
    private Path buildRoot;

    private SimBuild build() throws IOException {
        sourceRoot = folder.getRoot().toPath().resolve("src/main/java");
        Files.createDirectories(sourceRoot);
        buildRoot = folder.getRoot().toPath().resolve("build/sim/classes");
        return new SimBuild(sourceRoot, buildRoot);
    }

    private Path write(String relative, String source) throws IOException {
        Path file = sourceRoot.resolve(relative);
        Files.createDirectories(file.getParent());
        Files.write(file, source.getBytes(StandardCharsets.UTF_8));
        return file;
    }

    private static final String GREETER = "package demo;\n"
            + "public class Greeter { public String greet() { return \"hi\"; } }\n";

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
    public void theRealMainSourcesCompile() throws IOException {
        Path real = Paths.get("src/main/java").toAbsolutePath();
        assertTrue("tests run from the TeamCode module directory: " + real, Files.isDirectory(real));
        SimBuild build = new SimBuild(real, folder.getRoot().toPath().resolve("classes"));

        SimBuild.Result result = build.build();

        assertNotNull(result.diagnostics, result.classes);
        assertTrue(Files.isRegularFile(result.classes.resolve("org/firstinspires/ftc/teamcode/base/AutoOp.class")));
        assertTrue(Files.isRegularFile(result.classes.resolve("org/firstinspires/ftc/teamcode/auto/DriveForward.class")));
    }
}
