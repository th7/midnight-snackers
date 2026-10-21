package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.simcore.Deployment;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class GradleDeployTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static void write(Path file, String content) throws IOException {
        Files.write(file, content.getBytes(StandardCharsets.UTF_8));
    }

    private static String read(Path file) throws IOException {
        return new String(Files.readAllBytes(file), StandardCharsets.UTF_8);
    }

    private Path wrapper(String body) throws IOException {
        Path worktree = folder.newFolder("worktree with spaces").toPath();
        Path script = worktree.resolve("gradlew");
        write(script, "#!/bin/sh\n" + body + "\n");
        assertTrue(script.toFile().setExecutable(true));
        return worktree;
    }

    @Test
    public void gradleInstallsFromTheWorkingDirectoryIncludingUncommittedAndUntrackedFiles() throws IOException {
        Path worktree = wrapper("pwd\nprintf '%s\\n' \"$@\"\n"
                + "while IFS= read -r line; do printf '%s\\n' \"$line\"; done < unsaved.java\n"
                + "printf 'adb installed\\n' >&2");
        write(worktree.resolve("unsaved.java"), "uncommitted robot code\n");

        Deployment result = new GradleDeploy(folder.getRoot().toPath()).from(worktree);

        assertTrue(result.output(), result.ok());
        assertTrue(result.output(), result.output().contains(worktree.toString()));
        assertTrue(result.output(), result.output().contains(":TeamCode:installDebug"));
        assertTrue(result.output(), result.output().contains("--console=plain"));
        assertTrue(result.output(), result.output().contains("uncommitted robot code"));
        assertTrue(result.output(), result.output().contains("adb installed"));
    }

    @Test
    public void verboseBuildsKeepABoundedTailWithoutBlockingTheProcess() throws IOException {
        Path worktree = wrapper("i=0\nwhile [ \"$i\" -lt 10000 ]; do printf 'build output line\\n'; i=$((i+1)); done\n"
                + "printf 'the last diagnostic\\n'");

        Deployment result = new GradleDeploy(folder.getRoot().toPath()).from(worktree);

        assertTrue(result.output(), result.ok());
        assertTrue(result.output().length() <= 65536);
        assertTrue(result.output(), result.output().endsWith("the last diagnostic\n"));
    }

    @Test
    public void aCompileOrAdbFailureIsReportedWithBothOutputStreams() throws IOException {
        Path worktree = wrapper("printf 'compile output\\n'\nprintf 'adb: no devices\\n' >&2\nexit 7");

        Deployment result = new GradleDeploy(folder.getRoot().toPath()).from(worktree);

        assertFalse(result.ok());
        assertTrue(result.output(), result.output().contains("compile output"));
        assertTrue(result.output(), result.output().contains("adb: no devices"));
    }

    @Test
    public void aMissingWrapperCannotReportSuccess() {
        Deployment result = new GradleDeploy(folder.getRoot().toPath())
                .from(folder.getRoot().toPath());
        assertFalse(result.ok());
        assertTrue(result.output(), result.output().contains("gradlew"));
    }

    @Test
    public void aTimedOutGradleIsKilledAndTheNextPressCanTryAgain() throws Exception {
        Path worktree = wrapper("sleep 30 &\nprintf '%s\\n' \"$!\" > child\nprintf 'began\\n'\nwait");
        GradleDeploy deploy =
                new GradleDeploy(folder.getRoot().toPath(), Seconds.of(0.2).orElse(Seconds.zero()));

        Deployment result = deploy.from(worktree);

        assertFalse(result.ok());
        assertEquals(Deployment.Outcome.TIMED_OUT, result.outcome());
        assertTrue(result.output(), result.output().contains("began"));
        assertTrue(
                "the build left a child running",
                ProcessTree.endsWithin(
                        Long.parseLong(read(worktree.resolve("child")).trim()), 2));
        write(worktree.resolve("gradlew"), "#!/bin/sh\nprintf 'retried\\n'\n");
        assertTrue(deploy.from(worktree).ok());
    }

    @Test
    public void aSecondDeployIsRefusedRatherThanInstallingTwoWorktreesAtOnce() throws Exception {
        Path worktree = wrapper("printf 'began\\n' > began\nsleep 30");
        GradleDeploy deploy =
                new GradleDeploy(folder.getRoot().toPath(), Seconds.of(1).orElse(Seconds.zero()));
        CompletableFuture<Deployment> first = CompletableFuture.supplyAsync(() -> deploy.from(worktree));
        try {
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(5);
            while (!Files.exists(worktree.resolve("began")) && System.nanoTime() < deadline && !first.isDone()) {
                Thread.sleep(10);
            }
            assertTrue("the first deploy never started", Files.exists(worktree.resolve("began")));
            assertEquals(Deployment.Outcome.BUSY, deploy.from(worktree).outcome());
        } finally {
            assertEquals(
                    Deployment.Outcome.TIMED_OUT, first.get(5, TimeUnit.SECONDS).outcome());
        }
    }

    @Test
    public void anInterruptedDeployStopsItsProcessAndPreservesTheInterrupt() throws IOException {
        Path worktree = wrapper("printf 'started\\n'\nsleep 30");
        Thread.currentThread().interrupt();
        try {
            Deployment result = new GradleDeploy(folder.getRoot().toPath()).from(worktree);
            assertEquals(Deployment.Outcome.INTERRUPTED, result.outcome());
            assertTrue(Thread.currentThread().isInterrupted());
        } finally {
            Thread.interrupted();
        }
    }

    @Test
    public void theServersSdkLocationIsAvailableInAWorktreeWithoutLocalProperties() throws IOException {
        Path root = folder.newFolder("server").toPath();
        write(root.resolve("local.properties"), "sdk.dir=/the/host/android-sdk\n");
        Path worktree = wrapper("printf '%s\\n' \"$ANDROID_HOME\"");
        Deployment result = new GradleDeploy(root).from(worktree);
        assertTrue(result.output(), result.ok());
        assertEquals("/the/host/android-sdk", result.output().trim());
        assertFalse(Files.exists(worktree.resolve("local.properties")));
    }
}
