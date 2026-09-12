package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/** Temp repositories for the tests: made with the real git, with a {@code develop} branch and a commit on it. */
final class GitFixture {
    private GitFixture() {
    }

    /** A new repository at {@code root} whose only branch is {@code develop}, with one commit holding {@code README}. */
    static void init(Path root) throws IOException {
        Files.createDirectories(root);
        git(root, "init", "-q", "-b", Worktrees.DEVELOP);
        Files.write(root.resolve("README"), "hello\n".getBytes(StandardCharsets.UTF_8));
        commitAll(root, "first");
    }

    /** Commits everything under {@code cwd}'s tree, and returns the new commit. */
    static String commitAll(Path cwd, String message) throws IOException {
        git(cwd, "add", "-A");
        git(cwd, "-c", "user.name=fixture", "-c", "user.email=fixture@example.invalid", "commit", "-q", "--allow-empty", "-m", message);
        return head(cwd);
    }

    static String head(Path cwd) throws IOException {
        return git(cwd, "rev-parse", "HEAD").trim();
    }

    static String commitOf(Path cwd, String ref) throws IOException {
        return git(cwd, "rev-parse", ref).trim();
    }

    /** Runs git in {@code cwd}, asserting it succeeds, and returns its stdout. */
    static String git(Path cwd, String... args) throws IOException {
        List<String> command = new ArrayList<>();
        command.add("git");
        command.addAll(List.of(args));
        Process process = new ProcessBuilder(command).directory(cwd.toFile()).redirectErrorStream(false).start();
        String out;
        String err;
        try {
            out = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            err = new String(process.getErrorStream().readAllBytes(), StandardCharsets.UTF_8);
            process.waitFor();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IOException(e);
        }
        assertEquals("git " + String.join(" ", args) + " in " + cwd + "\n" + err, 0, process.exitValue());
        return out;
    }
}
