package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

final class GitFixture {
    private GitFixture() {}

    static void init(Path root) throws IOException {
        Files.createDirectories(root);
        git(root, "init", "-q", "-b", Worktrees.DEVELOP);
        Files.write(root.resolve("README"), "hello\n".getBytes(StandardCharsets.UTF_8));
        commitAll(root, "first");
    }

    static void withOrigin(Path root, Path bare) throws IOException {
        Files.createDirectories(bare);
        git(bare, "init", "-q", "--bare");
        git(root, "remote", "add", "origin", bare.toString());
        git(root, "push", "-q", "origin", Worktrees.DEVELOP);
    }

    static String commitAll(Path cwd, String message) throws IOException {
        git(cwd, "add", "-A");
        git(
                cwd,
                "-c",
                "user.name=fixture",
                "-c",
                "user.email=fixture@example.invalid",
                "commit",
                "-q",
                "--allow-empty",
                "-m",
                message);
        return head(cwd);
    }

    static String head(Path cwd) throws IOException {
        return git(cwd, "rev-parse", "HEAD").trim();
    }

    static String commitOf(Path cwd, String ref) throws IOException {
        return git(cwd, "rev-parse", ref).trim();
    }

    static String git(Path cwd, String... args) throws IOException {
        List<String> command = new ArrayList<>();
        command.add("git");
        command.addAll(List.of(args));
        Process process = new ProcessBuilder(command)
                .directory(cwd.toFile())
                .redirectErrorStream(false)
                .start();
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
