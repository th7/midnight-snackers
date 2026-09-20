package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

final class GitFixture {
    private GitFixture() {}

    /**
     * Where the repository every test starts from is made, once. Making one costs four git
     * processes and two hundred tests want the same one, so it is made here and copied after: a
     * copy of a fresh repository is a fresh repository, and copying forty small files is not worth
     * forking for.
     */
    private static final Path TEMPLATE = Paths.get("build", "sim", "git-template");

    private static boolean made;

    static void init(Path root) throws IOException {
        SimProject.copyTree(template(), root);
    }

    private static synchronized Path template() throws IOException {
        if (made) {
            return TEMPLATE;
        }
        deleteTree(TEMPLATE);
        Files.createDirectories(TEMPLATE);
        git(TEMPLATE, "init", "-q", "-b", Worktrees.DEVELOP);
        Files.write(TEMPLATE.resolve("README"), "hello\n".getBytes(StandardCharsets.UTF_8));
        commitAll(TEMPLATE, "first");
        made = true;
        return TEMPLATE;
    }

    private static void deleteTree(Path root) throws IOException {
        if (!Files.isDirectory(root)) {
            return;
        }
        try (java.util.stream.Stream<Path> walk = Files.walk(root)) {
            for (Path path : walk.sorted(java.util.Comparator.reverseOrder()).toList()) {
                Files.deleteIfExists(path);
            }
        }
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
        try (Cost.Spent spent = Cost.start(Cost.Kind.GIT)) {
            return running(cwd, args);
        }
    }

    private static String running(Path cwd, String... args) throws IOException {
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
