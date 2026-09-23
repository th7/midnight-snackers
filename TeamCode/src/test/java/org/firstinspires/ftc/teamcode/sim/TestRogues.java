package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.TestAutos.TestAuto;

/**
 * Stand-ins for a teammate's op mode that does what nobody meant it to: writes files where it
 * stands, reads the environment, starts processes, eats memory. They break the rules the server's
 * own code is held to, on purpose, so the child can be shown to contain them.
 */
public final class TestRogues {
    private TestRogues() {}

    /** The name every stray write uses, so a test knows what to look for and what to clean up. */
    public static final String STRAY = "sim-child-stray.txt";

    @Autonomous(name = "Stray", group = "Test")
    public static class StrayAuto extends TestAuto {
        @Override
        public PlanPart getPlan() {
            return new Step(
                    "write where it stands",
                    () -> {
                        System.err.println("cwd " + Paths.get("").toAbsolutePath());
                        write(Paths.get(STRAY));
                        write(Paths.get(System.getProperty("user.home"), STRAY));
                        write(Paths.get(System.getProperty("java.io.tmpdir"), STRAY));
                    },
                    () -> true);
        }

        private static void write(Path path) {
            try {
                Files.write(path, "left behind".getBytes(StandardCharsets.UTF_8));
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
    }

    @Autonomous(name = "Environment", group = "Test")
    public static class EnvironmentAuto extends TestAuto {
        @Override
        public PlanPart getPlan() {
            return new Step(
                    "say the environment",
                    () -> {
                        for (Map.Entry<String, String> variable :
                                System.getenv().entrySet()) {
                            System.err.println("env " + variable.getKey() + "=" + variable.getValue());
                        }
                    },
                    () -> true);
        }
    }

    /** Says where it stands, then never returns from its step. */
    @Autonomous(name = "Hangs where it stands", group = "Test")
    public static class HangingWhereItStandsAuto extends TestAuto {
        @Override
        public PlanPart getPlan() {
            return new Step(
                    "hang",
                    () -> {
                        System.err.println("cwd " + Paths.get("").toAbsolutePath());
                        while (true) {
                            try {
                                Thread.sleep(1000);
                            } catch (InterruptedException e) {
                            }
                        }
                    },
                    () -> false);
        }
    }

    /** Starts a process that would outlive it, then runs until it is stopped. */
    @Autonomous(name = "Spawns and runs", group = "Test")
    public static class SpawningAuto extends TestAuto {
        @Override
        public PlanPart getPlan() {
            return new Step("spawn", TestRogues::spawnASleeper, () -> false);
        }
    }

    /** Starts a process that would outlive it, then finishes. */
    @Autonomous(name = "Spawns and ends", group = "Test")
    public static class SpawnThenDoneAuto extends TestAuto {
        @Override
        public PlanPart getPlan() {
            return new Step("spawn", TestRogues::spawnASleeper, () -> true);
        }
    }

    @Autonomous(name = "Hog", group = "Test")
    public static class HogAuto extends TestAuto {
        public static final int CHUNK_MB = 16;

        @Override
        public PlanPart getPlan() {
            return new Step(
                    "hold everything",
                    () -> {
                        List<byte[]> held = new ArrayList<>();
                        while (true) {
                            held.add(new byte[CHUNK_MB << 20]);
                            System.err.println("held " + held.size() * CHUNK_MB);
                        }
                    },
                    () -> true);
        }
    }

    private static void spawnASleeper() {
        try (Cost.Spent ignored = Cost.start(Cost.Kind.CHILD_JVM)) {
            Process sleeper = new ProcessBuilder(
                            javaBin(), "-cp", System.getProperty("java.class.path"), Sleeper.class.getName())
                    .redirectErrorStream(true)
                    .start();
            System.err.println("spawned " + ProcessTree.pid(sleeper));
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private static String javaBin() {
        return Paths.get(System.getProperty("java.home"), "bin", "java").toString();
    }

    /** What a rogue starts: a process that would sit there for ten minutes. */
    public static final class Sleeper {
        public static void main(String[] args) throws InterruptedException {
            Thread.sleep(600_000);
        }
    }

    /**
     * A server in miniature: starts a child through {@link JvmChild}, lets its op mode hang, says
     * the child's pid and where it stands, and exits without stopping it.
     */
    public static final class Parent {
        public static void main(String[] args) throws Exception {
            java.util.concurrent.CompletableFuture<String> cwd = new java.util.concurrent.CompletableFuture<>();
            Child.Running child = new JvmChild()
                    .onThisClasspath(
                            line -> {
                                if (line.startsWith("cwd ")) {
                                    cwd.complete(line.substring("cwd ".length()));
                                }
                            },
                            "--run",
                            "Hangs where it stands",
                            "600",
                            Files.createTempDirectory("sim-parent").toString(),
                            HangingWhereItStandsAuto.class.getName());
            child.say(new Gson().toJson(SimDriverStation.startLine(new Pose2d(0, 0, 0))));
            for (String line = child.hear(); line != null; line = child.hear()) {
                if (line.contains("started")) {
                    break;
                }
            }
            ProcessTree.childrenOfThisJvm().forEach(pid -> System.out.println("child " + pid));
            System.out.println("scratch " + cwd.get(20, java.util.concurrent.TimeUnit.SECONDS));
            System.exit(0);
        }
    }

    /** Starts a {@link Parent}, the way the host starts the coding server. */
    public static Process startParent() throws IOException {
        try (Cost.Spent ignored = Cost.start(Cost.Kind.CHILD_JVM)) {
            return new ProcessBuilder(javaBin(), "-cp", System.getProperty("java.class.path"), Parent.class.getName())
                    .redirectError(ProcessBuilder.Redirect.INHERIT)
                    .start();
        }
    }
}
