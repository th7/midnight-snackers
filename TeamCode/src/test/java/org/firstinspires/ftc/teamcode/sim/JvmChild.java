package org.firstinspires.ftc.teamcode.sim;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalInt;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

/**
 * Starts the child JVM a simulation runs in. What runs there is a teammate's robot code, so the
 * child is fenced against the accidents robot code makes on a laptop -- not against anyone trying:
 * it stands in its own scratch directory, which goes when it does; it is given none of the server's
 * environment beyond what a JVM needs; its heap is capped; what it starts is killed with it; and no
 * more of them run at once than this was made to allow. JvmChildTest holds each.
 */
public final class JvmChild implements Child {

    /** The most heap one child may hold. A run needs a fraction of it. */
    public static final int HEAP_MB = 512;

    /** What of the server's environment a child is given: what a JVM needs to start, on each OS. */
    public static final Set<String> PASSED_THROUGH = Set.of("PATH", "SystemRoot", "windir", "LANG", "LC_ALL", "TZ");

    /** Where a process looks for home and for scratch space: the child's own scratch directory. */
    public static final Set<String> SET_TO_SCRATCH = Set.of("HOME", "TMPDIR", "TMP", "TEMP");

    /**
     * Every child any JvmChild in this JVM started that has not ended, with its scratch directory,
     * so neither outlives this JVM.
     */
    private static final Map<Process, Path> LIVE = new HashMap<>();

    static {
        Runtime.getRuntime().addShutdownHook(new Thread(JvmChild::killEveryLiveChild, "sim-children"));
    }

    private final int atOnce;
    private final List<Process> mine = new ArrayList<>();

    /** As many children at once as the machine has processors, since a run keeps one busy. */
    public JvmChild() {
        this(Runtime.getRuntime().availableProcessors());
    }

    public JvmChild(int atOnce) {
        this.atOnce = atOnce;
    }

    private static final class Process1 implements Running {
        private final Process process;
        private final BufferedReader out;
        private final Thread log;

        Process1(Process process, Consumer<String> onLog) {
            this.process = process;
            this.out = new BufferedReader(new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8));
            this.log = new Thread(
                    () -> {
                        try (BufferedReader err = new BufferedReader(
                                new InputStreamReader(process.getErrorStream(), StandardCharsets.UTF_8))) {
                            for (String line = err.readLine(); line != null; line = err.readLine()) {
                                onLog.accept(line);
                            }
                        } catch (IOException ignored) {
                        }
                    },
                    "sim-child-log");
            this.log.setDaemon(true);
            this.log.start();
        }

        @Override
        public boolean say(String line) {
            try {
                OutputStream in = process.getOutputStream();
                in.write((line + "\n").getBytes(StandardCharsets.UTF_8));
                in.flush();
                return true;
            } catch (IOException e) {
                return false;
            }
        }

        @Override
        public String hear() {
            try {
                return out.readLine();
            } catch (IOException e) {
                return null;
            }
        }

        @Override
        public boolean alive() {
            return process.isAlive();
        }

        @Override
        public boolean endedWithin(double seconds) {
            try {
                return process.waitFor((long) (seconds * 1000), TimeUnit.MILLISECONDS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return !process.isAlive();
            }
        }

        @Override
        public void kill() {
            killTree(process);
        }

        @Override
        public OptionalInt exitCode() {
            return process.isAlive() ? OptionalInt.empty() : OptionalInt.of(process.exitValue());
        }

        @Override
        public void close() {
            try {
                out.close();
            } catch (IOException ignored) {
            }
            try {
                log.join(2000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private Running started(Supplier<Process> launch, Consumer<String> log) {
        synchronized (mine) {
            mine.removeIf(process -> !process.isAlive());
            if (mine.size() >= atOnce) {
                throw new CouldNotStart(
                        "the server is already running " + atOnce + " simulation children, as many as it may at"
                                + " once; try again when one ends",
                        null);
            }
            try {
                Process process = launch.get();
                mine.add(process);
                return new Process1(process, log);
            } catch (UncheckedIOException e) {
                throw new CouldNotStart("could not start the simulation child: " + e.getMessage(), e);
            }
        }
    }

    /** Kills what a child started, and then the child, so nothing it started is left without it. */
    private static void killTree(Process process) {
        ProcessTree.killDescendantsOf(process);
        process.destroyForcibly();
    }

    /** On this JVM's way out, where no reaper will get to run: each child's tree, then its scratch. */
    private static void killEveryLiveChild() {
        Map<Process, Path> live;
        synchronized (LIVE) {
            live = new HashMap<>(LIVE);
        }
        for (Process process : live.keySet()) {
            killTree(process);
        }
        for (Map.Entry<Process, Path> child : live.entrySet()) {
            try {
                child.getKey().waitFor(2, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            deleteTree(child.getValue());
        }
    }

    @Override
    public Running onTheClassesAt(Path classes, Consumer<String> log, String... args) {
        return started(() -> launch(classes, args), log);
    }

    @Override
    public Running onThisClasspath(Consumer<String> log, String... args) {
        return started(() -> launchOnThisClasspath(args), log);
    }

    /** A child on freshly built classes, with the libraries under them. */
    public static Process launch(Path classes, String... args) {
        List<String> classpath = new ArrayList<>();
        classpath.add(classes.toAbsolutePath().toString());
        classpath.addAll(SimBuild.libraries());
        return launchWith(classpath, args);
    }

    /** A child on the classpath this JVM is already running: the tests' own way in. */
    public static Process launchOnThisClasspath(String... args) {
        return launchWith(List.of(System.getProperty("java.class.path")), args);
    }

    private static Process launchWith(List<String> classpath, String... args) {
        Path scratch;
        try {
            scratch = Files.createTempDirectory("sim-child-");
        } catch (IOException e) {
            throw new UncheckedIOException("could not make the simulation child a scratch directory", e);
        }
        List<String> command = new ArrayList<>();
        command.add(Paths.get(System.getProperty("java.home"), "bin", "java").toString());
        command.add("-Xmx" + HEAP_MB + "m");
        command.add("-XX:+ExitOnOutOfMemoryError");
        command.add("-Duser.home=" + scratch);
        command.add("-Djava.io.tmpdir=" + scratch);
        command.add("-cp");
        command.add(String.join(File.pathSeparator, classpath));
        command.add(SimChild.class.getName());
        command.addAll(List.of(args));
        ProcessBuilder builder = new ProcessBuilder(command).directory(scratch.toFile());
        Map<String, String> environment = builder.environment();
        environment.keySet().retainAll(PASSED_THROUGH);
        for (String name : SET_TO_SCRATCH) {
            environment.put(name, scratch.toString());
        }
        Process process;
        try (Cost.Spent ignored = Cost.start(Cost.Kind.CHILD_JVM)) {
            process = builder.start();
        } catch (IOException e) {
            deleteTree(scratch);
            throw new UncheckedIOException("could not start the simulation child: " + command.get(0), e);
        }
        synchronized (LIVE) {
            LIVE.put(process, scratch);
        }
        Thread reaper = new Thread(
                () -> {
                    try {
                        process.waitFor();
                    } catch (InterruptedException e) {
                        return;
                    }
                    synchronized (LIVE) {
                        LIVE.remove(process);
                    }
                    deleteTree(scratch);
                },
                "sim-child-reaper");
        reaper.setDaemon(true);
        reaper.start();
        return process;
    }

    /** Deletes what a child left in its scratch directory; what cannot be deleted is left. */
    private static void deleteTree(Path root) {
        try (Stream<Path> paths = Files.walk(root)) {
            paths.sorted(Comparator.reverseOrder()).forEach(path -> {
                try {
                    Files.deleteIfExists(path);
                } catch (IOException ignored) {
                }
            });
        } catch (IOException | UncheckedIOException ignored) {
        }
    }
}
