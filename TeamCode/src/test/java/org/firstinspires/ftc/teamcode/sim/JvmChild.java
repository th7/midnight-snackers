package org.firstinspires.ftc.teamcode.sim;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

public final class JvmChild implements Child {

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
            process.destroyForcibly();
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

    private static Running started(java.util.function.Supplier<Process> launch, Consumer<String> log) {
        try {
            return new Process1(launch.get(), log);
        } catch (UncheckedIOException e) {
            throw new CouldNotStart("could not start the simulation child: " + e.getMessage(), e);
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
        List<String> command = new ArrayList<>();
        command.add(Paths.get(System.getProperty("java.home"), "bin", "java").toString());
        command.add("-cp");
        command.add(String.join(File.pathSeparator, classpath));
        command.add(SimChild.class.getName());
        command.addAll(List.of(args));
        try (Cost.Spent ignored = Cost.start(Cost.Kind.CHILD_JVM)) {
            return new ProcessBuilder(command).start();
        } catch (IOException e) {
            throw new UncheckedIOException("could not start the simulation child: " + command.get(0), e);
        }
    }
}
