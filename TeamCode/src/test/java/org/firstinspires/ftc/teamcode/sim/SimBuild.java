package org.firstinspires.ftc.teamcode.sim;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;
import org.firstinspires.ftc.teamcode.base.OpMode;

/**
 * Compiles the robot's main sources, as they are on disk right now, with the JDK's own compiler
 * against the {@link #libraries()}, and with them the simulator's own sources: everything under the
 * harness root that is not a test, with the resources next to it copied along. The simulator
 * runs in the child against the robot sources it was built with, so a simulator that does not
 * fit them fails the build naming the seam, rather than the child failing at run time with a
 * linkage error nobody can read. About a second for the whole tree, so a simulated run can
 * always execute what was last saved. Output goes to a fresh directory under the build root each
 * time any of the trees changes; only the latest is kept. This is not the Android build: no
 * Kotlin, no desugaring, no annotation processing. A Kotlin file is refused by name rather than
 * skipped.
 */
public final class SimBuild {
    /** What a compile error in the simulator's own sources means, said once ahead of them. */
    static final String SIMULATOR_DOES_NOT_FIT =
            "the simulator does not fit these robot sources: it was built from the code"
                    + " the server runs from, and these sources came from another version of it. Pull develop; if that does not help,"
                    + " the coach brings develop and the server's checkout to the same code and restarts the server.";

    /**
     * One compiler error: the file relative to the source root, its line, and the message. A
     * problem in the simulator's own sources has no file the user can open: the file is empty
     * and the message names it.
     */
    public static final class Problem {
        public final String file;
        public final long line;
        public final String message;

        Problem(String file, long line, String message) {
            this.file = file;
            this.line = line;
            this.message = message;
        }
    }

    public static final class Result {
        /** The compiled classes, or null when the build failed. */
        public final Path classes;
        /** Compiler errors when the build failed, otherwise empty. */
        public final String diagnostics;

        public final List<Problem> problems;
        /** False when the sources had not changed and the previous output was reused. */
        public final boolean rebuilt;

        Result(Path classes, List<Problem> problems, boolean rebuilt) {
            this.classes = classes;
            this.problems = Collections.unmodifiableList(problems);
            this.rebuilt = rebuilt;
            StringBuilder text = new StringBuilder();
            for (Problem problem : problems) {
                if (text.length() > 0) {
                    text.append('\n');
                }
                text.append(problem.file.isEmpty() ? "" : problem.file + ":" + problem.line + ": ")
                        .append(problem.message);
            }
            this.diagnostics = text.toString();
        }
    }

    private final Path sourceRoot;
    private final Path harnessRoot;
    /** The simulator's resources, next to its sources ({@code src/test/resources}); copied into the output as they are. */
    private final Path resourcesRoot;

    private final Path buildRoot;
    private String lastFingerprint;
    private Result lastResult;
    private int builds = 0;

    /**
     * @param sourceRoot  the package root, e.g. {@code TeamCode/src/main/java}
     * @param harnessRoot the package root of the simulator's own sources, e.g. {@code TeamCode/src/test/java}
     * @param buildRoot   where compiled output goes, e.g. {@code TeamCode/build/sim/classes}
     * @throws IllegalArgumentException when there are no simulator sources at {@code harnessRoot}
     */
    public SimBuild(Path sourceRoot, Path harnessRoot, Path buildRoot) {
        this.sourceRoot = sourceRoot.toAbsolutePath().normalize();
        this.harnessRoot = harnessRoot.toAbsolutePath().normalize();
        this.resourcesRoot = this.harnessRoot.resolveSibling("resources");
        this.buildRoot = buildRoot.toAbsolutePath().normalize();
        if (!Files.isDirectory(this.harnessRoot)) {
            throw new IllegalArgumentException("no simulator sources at " + this.harnessRoot);
        }
    }

    public Path sourceRoot() {
        return sourceRoot;
    }

    /**
     * What a project is built against and run with: the libraries on this JVM's classpath, and
     * none of this server's own code. The server's code is the directories on its classpath (its
     * simulator and tests) and the jar its robot classes come from; the FTC SDK, Road Runner, and
     * the FtcRobotController module's jar, which no project rebuilds, are libraries. So a class a
     * project lacks is missing, in its build and in its child, rather than quietly this server's.
     */
    public static List<String> libraries() {
        return librariesOf(
                List.of(System.getProperty("java.class.path").split(Pattern.quote(File.pathSeparator))),
                locationOf(OpMode.class));
    }

    /** {@code classpath} without its directories, its entries that do not exist, and {@code serverRobotClasses}. */
    static List<String> librariesOf(List<String> classpath, Path serverRobotClasses) {
        Path robot = serverRobotClasses.toAbsolutePath().normalize();
        List<String> libraries = new ArrayList<>();
        for (String entry : classpath) {
            Path path = Paths.get(entry).toAbsolutePath().normalize();
            if (Files.isRegularFile(path) && !path.equals(robot)) {
                libraries.add(entry);
            }
        }
        return libraries;
    }

    /** Where a class was loaded from: a jar, or a directory of classes. */
    private static Path locationOf(Class<?> type) {
        try {
            return Paths.get(
                    type.getProtectionDomain().getCodeSource().getLocation().toURI());
        } catch (URISyntaxException e) {
            throw new IllegalStateException("no location for " + type, e);
        }
    }

    public synchronized Result build() {
        List<Path> sources = sourcesUnder(sourceRoot);
        List<Path> harness = harnessUnder(harnessRoot);
        List<Path> resources = filesUnder(resourcesRoot);
        String fingerprint = fingerprintOf(sourceRoot, sources)
                + fingerprintOf(harnessRoot, harness)
                + fingerprintOf(resourcesRoot, resources);
        if (fingerprint.equals(lastFingerprint) && lastResult != null) {
            return new Result(lastResult.classes, lastResult.problems, false);
        }
        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        if (compiler == null) {
            throw new IllegalStateException("this JVM has no Java compiler; run the server on a JDK, not a JRE");
        }
        Path output = buildRoot.resolve("build-" + (++builds));
        try {
            Files.createDirectories(output);
        } catch (IOException e) {
            throw new UncheckedIOException("could not create " + output, e);
        }
        DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<>();
        boolean ok;
        try (StandardJavaFileManager files =
                compiler.getStandardFileManager(diagnostics, null, StandardCharsets.UTF_8)) {
            List<String> options = List.of(
                    "-d",
                    output.toString(),
                    "-cp",
                    String.join(File.pathSeparator, libraries()),
                    "--release",
                    "17",
                    "-proc:none",
                    "-nowarn",
                    "-encoding",
                    "UTF-8");
            List<java.io.File> sourceFiles = new ArrayList<>();
            for (Path source : sources) {
                sourceFiles.add(source.toFile());
            }
            for (Path source : harness) {
                sourceFiles.add(source.toFile());
            }
            Iterable<? extends JavaFileObject> units = files.getJavaFileObjectsFromFiles(sourceFiles);
            ok = sourceFiles.isEmpty()
                    || compiler.getTask(null, files, diagnostics, options, null, units)
                            .call();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        Result result;
        if (ok) {
            copyResources(resources, output);
            Path previous = lastResult == null ? null : lastResult.classes;
            result = new Result(output, List.of(), true);
            if (previous != null) {
                deleteTree(previous);
            }
        } else {
            deleteTree(output);
            result = new Result(null, problems(diagnostics), true);
        }
        lastFingerprint = fingerprint;
        lastResult = result;
        return result;
    }

    /** Every .java file under the source root, sorted by path; refuses a tree with Kotlin next to it. */
    static List<Path> sourcesUnder(Path sourceRoot) {
        Path mainDir = sourceRoot.getParent() == null ? sourceRoot : sourceRoot.getParent();
        try (Stream<Path> walk = Files.walk(mainDir)) {
            List<Path> kotlin = walk.filter(p -> p.toString().endsWith(".kt")).collect(Collectors.toList());
            if (!kotlin.isEmpty()) {
                throw new IllegalStateException(
                        "cannot build for the simulator: Kotlin sources are not compiled here: " + kotlin);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        try (Stream<Path> walk = Files.walk(sourceRoot)) {
            return walk.filter(p -> p.toString().endsWith(".java") && Files.isRegularFile(p))
                    .sorted(Comparator.comparing(Path::toString))
                    .collect(Collectors.toList());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /**
     * Every .java file under the harness root that is not a test, sorted by path: the simulator
     * the child runs, without what only tests it.
     */
    static List<Path> harnessUnder(Path harnessRoot) {
        try (Stream<Path> walk = Files.walk(harnessRoot)) {
            return walk.filter(p -> p.toString().endsWith(".java")
                            && !p.getFileName().toString().endsWith("Test.java")
                            && Files.isRegularFile(p))
                    .sorted(Comparator.comparing(Path::toString))
                    .collect(Collectors.toList());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /** Every regular file under a root that may not exist, sorted by path. */
    static List<Path> filesUnder(Path root) {
        if (!Files.isDirectory(root)) {
            return List.of();
        }
        try (Stream<Path> walk = Files.walk(root)) {
            return walk.filter(Files::isRegularFile)
                    .sorted(Comparator.comparing(Path::toString))
                    .collect(Collectors.toList());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private void copyResources(List<Path> resources, Path output) {
        for (Path resource : resources) {
            Path target = output.resolve(resourcesRoot.relativize(resource).toString());
            try {
                Files.createDirectories(target.getParent());
                Files.copy(resource, target);
            } catch (IOException e) {
                throw new UncheckedIOException("could not copy " + resource + " to " + target, e);
            }
        }
    }

    /** Path, size, and modification time of every source: enough to notice a save. */
    static String fingerprintOf(Path sourceRoot, List<Path> sources) {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            for (Path source : sources) {
                String line = sourceRoot.relativize(source) + "|" + Files.size(source) + "|"
                        + Files.getLastModifiedTime(source).toMillis() + "\n";
                digest.update(line.getBytes(StandardCharsets.UTF_8));
            }
            StringBuilder hex = new StringBuilder();
            for (byte b : digest.digest()) {
                hex.append(Character.forDigit((b >> 4) & 0xf, 16)).append(Character.forDigit(b & 0xf, 16));
            }
            return hex.toString();
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private List<Problem> problems(DiagnosticCollector<JavaFileObject> diagnostics) {
        List<Problem> problems = new ArrayList<>();
        boolean simulator = false;
        for (Diagnostic<? extends JavaFileObject> d : diagnostics.getDiagnostics()) {
            if (d.getKind() != Diagnostic.Kind.ERROR) {
                continue;
            }
            Path file = d.getSource() == null
                    ? null
                    : Path.of(d.getSource().toUri()).toAbsolutePath().normalize();
            if (file != null && file.startsWith(harnessRoot)) {
                simulator = true;
                problems.add(new Problem(
                        "",
                        d.getLineNumber(),
                        "simulator " + relative(harnessRoot, file) + ":" + d.getLineNumber() + ": "
                                + d.getMessage(null)));
            } else {
                problems.add(new Problem(
                        file == null ? "" : relative(sourceRoot, file), d.getLineNumber(), d.getMessage(null)));
            }
        }
        if (simulator) {
            problems.add(0, new Problem("", 0, SIMULATOR_DOES_NOT_FIT));
        }
        return problems;
    }

    private static String relative(Path root, Path file) {
        return root.relativize(file).toString().replace('\\', '/');
    }

    private static void deleteTree(Path root) {
        try (Stream<Path> walk = Files.walk(root)) {
            walk.sorted(Comparator.reverseOrder()).forEach(p -> {
                try {
                    Files.deleteIfExists(p);
                } catch (IOException ignored) {
                    // a stale build directory is harmless
                }
            });
        } catch (IOException ignored) {
            // already gone
        }
    }
}
