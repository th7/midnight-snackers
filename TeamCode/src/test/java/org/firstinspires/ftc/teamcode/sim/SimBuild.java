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
import org.firstinspires.ftc.teamcode.opmode.OpMode;

public final class SimBuild {
    static final String SIMULATOR_DOES_NOT_FIT =
            "the simulator does not fit these robot sources: it was built from the code"
                    + " the server runs from, and these sources came from another version of it. Pull develop; if that does not help,"
                    + " the coach brings develop and the server's checkout to the same code and restarts the server.";

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
        public final Path classes;

        public final String diagnostics;

        public final List<Problem> problems;

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

    private final Path resourcesRoot;

    private final Path buildRoot;
    private String lastFingerprint;
    private Result lastResult;

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

    public static List<String> libraries() {
        return librariesOf(
                List.of(System.getProperty("java.class.path").split(Pattern.quote(File.pathSeparator))),
                locationOf(OpMode.class));
    }

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
        Path output;
        try {
            if (lastResult == null) {
                clearBuildRoot();
            }
            Files.createDirectories(buildRoot);
            output = Files.createTempDirectory(buildRoot, "build-");
        } catch (IOException e) {
            throw new UncheckedIOException("could not create a build directory under " + buildRoot, e);
        }
        DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<>();
        boolean ok;
        try (Cost.Spent spent = Cost.start(Cost.Kind.COMPILE);
                StandardJavaFileManager files =
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

    private void clearBuildRoot() throws IOException {
        if (!Files.isDirectory(buildRoot)) {
            return;
        }
        try (Stream<Path> children = Files.list(buildRoot)) {
            children.forEach(SimBuild::deleteTree);
        }
    }

    private static void deleteTree(Path root) {
        try (Stream<Path> walk = Files.walk(root)) {
            walk.sorted(Comparator.reverseOrder()).forEach(p -> {
                try {
                    Files.deleteIfExists(p);
                } catch (IOException ignored) {
                }
            });
        } catch (IOException ignored) {
        }
    }
}
