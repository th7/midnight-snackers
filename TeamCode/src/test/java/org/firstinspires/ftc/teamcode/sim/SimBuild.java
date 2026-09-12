package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;

/**
 * Compiles the robot's main sources, as they are on disk right now, with the JDK's own compiler
 * against this JVM's classpath. About a second for the whole tree, so a simulated run can always
 * execute what was last saved. Output goes to a fresh directory under the build root each time
 * the sources change; only the latest is kept. This is not the Android build: no Kotlin, no
 * desugaring, no annotation processing. A Kotlin file is refused by name rather than skipped.
 */
public final class SimBuild {
    public static final class Result {
        /** The compiled classes, or null when the build failed. */
        public final Path classes;
        /** Compiler errors when the build failed, otherwise empty. */
        public final String diagnostics;
        /** False when the sources had not changed and the previous output was reused. */
        public final boolean rebuilt;

        Result(Path classes, String diagnostics, boolean rebuilt) {
            this.classes = classes;
            this.diagnostics = diagnostics;
            this.rebuilt = rebuilt;
        }
    }

    private final Path sourceRoot;
    private final Path buildRoot;
    private String lastFingerprint;
    private Result lastResult;
    private int builds = 0;

    /**
     * @param sourceRoot the package root, e.g. {@code TeamCode/src/main/java}
     * @param buildRoot  where compiled output goes, e.g. {@code TeamCode/build/sim/classes}
     */
    public SimBuild(Path sourceRoot, Path buildRoot) {
        this.sourceRoot = sourceRoot.toAbsolutePath().normalize();
        this.buildRoot = buildRoot.toAbsolutePath().normalize();
    }

    public Path sourceRoot() {
        return sourceRoot;
    }

    public synchronized Result build() {
        List<Path> sources = sources();
        String fingerprint = fingerprint(sources);
        if (fingerprint.equals(lastFingerprint) && lastResult != null) {
            return new Result(lastResult.classes, lastResult.diagnostics, false);
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
        try (StandardJavaFileManager files = compiler.getStandardFileManager(diagnostics, null, StandardCharsets.UTF_8)) {
            List<String> options = List.of(
                    "-d", output.toString(),
                    "-cp", System.getProperty("java.class.path"),
                    "--release", "17",
                    "-proc:none",
                    "-nowarn",
                    "-encoding", "UTF-8");
            List<java.io.File> sourceFiles = new ArrayList<>();
            for (Path source : sources) {
                sourceFiles.add(source.toFile());
            }
            Iterable<? extends JavaFileObject> units = files.getJavaFileObjectsFromFiles(sourceFiles);
            ok = sources.isEmpty() || compiler.getTask(null, files, diagnostics, options, null, units).call();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        Result result;
        if (ok) {
            Path previous = lastResult == null ? null : lastResult.classes;
            result = new Result(output, "", true);
            if (previous != null) {
                deleteTree(previous);
            }
        } else {
            deleteTree(output);
            result = new Result(null, format(diagnostics), true);
        }
        lastFingerprint = fingerprint;
        lastResult = result;
        return result;
    }

    private List<Path> sources() {
        Path mainDir = sourceRoot.getParent() == null ? sourceRoot : sourceRoot.getParent();
        try (Stream<Path> walk = Files.walk(mainDir)) {
            List<Path> kotlin = walk.filter(p -> p.toString().endsWith(".kt")).collect(Collectors.toList());
            if (!kotlin.isEmpty()) {
                throw new IllegalStateException("cannot build for the simulator: Kotlin sources are not compiled here: " + kotlin);
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

    /** Path, size, and modification time of every source: enough to notice a save. */
    private String fingerprint(List<Path> sources) {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            for (Path source : sources) {
                String line = sourceRoot.relativize(source) + "|" + Files.size(source) + "|" + Files.getLastModifiedTime(source).toMillis() + "\n";
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

    private String format(DiagnosticCollector<JavaFileObject> diagnostics) {
        List<String> lines = new ArrayList<>();
        for (Diagnostic<? extends JavaFileObject> d : diagnostics.getDiagnostics()) {
            if (d.getKind() != Diagnostic.Kind.ERROR) {
                continue;
            }
            String where = d.getSource() == null ? "" : sourceRoot.relativize(Path.of(d.getSource().toUri())) + ":" + d.getLineNumber() + ": ";
            lines.add(where + d.getMessage(null));
        }
        return String.join("\n", lines);
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
