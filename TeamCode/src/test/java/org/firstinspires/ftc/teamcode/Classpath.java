package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.CodeSource;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.stream.Stream;

/** What this JVM has on its classpath, for the tests and tools that ask what the team code holds. */
public final class Classpath {
    private Classpath() {}

    /**
     * The top-level classes in a package and its subpackages, found by listing every entry of this
     * JVM's classpath (directories and jars), so a freshly compiled directory placed first is seen
     * the same way the class loader sees it. Classes are loaded without being initialized, and one
     * that cannot be loaded on this JVM is noted on standard error and left out.
     */
    public static List<Class<?>> classesUnder(String packageName) {
        String prefix = packageName.replace('.', '/') + "/";
        Set<String> classFiles = new LinkedHashSet<>();
        try {
            for (String entry : System.getProperty("java.class.path").split(File.pathSeparator)) {
                File location = new File(entry);
                if (location.isDirectory()) {
                    Path root = location.toPath();
                    Path packageDir = root.resolve(prefix);
                    if (Files.isDirectory(packageDir)) {
                        try (Stream<Path> files = Files.walk(packageDir)) {
                            files.filter(Files::isRegularFile)
                                    .forEach(file -> classFiles.add(
                                            root.relativize(file).toString().replace(File.separatorChar, '/')));
                        }
                    }
                } else if (location.isFile()) {
                    try (JarFile jar = new JarFile(location)) {
                        Enumeration<JarEntry> entries = jar.entries();
                        while (entries.hasMoreElements()) {
                            String name = entries.nextElement().getName();
                            if (name.startsWith(prefix)) {
                                classFiles.add(name);
                            }
                        }
                    }
                }
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        List<Class<?>> classes = new ArrayList<>();
        for (String classFile : classFiles) {
            if (!classFile.endsWith(".class")) {
                continue;
            }
            String rest = classFile
                    .substring(0, classFile.length() - ".class".length())
                    .replace('/', '.');
            if (rest.contains("$") || rest.endsWith(".package-info") || rest.endsWith(".module-info")) {
                continue; // a nested class is part of the class that declares it
            }
            try {
                classes.add(Class.forName(rest, false, Classpath.class.getClassLoader()));
            } catch (ClassNotFoundException | LinkageError e) {
                System.err.println("classpath: skipping " + rest + ", not loadable on this JVM: " + e);
            }
        }
        return classes;
    }

    /**
     * Whether a class was compiled alongside the tests rather than as robot code. The test source
     * set compiles to its own directory, so a class from the same place as this one is a test's
     * own: a fake or a fixture, and not something the robot is expected to run.
     */
    public static boolean isTestClass(Class<?> type) {
        return sourceOf(type) != null && sourceOf(type).equals(sourceOf(Classpath.class));
    }

    private static String sourceOf(Class<?> type) {
        CodeSource source = type.getProtectionDomain().getCodeSource();
        return source == null || source.getLocation() == null
                ? null
                : source.getLocation().toString();
    }
}
