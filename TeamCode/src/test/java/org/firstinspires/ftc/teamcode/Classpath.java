package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.CodeSource;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.stream.Stream;

/** What this JVM has on its classpath, for the tests and tools that ask what the team code holds. */
public final class Classpath {
    private Classpath() {}

    /**
     * What one scan of a package found: the classes it read, and the ones it could not.
     *
     * <p>A class of ours that will not load on a plain JVM is a thing this scan could not judge,
     * not a thing that is not there. Left out quietly, it would take every invariant that reads a
     * scan down with it -- that every subsystem is ticked, that every op mode is listed -- and
     * they would pass by looking at less. So {@link #all()} is how the classes are had, and it
     * refuses to hand over a partial answer.
     */
    public static final class Scan {
        /** The classes that loaded, in the order the classpath holds them. */
        public final List<Class<?>> classes;
        /** The ones that did not, by name, each with what went wrong. */
        public final Map<String, String> unreadable;

        Scan(List<Class<?>> classes, Map<String, String> unreadable) {
            this.classes = List.copyOf(classes);
            this.unreadable = Map.copyOf(unreadable);
        }

        /**
         * The classes, when the scan read every one of them.
         *
         * @throws IllegalStateException naming the classes it could not read, and why
         */
        public List<Class<?>> all() {
            if (!unreadable.isEmpty()) {
                throw new IllegalStateException("the classpath scan could not read " + unreadable.size()
                        + " class(es), so it judged nothing: " + unreadable);
            }
            return classes;
        }
    }

    /**
     * The top-level classes in a package and its subpackages, and the ones that would not load,
     * found by listing every entry of this JVM's classpath (directories and jars), so a freshly
     * compiled directory placed first is seen the same way the class loader sees it. Classes are
     * loaded without being initialized.
     */
    public static Scan scan(String packageName) {
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
        Map<String, String> unreadable = new LinkedHashMap<>();
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
                unreadable.put(rest, String.valueOf(e));
            }
        }
        return new Scan(classes, unreadable);
    }

    /**
     * The top-level classes in a package and its subpackages.
     *
     * @throws IllegalStateException when any class in the package would not load, since a scan
     *                               that read only some of them cannot answer for the package
     */
    public static List<Class<?>> classesUnder(String packageName) {
        return scan(packageName).all();
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
