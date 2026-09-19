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

public final class Classpath {
    private Classpath() {}

    public static final class Scan {
        public final List<Class<?>> classes;

        public final Map<String, String> unreadable;

        Scan(List<Class<?>> classes, Map<String, String> unreadable) {
            this.classes = List.copyOf(classes);
            this.unreadable = Map.copyOf(unreadable);
        }

        public List<Class<?>> all() {
            if (!unreadable.isEmpty()) {
                throw new IllegalStateException("the classpath scan could not read " + unreadable.size()
                        + " class(es), so it judged nothing: " + unreadable);
            }
            return classes;
        }
    }

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
                continue;
            }
            try {
                classes.add(Class.forName(rest, false, Classpath.class.getClassLoader()));
            } catch (ClassNotFoundException | LinkageError e) {
                unreadable.put(rest, String.valueOf(e));
            }
        }
        return new Scan(classes, unreadable);
    }

    public static List<Class<?>> classesUnder(String packageName) {
        return scan(packageName).all();
    }

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
