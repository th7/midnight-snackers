package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.reflect.Modifier;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.List;
import java.util.Optional;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * The autonomous op modes that can be run in the simulator: every concrete {@link AutoOp} in the
 * auto package that carries the {@link Autonomous} annotation, exactly as the robot controller
 * would list them.
 */
public final class SimCatalog {
    public static final String AUTO_PACKAGE = "org.firstinspires.ftc.teamcode.auto";

    public static final class Entry {
        public final String name;
        public final String group;
        public final Class<? extends AutoOp> type;

        Entry(String name, String group, Class<? extends AutoOp> type) {
            this.name = name;
            this.group = group;
            this.type = type;
        }

        public AutoOp create() {
            try {
                return type.getDeclaredConstructor().newInstance();
            } catch (ReflectiveOperationException e) {
                throw new IllegalStateException("could not construct " + type.getName(), e);
            }
        }
    }

    private final List<Entry> entries;

    private SimCatalog(List<Entry> entries) {
        this.entries = entries;
    }

    /**
     * A catalog of exactly these op modes, for tests and tools that know what they want to run.
     */
    @SafeVarargs
    public static SimCatalog of(Class<? extends AutoOp>... types) {
        List<Entry> entries = new ArrayList<>();
        for (Class<? extends AutoOp> type : types) {
            Autonomous annotation = type.getAnnotation(Autonomous.class);
            String name = annotation == null || annotation.name().isEmpty() ? type.getSimpleName() : annotation.name();
            entries.add(new Entry(name, annotation == null ? "" : annotation.group(), type));
        }
        entries.sort(Comparator.comparing(e -> e.name));
        return new SimCatalog(Collections.unmodifiableList(entries));
    }

    public static SimCatalog discover() {
        List<Entry> entries = new ArrayList<>();
        for (Class<?> type : classesIn(AUTO_PACKAGE)) {
            Autonomous annotation = type.getAnnotation(Autonomous.class);
            if (annotation == null || Modifier.isAbstract(type.getModifiers()) || !AutoOp.class.isAssignableFrom(type)) {
                continue;
            }
            String name = annotation.name().isEmpty() ? type.getSimpleName() : annotation.name();
            entries.add(new Entry(name, annotation.group(), type.asSubclass(AutoOp.class)));
        }
        entries.sort(Comparator.comparing(e -> e.name));
        return new SimCatalog(Collections.unmodifiableList(entries));
    }

    public List<Entry> entries() {
        return entries;
    }

    public Optional<Entry> find(String className) {
        return entries.stream().filter(e -> e.type.getName().equals(className)).findFirst();
    }

    /**
     * Classes directly in a package of the main code, found by listing wherever {@link AutoOp}
     * itself was loaded from: a jar when the unit tests run, a directory in other setups.
     */
    private static List<Class<?>> classesIn(String packageName) {
        String prefix = packageName.replace('.', '/') + "/";
        List<String> classFiles = new ArrayList<>();
        try {
            File location = new File(AutoOp.class.getProtectionDomain().getCodeSource().getLocation().toURI());
            if (location.isDirectory()) {
                File[] files = new File(location, prefix).listFiles();
                if (files != null) {
                    for (File file : files) {
                        classFiles.add(prefix + file.getName());
                    }
                }
            } else {
                try (JarFile jar = new JarFile(location)) {
                    Enumeration<JarEntry> entries = jar.entries();
                    while (entries.hasMoreElements()) {
                        classFiles.add(entries.nextElement().getName());
                    }
                }
            }
            List<Class<?>> classes = new ArrayList<>();
            for (String classFile : classFiles) {
                if (!classFile.startsWith(prefix) || !classFile.endsWith(".class")) {
                    continue;
                }
                String rest = classFile.substring(prefix.length(), classFile.length() - ".class".length());
                if (rest.contains("/") || rest.contains("$")) {
                    continue; // subpackages and nested classes are not op modes
                }
                classes.add(Class.forName(packageName + "." + rest));
            }
            return classes;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        } catch (ClassNotFoundException | URISyntaxException e) {
            throw new IllegalStateException(e);
        }
    }
}
