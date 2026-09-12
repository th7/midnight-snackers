package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * The autonomous op modes that can be run in the simulator: every concrete {@link AutoOp} in the
 * auto package that carries the {@link Autonomous} annotation, exactly as the robot controller
 * would list them. A catalog discovered here can build its op modes; one parsed from another
 * JVM's listing ({@link #fromJson}) only names them.
 */
public final class SimCatalog {
    public static final String AUTO_PACKAGE = "org.firstinspires.ftc.teamcode.auto";

    public static final class Entry {
        public final String name;
        public final String group;
        public final String className;
        /** The class when it is loadable in this JVM, null for an entry parsed from a listing. */
        public final Class<? extends AutoOp> type;

        Entry(String name, String group, Class<? extends AutoOp> type) {
            this(name, group, type.getName(), type);
        }

        Entry(String name, String group, String className, Class<? extends AutoOp> type) {
            this.name = name;
            this.group = group;
            this.className = className;
            this.type = type;
        }

        public AutoOp create() {
            if (type == null) {
                throw new IllegalStateException(className + " was listed by another JVM and cannot be built here");
            }
            try {
                return type.getDeclaredConstructor().newInstance();
            } catch (ReflectiveOperationException e) {
                throw new IllegalStateException("could not construct " + type.getName(), e);
            }
        }

        public JsonObject toJson() {
            JsonObject item = new JsonObject();
            item.addProperty("name", name);
            item.addProperty("group", group);
            item.addProperty("opMode", className);
            return item;
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

    /** Entries as another JVM listed them: names only, nothing to build. */
    public static SimCatalog fromJson(JsonArray json) {
        List<Entry> entries = new ArrayList<>();
        for (JsonElement element : json) {
            JsonObject item = element.getAsJsonObject();
            entries.add(new Entry(item.get("name").getAsString(), item.get("group").getAsString(),
                    item.get("opMode").getAsString(), null));
        }
        return new SimCatalog(Collections.unmodifiableList(entries));
    }

    public JsonArray toJson() {
        JsonArray json = new JsonArray();
        for (Entry entry : entries) {
            json.add(entry.toJson());
        }
        return json;
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
        return entries.stream().filter(e -> e.className.equals(className)).findFirst();
    }

    /**
     * Classes directly in a package of the main code, found by listing every entry of this JVM's
     * classpath (directories and jars), so a freshly compiled directory placed first is seen the
     * same way the class loader sees it.
     */
    private static List<Class<?>> classesIn(String packageName) {
        String prefix = packageName.replace('.', '/') + "/";
        Set<String> classFiles = new LinkedHashSet<>();
        try {
            for (String entry : System.getProperty("java.class.path").split(File.pathSeparator)) {
                File location = new File(entry);
                if (location.isDirectory()) {
                    File[] files = new File(location, prefix).listFiles();
                    if (files != null) {
                        for (File file : files) {
                            classFiles.add(prefix + file.getName());
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
            List<Class<?>> classes = new ArrayList<>();
            for (String classFile : classFiles) {
                if (!classFile.endsWith(".class")) {
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
        } catch (ClassNotFoundException e) {
            throw new IllegalStateException(e);
        }
    }
}
