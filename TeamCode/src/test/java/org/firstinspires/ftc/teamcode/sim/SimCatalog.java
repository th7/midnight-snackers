package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.OpMode;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
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
import java.util.stream.Stream;

/**
 * The op modes that can be run in the simulator: every concrete {@link OpMode} of ours, anywhere
 * under the team code package, that carries the {@link Autonomous} or {@link TeleOp} annotation,
 * exactly as the robot controller would list them. A catalog discovered here can build its op
 * modes; one parsed from another JVM's listing ({@link #fromJson}) only names them.
 */
public final class SimCatalog {
    public static final String TEAMCODE_PACKAGE = "org.firstinspires.ftc.teamcode";
    public static final String AUTO = "auto";
    public static final String TELEOP = "teleop";

    public static final class Entry {
        public final String name;
        public final String group;
        /** {@link #AUTO} or {@link #TELEOP}: an auto runs until its plan is done, a TeleOp until the driver stops it. */
        public final String kind;
        public final String className;
        /** The class when it is loadable in this JVM, null for an entry parsed from a listing. */
        public final Class<? extends OpMode> type;

        Entry(String name, String group, Class<? extends OpMode> type) {
            this(name, group, kindOf(type), type.getName(), type);
        }

        Entry(String name, String group, String kind, String className, Class<? extends OpMode> type) {
            this.name = name;
            this.group = group;
            this.kind = kind;
            this.className = className;
            this.type = type;
        }

        public OpMode create() {
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
            item.addProperty("kind", kind);
            item.addProperty("opMode", className);
            return item;
        }
    }

    private final List<Entry> entries;

    private SimCatalog(List<Entry> entries) {
        this.entries = entries;
    }

    /**
     * Which kind of run an op mode class gets: {@link #TELEOP} when it carries the TeleOp
     * annotation, else {@link #AUTO}.
     */
    public static String kindOf(Class<?> type) {
        return type.getAnnotation(TeleOp.class) != null ? TELEOP : AUTO;
    }

    /**
     * A catalog of exactly these op modes, for tests and tools that know what they want to run.
     */
    @SafeVarargs
    public static SimCatalog of(Class<? extends OpMode>... types) {
        List<Entry> entries = new ArrayList<>();
        for (Class<? extends OpMode> type : types) {
            entries.add(entryFor(type));
        }
        return sorted(entries);
    }

    /** Entries as another JVM listed them: names only, nothing to build. */
    public static SimCatalog fromJson(JsonArray json) {
        List<Entry> entries = new ArrayList<>();
        for (JsonElement element : json) {
            JsonObject item = element.getAsJsonObject();
            if (!item.has("kind")) {
                throw new IllegalArgumentException("a listing without the kind of each op mode: " + item);
            }
            entries.add(new Entry(item.get("name").getAsString(), item.get("group").getAsString(),
                    item.get("kind").getAsString(), item.get("opMode").getAsString(), null));
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
        for (Class<?> type : classesUnder(TEAMCODE_PACKAGE)) {
            boolean registered = type.getAnnotation(Autonomous.class) != null || type.getAnnotation(TeleOp.class) != null;
            if (!registered || Modifier.isAbstract(type.getModifiers()) || !OpMode.class.isAssignableFrom(type)) {
                continue;
            }
            entries.add(entryFor(type.asSubclass(OpMode.class)));
        }
        return sorted(entries);
    }

    /** Autos first, then TeleOps, each by name. */
    private static SimCatalog sorted(List<Entry> entries) {
        entries.sort(Comparator.comparing((Entry e) -> e.kind).thenComparing(e -> e.name));
        return new SimCatalog(Collections.unmodifiableList(entries));
    }

    private static Entry entryFor(Class<? extends OpMode> type) {
        Autonomous auto = type.getAnnotation(Autonomous.class);
        TeleOp teleOp = type.getAnnotation(TeleOp.class);
        String name = auto != null ? auto.name() : teleOp != null ? teleOp.name() : "";
        String group = auto != null ? auto.group() : teleOp != null ? teleOp.group() : "";
        return new Entry(name.isEmpty() ? type.getSimpleName() : name, group, type);
    }

    public List<Entry> entries() {
        return entries;
    }

    public Optional<Entry> find(String className) {
        return entries.stream().filter(e -> e.className.equals(className)).findFirst();
    }

    /**
     * The top-level classes in a package and its subpackages, found by listing every entry of this
     * JVM's classpath (directories and jars), so a freshly compiled directory placed first is seen
     * the same way the class loader sees it. Classes are loaded without being initialized, and one
     * that cannot be loaded on this JVM is noted on standard error and left out: it could not be
     * an op mode the simulator runs.
     */
    private static List<Class<?>> classesUnder(String packageName) {
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
                                    .forEach(file -> classFiles.add(root.relativize(file).toString().replace(File.separatorChar, '/')));
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
            String rest = classFile.substring(0, classFile.length() - ".class".length()).replace('/', '.');
            if (rest.contains("$") || rest.endsWith(".package-info") || rest.endsWith(".module-info")) {
                continue; // nested classes are not op modes
            }
            try {
                classes.add(Class.forName(rest, false, SimCatalog.class.getClassLoader()));
            } catch (ClassNotFoundException | LinkageError e) {
                System.err.println("catalog: skipping " + rest + ", not loadable on this JVM: " + e);
            }
        }
        return classes;
    }
}
