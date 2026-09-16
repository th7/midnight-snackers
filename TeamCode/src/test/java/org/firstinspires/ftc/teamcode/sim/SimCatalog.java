package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.Classpath;
import org.firstinspires.ftc.teamcode.fakes.FakeOpModeManager;
import org.firstinspires.ftc.teamcode.opmode.OpMode;

/**
 * The op modes that can be run in the simulator, exactly as the robot controller would list them:
 * every concrete {@link OpMode} of ours that carries the {@link Autonomous} or {@link TeleOp}
 * annotation, plus every op mode of ours that a registrar ({@link OpModeRegistrar}) registers,
 * found by calling the registrar itself. Entries are keyed by name, which the robot controller
 * requires to be unique. A catalog built here can produce its op modes; one parsed from another
 * JVM's listing ({@link #fromJson}) only names them.
 */
public final class SimCatalog {
    public static final String TEAMCODE_PACKAGE = "org.firstinspires.ftc.teamcode";
    /** Vendored Road Runner code: its op modes and registrars are not ours to simulate. */
    public static final String ROADRUNNER_PACKAGE = TEAMCODE_PACKAGE + ".roadrunner";

    public static final String AUTO = "auto";
    public static final String TELEOP = "teleop";

    public static final class Entry {
        /** The name on the driver station, unique in the catalog. */
        public final String name;

        public final String group;
        /** {@link #AUTO} or {@link #TELEOP}: an auto runs until its plan is done, a TeleOp until the driver stops it. */
        public final String kind;
        /** Where a person finds the op mode's code: a class, or a plan method. */
        public final String where;
        /** Produces the op mode as the robot controller would, or null for an entry parsed from a listing. */
        private final Supplier<OpMode> opMode;

        Entry(String name, String group, String kind, String where, Supplier<OpMode> opMode) {
            this.name = name;
            this.group = group;
            this.kind = kind;
            this.where = where;
            this.opMode = opMode;
        }

        /**
         * The op mode as the robot controller runs it: the registered instance every time for an
         * op mode registered by instance, a new instance each time for one registered as a class.
         */
        public OpMode opMode() {
            if (opMode == null) {
                throw new IllegalStateException(name + " was listed by another JVM and cannot be built here");
            }
            return opMode.get();
        }

        public JsonObject toJson() {
            JsonObject item = new JsonObject();
            item.addProperty("name", name);
            item.addProperty("group", group);
            item.addProperty("kind", kind);
            item.addProperty("where", where);
            return item;
        }
    }

    private final List<Entry> entries;
    private final List<String> sources;

    private SimCatalog(List<Entry> entries, List<String> sources) {
        this.entries = entries;
        this.sources = sources;
    }

    /**
     * Which kind of run an op mode class gets: {@link #TELEOP} when it carries the TeleOp
     * annotation, else {@link #AUTO}.
     */
    public static String kindOf(Class<?> type) {
        return type.getAnnotation(TeleOp.class) != null ? TELEOP : AUTO;
    }

    /**
     * A catalog of exactly these sources, for tests and tools that know what they want to run.
     * A source is an annotated op mode class, or a class whose registrar registers op modes.
     *
     * @throws IllegalArgumentException for a class that is neither
     */
    public static SimCatalog of(Class<?>... sources) {
        List<Entry> entries = new ArrayList<>();
        List<String> names = new ArrayList<>();
        for (Class<?> source : sources) {
            List<Entry> found = entriesFrom(source);
            if (found == null) {
                throw new IllegalArgumentException(
                        source.getName() + " is neither an annotated op mode nor a registrar");
            }
            entries.addAll(found);
            names.add(source.getName());
        }
        return sorted(entries, Collections.unmodifiableList(names));
    }

    /** Entries as another JVM listed them: names only, nothing to build. */
    public static SimCatalog fromJson(JsonArray json) {
        List<Entry> entries = new ArrayList<>();
        for (JsonElement element : json) {
            JsonObject item = element.getAsJsonObject();
            if (!item.has("kind") || !item.has("where")) {
                throw new IllegalArgumentException("a listing without the kind and place of each op mode: " + item);
            }
            entries.add(new Entry(
                    item.get("name").getAsString(),
                    item.get("group").getAsString(),
                    item.get("kind").getAsString(),
                    item.get("where").getAsString(),
                    null));
        }
        return new SimCatalog(Collections.unmodifiableList(entries), List.of());
    }

    public JsonArray toJson() {
        JsonArray json = new JsonArray();
        for (Entry entry : entries) {
            json.add(entry.toJson());
        }
        return json;
    }

    /**
     * Every op mode a top-level class under the team code package declares or registers, other
     * than Road Runner's.
     */
    public static SimCatalog discover() {
        List<Entry> entries = new ArrayList<>();
        for (Class<?> type : Classpath.classesUnder(TEAMCODE_PACKAGE)) {
            if (type.getName().startsWith(ROADRUNNER_PACKAGE + ".")) {
                continue;
            }
            List<Entry> found = entriesFrom(type);
            if (found != null) {
                entries.addAll(found);
            }
        }
        return sorted(entries, List.of());
    }

    /**
     * The classes this catalog was built from, for another JVM to build the same; empty when it
     * was discovered, which another JVM does alike.
     */
    public List<String> sources() {
        return sources;
    }

    /** Autos first, then TeleOps, each by name; a name used twice is refused as the robot controller would. */
    private static SimCatalog sorted(List<Entry> entries, List<String> sources) {
        Map<String, Entry> byName = new HashMap<>();
        for (Entry entry : entries) {
            Entry other = byName.put(entry.name, entry);
            if (other != null) {
                throw new IllegalStateException(
                        "two op modes are named " + entry.name + ": " + other.where + " and " + entry.where);
            }
        }
        entries.sort(Comparator.comparing((Entry e) -> e.kind).thenComparing(e -> e.name));
        return new SimCatalog(Collections.unmodifiableList(entries), sources);
    }

    /**
     * The op modes a class contributes: itself when it is a concrete annotated {@link OpMode} of
     * ours, what its registrars register when it has any, and null when it is neither.
     */
    private static List<Entry> entriesFrom(Class<?> type) {
        boolean annotated = type.getAnnotation(Autonomous.class) != null || type.getAnnotation(TeleOp.class) != null;
        List<Method> registrars = new ArrayList<>();
        for (Method method : type.getDeclaredMethods()) {
            if (method.getAnnotation(OpModeRegistrar.class) != null && Modifier.isStatic(method.getModifiers())) {
                registrars.add(method);
            }
        }
        if (!annotated && registrars.isEmpty()) {
            return null;
        }
        List<Entry> entries = new ArrayList<>();
        if (annotated && !Modifier.isAbstract(type.getModifiers()) && OpMode.class.isAssignableFrom(type)) {
            entries.add(entryFor(type.asSubclass(OpMode.class)));
        }
        for (Method registrar : registrars) {
            entries.addAll(registeredBy(registrar));
        }
        return entries;
    }

    private static Entry entryFor(Class<? extends OpMode> type) {
        Autonomous auto = type.getAnnotation(Autonomous.class);
        TeleOp teleOp = type.getAnnotation(TeleOp.class);
        String name = auto != null ? auto.name() : teleOp != null ? teleOp.name() : "";
        String group = auto != null ? auto.group() : teleOp != null ? teleOp.group() : "";
        return new Entry(
                name.isEmpty() ? type.getSimpleName() : name,
                group,
                kindOf(type),
                type.getName(),
                () -> construct(type));
    }

    private static OpMode construct(Class<? extends OpMode> type) {
        try {
            return type.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("could not construct " + type.getName(), e);
        }
    }

    /**
     * What a registrar registers that is ours to run. A registrar that fails would stop the robot
     * controller too, so it fails the catalog rather than listing less.
     */
    private static List<Entry> registeredBy(Method registrar) {
        FakeOpModeManager manager = new FakeOpModeManager();
        try {
            registrar.setAccessible(true);
            registrar.invoke(null, manager);
        } catch (InvocationTargetException e) {
            throw new IllegalStateException(
                    "the registrar " + registrar.getDeclaringClass().getName() + "." + registrar.getName() + " failed",
                    e.getCause());
        } catch (IllegalAccessException | IllegalArgumentException e) {
            throw new IllegalStateException("could not call the registrar " + registrar, e);
        }
        List<Entry> entries = new ArrayList<>();
        for (FakeOpModeManager.Registration registration : manager.registrations) {
            boolean ours = registration.instance != null
                    ? registration.instance instanceof OpMode
                    : OpMode.class.isAssignableFrom(registration.type);
            if (!ours) {
                continue;
            }
            String where = registration.instance != null
                    ? ((OpMode) registration.instance).where()
                    : registration.type.getName();
            String kind = registration.meta.flavor == OpModeMeta.Flavor.TELEOP ? TELEOP : AUTO;
            entries.add(new Entry(registration.meta.name, registration.meta.group, kind, where, () ->
                    (OpMode) registration.opMode()));
        }
        return entries;
    }

    public List<Entry> entries() {
        return entries;
    }

    public Optional<Entry> find(String name) {
        return entries.stream().filter(e -> e.name.equals(name)).findFirst();
    }
}
