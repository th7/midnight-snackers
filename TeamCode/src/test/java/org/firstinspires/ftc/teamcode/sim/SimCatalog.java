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

public final class SimCatalog {
    public static final String TEAMCODE_PACKAGE = "org.firstinspires.ftc.teamcode";

    public static final String ROADRUNNER_PACKAGE = TEAMCODE_PACKAGE + ".roadrunner";

    public static final String AUTO = "auto";
    public static final String TELEOP = "teleop";

    public static final class Entry {
        public final String name;

        public final String group;

        public final String kind;

        public final String where;

        private final Supplier<OpMode> opMode;

        Entry(String name, String group, String kind, String where, Supplier<OpMode> opMode) {
            this.name = name;
            this.group = group;
            this.kind = kind;
            this.where = where;
            this.opMode = opMode;
        }

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

    public static String kindOf(Class<?> type) {
        return type.getAnnotation(TeleOp.class) != null ? TELEOP : AUTO;
    }

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

    public List<String> sources() {
        return sources;
    }

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
