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
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.firstinspires.ftc.nugget.NuggetOpMode;
import org.firstinspires.ftc.reginald.opmode.OpMode;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.Classpath;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeOpModeManager;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;

public final class SimCatalog {
    public static final String ROADRUNNER_PACKAGE = TeamRobot.REGINALD.javaPackage() + ".roadrunner";

    public static final String AUTO = "auto";
    public static final String TELEOP = "teleop";

    /** The alliances an op mode can play for, as the field names them. */
    public static final List<String> ALLIANCES = List.of("Blue", "Red");

    public static final class Entry {
        public final String name;

        public final String group;

        public final String kind;

        public final String where;

        /** Which of the {@link #ALLIANCES} it plays for, or null for one that plays for none. */
        public final String alliance;

        private final Supplier<? extends com.qualcomm.robotcore.eventloop.opmode.OpMode> opMode;

        Entry(
                String name,
                String group,
                String kind,
                String where,
                String alliance,
                Supplier<? extends com.qualcomm.robotcore.eventloop.opmode.OpMode> opMode) {
            this.name = name;
            this.group = group;
            this.kind = kind;
            this.where = where;
            this.alliance = alliance;
            this.opMode = opMode;
        }

        /**
         * Whose area its drivers stand in: its own alliance's, and blue's for an op mode that plays for
         * none, which plays the blue way round.
         */
        public String drivenFrom() {
            return alliance != null ? alliance : "Blue";
        }

        public com.qualcomm.robotcore.eventloop.opmode.OpMode opMode() {
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
            item.addProperty("alliance", alliance);
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
            List<Entry> found = entriesFrom(source, SimCatalog::anyRobots);
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
                    allianceIn(item),
                    null));
        }
        return new SimCatalog(Collections.unmodifiableList(entries), List.of());
    }

    private static String allianceIn(JsonObject item) {
        JsonElement alliance = item.get("alliance");
        if (alliance == null || alliance.isJsonNull()) {
            return null;
        }
        if (!ALLIANCES.contains(alliance.getAsString())) {
            throw new IllegalArgumentException(item.get("name").getAsString() + " plays for " + alliance
                    + ", and nobody does: an op mode plays for one of " + ALLIANCES + " or for none");
        }
        return alliance.getAsString();
    }

    public JsonArray toJson() {
        JsonArray json = new JsonArray();
        for (Entry entry : entries) {
            json.add(entry.toJson());
        }
        return json;
    }

    public static String packageOf(TeamRobot robot) {
        return robot.javaPackage();
    }

    private static Class<? extends com.qualcomm.robotcore.eventloop.opmode.OpMode> opModeOf(TeamRobot robot) {
        return switch (robot) {
            case REGINALD -> OpMode.class;
            case NUGGET -> NuggetOpMode.class;
        };
    }

    private static boolean anyRobots(Class<?> type) {
        for (TeamRobot robot : TeamRobot.values()) {
            if (opModeOf(robot).isAssignableFrom(type)) {
                return true;
            }
        }
        return false;
    }

    public static SimCatalog discover(TeamRobot robot) {
        List<Entry> entries = new ArrayList<>();
        for (Class<?> type : Classpath.classesUnder(packageOf(robot))) {
            if (type.getName().startsWith(ROADRUNNER_PACKAGE + ".")) {
                continue;
            }
            List<Entry> found = entriesFrom(type, opModeOf(robot)::isAssignableFrom);
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

    private static List<Entry> entriesFrom(Class<?> type, Predicate<Class<?>> ours) {
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
        if (annotated && !Modifier.isAbstract(type.getModifiers()) && ours.test(type)) {
            entries.add(entryFor(type.asSubclass(com.qualcomm.robotcore.eventloop.opmode.OpMode.class)));
        }
        for (Method registrar : registrars) {
            entries.addAll(registeredBy(registrar, ours));
        }
        return entries;
    }

    private static Entry entryFor(Class<? extends com.qualcomm.robotcore.eventloop.opmode.OpMode> type) {
        Autonomous auto = type.getAnnotation(Autonomous.class);
        TeleOp teleOp = type.getAnnotation(TeleOp.class);
        String name = auto != null ? auto.name() : teleOp != null ? teleOp.name() : "";
        String group = auto != null ? auto.group() : teleOp != null ? teleOp.group() : "";
        return new Entry(
                name.isEmpty() ? type.getSimpleName() : name,
                group,
                kindOf(type),
                type.getName(),
                allianceOfA(type),
                () -> construct(type));
    }

    private static com.qualcomm.robotcore.eventloop.opmode.OpMode construct(
            Class<? extends com.qualcomm.robotcore.eventloop.opmode.OpMode> type) {
        try {
            return type.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("could not construct " + type.getName(), e);
        }
    }

    /**
     * An op mode says which alliance it plays for only once it is made, so one is made to ask. One
     * that cannot be made is listed all the same, as it always was, and says why when it is run.
     */
    private static String allianceOfA(Class<? extends com.qualcomm.robotcore.eventloop.opmode.OpMode> type) {
        if (!OpMode.class.isAssignableFrom(type)) {
            return null;
        }
        com.qualcomm.robotcore.eventloop.opmode.OpMode made;
        try {
            made = construct(type);
        } catch (RuntimeException e) {
            return null;
        }
        return allianceOf(made);
    }

    private static String allianceOf(com.qualcomm.robotcore.eventloop.opmode.OpMode opMode) {
        if (!(opMode instanceof OpMode reginalds)) {
            return null;
        }
        Alliance alliance = reginalds.alliance();
        if (alliance == Alliance.BLUE) {
            return "Blue";
        }
        if (alliance == Alliance.RED) {
            return "Red";
        }
        return null;
    }

    private static List<Entry> registeredBy(Method registrar, Predicate<Class<?>> ours) {
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
            Class<?> type = registration.instance != null ? registration.instance.getClass() : registration.type;
            if (!ours.test(type)) {
                continue;
            }
            String where = registration.instance instanceof OpMode reginalds ? reginalds.where() : type.getName();
            String kind = registration.meta.flavor == OpModeMeta.Flavor.TELEOP ? TELEOP : AUTO;
            String alliance =
                    registration.instance != null ? allianceOf(registration.instance) : allianceOfA(registration.type);
            entries.add(new Entry(
                    registration.meta.name, registration.meta.group, kind, where, alliance, registration::opMode));
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
