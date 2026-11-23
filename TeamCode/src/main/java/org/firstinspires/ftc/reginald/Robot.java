package org.firstinspires.ftc.reginald;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.reginald.hardware.SwitchedOff;
import org.firstinspires.ftc.reginald.roadrunner.PoseEstimate;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Channels;
import org.firstinspires.ftc.teamcode.base.Gamepads;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;

public final class Robot implements Loopable {
    private static final String OWN_CLASSES = "org.firstinspires.ftc.reginald.";

    public final Channels channels;

    private final Hardware hardware;
    private final Map<Class<?>, Object> supplied = new HashMap<>();
    private final Map<Class<?>, Class<?>> standsFor = new HashMap<>();
    private final Map<Class<?>, Object> built = new HashMap<>();
    private final Set<Class<?>> building = new LinkedHashSet<>();
    private final List<SubSystem> tickOrder = new ArrayList<>();

    @SafeVarargs
    public Robot(
            Hardware hardware,
            Alliance alliance,
            Telemetry telemetry,
            Gamepads gamepads,
            Class<? extends SubSystem>... needs) {
        this.hardware = hardware;
        this.channels = new Channels(telemetry);
        supplied.put(Alliance.class, alliance);
        supplied.put(Gamepads.class, gamepads);
        supplied.put(Dashboard.class, hardware.dashboard());
        standsFor.put(PoseEstimate.class, Localizer.class);

        for (Class<? extends SubSystem> need : needs) {
            build(need);
        }
        for (SubSystem subSystem : tickOrder) {
            subSystem.init();
        }
    }

    public <T> T part(Class<T> kind) {
        Object part = built.get(kind);
        if (part == null) {
            throw new IllegalArgumentException("the robot built no " + kind.getSimpleName()
                    + ": the op mode names the parts it needs, and the robot builds only those and what they need");
        }
        return kind.cast(part);
    }

    @Override
    public void loop() {
        for (SubSystem subSystem : tickOrder) {
            subSystem.loop();
        }
    }

    private Object supply(Class<?> need, Class<?> part) {
        if (need == Hardware.class) {
            return part.isAnnotationPresent(SwitchedOff.class) ? hardware.switchedOff() : hardware;
        }
        if (need == Prints.class) {
            return channels.channel(part.getSimpleName());
        }
        if (supplied.containsKey(need)) {
            return supplied.get(need);
        }
        Class<?> kind = standsFor.get(need);
        return build(kind != null ? kind : need);
    }

    private Object build(Class<?> kind) {
        Object already = built.get(kind);
        if (already != null) {
            return already;
        }
        if (!kind.getName().startsWith(OWN_CLASSES)) {
            throw new IllegalStateException(neededBy() + "a " + kind.getSimpleName()
                    + ", which the robot does not supply, and it builds only Reginald's own classes");
        }
        if (kind.isInterface() || Modifier.isAbstract(kind.getModifiers())) {
            throw new IllegalStateException(neededBy() + "a " + kind.getSimpleName()
                    + ", which is not a class the robot can build, and the robot has not been told which class stands for it");
        }
        if (!building.add(kind)) {
            throw new IllegalStateException(neededBy() + "a " + kind.getSimpleName()
                    + ", which is already being built: a part cannot need itself");
        }
        Constructor<?>[] constructors = kind.getConstructors();
        if (constructors.length != 1) {
            throw new IllegalStateException("the robot builds " + kind.getSimpleName()
                    + " through its one public constructor, and it has " + constructors.length);
        }
        Class<?>[] needs = constructors[0].getParameterTypes();
        Object[] args = new Object[needs.length];
        for (int i = 0; i < needs.length; i++) {
            args[i] = supply(needs[i], kind);
        }
        Object part = construct(constructors[0], args);
        building.remove(kind);
        built.put(kind, part);
        if (part instanceof SubSystem) {
            tickOrder.add((SubSystem) part);
        }
        return part;
    }

    private String neededBy() {
        List<String> chain = new ArrayList<>();
        for (Class<?> kind : building) {
            chain.add(kind.getSimpleName());
        }
        if (chain.isEmpty()) {
            return "the op mode needs ";
        }
        return "the robot cannot build " + String.join(", which needs ", chain) + ", because that needs ";
    }

    private static Object construct(Constructor<?> constructor, Object[] args) {
        try {
            return constructor.newInstance(args);
        } catch (InvocationTargetException failed) {
            Throwable cause = failed.getCause();
            if (cause instanceof RuntimeException) {
                throw (RuntimeException) cause;
            }
            if (cause instanceof Error) {
                throw (Error) cause;
            }
            throw new IllegalStateException(
                    "building " + constructor.getDeclaringClass().getSimpleName() + " failed", cause);
        } catch (ReflectiveOperationException unreachable) {
            throw new IllegalStateException(
                    "the robot could not call " + constructor.getDeclaringClass().getSimpleName()
                            + "'s public constructor",
                    unreachable);
        }
    }
}
