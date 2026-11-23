package org.firstinspires.ftc.reginald.framework;

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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Channels;
import org.firstinspires.ftc.teamcode.base.Gamepads;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;

public final class Robot implements Loopable {
    public final Channels channels;

    private final Hardware hardware;
    private final Gamepads gamepads;
    private final Discovered discovered;
    private final Map<Class<?>, Object> supplied = new HashMap<>();
    private final Map<Class<?>, Object> built = new HashMap<>();
    private final Set<Class<?>> building = new LinkedHashSet<>();
    private final List<SubSystem> tickOrder = new ArrayList<>();

    public Robot(Hardware hardware, Alliance alliance, Telemetry telemetry, Gamepads gamepads, Discovered discovered) {
        this.hardware = hardware;
        this.gamepads = gamepads;
        this.discovered = discovered;
        this.channels = new Channels(telemetry);
        supplied.put(Alliance.class, alliance);
        supplied.put(Gamepads.class, gamepads);
        supplied.put(Dashboard.class, hardware.dashboard());

        for (Class<? extends SubSystem> subsystem : discovered.subsystems()) {
            build(subsystem);
        }
        for (SubSystem subsystem : tickOrder) {
            subsystem.init();
        }
    }

    public <T> T part(Class<T> kind) {
        return kind.cast(partFor(kind, kind));
    }

    Object partFor(Class<?> need, Class<?> asker) {
        if (need == Hardware.class) {
            return asker.isAnnotationPresent(SwitchedOff.class) ? hardware.switchedOff() : hardware;
        }
        if (need == Prints.class) {
            return channels.channel(asker.getSimpleName());
        }
        if (supplied.containsKey(need)) {
            return supplied.get(need);
        }
        return build(standingFor(need));
    }

    void teleOp() {
        for (SubSystem subsystem : tickOrder) {
            subsystem.teleOp(gamepads);
        }
    }

    @Override
    public void loop() {
        for (SubSystem subsystem : tickOrder) {
            subsystem.loop();
        }
    }

    private Class<?> standingFor(Class<?> need) {
        if (!need.isInterface() && !Modifier.isAbstract(need.getModifiers())) {
            return need;
        }
        List<Class<?>> implementing = discovered.implementing(need);
        if (implementing.size() == 1) {
            return implementing.get(0);
        }
        String kind = need.getSimpleName();
        if (implementing.isEmpty()) {
            throw new IllegalStateException(
                    neededBy() + "a " + kind + ", and no class it found implements " + kind);
        }
        List<String> names = new ArrayList<>();
        for (Class<?> each : implementing) {
            names.add(each.getSimpleName());
        }
        throw new IllegalStateException(neededBy() + "a " + kind + ", and " + String.join(" and ", names)
                + " both implement it: the robot builds an interface only when one class it found implements it");
    }

    private Object build(Class<?> kind) {
        Object already = built.get(kind);
        if (already != null) {
            return already;
        }
        if (!discovered.has(kind)) {
            throw new IllegalStateException(neededBy() + "a " + kind.getSimpleName()
                    + ", which the robot does not supply, and it builds only the classes it found in Reginald's code");
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
            args[i] = partFor(needs[i], kind);
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
