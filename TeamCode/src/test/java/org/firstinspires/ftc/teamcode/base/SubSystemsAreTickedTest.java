package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Classpath;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.opmode.OpMode;
import org.firstinspires.ftc.teamcode.sim.SimCatalog;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

public class SubSystemsAreTickedTest {
    private static class Stray implements Loopable {
        @Override
        public void loop() {}
    }

    private static List<Class<?>> untickedAmong(Collection<Class<?>> subSystems, Set<Class<?>> ticked) {
        return subSystems.stream().filter(type -> !ticked.contains(type)).collect(Collectors.toList());
    }

    private static List<Class<?>> subSystems() {
        List<Class<?>> subSystems = new ArrayList<>();
        for (Class<?> type : Classpath.classesUnder(SimCatalog.TEAMCODE_PACKAGE)) {
            if (type.getName().startsWith(SimCatalog.ROADRUNNER_PACKAGE + ".")) {
                continue;
            }
            if (Loopable.class.isAssignableFrom(type)
                    && type.getPackageName().equals(SimCatalog.TEAMCODE_PACKAGE)
                    && type != Robot.class
                    && !Modifier.isAbstract(type.getModifiers())
                    && !Classpath.isTestClass(type)) {
                subSystems.add(type);
            }
        }
        return subSystems;
    }

    private static Set<Class<?>> tickedByAnOpMode() {
        Set<Class<?>> ticked = new LinkedHashSet<>();
        for (SimCatalog.Entry entry : SimCatalog.discover().entries()) {
            OpMode opMode = entry.opMode();
            opMode.useHardware(new SimRobot().hardware());
            opMode.telemetry = new FakeTelemetry();
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad2 = new Gamepad();
            opMode.init();
            for (Loopable loopable : opMode.loopOrder()) {
                ticked.add(loopable.getClass());
            }

            ticked.addAll(ownLoopables(opMode));
        }
        return ticked;
    }

    private static Set<Class<?>> ownLoopables(OpMode opMode) {
        Set<Class<?>> own = new LinkedHashSet<>();
        for (Class<?> type = opMode.getClass(); type != null; type = type.getSuperclass()) {
            for (Field field : type.getDeclaredFields()) {
                if (!Loopable.class.isAssignableFrom(field.getType())) {
                    continue;
                }
                field.setAccessible(true);
                try {
                    Object held = field.get(opMode);
                    if (held != null) {
                        own.add(held.getClass());
                    }
                } catch (IllegalAccessException e) {
                    throw new IllegalStateException(e);
                }
            }
        }
        return own;
    }

    @Test
    public void everySubSystemIsTickedByAnOpMode() {
        List<Class<?>> unticked = untickedAmong(subSystems(), tickedByAnOpMode());

        assertEquals(
                "a subsystem no op mode ticks: add it to Robot's constructor, or to an op mode's init()",
                List.of(),
                unticked);
    }

    @Test
    public void findsTheSubSystemsToCheck() {
        List<Class<?>> subSystems = subSystems();

        assertTrue(subSystems.toString(), subSystems.containsAll(List.of(Launcher.class, Drive.class, Brain.class)));
        assertTrue(subSystems.toString(), subSystems.contains(Driver.class));
        assertTrue(subSystems.toString(), subSystems.contains(Plans.class));
    }

    @Test
    public void noticesASubSystemNoOpModeTicks() {
        List<Class<?>> candidates = new ArrayList<>(subSystems());
        candidates.add(Stray.class);

        assertEquals(List.of(Stray.class), untickedAmong(candidates, tickedByAnOpMode()));
    }
}
