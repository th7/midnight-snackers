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

/**
 * A subsystem is wired in by hand: a field on {@link Robot} and a place in the list that sets loop
 * order, or a field on an {@link OpMode} that the op mode ticks itself. Forgetting either still
 * compiles and still runs -- the subsystem simply never ticks, silently. Loop order is a decision
 * a person makes and Robot spells out; that every subsystem is in it at all is mechanical, so this
 * proves it here.
 *
 * <p>What counts as a subsystem is the package: {@code teamcode} holds the Robot and nothing but
 * the subsystems it is made of, so a {@link Loopable} that lives directly in it is one. That used
 * to be a base class to extend; the package says the same thing and is the rule the glossary
 * already states.
 */
public class SubSystemsAreTickedTest {

    /** A subsystem written but never wired in, for the check below to catch. */
    private static class Stray implements Loopable {
        @Override
        public void loop() {}
    }

    /** Of these subsystems, the ones no op mode ticks. */
    private static List<Class<?>> untickedAmong(Collection<Class<?>> subSystems, Set<Class<?>> ticked) {
        return subSystems.stream().filter(type -> !ticked.contains(type)).collect(Collectors.toList());
    }

    /** Every subsystem in the robot code: concrete, ours, and not a test's own fake. */
    private static List<Class<?>> subSystems() {
        List<Class<?>> subSystems = new ArrayList<>();
        for (Class<?> type : Classpath.classesUnder(SimCatalog.TEAMCODE_PACKAGE)) {
            if (type.getName().startsWith(SimCatalog.ROADRUNNER_PACKAGE + ".")) {
                continue; // vendored Road Runner code is not ours to wire in
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

    /** What the op modes on the driver station tick between them, each initialised as a run would. */
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
            // and whatever the op mode owns and ticks itself, in its own onLoop
            ticked.addAll(ownLoopables(opMode));
        }
        return ticked;
    }

    /**
     * The {@link Loopable}s an op mode holds as its own: a TeleOp's Driver, an auto's PlanRunner.
     * The op mode ticks them in its own {@code onLoop}, which is where anything it owns is ticked.
     */
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

    /**
     * The check above passes when nothing is missing and when it looked at nothing at all. This
     * says which: the subsystems Robot builds, and one an op mode adds.
     */
    @Test
    public void findsTheSubSystemsToCheck() {
        List<Class<?>> subSystems = subSystems();

        assertTrue(subSystems.toString(), subSystems.containsAll(List.of(Launcher.class, Drive.class, Brain.class)));
        assertTrue(subSystems.toString(), subSystems.contains(Driver.class));
        assertTrue(subSystems.toString(), subSystems.contains(Plans.class));
    }

    /** And that it would fail: a subsystem no op mode ticks is reported, not passed over. */
    @Test
    public void noticesASubSystemNoOpModeTicks() {
        List<Class<?>> candidates = new ArrayList<>(subSystems());
        candidates.add(Stray.class);

        assertEquals(List.of(Stray.class), untickedAmong(candidates, tickedByAnOpMode()));
    }
}
