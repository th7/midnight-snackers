package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.junit.Test;

public class SubSystemTest {
    private final List<String> ticks = new ArrayList<>();
    private final FakeTelemetry fakeTelemetry = new FakeTelemetry();

    /** A subsystem with one registered helper, the way Launcher owns a PlanRunner. */
    private class WithHelper extends SubSystem {
        final Loopable helper = add(() -> ticks.add("helper"));

        WithHelper() {
            telemetry = fakeTelemetry;
        }

        @Override
        protected void onInit() {
            ticks.add("onInit");
        }

        @Override
        protected void onLoop() {
            ticks.add("onLoop");
        }

        @Override
        protected void onTelemetry() {
            ticks.add("onTelemetry");
        }
    }

    @Test
    public void ticksRegisteredHelpersBeforeItsOwnWork() {
        new WithHelper().loop();

        assertEquals(List.of("helper", "onLoop"), ticks);
    }

    @Test
    public void ticksHelpersOnEveryLoop() {
        WithHelper subSystem = new WithHelper();

        subSystem.loop();
        subSystem.loop();

        assertEquals(List.of("helper", "onLoop", "helper", "onLoop"), ticks);
    }

    @Test
    public void setsUpItsHardwareWhenItIsInitialised() {
        new WithHelper().init(new FakeTelemetry());

        assertEquals(List.of("onInit"), ticks);
    }

    /** Telemetry costs a loop time and a crowded screen, so a subsystem prints only when asked. */
    @Test
    public void printsNoTelemetryUntilItIsToggledOn() {
        WithHelper subSystem = new WithHelper();

        subSystem.loop();

        assertEquals(List.of("helper", "onLoop"), ticks);
    }

    @Test
    public void printsItsTelemetryAfterItsOwnWorkOnceToggledOn() {
        WithHelper subSystem = new WithHelper();

        subSystem.toggleTelemetry();
        subSystem.loop();

        assertEquals(List.of("helper", "onLoop", "onTelemetry"), ticks);
    }

    @Test
    public void stopsPrintingWhenToggledBackOff() {
        WithHelper subSystem = new WithHelper();

        subSystem.toggleTelemetry();
        subSystem.loop();
        subSystem.toggleTelemetry();
        subSystem.loop();

        assertEquals(List.of("helper", "onLoop", "onTelemetry", "helper", "onLoop"), ticks);
    }

    /** So the screen says whose numbers these are, without every subsystem remembering to. */
    @Test
    public void saysWhichSubsystemIsPrinting() {
        WithHelper subSystem = new WithHelper();

        subSystem.toggleTelemetry();
        subSystem.loop();

        assertEquals(List.of("WithHelper"), fakeTelemetry.captions);
    }

    /**
     * The point of the three hooks: a new subsystem is asked for all of them by the compiler, and
     * cannot answer by overriding the methods that call them at the right moment.
     */
    @Test
    public void asksEverySubsystemWhatItSetsUpDoesAndPrints() throws NoSuchMethodException {
        for (String hook : List.of("onInit", "onLoop", "onTelemetry")) {
            assertTrue(
                    hook + " must be abstract, so the compiler asks for it",
                    Modifier.isAbstract(SubSystem.class.getDeclaredMethod(hook).getModifiers()));
        }
        assertTrue(
                "init must be final, so a subsystem cannot take over when its hooks run",
                Modifier.isFinal(SubSystem.class
                        .getDeclaredMethod("init", Telemetry.class)
                        .getModifiers()));
        for (String caller : List.of("loop", "toggleTelemetry")) {
            assertTrue(
                    caller + " must be final, so a subsystem cannot take over when its hooks run",
                    Modifier.isFinal(SubSystem.class.getDeclaredMethod(caller).getModifiers()));
        }
    }
}
