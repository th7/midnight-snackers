package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class SubSystemTest {
    private final List<String> ticks = new ArrayList<>();

    /** A subsystem with one registered helper, the way Launcher owns a PlanRunner. */
    private class WithHelper extends SubSystem {
        final Loopable helper = add(() -> ticks.add("helper"));

        @Override
        protected void onLoop() {
            ticks.add("onLoop");
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
}
