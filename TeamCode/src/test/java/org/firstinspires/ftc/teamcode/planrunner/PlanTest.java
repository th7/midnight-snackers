package org.firstinspires.ftc.teamcode.planrunner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import org.junit.Test;

public class PlanTest {
    private final ArrayList<String> started = new ArrayList<>();

    private Step instantStep(String name) {
        return new Step(name, () -> started.add(name), () -> true);
    }

    @Test
    public void advancesOnePartPerTickAndOnlyThenReportsFinished() {
        Plan plan = new Plan(instantStep("first"), instantStep("second"));

        assertFalse(plan.tick());
        assertFalse(plan.tick());
        assertTrue(plan.tick());
    }

    @Test
    public void startsPartsInOrder() {
        Plan plan = new Plan(instantStep("first"), instantStep("second"));

        plan.tick();
        assertEquals(1, started.size());
        plan.tick();

        assertEquals("first", started.get(0));
        assertEquals("second", started.get(1));
    }

    @Test
    public void doesNotAdvancePastAPartThatIsNotDone() {
        boolean[] finished = {false};
        Step blocking = new Step("blocking", () -> started.add("blocking"), () -> finished[0]);
        Plan plan = new Plan(blocking, instantStep("after"));

        assertFalse(plan.tick());
        assertFalse(plan.tick());
        assertEquals(1, started.size());

        finished[0] = true;
        assertFalse(plan.tick());
        assertFalse(plan.tick());
        assertEquals(2, started.size());
    }

    @Test
    public void currentStepShowsIndexAndPartName() {
        Plan plan = new Plan(instantStep("first"), instantStep("second"));

        assertEquals("0. first", plan.currentStep());
        plan.tick();
        assertEquals("1. second", plan.currentStep());
    }

    @Test
    public void nestedPlansRunTheirPartsBeforeTheOuterPlanMovesOn() {
        Plan inner = new Plan(instantStep("inner1"), instantStep("inner2"));
        Plan plan = new Plan(inner, instantStep("outer"));

        while (!plan.tick()) {}

        assertEquals(3, started.size());
        assertEquals("inner1", started.get(0));
        assertEquals("inner2", started.get(1));
        assertEquals("outer", started.get(2));
    }

    @Test
    public void emptyPlanIsImmediatelyDone() {
        assertTrue(new Plan().tick());
    }
}
