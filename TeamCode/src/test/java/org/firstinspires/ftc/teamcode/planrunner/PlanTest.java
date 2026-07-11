package org.firstinspires.ftc.teamcode.planrunner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;

public class PlanTest {
    private final ArrayList<String> started = new ArrayList<>();

    private Step instantStep(String name) {
        return new Step(name, () -> started.add(name), () -> true);
    }

    @Test
    public void advancesOnePartPerDoneCallAndOnlyThenReportsDone() {
        Plan plan = new Plan(instantStep("first"), instantStep("second"));

        assertFalse(plan.done());
        assertFalse(plan.done());
        assertTrue(plan.done());
    }

    @Test
    public void startsPartsInOrder() {
        Plan plan = new Plan(instantStep("first"), instantStep("second"));

        plan.done();
        assertEquals(1, started.size());
        plan.done();

        assertEquals("first", started.get(0));
        assertEquals("second", started.get(1));
    }

    @Test
    public void doesNotAdvancePastAPartThatIsNotDone() {
        boolean[] finished = {false};
        Step blocking = new Step("blocking", () -> started.add("blocking"), () -> finished[0]);
        Plan plan = new Plan(blocking, instantStep("after"));

        assertFalse(plan.done());
        assertFalse(plan.done());
        assertEquals(1, started.size());

        finished[0] = true;
        assertFalse(plan.done());
        assertFalse(plan.done());
        assertEquals(2, started.size());
    }

    @Test
    public void currentStepShowsIndexAndPartName() {
        Plan plan = new Plan(instantStep("first"), instantStep("second"));

        assertEquals("0. first", plan.currentStep());
        plan.done();
        assertEquals("1. second", plan.currentStep());
    }

    @Test
    public void nestedPlansRunTheirPartsBeforeTheOuterPlanMovesOn() {
        Plan inner = new Plan(instantStep("inner1"), instantStep("inner2"));
        Plan plan = new Plan(inner, instantStep("outer"));

        while (!plan.done()) {
        }

        assertEquals(3, started.size());
        assertEquals("inner1", started.get(0));
        assertEquals("inner2", started.get(1));
        assertEquals("outer", started.get(2));
    }

    @Test
    public void emptyPlanIsImmediatelyDone() {
        assertTrue(new Plan().done());
    }
}
