package org.firstinspires.ftc.teamcode.planrunner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PlanRunnerTest {
    private final PlanRunner planRunner = new PlanRunner();

    private Step instantStep(String name) {
        return new Step(name, () -> {
        }, () -> true);
    }

    @Test
    public void doneWithNothingRunning() {
        assertTrue(planRunner.done());
        assertEquals("no plan", planRunner.currentStep());
    }

    @Test
    public void runsAPlanToCompletionAcrossLoops() {
        planRunner.run(new Plan(instantStep("only")));

        assertFalse(planRunner.done());
        planRunner.loop();
        assertFalse(planRunner.done());
        planRunner.loop();
        assertTrue(planRunner.done());
    }

    @Test
    public void ignoresANewPlanWhileBusy() {
        planRunner.run(new Plan(instantStep("first")));
        planRunner.run(new Plan(instantStep("second")));

        assertEquals("0. first", planRunner.currentStep());
    }

    @Test
    public void cancelDropsTheCurrentPlan() {
        planRunner.run(new Plan(instantStep("only")));
        assertFalse(planRunner.done());

        planRunner.cancel();

        assertTrue(planRunner.done());
        assertEquals("no plan", planRunner.currentStep());
    }

    @Test
    public void loopWithNoPlanIsANoOp() {
        planRunner.loop();
        assertTrue(planRunner.done());
    }
}
