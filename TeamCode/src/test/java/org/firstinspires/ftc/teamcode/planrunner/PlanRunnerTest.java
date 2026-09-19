package org.firstinspires.ftc.teamcode.planrunner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PlanRunnerTest {
    private final PlanRunner planRunner = new PlanRunner();
    private final java.util.List<String> started = new java.util.ArrayList<>();

    private Step instantStep(String name) {
        return new Step(name, () -> {}, () -> true);
    }

    private Step recordingStep(String name) {
        return new Step(name, () -> started.add(name), () -> true);
    }

    @Test
    public void askingWhetherItIsDoneAndWhatStepItIsOnAdvancesNothingHoweverOftenAnyoneAsks() {
        planRunner.run(new Plan(recordingStep("first"), recordingStep("second")));

        for (int asked = 0; asked < 10; asked++) {
            planRunner.done();
            planRunner.currentStep();
        }
        planRunner.loop();

        assertEquals(java.util.List.of("first"), started);
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
