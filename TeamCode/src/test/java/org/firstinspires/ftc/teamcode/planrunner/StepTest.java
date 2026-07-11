package org.firstinspires.ftc.teamcode.planrunner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;

public class StepTest {
    private long nanoNow = 0;

    @Test
    public void startRunsExactlyOnceOnFirstDoneCall() {
        AtomicInteger startCount = new AtomicInteger();
        Step step = new Step("step", startCount::incrementAndGet, () -> false, () -> nanoNow);

        assertFalse(step.done());
        assertFalse(step.done());
        assertFalse(step.done());

        assertEquals(1, startCount.get());
    }

    @Test
    public void doneReflectsTheSupplier() {
        boolean[] finished = {false};
        Step step = new Step("step", () -> {
        }, () -> finished[0], () -> nanoNow);

        assertFalse(step.done());
        finished[0] = true;
        assertTrue(step.done());
    }

    @Test
    public void timedStepMeasuresElapsedTimeFromItsClock() {
        nanoNow = 5_000_000_000L;
        Step step = new Step("step", () -> {
        }, Step.secondsElapsed(1), () -> nanoNow);

        assertFalse(step.done());

        nanoNow += 999_999_999L;
        assertFalse(step.done());

        nanoNow += 2;
        assertTrue(step.done());
    }

    @Test
    public void currentStepIsTheName() {
        Step step = new Step("step name", () -> {
        }, () -> true, () -> nanoNow);

        assertEquals("step name", step.currentStep());
    }
}
