package org.firstinspires.ftc.teamcode.planrunner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.Test;

public class StepTest {
    private long nanoNow = 0;

    @Test
    public void startRunsExactlyOnceOnTheFirstTick() {
        AtomicInteger startCount = new AtomicInteger();
        Step step = new Step("step", startCount::incrementAndGet, () -> false, () -> nanoNow);

        assertFalse(step.tick());
        assertFalse(step.tick());
        assertFalse(step.tick());

        assertEquals(1, startCount.get());
    }

    @Test
    public void aTickAnswersWhatTheStepsOwnDoneSupplierSays() {
        boolean[] finished = {false};
        Step step = new Step("step", () -> {}, () -> finished[0], () -> nanoNow);

        assertFalse(step.tick());
        finished[0] = true;
        assertTrue(step.tick());
    }

    @Test
    public void timedStepMeasuresElapsedTimeFromItsClock() {
        nanoNow = 5_000_000_000L;
        Step step = new Step("step", () -> {}, Step.secondsElapsed(1), () -> nanoNow);

        assertFalse(step.tick());

        nanoNow += 999_999_999L;
        assertFalse(step.tick());

        nanoNow += 2;
        assertTrue(step.tick());
    }

    @Test
    public void waitForCountsOnTheGivenClock() {
        nanoNow = 7_000_000_000L;
        Step step = Step.waitFor("settle", 0.25, () -> nanoNow);

        assertFalse(step.tick());
        nanoNow += 250_000_000L;
        assertFalse(step.tick());

        nanoNow += 1;
        assertTrue(step.tick());
        assertEquals("settle waitFor 0.25", step.currentStep());
    }

    @Test
    public void currentStepIsTheName() {
        Step step = new Step("step name", () -> {}, () -> true, () -> nanoNow);

        assertEquals("step name", step.currentStep());
    }
}
