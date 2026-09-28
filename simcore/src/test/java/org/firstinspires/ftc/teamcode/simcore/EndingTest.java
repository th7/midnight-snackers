package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Optional;
import org.junit.Test;

public class EndingTest {
    private static final Budget THIRTY = Valid.value(Budget.of(30));
    private static final Budget NO_LIMIT = Valid.value(Budget.of(Double.POSITIVE_INFINITY));

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static Optional<Ending> before(boolean stopRequested, Ending.Plan plan, double elapsed, Budget budget) {
        return Ending.before(stopRequested, plan, seconds(elapsed), budget);
    }

    @Test
    public void aRunLoopsAgainUntilSomethingEndsIt() {
        for (Ending.Plan plan : new Ending.Plan[] {Ending.Plan.NONE, Ending.Plan.UNDER_WAY}) {
            assertEquals(plan.toString(), Optional.empty(), before(false, plan, 0, THIRTY));
            assertEquals(plan.toString(), Optional.empty(), before(false, plan, 29.99, THIRTY));
        }
    }

    @Test
    public void stopEndsARunWhateverElseIsTrueOfIt() {
        for (Ending.Plan plan : Ending.Plan.values()) {
            assertEquals(plan.toString(), Optional.of(Ending.STOPPED), before(true, plan, 0, THIRTY));
            assertEquals(plan.toString(), Optional.of(Ending.STOPPED), before(true, plan, 31, THIRTY));
        }
    }

    @Test
    public void anAutoIsDoneWhenItsPlanIsEvenPastItsPeriod() {
        assertEquals(Optional.of(Ending.DONE), before(false, Ending.Plan.DONE, 0, THIRTY));
        assertEquals(Optional.of(Ending.DONE), before(false, Ending.Plan.DONE, 31, THIRTY));
    }

    @Test
    public void anAutoWhosePlanIsNotDoneWithinItsPeriodTimesOut() {
        assertEquals(Optional.of(Ending.TIMED_OUT), before(false, Ending.Plan.UNDER_WAY, 30.02, THIRTY));
    }

    @Test
    public void aTeleOpIsDoneWhenItsPeriodIsOver() {
        assertEquals(Optional.of(Ending.DONE), before(false, Ending.Plan.NONE, 30.02, THIRTY));
    }

    @Test
    public void aPeriodIsOverOnlyOnceARunHasGonePastIt() {
        assertEquals(Optional.empty(), before(false, Ending.Plan.UNDER_WAY, 30, THIRTY));
        assertEquals(Optional.empty(), before(false, Ending.Plan.NONE, 30, THIRTY));
    }

    @Test
    public void inFreePlayNothingEndsARunButItsPlanOrStop() {
        assertEquals(Optional.empty(), before(false, Ending.Plan.UNDER_WAY, Double.MAX_VALUE, NO_LIMIT));
        assertEquals(Optional.empty(), before(false, Ending.Plan.NONE, Double.MAX_VALUE, NO_LIMIT));
        assertEquals(Optional.of(Ending.DONE), before(false, Ending.Plan.DONE, 5, NO_LIMIT));
        assertEquals(Optional.of(Ending.STOPPED), before(true, Ending.Plan.NONE, 5, NO_LIMIT));
    }

    @Test
    public void aBudgetIsATimeOrInfinityForNoLimit() {
        assertEquals(new Budget.Period(seconds(30)), THIRTY);
        assertEquals(new Budget.Period(seconds(0)), Valid.value(Budget.of(0)));
        assertEquals(new Budget.NoLimit(), NO_LIMIT);
        for (double notABudget : new double[] {-0.02, Double.NEGATIVE_INFINITY, Double.NaN}) {
            Checked<Budget> refused = Budget.of(notABudget);
            assertTrue(notABudget + " was taken", refused instanceof Checked.Rejected);
            assertTrue(
                    refused.toString(), refused.fold(budget -> "", rule -> rule).contains("Infinity for no limit"));
        }
    }
}
