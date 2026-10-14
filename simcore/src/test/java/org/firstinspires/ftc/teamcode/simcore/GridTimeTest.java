package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class GridTimeTest {
    private static final Bodies.Step STEP = Valid.value(Flight.stepping());

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted", rule -> rule);
    }

    @Test
    public void aPartOfAStepIsAtLeastNoneAndLessThanAWholeStep() {
        for (double outside : new double[] {-1e-12, 1, Double.NaN, Double.POSITIVE_INFINITY}) {
            String said = rejection(GridTime.PartOfAStep.of(outside));

            assertTrue(said, said.contains("a part of a step is at least 0 and less than 1, not " + outside));
        }
        assertEquals("accepted", rejection(GridTime.PartOfAStep.of(0)));
        assertEquals("accepted", rejection(GridTime.PartOfAStep.of(Math.nextDown(1.0))));
    }

    @Test
    public void aTimeInSecondsIsTheWholeStepsAtOrBeforeItAndThePartOfAStepAfter() {
        GridTime later = Valid.value(GridTime.of(0.00025, STEP));
        GridTime earlier = Valid.value(GridTime.of(-0.00025, STEP));

        assertEquals(2, later.steps());
        assertEquals(0.5, later.part().value(), 1e-9);
        assertEquals(-3, earlier.steps());
        assertEquals(0.5, earlier.part().value(), 1e-9);
        for (double nowhere : new double[] {Double.NaN, Double.POSITIVE_INFINITY, 1e300}) {
            String said = rejection(GridTime.of(nowhere, STEP));

            assertTrue(said, said.contains("is a finite number of steps a long can count, not " + nowhere + " s"));
        }
    }

    @Test
    public void aTimeLaterCarriesItsPartsOfAStepIntoWholeStepsAndStopsAtTheLastStepALongCanCount() {
        GridTime later = GridTime.partWay(5, Valid.value(GridTime.PartOfAStep.of(0.75)), STEP)
                .after(Valid.value(Seconds.of(0.0005)));

        assertEquals(10, later.steps());
        assertEquals(0.75, later.part().value(), 1e-9);
        assertEquals(
                Long.MAX_VALUE,
                GridTime.at(5, STEP)
                        .after(Valid.value(Seconds.of(Double.MAX_VALUE)))
                        .steps());
    }

    @Test
    public void theTimeSinceAStepIsTheWholeStepsSinceItTimesTheStepWhereverOnTheStepsItBegan() {
        for (long began : List.of(0L, 45L, 46L, 66L, 187L, 1235L, 123_456_789L)) {
            assertEquals(1.0, GridTime.at(began + 10_000, STEP).secondsSince(began), 0);
            assertEquals(
                    0.5 * STEP.seconds().value(),
                    GridTime.partWay(began, Valid.value(GridTime.PartOfAStep.of(0.5)), STEP)
                            .secondsSince(began),
                    0);
        }
    }
}
