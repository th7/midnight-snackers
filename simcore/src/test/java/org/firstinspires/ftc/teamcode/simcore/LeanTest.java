package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.Optional;
import org.junit.Test;

public class LeanTest {
    private static final Tilt THIRTY = Valid.value(Tilt.of(30));

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    @Test
    public void aRestingHiveLeansWhereItIsAndIsStill() {
        Lean resting = new Lean.Resting(THIRTY);

        assertEquals(30, resting.degrees(), 0);
        assertEquals(0, resting.degreesPerSecond(), 0);
        assertSame(resting, resting.after(seconds(5)));
        assertEquals(Optional.empty(), resting.tippingTo());
    }

    @Test
    public void aTipStartsWhereTheHiveRestedAndHeadsForTheOtherSideOfLevel() {
        Lean tipping = new Lean.Resting(THIRTY).tipped();

        assertEquals(30, tipping.degrees(), 0);
        assertEquals(Optional.of(THIRTY.opposite()), tipping.tippingTo());
        assertEquals(0, tipping.after(seconds(Lean.TIP_SECONDS / 2)).degrees(), 1e-9);
    }

    @Test
    public void aTipThatHasRunItsCourseRestsOnTheOtherSide() {
        Lean tipped = new Lean.Resting(THIRTY).tipped().after(seconds(Lean.TIP_SECONDS));

        assertEquals(new Lean.Resting(THIRTY.opposite()), tipped);
    }

    @Test
    public void aHiveThatIsTippingIsNotTippedAgain() {
        Lean tipping = new Lean.Resting(THIRTY).tipped().after(seconds(0.3));

        assertSame(tipping, tipping.tipped());
    }

    @Test
    public void aTimeIsFiniteAndNotBelowZero() {
        assertEquals(0.5, seconds(0.5).value(), 0);
        assertEquals(0.25, seconds(1).dividedInto(4).value(), 0);
        assertEquals(1, seconds(1).dividedInto(0).value(), 0);
        assertEquals(
                Double.MAX_VALUE,
                seconds(Double.MAX_VALUE).plus(seconds(Double.MAX_VALUE)).value(),
                0);
        assertTrue(Seconds.of(-0.1) instanceof Checked.Rejected);
        assertTrue(Seconds.of(Double.POSITIVE_INFINITY) instanceof Checked.Rejected);
        assertTrue(Seconds.of(Double.NaN) instanceof Checked.Rejected);
    }
}
