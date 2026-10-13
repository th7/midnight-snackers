package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class BounceTest {
    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    @Test
    public void aBounceIsWhatABallKeepsOfItsApproachHowHardTheSurfaceRubsAndHowMuchOfItsSlipComesBack() {
        Bounce bounce = Valid.value(Bounce.of(0.55, 0.5, 0.2));

        assertEquals(0.55, bounce.restitution(), 0);
        assertEquals(0.5, bounce.friction(), 0);
        assertEquals(0.2, bounce.grip(), 0);
    }

    @Test
    public void theEdgesOfEachRangeAreBounces() {
        assertEquals(0, Valid.value(Bounce.of(0, 0, 0)).restitution(), 0);
        assertEquals(1, Valid.value(Bounce.of(1, 1.5, 1)).grip(), 0);
        assertEquals(40, Valid.value(Bounce.of(1, 40, 1)).friction(), 0);
    }

    @Test
    public void aBallKeepsBetweenNoneAndAllOfItsApproach() {
        assertRejected("restitution is between 0 and 1", Bounce.of(1.01, 0.5, 0.2));
        assertRejected("restitution is between 0 and 1", Bounce.of(-0.01, 0.5, 0.2));
        assertRejected("restitution is between 0 and 1", Bounce.of(Double.NaN, 0.5, 0.2));
    }

    @Test
    public void aSurfaceRubsAFiniteAmountNotBelowNothing() {
        assertRejected("friction is a finite number not below 0", Bounce.of(0.55, -0.1, 0.2));
        assertRejected("friction is a finite number not below 0", Bounce.of(0.55, Double.NaN, 0.2));
        assertRejected("friction is a finite number not below 0", Bounce.of(0.55, Double.POSITIVE_INFINITY, 0.2));
    }

    @Test
    public void aGripTurnsBackBetweenNoneAndAllOfTheSlip() {
        assertRejected("grip is between 0 and 1", Bounce.of(0.55, 0.5, -0.1));
        assertRejected("grip is between 0 and 1", Bounce.of(0.55, 0.5, 1.1));
        assertRejected("grip is between 0 and 1", Bounce.of(0.55, 0.5, Double.NaN));
    }

    @Test
    public void bouncesAreEqualWhenTheirNumbersAre() {
        assertEquals(Valid.value(Bounce.of(0.55, 0.5, 0.2)), Valid.value(Bounce.of(0.55, 0.5, 0.2)));
        assertEquals(
                Valid.value(Bounce.of(0.55, 0.5, 0.2)).hashCode(),
                Valid.value(Bounce.of(0.55, 0.5, 0.2)).hashCode());
        assertNotEquals(Valid.value(Bounce.of(0.55, 0.5, 0.2)), Valid.value(Bounce.of(0.55, 0.5, 0.15)));
        assertTrue(Valid.value(Bounce.of(0.55, 0.5, 0.2)).toString().contains("0.55"));
    }
}
