package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PollenTest {
    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    @Test
    public void aPollenIsItsDiameterItsMassAndHowFarOutItsMassIs() {
        Pollen pollen = Valid.value(Pollen.of(2.8, 24.9, 0.634));

        assertEquals(1.4, pollen.radius(), 1e-12);
        assertEquals(0.0249, pollen.kilograms(), 1e-12);
        assertEquals(0.634, pollen.inertiaFactor(), 0);
        assertEquals(1.4 * Length.METRES_PER_INCH, pollen.radiusMetres(), 1e-15);
    }

    @Test
    public void itsAreaAndVolumeAreASpheres() {
        Pollen pollen = Valid.value(Pollen.of(2.8, 24.9, 0.634));
        double r = 1.4 * Length.METRES_PER_INCH;

        assertEquals(Math.PI * r * r, pollen.areaSquareMetres(), 1e-15);
        assertEquals(4.0 / 3 * Math.PI * r * r * r, pollen.volumeCubicMetres(), 1e-18);
    }

    @Test
    public void aSpheresMassIsNoFurtherOutThanItsSurface() {
        assertEquals(2.0 / 3, Valid.value(Pollen.of(2.8, 24.9, 2.0 / 3)).inertiaFactor(), 0);
        assertRejected("2/3", Pollen.of(2.8, 24.9, 0.7));
        assertRejected("more than 0", Pollen.of(2.8, 24.9, 0));
        assertRejected("more than 0", Pollen.of(2.8, 24.9, Double.NaN));
    }

    @Test
    public void aPollenHasASizeAndAWeight() {
        assertRejected("diameter", Pollen.of(0, 24.9, 0.634));
        assertRejected("diameter", Pollen.of(Double.POSITIVE_INFINITY, 24.9, 0.634));
        assertRejected("weighs", Pollen.of(2.8, 0, 0.634));
        assertRejected("weighs", Pollen.of(2.8, Double.NaN, 0.634));
    }
}
