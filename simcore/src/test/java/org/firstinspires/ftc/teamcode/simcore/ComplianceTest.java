package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ComplianceTest {
    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted", rule -> rule);
    }

    private static Compliance compliance(double restitution) {
        return Valid.value(Compliance.of(40, restitution, 0.6, 0.02));
    }

    @Test
    public void theLaunchersSurfacesAreAsStiffBouncyGrippyAndRegularisedAsTheirSourcesSay() {
        assertEquals(Compliance.STIFFNESS_SOURCE, 40, Compliance.STIFFNESS_N_PER_MM, 0);
        assertEquals(Compliance.RESTITUTION_SOURCE, 0.6, Compliance.RESTITUTION, 0);
        assertEquals(Compliance.FRICTION_SOURCE, 0.6, Compliance.WHEEL_FRICTION, 0);
        assertEquals(Compliance.FRICTION_SOURCE, 0.3, Compliance.HOOD_FRICTION, 0);
        assertEquals(Compliance.REGULARISATION_SOURCE, 0.02, Compliance.REGULARISATION_M_PER_S, 0);
        for (double friction : new double[] {Compliance.WHEEL_FRICTION, Compliance.HOOD_FRICTION}) {
            assertEquals(
                    "accepted",
                    rejection(Compliance.of(
                            Compliance.STIFFNESS_N_PER_MM,
                            Compliance.RESTITUTION,
                            friction,
                            Compliance.REGULARISATION_M_PER_S)));
        }
    }

    @Test
    public void
            aCompliantSurfaceIsStiffBouncesBackSomeOfAnImpactGripsNoLessThanNotAtAllAndIsRegularisedOverSomeSpeed() {
        String stiffness = "a compliant surface's stiffness is a positive, finite number of newtons a millimetre, not ";
        String restitution = "a compliant surface's restitution is more than 0 and at most 1, not ";
        String friction = "a compliant surface's friction coefficient is a finite number not below 0, not ";
        String regularisation =
                "a compliant surface's regularisation speed is a positive, finite number of metres a second, not ";

        assertEquals(stiffness + "0.0", rejection(Compliance.of(0, 0.6, 0.6, 0.02)));
        assertEquals(stiffness + "-1.0", rejection(Compliance.of(-1, 0.6, 0.6, 0.02)));
        assertEquals(stiffness + "NaN", rejection(Compliance.of(Double.NaN, 0.6, 0.6, 0.02)));
        assertEquals(stiffness + "Infinity", rejection(Compliance.of(Double.POSITIVE_INFINITY, 0.6, 0.6, 0.02)));
        assertEquals(restitution + "0.0", rejection(Compliance.of(40, 0, 0.6, 0.02)));
        assertEquals(restitution + "-0.1", rejection(Compliance.of(40, -0.1, 0.6, 0.02)));
        assertEquals(restitution + "1.1", rejection(Compliance.of(40, 1.1, 0.6, 0.02)));
        assertEquals(restitution + "NaN", rejection(Compliance.of(40, Double.NaN, 0.6, 0.02)));
        assertEquals(friction + "-0.1", rejection(Compliance.of(40, 0.6, -0.1, 0.02)));
        assertEquals(friction + "NaN", rejection(Compliance.of(40, 0.6, Double.NaN, 0.02)));
        assertEquals(friction + "Infinity", rejection(Compliance.of(40, 0.6, Double.POSITIVE_INFINITY, 0.02)));
        assertEquals(regularisation + "0.0", rejection(Compliance.of(40, 0.6, 0.6, 0)));
        assertEquals(regularisation + "-0.02", rejection(Compliance.of(40, 0.6, 0.6, -0.02)));
        assertEquals(regularisation + "NaN", rejection(Compliance.of(40, 0.6, 0.6, Double.NaN)));
        assertEquals(regularisation + "Infinity", rejection(Compliance.of(40, 0.6, 0.6, Double.POSITIVE_INFINITY)));
        assertEquals("accepted", rejection(Compliance.of(1e-6, 1, 0, 1e-9)));
    }

    @Test
    public void aSurfaceSampledByRingsSharesItsStiffnessAndItsDampingAmongThemAndIsSharedByAtLeastOne() {
        Compliance whole = compliance(0.6);
        Compliance fifth = Valid.value(whole.sharedBy(5));
        double kilograms = 0.0249;

        assertEquals(40_000, whole.newtonsPerMetre(), 1e-9);
        assertEquals(8_000, fifth.newtonsPerMetre(), 1e-9);
        assertEquals(whole.dampingFor(kilograms) / 5, fifth.dampingFor(kilograms), 1e-15);
        assertEquals(whole.friction(), fifth.friction(), 0);
        assertEquals(whole.regularisation(), fifth.regularisation(), 0);
        assertEquals("a compliant surface is shared by at least one ring, not 0", rejection(whole.sharedBy(0)));
        assertEquals("a compliant surface is shared by at least one ring, not -2", rejection(whole.sharedBy(-2)));
    }

    @Test
    public void aSurfacesDampingIsWhatMakesABallOfItsMassStrikingItAloneComeBackAtItsRestitution() {
        double kilograms = 0.0249;
        for (double restitution : new double[] {0.2, 0.6, 0.9, 1}) {
            Compliance surface = compliance(restitution);
            double k = surface.newtonsPerMetre(), c = surface.dampingFor(kilograms);
            double ratio = c / (2 * StrictMath.sqrt(k * kilograms));
            double decayed = StrictMath.exp(-StrictMath.PI * ratio / StrictMath.sqrt(1 - ratio * ratio));
            assertEquals("a linear spring and dashpot of damping ratio " + ratio, restitution, decayed, 1e-12);
        }
    }
}
