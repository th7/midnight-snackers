package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

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
        Ball pollen = Valid.value(Ball.of(2.78, 24.9, 0.634));

        assertEquals(40_000, whole.newtonsPerMetre(), 1e-9);
        assertEquals(8_000, fifth.newtonsPerMetre(), 1e-9);
        assertEquals(whole.dampingFor(pollen) / 5, fifth.dampingFor(pollen), 1e-15);
        assertEquals(whole.friction(), fifth.friction(), 0);
        assertEquals(whole.regularisation(), fifth.regularisation(), 0);
        assertEquals("a compliant surface is shared by at least one ring, not 0", rejection(whole.sharedBy(0)));
        assertEquals("a compliant surface is shared by at least one ring, not -2", rejection(whole.sharedBy(-2)));
    }

    private static double returnedAt(double ratio) {
        if (ratio < 1) {
            return StrictMath.exp(-2 * ratio * StrictMath.acos(ratio) / StrictMath.sqrt(1 - ratio * ratio));
        }
        double acosh = StrictMath.log(ratio + StrictMath.sqrt(ratio * ratio - 1));
        return StrictMath.exp(-2 * ratio * acosh / StrictMath.sqrt(ratio * ratio - 1));
    }

    @Test
    public void aSurfacesDampingRatioIsTheOneAtWhichASpringAndDashpotThatNeverPullsReturnsABallAtItsRestitution() {
        Ball pollen = Valid.value(Ball.of(2.78, 24.9, 0.634));
        for (double restitution : new double[] {0.01, 0.1, 0.2, 0.6, 0.9, 0.999}) {
            Compliance surface = compliance(restitution);
            double ratio =
                    surface.dampingFor(pollen) / (2 * StrictMath.sqrt(surface.newtonsPerMetre() * pollen.kilograms()));
            assertEquals(
                    Compliance.DAMPING_SOURCE + ": a damping ratio of " + ratio,
                    restitution,
                    returnedAt(ratio),
                    1e-12 * restitution);
        }
        assertEquals(0, compliance(1).dampingFor(pollen), 0);
        for (double restitution : new double[] {Double.MIN_VALUE, 1e-300, StrictMath.exp(-2), 1 - 1e-16}) {
            double damping = compliance(restitution).dampingFor(pollen);
            assertTrue("a damping of " + damping, damping > 0 && Double.isFinite(damping));
        }
    }
}
