package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

public class SurfacesTest {
    private static final Bounce FLOOR = bounce(0.55, 0.5, 0.2);
    private static final Bounce WALL = bounce(0.5, 0.3, 0.15);
    private static final Bounce FRAME = bounce(0.62, 0.35, 0.15);
    private static final Bounce FLOWER = bounce(0.56, 0.36, 0.16);
    private static final Bounce CELL = bounce(0.51, 0.31, 0.14);
    private static final Bounce PIECE = bounce(0.57, 0.29, 0.13);

    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    private static Bounce bounce(double restitution, double friction, double grip) {
        return Valid.value(Bounce.of(restitution, friction, grip));
    }

    private static Checked<Surfaces> surfaces(
            double loss, double threshold, double impactTwist, double spinTwist, double rollingResistance) {
        return Surfaces.of(
                FLOOR, WALL, FRAME, FLOWER, CELL, PIECE, loss, threshold, impactTwist, spinTwist, rollingResistance);
    }

    private static Surfaces withLoss(double loss) {
        return Valid.value(surfaces(loss, 4, 0.1, 0.06, 0.04));
    }

    @Test
    public void eachSurfaceBouncesAsItWasGiven() {
        Surfaces surfaces = withLoss(0.038);

        assertSame(FLOOR, surfaces.of(Surface.FLOOR));
        assertSame(WALL, surfaces.of(Surface.WALL));
        assertSame(FRAME, surfaces.of(Surface.FRAME));
        assertSame(FLOWER, surfaces.of(Surface.FLOWER));
        assertSame(CELL, surfaces.of(Surface.CELL));
        assertSame(PIECE, surfaces.of(Surface.PIECE));
    }

    @Test
    public void theContactsNumbersAreKept() {
        Surfaces surfaces = Valid.value(surfaces(0.038, 4, 0.1, 0.06, 0.04));

        assertEquals(0.038, surfaces.loss(), 0);
        assertEquals(4, surfaces.threshold(), 0);
        assertEquals(0.1, surfaces.impactTwist(), 0);
        assertEquals(0.06, surfaces.spinTwist(), 0);
        assertEquals(0.04, surfaces.rollingResistance(), 0);
    }

    @Test
    public void aSurfaceBouncesAtItsOwnRestitutionAtThePickleballsDropSpeed() {
        for (double loss = 0; loss <= 0.2; loss += 0.01) {
            for (Surface surface : Surface.values()) {
                Surfaces surfaces = withLoss(loss);
                assertEquals(
                        surface + " at loss " + loss,
                        surfaces.of(surface).restitution(),
                        surfaces.restitution(surface, Surfaces.REFERENCE_APPROACH_IN_PER_S),
                        0);
            }
        }
        assertEquals(244, Surfaces.REFERENCE_APPROACH_IN_PER_S, 0);
    }

    @Test
    public void aSurfaceLosesTheLossForEachHundredInchesASecondFasterItIsMet() {
        Surfaces surfaces = withLoss(0.038);

        assertEquals(0.55 - 0.038 * 2.56, surfaces.restitution(Surface.FLOOR, 500), 1e-12);
        assertEquals(0.55 + 0.038 * 1.44, surfaces.restitution(Surface.FLOOR, 100), 1e-12);
        assertEquals(surfaces.restitution(Surface.FLOOR, 100), surfaces.restitution(Surface.FLOOR, -100), 0);
    }

    @Test
    public void aBallThatKeptLessThanAllOfItsApproachNeverKeepsAllOfIt() {
        for (int hundredths = 0; hundredths < 100; hundredths++) {
            double reference = hundredths / 100.0;
            for (int thousandths = 0; thousandths <= 200; thousandths += 5) {
                double loss = thousandths / 1000.0;
                Surfaces surfaces = Valid.value(Surfaces.of(
                        bounce(reference, 0.5, 0.2), WALL, FRAME, FLOWER, CELL, PIECE, loss, 4, 0.1, 0.06, 0.04));
                double before = Double.POSITIVE_INFINITY;
                for (int approach = 0; approach <= 5000; approach += 2) {
                    double kept = surfaces.restitution(Surface.FLOOR, approach);
                    boolean below = kept < 1
                            && kept >= Math.min(reference, Surfaces.LEAST_RESTITUTION)
                            && kept <= Math.max(reference, Surfaces.MOST_RESTITUTION)
                            && kept <= before;
                    if (!below) {
                        fail("e_ref " + reference + ", loss " + loss + ", at " + approach + " in/s: " + kept + " after "
                                + before);
                    }
                    before = kept;
                }
            }
        }
    }

    @Test
    public void theClampHoldsRestitutionBetweenATenthAndNineTenthsUnlessTheSurfaceIsOutsideThem() {
        Surfaces lossy = Valid.value(Surfaces.of(
                bounce(0.55, 0.5, 0.2),
                bounce(1, 0.3, 0.15),
                bounce(0.05, 0.3, 0.1),
                FLOWER,
                CELL,
                PIECE,
                0.2,
                4,
                0,
                0,
                0));

        assertEquals(0.1, lossy.restitution(Surface.FLOOR, 10000), 0);
        assertEquals(0.9, lossy.restitution(Surface.FLOOR, 0), 0);
        assertEquals(1, lossy.restitution(Surface.WALL, 0), 0);
        assertEquals(0.05, lossy.restitution(Surface.FRAME, 10000), 0);
        assertEquals(0.1, Surfaces.LEAST_RESTITUTION, 0);
        assertEquals(0.9, Surfaces.MOST_RESTITUTION, 0);
    }

    @Test
    public void withNoLossASurfaceBouncesAtItsOwnRestitutionAtEverySpeed() {
        Surfaces lossless = withLoss(0);

        for (double approach = 0; approach < 3000; approach += 7) {
            assertEquals(0.62, lossless.restitution(Surface.FRAME, approach), 0);
        }
    }

    @Test
    public void theContactsNumbersAreFiniteAndNotBelowNothing() {
        assertRejected("bounce loss is a finite number not below 0", surfaces(-0.01, 4, 0.1, 0.06, 0.04));
        assertRejected("bounce loss is a finite number not below 0", surfaces(Double.NaN, 4, 0.1, 0.06, 0.04));
        assertRejected("bounce threshold is a positive, finite speed", surfaces(0.038, 0, 0.1, 0.06, 0.04));
        assertRejected(
                "bounce threshold is a positive, finite speed",
                surfaces(0.038, Double.POSITIVE_INFINITY, 0.1, 0.06, 0.04));
        assertRejected("impact twist is a finite lever not below 0", surfaces(0.038, 4, -0.1, 0.06, 0.04));
        assertRejected("impact twist is a finite lever not below 0", surfaces(0.038, 4, Double.NaN, 0.06, 0.04));
        assertRejected("spin twist is a finite lever not below 0", surfaces(0.038, 4, 0.1, -0.06, 0.04));
        assertRejected(
                "spin twist is a finite lever not below 0", surfaces(0.038, 4, 0.1, Double.POSITIVE_INFINITY, 0.04));
        assertRejected("rolling resistance is a finite number not below 0", surfaces(0.038, 4, 0.1, 0.06, -0.04));
        assertRejected("rolling resistance is a finite number not below 0", surfaces(0.038, 4, 0.1, 0.06, Double.NaN));
    }

    @Test
    public void theEdgesOfEachRangeAreSurfaces() {
        Surfaces least = Valid.value(surfaces(0, 0.5, 0, 0, 0));

        assertEquals(0, least.loss(), 0);
        assertEquals(0.5, least.threshold(), 0);
        assertEquals(0, least.rollingResistance(), 0);
    }

    @Test
    public void surfacesAreEqualWhenAllTheirNumbersAre() {
        assertEquals(withLoss(0.038), withLoss(0.038));
        assertEquals(withLoss(0.038).hashCode(), withLoss(0.038).hashCode());
        assertNotEquals(withLoss(0.038), withLoss(0.039));
        assertTrue(withLoss(0.038).toString().contains("0.038"));
    }
}
