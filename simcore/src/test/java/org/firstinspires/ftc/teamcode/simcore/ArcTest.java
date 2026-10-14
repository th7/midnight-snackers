package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ArcTest {
    private static final double WITHIN = 1e-12;
    private static final Vec3 CENTRE = new Vec3(1, 2, 3);
    private static final Vec3 AXLE = new Vec3(0, 1, 0);
    private static final double RADIUS = 4;
    private static final double DEGREE = StrictMath.PI / 180;

    private static Arc arc(Arc.Span span) {
        return Valid.value(Arc.of(CENTRE, AXLE, RADIUS, span));
    }

    private static Arc.Span part(double fromDegrees, double widthDegrees) {
        return Valid.value(Arc.Span.of(fromDegrees * DEGREE, widthDegrees * DEGREE));
    }

    private static Vec3 outward(double degrees) {
        double angle = degrees * DEGREE;
        return new Vec3(-StrictMath.cos(angle), 0, StrictMath.sin(angle));
    }

    private static Vec3 onTheCircle(double degrees) {
        return CENTRE.plus(outward(degrees).times(RADIUS));
    }

    private static Vec3 beside(double degrees, double out, double along) {
        return CENTRE.plus(outward(degrees).times(out)).plus(AXLE.times(along));
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted", rule -> rule);
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual) {
        assertEquals(what + ": x of " + actual, expected.x(), actual.x(), WITHIN);
        assertEquals(what + ": y of " + actual, expected.y(), actual.y(), WITHIN);
        assertEquals(what + ": z of " + actual, expected.z(), actual.z(), WITHIN);
    }

    private static void assertNearest(String what, Arc arc, Vec3 query, Vec3 point) {
        Arc.Nearest nearest = arc.nearestTo(query);
        Vec3 apart = query.minus(point);
        double distance = StrictMath.sqrt(apart.dot(apart));
        assertVec(what + ", the nearest point", point, nearest.point());
        assertEquals(what + ", the distance", distance, nearest.distance(), WITHIN);
        assertVec(what + ", the normal", apart.times(1 / distance), nearest.normal());
    }

    @Test
    public void aPointBesideTheArcWithinItsSpanIsNearestTheCircleStraightOutFromTheAxisOutsideItOrInside() {
        Arc hood = arc(part(-45, 90));

        assertNearest("outside", hood, beside(10, 6, 1.5), onTheCircle(10));
        assertEquals(2.5, hood.nearestTo(beside(10, 6, 1.5)).distance(), WITHIN);
        assertNearest("inside", hood, beside(-30, 1, -4), onTheCircle(-30));
        assertEquals(5, hood.nearestTo(beside(-30, 1, -4)).distance(), WITHIN);
        assertNearest("at its end", hood, beside(45, 5, 0), onTheCircle(45));
    }

    @Test
    public void aPointPastEitherEndOfTheSpanIsNearestTheEndItIsNearerToRoundTheCircle() {
        Arc hood = arc(part(-45, 90));

        assertNearest("just past the end", hood, beside(60, 5, 0.5), onTheCircle(45));
        assertNearest("just before the start", hood, beside(-70, 3, -0.5), onTheCircle(-45));
        assertNearest("across from it, nearer the end", hood, beside(170, 5, 0), onTheCircle(45));
        assertNearest("across from it, nearer the start", hood, beside(-150, 5, 0), onTheCircle(-45));
    }

    @Test
    public void anArcAcrossTheSeamAtPiHoldsBothSidesOfItAndItsEndsAreWhereItsSpanSays() {
        Arc hood = arc(part(170, 40));

        assertNearest("at pi", hood, beside(180, 6, 1), onTheCircle(180));
        assertNearest("just past the seam", hood, beside(-175, 2, 1), onTheCircle(-175));
        assertNearest("further past the seam", hood, beside(-160, 6, -1), onTheCircle(-160));
        assertNearest("past its end", hood, beside(-140, 6, 0), onTheCircle(210));
        assertNearest("before its start", hood, beside(160, 6, 0), onTheCircle(170));
        assertNearest("across from it", hood, beside(-10, 6, 0), onTheCircle(210));
    }

    @Test
    public void aWholeRingIsNearestStraightOutFromTheAxisAtEveryAngle() {
        Arc ring = arc(Arc.Span.whole());

        for (double degrees = -180; degrees < 180; degrees += 15) {
            assertNearest("at " + degrees, ring, beside(degrees, 6, 2), onTheCircle(degrees));
            assertNearest("inside at " + degrees, ring, beside(degrees, 0.5, -1), onTheCircle(degrees));
        }
    }

    @Test
    public void onItsAxisAnArcIsNearestItsStartAndAWholeRingTheStartOfItsAnglesAsFarAsTheRimIs() {
        Vec3 above = CENTRE.plus(AXLE.times(3));

        assertNearest("a part of a circle", arc(part(20, 50)), above, onTheCircle(20));
        assertEquals(5, arc(part(20, 50)).nearestTo(above).distance(), WITHIN);
        assertNearest("a whole ring", arc(Arc.Span.whole()), above, onTheCircle(0));
        assertEquals(5, arc(Arc.Span.whole()).nearestTo(above).distance(), WITHIN);
        assertNearest("its centre", arc(Arc.Span.whole()), CENTRE, onTheCircle(0));
        assertEquals(RADIUS, arc(Arc.Span.whole()).nearestTo(CENTRE).distance(), WITHIN);
    }

    @Test
    public void onTheCircleItselfTheNormalIsStraightOutFromTheAxisInTheCirclesPlane() {
        for (double degrees : new double[] {0, 90}) {
            Arc.Nearest nearest = arc(part(-45, 180)).nearestTo(onTheCircle(degrees));
            assertEquals("at " + degrees, 0, nearest.distance(), 0);
            assertVec("at " + degrees, onTheCircle(degrees), nearest.point());
            assertVec("at " + degrees, outward(degrees), nearest.normal());
        }
        Arc.Nearest across = arc(Arc.Span.whole()).nearestTo(onTheCircle(180));
        assertEquals(0, across.distance(), 0);
        assertVec("a whole ring at 180", outward(180), across.normal());
    }

    @Test
    public void anArcsAnglesRunAboutItsAxisFromTheLevelWayStraightBackForAnAxleAlongYAndUpwardFromThere() {
        Arc level = arc(Arc.Span.whole());
        assertVec("along y, 0", CENTRE.plus(new Vec3(-RADIUS, 0, 0)), level.at(0));
        assertVec("along y, a quarter turn", CENTRE.plus(new Vec3(0, 0, RADIUS)), level.at(StrictMath.PI / 2));

        Arc reversed = Valid.value(Arc.of(CENTRE, new Vec3(0, -1, 0), RADIUS, Arc.Span.whole()));
        assertVec("along -y, 0", CENTRE.plus(new Vec3(RADIUS, 0, 0)), reversed.at(0));
        assertVec("along -y, a quarter turn", CENTRE.plus(new Vec3(0, 0, RADIUS)), reversed.at(StrictMath.PI / 2));

        double tilt = 0.3;
        Arc tilted = Valid.value(
                Arc.of(CENTRE, new Vec3(0, StrictMath.cos(tilt), StrictMath.sin(tilt)), RADIUS, Arc.Span.whole()));
        assertVec("tilted, 0", CENTRE.plus(new Vec3(-RADIUS, 0, 0)), tilted.at(0));
        assertVec(
                "tilted, a quarter turn",
                CENTRE.plus(new Vec3(0, -StrictMath.sin(tilt), StrictMath.cos(tilt)).times(RADIUS)),
                tilted.at(StrictMath.PI / 2));

        Arc upright = Valid.value(Arc.of(CENTRE, new Vec3(0, 0, 1), RADIUS, Arc.Span.whole()));
        assertVec("upright, 0", CENTRE.plus(new Vec3(RADIUS, 0, 0)), upright.at(0));
        assertVec("upright, a quarter turn", CENTRE.plus(new Vec3(0, RADIUS, 0)), upright.at(StrictMath.PI / 2));
    }

    @Test
    public void everyFinitePointHasANearestPointOnTheArcWithinItsSpanAFiniteDistanceAndAUnitNormal() {
        Arc hood = arc(part(150, 100));
        for (double x = -6; x <= 8; x += 0.5) {
            for (double y = -1; y <= 5; y += 1.5) {
                for (double z = -2; z <= 8; z += 0.5) {
                    Vec3 query = new Vec3(x, y, z);
                    Arc.Nearest nearest = hood.nearestTo(query);
                    Vec3 radial = nearest.point().minus(CENTRE);
                    double angle = StrictMath.atan2(radial.z(), -radial.x()) / DEGREE;
                    double past = (angle - 150 + 720) % 360;
                    String where = " for " + query + ": " + nearest;
                    assertEquals("on the circle" + where, RADIUS, StrictMath.sqrt(radial.dot(radial)), WITHIN);
                    assertEquals("in its plane" + where, 0, radial.dot(AXLE), WITHIN);
                    assertTrue("within its span" + where, past <= 100 + 1e-9 || past >= 360 - 1e-9);
                    assertEquals(
                            "as far as the point" + where,
                            StrictMath.sqrt(query.minus(nearest.point()).dot(query.minus(nearest.point()))),
                            nearest.distance(),
                            WITHIN);
                    assertEquals("a unit normal" + where, 1, nearest.normal().dot(nearest.normal()), WITHIN);
                }
            }
        }
    }

    @Test
    public void anArcHasAFiniteCentreAUnitAxisAndAPositiveFiniteRadius() {
        Arc.Span whole = Arc.Span.whole();

        assertEquals(
                "an arc's radius is a positive, finite number of inches, not 0.0",
                rejection(Arc.of(CENTRE, AXLE, 0, whole)));
        assertEquals(
                "an arc's radius is a positive, finite number of inches, not -1.0",
                rejection(Arc.of(CENTRE, AXLE, -1, whole)));
        assertEquals(
                "an arc's radius is a positive, finite number of inches, not NaN",
                rejection(Arc.of(CENTRE, AXLE, Double.NaN, whole)));
        assertEquals(
                "an arc's radius is a positive, finite number of inches, not Infinity",
                rejection(Arc.of(CENTRE, AXLE, Double.POSITIVE_INFINITY, whole)));
        assertEquals(
                "an arc's axis is a unit vector, not Vec3[x=0.0, y=2.0, z=0.0]",
                rejection(Arc.of(CENTRE, new Vec3(0, 2, 0), RADIUS, whole)));
        assertEquals(
                "an arc's axis is a unit vector, not Vec3[x=0.0, y=0.0, z=0.0]",
                rejection(Arc.of(CENTRE, Vec3.zero(), RADIUS, whole)));
        assertEquals(
                "an arc's axis is a unit vector, not Vec3[x=NaN, y=1.0, z=0.0]",
                rejection(Arc.of(CENTRE, new Vec3(Double.NaN, 1, 0), RADIUS, whole)));
        assertEquals(
                "an arc's axis is a unit vector, not Vec3[x=0.0, y=1.000001, z=0.0]",
                rejection(Arc.of(CENTRE, new Vec3(0, 1.000001, 0), RADIUS, whole)));
        assertEquals(
                "an arc's centre is a finite point, not Vec3[x=NaN, y=2.0, z=3.0]",
                rejection(Arc.of(new Vec3(Double.NaN, 2, 3), AXLE, RADIUS, whole)));
        assertEquals(
                "an arc's centre is a finite point, not Vec3[x=1.0, y=Infinity, z=3.0]",
                rejection(Arc.of(new Vec3(1, Double.POSITIVE_INFINITY, 3), AXLE, RADIUS, whole)));
        assertEquals("accepted", rejection(Arc.of(CENTRE, new Vec3(0, 1 + 1e-10, 0), RADIUS, whole)));
        assertEquals("accepted", rejection(Arc.of(CENTRE, AXLE, 1e-9, whole)));
    }

    @Test
    public void anArcsSpanStartsAtAFiniteAngleAndIsMoreThanNothingAndLessThanAWholeTurnWide() {
        String wide = "an arc's span is more than 0 and less than a whole turn, 2 pi, wide, not ";

        assertEquals(wide + "0.0", rejection(Arc.Span.of(0, 0)));
        assertEquals(wide + "-0.5", rejection(Arc.Span.of(0, -0.5)));
        assertEquals(wide + (2 * StrictMath.PI), rejection(Arc.Span.of(0, 2 * StrictMath.PI)));
        assertEquals(wide + "7.0", rejection(Arc.Span.of(0, 7)));
        assertEquals(wide + "NaN", rejection(Arc.Span.of(0, Double.NaN)));
        assertEquals(wide + "Infinity", rejection(Arc.Span.of(0, Double.POSITIVE_INFINITY)));
        assertEquals("an arc's span starts at a finite angle, not NaN", rejection(Arc.Span.of(Double.NaN, 1)));
        assertEquals(
                "an arc's span starts at a finite angle, not -Infinity",
                rejection(Arc.Span.of(Double.NEGATIVE_INFINITY, 1)));
        assertEquals("accepted", rejection(Arc.Span.of(100, StrictMath.nextDown(2 * StrictMath.PI))));
        assertEquals("accepted", rejection(Arc.Span.of(-3, 1e-12)));
    }
}
