package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class FeedDrawingTest {
    private static final double WITHIN = 1e-4;

    private static Feed.Drawing drawing() {
        return Valid.value(Valid.value(Feed.Plan.defaults()).run()).drawing();
    }

    private static void assertNear(Vec3 expected, Vec3 actual) {
        assertEquals(actual.toString(), expected.x(), actual.x(), WITHIN);
        assertEquals(actual.toString(), expected.y(), actual.y(), WITHIN);
        assertEquals(actual.toString(), expected.z(), actual.z(), WITHIN);
    }

    @Test
    public void theFeedersAxesPutItsOriginWhereTheMountDoesAndTipItsAxisBack16Point46Degrees() {
        Feed.Axes feeder = drawing().feeder();

        assertNear(new Vec3(1.18268, 0, 1.13307), feeder.origin());
        assertNear(new Vec3(0.28336, 0, 0.95901), feeder.z());
        assertNear(new Vec3(0, -1, 0), feeder.x());
    }

    @Test
    public void theFeedersMeasuresAreItsShapesInInchesWithTheBackSillWhereThePlanPutsIt() {
        Feed.Drawing drawing = drawing();

        assertEquals(3.23898, drawing.feederInches().get("wallRadius"), WITHIN);
        assertEquals(1.18110, drawing.feederInches().get("backSill"), WITHIN);
        assertEquals(1.96850, drawing.feederInches().get("armReach"), WITHIN);
        assertEquals(-1.25827, drawing.feederInches().get("barFreeEndX"), WITHIN);
    }

    @Test
    public void theLaunchersAxesCentreOnTheAxleWithItsHoodFromMinus64Point8To20DegreesUpFromTheBack() {
        Feed.Drawing drawing = drawing();

        assertNear(new Vec3(-0.51181, 0, 7.08661), drawing.launcher().origin());
        assertNear(new Vec3(-1, 0, 0), drawing.launcher().x());
        assertEquals(-64.8, drawing.launcherInches().get("hoodFromDegrees"), 0);
        assertEquals(20, drawing.launcherInches().get("hoodToDegrees"), 0);
        assertEquals(2.96412, drawing.launcherInches().get("wheelTorus"), WITHIN);
    }
}
