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
    public void theSpindexersAxesPutItsOriginWhereTheMountDoesAndTipItsAxisBack16Point46Degrees() {
        Feed.Axes spindexer = drawing().spindexer();

        assertNear(new Vec3(1.18268, 0, 1.13307), spindexer.origin());
        assertNear(new Vec3(0.28336, 0, 0.95901), spindexer.z());
        assertNear(new Vec3(0, -1, 0), spindexer.x());
    }

    @Test
    public void theSpindexersMeasuresAreItsShapesInInchesWithTheBackSillWhereThePlanPutsIt() {
        Feed.Drawing drawing = drawing();

        assertEquals(3.23898, drawing.spindexerInches().get("wallRadius"), WITHIN);
        assertEquals(1.18110, drawing.spindexerInches().get("backSill"), WITHIN);
        assertEquals(1.96850, drawing.spindexerInches().get("armReach"), WITHIN);
        assertEquals(-1.25827, drawing.spindexerInches().get("barFreeEndX"), WITHIN);
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
