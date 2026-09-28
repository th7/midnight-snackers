package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Hives;
import org.firstinspires.ftc.teamcode.simcore.Lean;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Ring;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class HivesTest {
    private static final double DELTA = 0.001;
    private static final double POLLEN_RADIUS = 1.39;
    private static final Length POLLEN = Valid.value(Length.of(POLLEN_RADIUS));
    private static final double STEP = 0.01;

    private final Field field = SimPlacement.FIELD;
    private Hives hives = Hives.of(field);
    private final Field.Hive blue = hives.hiveOf("Blue").orElseThrow();
    private final Field.Hive red = hives.hiveOf("Red").orElseThrow();

    private double tilt(Field.Hive hive) {
        return hives.tilt(hive);
    }

    private void advance(double seconds) {
        hives = hives.after(Valid.value(Seconds.of(seconds)));
    }

    private Field.Cell upturned(Field.Hive hive) {
        return hives.upturnedCell(hive).orElseThrow();
    }

    private Vec3 inBlue(Vec3 local) {
        return blue.at(tilt(blue), local);
    }

    private static Vec3 centreOf(Ring<Vec3> ring) {
        Vec3 sum = Vec3.zero();
        for (Vec3 corner : ring.all()) {
            sum = sum.plus(corner.times(1.0 / ring.size()));
        }
        return sum;
    }

    @Test
    public void aHiveStartsLeaningTheWayTheFieldIsSetUp() {
        assertEquals(blue.tilt().degrees(), tilt(blue), DELTA);
        assertEquals(30, Math.abs(tilt(blue)), DELTA);
        assertEquals(30, Math.abs(tilt(red)), DELTA);
        assertNotEquals("the two hives lean opposite ways", Math.signum(tilt(blue)), Math.signum(tilt(red)));
        assertTrue("and neither is tipping", !hives.tipping(blue) && !hives.tipping(red));
    }

    @Test
    public void oneCellOfEachHiveIsUpturnedAndItIsTheOneThatCanBeScoredIn() {
        for (Field.Hive hive : List.of(blue, red)) {
            Field.Cell up = upturned(hive);
            assertTrue(hives.upturned(up));
            for (Field.Cell cell : hive.cells()) {
                assertEquals(cell.name() + " upturned?", cell == up, hives.upturned(cell));
            }
        }
    }

    @Test
    public void aNectarIsAFifthOfAHiveAndAPollenAnEighth() {
        assertEquals(Hives.FULL / 5, Hives.fills(Field.Kind.NECTAR));
        assertEquals(Hives.FULL / 8, Hives.fills(Field.Kind.POLLEN));
    }

    @Test
    public void aTipTurnsTheHiveOverGraduallyToTheSameTiltTheOtherSideOfLevel() {
        double leaning = tilt(blue);
        hives = hives.tipped(blue);

        List<Double> tilts = new ArrayList<>();
        for (double t = 0; t < Lean.TIP_SECONDS + 0.5; t += STEP) {
            advance(STEP);
            tilts.add(tilt(blue));
        }

        double previous = leaning;
        for (double tilt : tilts) {
            assertTrue(
                    "it only ever turns toward the other side: " + previous + " then " + tilt,
                    Math.signum(leaning) * (previous - tilt) >= 0);
            previous = tilt;
        }
        int halfway = (int) Math.round(Lean.TIP_SECONDS / 2 / STEP) - 1;
        assertEquals("level halfway through", 0, tilts.get(halfway), 1);
        int nearlyThere = (int) Math.round(Lean.TIP_SECONDS * 0.9 / STEP) - 1;
        assertTrue(
                "still on its way nine tenths of the way through: " + tilts.get(nearlyThere),
                Math.abs(tilts.get(nearlyThere) + leaning) > DELTA);
        assertEquals("there by the end of it", -leaning, previous, DELTA);
        assertTrue("and at rest", !hives.tipping(blue));
    }

    @Test
    public void aTipStartsFromRestAndComesToRestSoItThrowsNothing() {
        hives = hives.tipped(blue);
        List<Double> turned = new ArrayList<>();
        double was = tilt(blue);
        for (double t = 0; t < Lean.TIP_SECONDS - STEP / 2; t += STEP) {
            advance(STEP);
            turned.add(Math.abs(tilt(blue) - was));
            was = tilt(blue);
        }

        double fastest = 0;
        for (double step : turned) {
            fastest = Math.max(fastest, step);
        }
        assertTrue("it speeds up from rest: " + turned.get(0), turned.get(0) < fastest / 10);
        assertTrue(
                "and slows to a stop: " + turned.get(turned.size() - 1), turned.get(turned.size() - 1) < fastest / 10);
        assertEquals("fastest in the middle", fastest, turned.get(turned.size() / 2), fastest / 20);
    }

    @Test
    public void aHiveThatIsTippingFinishesItsTipBeforeItCanTipAgain() {
        double leaning = tilt(blue);
        hives = hives.tipped(blue);
        advance(Lean.TIP_SECONDS / 2);

        hives = hives.tipped(blue);
        advance(Lean.TIP_SECONDS / 2 + STEP);

        assertEquals("it went on over", -leaning, tilt(blue), DELTA);

        hives = hives.tipped(blue);
        advance(Lean.TIP_SECONDS + STEP);

        assertEquals("and at rest it can tip back", leaning, tilt(blue), DELTA);
    }

    @Test
    public void whileAHiveTipsOneCellIsUpturnedAtEveryMomentAndItChangesAsTheHivePassesLevel() {
        Field.Cell was = upturned(blue);
        Field.Cell other =
                blue.cells().get(0) == was ? blue.cells().get(1) : blue.cells().get(0);
        hives = hives.tipped(blue);

        for (double t = STEP; t < Lean.TIP_SECONDS; t += STEP) {
            advance(STEP);
            Field.Cell up = upturned(blue);
            assertEquals(
                    "at " + tilt(blue) + " degrees",
                    Math.signum(tilt(blue)) == Math.signum(blue.tilt().degrees()) ? was : other,
                    up);
            assertTrue(hives.upturned(up) && !hives.upturned(up == was ? other : was));
        }
    }

    @Test
    public void aBallInsideACellAgainstItsBackIsPushedBackIntoTheCell() {
        Field.Cell cell = upturned(blue);
        Vec3 inward = cell.mouthCentre().minus(centreOf(cell.back())).unit();
        Vec3 local = centreOf(cell.back()).along(inward, 1.0);

        List<Hives.Touch> touches = hives.touching(inBlue(local), POLLEN);

        assertEquals("the back and nothing else", 1, touches.size());
        Hives.Touch touch = touches.get(0);
        assertEquals(POLLEN_RADIUS - 1.0, touch.depth(), DELTA);
        assertVector("away from the back, toward the mouth", blue.direction(tilt(blue), inward), touch.normal());
        assertVector("a hive at rest is still", Vec3.zero(), touch.velocity());
    }

    @Test
    public void theMouthOfACellIsOpen() {
        Field.Cell cell = upturned(blue);

        assertEquals(0, hives.touching(cell.mouthCentreAt(tilt(blue)), POLLEN).size());
    }

    @Test
    public void aWallIsSolidFromOutsideTooAndKeepsABallItsRadiusAway() {
        List<Hives.Touch> touches = hives.touching(inBlue(new Vec3(15, 10.03 + 0.7, 3)), POLLEN);

        assertEquals(1, touches.size());
        assertEquals(POLLEN_RADIUS - 0.7, touches.get(0).depth(), 0.01);
        assertVector(
                "straight out from the wall, which the CAD stands all but upright",
                new Vec3(0, 1, 0),
                touches.get(0).normal(),
                0.01);
    }

    @Test
    public void aBallJustBeyondTheEndOfAWallTouchesItsEdgeAndIsPushedAwayFromTheEdge() {
        Vec3 lip = new Vec3(21.46, 5, -1.42);
        Vec3 beyond = new Vec3(lip.x() + 0.6, lip.y(), lip.z() - 0.6);

        List<Hives.Touch> touches = hives.touching(inBlue(beyond), POLLEN);

        assertEquals(1, touches.size());
        assertEquals(POLLEN_RADIUS - Math.hypot(0.6, 0.6), touches.get(0).depth(), 0.01);
        assertVector(
                "from the edge to the ball, not square to the wall",
                blue.direction(tilt(blue), new Vec3(0.6, 0, -0.6).unit()),
                touches.get(0).normal());
    }

    @Test
    public void theHivesOtherPartsAreSolidToo() {
        List<Hives.Touch> touches = hives.touching(inBlue(new Vec3(15, 0, -2.71 - 1.0)), POLLEN);

        assertEquals("the base tube, which nothing else is near", 1, touches.size());
        assertEquals(POLLEN_RADIUS - 1.0, touches.get(0).depth(), DELTA);
        assertVector(
                blue.direction(tilt(blue), new Vec3(0, 0, -1)), touches.get(0).normal());
    }

    @Test
    public void aBallFarFromEveryHiveTouchesNothing() {
        assertEquals(0, hives.touching(new Vec3(0, 0, POLLEN_RADIUS), POLLEN).size());
        assertEquals(0, hives.touching(new Vec3(60, 60, 50), POLLEN).size());
    }

    @Test
    public void whileAHiveTurnsWhereItTouchesABallMovesWithIt() {
        Field.Cell cell = upturned(blue);
        Vec3 inward = cell.mouthCentre().minus(centreOf(cell.back())).unit();
        Vec3 local = centreOf(cell.back()).along(inward, 1.0);
        hives = hives.tipped(blue);
        advance(Lean.TIP_SECONDS / 2);

        double before = tilt(blue);
        Hives.Touch touch = hives.touching(inBlue(local), POLLEN).get(0);
        Vec3 touched = blue.at(before, centreOf(cell.back()));
        advance(0.001);
        Vec3 after = blue.at(tilt(blue), centreOf(cell.back()));

        assertVector(
                "as fast as the hive moves that part of it",
                after.minus(touched).times(1 / 0.001),
                touch.velocity(),
                0.5);
        assertTrue("which it does", touch.velocity().length() > 10);
    }

    @Test
    public void aPointIsInACellWhenItIsBetweenItsMouthItsBackAndItsWalls() {
        Field.Cell cell = upturned(blue);
        double tilt = tilt(blue);
        Vec3 mouth = cell.mouthCentreAt(tilt);
        Vec3 normal = cell.mouthNormalAt(tilt);

        assertEquals(Optional.of(cell), hives.cellHolding(cell.centreAt(tilt)));
        assertEquals("just inside the mouth", Optional.of(cell), hives.cellHolding(mouth.along(normal, -0.1)));
        assertEquals("just outside it", Optional.empty(), hives.cellHolding(mouth.along(normal, 0.1)));
        assertEquals(
                "between the two cells' backs", Optional.empty(), hives.cellHolding(blue.at(tilt, new Vec3(0, 0, 4))));
        assertEquals("above the roof", Optional.empty(), hives.cellHolding(inBlue(new Vec3(15, 0, 13))));
    }

    @Test
    public void aCellsRestingSpotsAreInsideItOnItsFloorAgainstItsBackAndApart() {
        Field.Cell cell = upturned(blue);
        double tilt = tilt(blue);
        double[][] back = Points.arrays(blue.at(tilt, cell.back()));
        Vec3 backNormal = Points.vec(Points.normal(back));

        List<Vec3> spots = hives.restingSpots(cell, POLLEN);

        assertTrue("room for a hive's worth", spots.size() >= Hives.FULL / Hives.fills(Field.Kind.POLLEN));
        assertEquals(Optional.of(cell), hives.cellHolding(spots.get(0)));
        assertEquals(
                "against the back",
                POLLEN_RADIUS,
                Math.abs(spots.get(0).minus(Points.vec(back[0])).dot(backNormal)),
                DELTA);
        for (int i = 0; i < spots.size(); i++) {
            assertEquals(Optional.of(cell), hives.cellHolding(spots.get(i)));
            for (int j = 0; j < i; j++) {
                double apart = spots.get(i).minus(spots.get(j)).length();
                assertTrue("spots " + i + " and " + j + " are " + apart + " apart", apart >= 2 * POLLEN_RADIUS - DELTA);
            }
        }
    }

    private static void assertVector(Vec3 expected, Vec3 actual) {
        assertVector("", expected, actual);
    }

    private static void assertVector(String message, Vec3 expected, Vec3 actual) {
        assertVector(message, expected, actual, DELTA);
    }

    static void assertVector(String message, Vec3 expected, Vec3 actual, double delta) {
        assertTrue(
                message + ": expected " + expected + " but was " + actual,
                Math.abs(expected.x() - actual.x()) <= delta
                        && Math.abs(expected.y() - actual.y()) <= delta
                        && Math.abs(expected.z() - actual.z()) <= delta);
    }
}
