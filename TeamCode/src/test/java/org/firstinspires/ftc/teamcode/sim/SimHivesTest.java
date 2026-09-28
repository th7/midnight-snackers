package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.junit.Test;

public class SimHivesTest {
    private static final double DELTA = 0.001;
    private static final double POLLEN_RADIUS = 1.39;
    private static final double STEP = 0.01;

    private final Field field = SimPlacement.FIELD;
    private final SimHives hives = new SimHives(field);
    private final Field.Hive blue = hives.hiveOf("Blue");

    @Test
    public void aHiveStartsLeaningTheWayTheFieldIsSetUp() {
        assertEquals(field.hive("Blue Hive <1>").orElseThrow().tilt().degrees(), hives.tilt("Blue"), DELTA);
        assertEquals(30, Math.abs(hives.tilt("Blue")), DELTA);
        assertEquals(30, Math.abs(hives.tilt("Red")), DELTA);
        assertNotEquals(
                "the two hives lean opposite ways", Math.signum(hives.tilt("Blue")), Math.signum(hives.tilt("Red")));
        assertTrue("and neither is tipping", !hives.tipping(blue) && !hives.tipping(hives.hiveOf("Red")));
    }

    @Test
    public void oneCellOfEachHiveIsUpturnedAndItIsTheOneThatCanBeScoredIn() {
        for (String alliance : List.of("Blue", "Red")) {
            Field.Cell up = hives.upturnedCell(alliance);
            assertTrue(hives.upturned(up));
            for (Field.Cell cell : hives.hiveOf(alliance).cells()) {
                assertEquals(cell.name() + " upturned?", cell == up, hives.upturned(cell));
            }
        }
    }

    @Test
    public void aNectarIsAFifthOfAHiveAndAPollenAnEighth() {
        assertEquals(SimHives.FULL / 5, SimHives.fills(Field.Kind.NECTAR));
        assertEquals(SimHives.FULL / 8, SimHives.fills(Field.Kind.POLLEN));
    }

    @Test
    public void aTipTurnsTheHiveOverGraduallyToTheSameTiltTheOtherSideOfLevel() {
        double leaning = hives.tilt("Blue");
        hives.tip(blue);

        List<Double> tilts = new ArrayList<>();
        for (double t = 0; t < SimHives.TIP_SECONDS + 0.5; t += STEP) {
            hives.advance(STEP);
            tilts.add(hives.tilt("Blue"));
        }

        double previous = leaning;
        for (double tilt : tilts) {
            assertTrue(
                    "it only ever turns toward the other side: " + previous + " then " + tilt,
                    Math.signum(leaning) * (previous - tilt) >= 0);
            previous = tilt;
        }
        int halfway = (int) Math.round(SimHives.TIP_SECONDS / 2 / STEP) - 1;
        assertEquals("level halfway through", 0, tilts.get(halfway), 1);
        int nearlyThere = (int) Math.round(SimHives.TIP_SECONDS * 0.9 / STEP) - 1;
        assertTrue(
                "still on its way nine tenths of the way through: " + tilts.get(nearlyThere),
                Math.abs(tilts.get(nearlyThere) + leaning) > DELTA);
        assertEquals("there by the end of it", -leaning, previous, DELTA);
        assertTrue("and at rest", !hives.tipping(blue));
    }

    @Test
    public void aTipStartsFromRestAndComesToRestSoItThrowsNothing() {
        hives.tip(blue);
        List<Double> turned = new ArrayList<>();
        double was = hives.tilt("Blue");
        for (double t = 0; t < SimHives.TIP_SECONDS - STEP / 2; t += STEP) {
            hives.advance(STEP);
            turned.add(Math.abs(hives.tilt("Blue") - was));
            was = hives.tilt("Blue");
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
        double leaning = hives.tilt("Blue");
        hives.tip(blue);
        hives.advance(SimHives.TIP_SECONDS / 2);

        hives.tip(blue);
        hives.advance(SimHives.TIP_SECONDS / 2 + STEP);

        assertEquals("it went on over", -leaning, hives.tilt("Blue"), DELTA);

        hives.tip(blue);
        hives.advance(SimHives.TIP_SECONDS + STEP);

        assertEquals("and at rest it can tip back", leaning, hives.tilt("Blue"), DELTA);
    }

    @Test
    public void whileAHiveTipsOneCellIsUpturnedAtEveryMomentAndItChangesAsTheHivePassesLevel() {
        Field.Cell was = hives.upturnedCell("Blue");
        Field.Cell other =
                blue.cells().get(0) == was ? blue.cells().get(1) : blue.cells().get(0);
        hives.tip(blue);

        for (double t = STEP; t < SimHives.TIP_SECONDS; t += STEP) {
            hives.advance(STEP);
            Field.Cell up = hives.upturnedCell("Blue");
            assertSame(
                    "at " + hives.tilt("Blue") + " degrees",
                    Math.signum(hives.tilt("Blue"))
                                    == Math.signum(field.hive("Blue Hive <1>")
                                            .orElseThrow()
                                            .tilt()
                                            .degrees())
                            ? was
                            : other,
                    up);
            assertTrue(hives.upturned(up) && !hives.upturned(up == was ? other : was));
        }
    }

    @Test
    public void aBallInsideACellAgainstItsBackIsPushedBackIntoTheCell() {
        Field.Cell cell = hives.upturnedCell("Blue");
        double[] inward = unit(sub(Points.array(cell.mouthCentre()), centreOf(Points.arrays(cell.back()))));
        double[] local = along(centreOf(Points.arrays(cell.back())), inward, 1.0);

        List<SimHives.Touch> touches = hives.touching(inBlue(local), POLLEN_RADIUS);

        assertEquals("the back and nothing else", 1, touches.size());
        SimHives.Touch touch = touches.get(0);
        assertEquals(POLLEN_RADIUS - 1.0, touch.depth, DELTA);
        assertVector("away from the back, toward the mouth", direction(hives.tilt("Blue"), inward), touch.normal);
        assertVector("a hive at rest is still", new double[3], touch.velocity);
    }

    @Test
    public void theMouthOfACellIsOpen() {
        Field.Cell cell = hives.upturnedCell("Blue");

        assertEquals(
                0,
                hives.touching(Points.array(cell.mouthCentreAt(hives.tilt("Blue"))), POLLEN_RADIUS)
                        .size());
    }

    @Test
    public void aWallIsSolidFromOutsideTooAndKeepsABallItsRadiusAway() {
        double[] outside = {15, 10.03 + 0.7, 3};

        List<SimHives.Touch> touches = hives.touching(inBlue(outside), POLLEN_RADIUS);

        assertEquals(1, touches.size());
        assertEquals(POLLEN_RADIUS - 0.7, touches.get(0).depth, 0.01);
        assertVector(
                "straight out from the wall, which the CAD stands all but upright",
                new double[] {0, 1, 0},
                touches.get(0).normal,
                0.01);
    }

    @Test
    public void aBallJustBeyondTheEndOfAWallTouchesItsEdgeAndIsPushedAwayFromTheEdge() {
        double[] lip = {21.46, 5, -1.42};
        double[] beyond = {lip[0] + 0.6, lip[1], lip[2] - 0.6};

        List<SimHives.Touch> touches = hives.touching(inBlue(beyond), POLLEN_RADIUS);

        assertEquals(1, touches.size());
        assertEquals(POLLEN_RADIUS - Math.hypot(0.6, 0.6), touches.get(0).depth, 0.01);
        assertVector(
                "from the edge to the ball, not square to the wall",
                direction(hives.tilt("Blue"), unit(new double[] {0.6, 0, -0.6})),
                touches.get(0).normal);
    }

    @Test
    public void theHivesOtherPartsAreSolidToo() {
        double[] underTheBaseTube = {15, 0, -2.71 - 1.0};

        List<SimHives.Touch> touches = hives.touching(inBlue(underTheBaseTube), POLLEN_RADIUS);

        assertEquals("the base tube, which nothing else is near", 1, touches.size());
        assertEquals(POLLEN_RADIUS - 1.0, touches.get(0).depth, DELTA);
        assertVector(direction(hives.tilt("Blue"), new double[] {0, 0, -1}), touches.get(0).normal);
    }

    @Test
    public void aBallFarFromEveryHiveTouchesNothing() {
        assertEquals(
                0,
                hives.touching(new double[] {0, 0, POLLEN_RADIUS}, POLLEN_RADIUS)
                        .size());
        assertEquals(0, hives.touching(new double[] {60, 60, 50}, POLLEN_RADIUS).size());
    }

    @Test
    public void whileAHiveTurnsWhereItTouchesABallMovesWithIt() {
        Field.Cell cell = hives.upturnedCell("Blue");
        double[] inward = unit(sub(Points.array(cell.mouthCentre()), centreOf(Points.arrays(cell.back()))));
        double[] local = along(centreOf(Points.arrays(cell.back())), inward, 1.0);
        hives.tip(blue);
        hives.advance(SimHives.TIP_SECONDS / 2);

        double before = hives.tilt("Blue");
        SimHives.Touch touch = hives.touching(inBlue(local), POLLEN_RADIUS).get(0);
        double[] touched = at(before, centreOf(Points.arrays(cell.back())));
        hives.advance(0.001);
        double[] after = at(hives.tilt("Blue"), centreOf(Points.arrays(cell.back())));

        assertVector(
                "as fast as the hive moves that part of it",
                scale(sub(after, touched), 1 / 0.001),
                touch.velocity,
                0.5);
        assertTrue("which it does", Math.sqrt(dot(touch.velocity, touch.velocity)) > 10);
    }

    @Test
    public void aPointIsInACellWhenItIsBetweenItsMouthItsBackAndItsWalls() {
        Field.Cell cell = hives.upturnedCell("Blue");
        double tilt = hives.tilt("Blue");
        double[] mouth = Points.array(cell.mouthCentreAt(tilt));
        double[] normal = Points.array(cell.mouthNormalAt(tilt));

        assertSame(cell, hives.cellHolding(Points.array(cell.centreAt(tilt))));
        assertSame("just inside the mouth", cell, hives.cellHolding(along(mouth, normal, -0.1)));
        assertNull("just outside it", hives.cellHolding(along(mouth, normal, 0.1)));
        assertNull("between the two cells' backs", hives.cellHolding(at(tilt, new double[] {0, 0, 4})));
        assertNull("above the roof", hives.cellHolding(inBlue(new double[] {15, 0, 13})));
    }

    @Test
    public void aCellsRestingSpotsAreInsideItOnItsFloorAgainstItsBackAndApart() {
        Field.Cell cell = hives.upturnedCell("Blue");
        double tilt = hives.tilt("Blue");
        double[][] back = at(tilt, Points.arrays(cell.back()));
        double[] backNormal = Points.normal(back);

        List<double[]> spots = hives.restingSpots(cell, POLLEN_RADIUS);

        assertTrue("room for a hive's worth", spots.size() >= SimHives.FULL / SimHives.fills(Field.Kind.POLLEN));
        double[] first = spots.get(0);
        assertSame(cell, hives.cellHolding(first));
        assertEquals("against the back", POLLEN_RADIUS, Math.abs(side(backNormal, back[0], first)), DELTA);
        for (int i = 0; i < spots.size(); i++) {
            assertSame(cell, hives.cellHolding(spots.get(i)));
            for (int j = 0; j < i; j++) {
                double apart = Math.sqrt(dot(sub(spots.get(i), spots.get(j)), sub(spots.get(i), spots.get(j))));
                assertTrue("spots " + i + " and " + j + " are " + apart + " apart", apart >= 2 * POLLEN_RADIUS - DELTA);
            }
        }
    }

    @Test
    public void theHivesAreGeometryAndNeedNeitherTheRigidBodyEngineNorTheSimulatedRobot() throws Exception {
        List<String> forbidden = List.of("org.dyn4j.", SimRobot.class.getName());
        try (URLClassLoader loader = new URLClassLoader(classpath(), null) {
            @Override
            protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
                for (String banned : forbidden) {
                    if (name.startsWith(banned)) {
                        throw new ClassNotFoundException(
                                name + " must not be needed to turn a hive; that is the seam this test holds");
                    }
                }
                return super.loadClass(name, resolve);
            }
        }) {
            Class<?> hivesClass = Class.forName(SimHives.class.getName(), true, loader);
            Class<?> placement = Class.forName(SimPlacement.class.getName(), true, loader);
            Object theirField = placement.getField("FIELD").get(null);
            Object theirHives = hivesClass
                    .getConstructor(Class.forName(Field.class.getName(), true, loader))
                    .newInstance(theirField);

            double tilt = (double) hivesClass.getMethod("tilt", String.class).invoke(theirHives, "Blue");

            assertEquals("it answered with no engine on the classpath it could reach", hives.tilt("Blue"), tilt, DELTA);
        }
    }

    @Test
    public void theGateWouldNoticeIfTheSimulatorCameBackInThroughTheBackDoor() throws Exception {
        try (URLClassLoader loader = new URLClassLoader(classpath(), null) {
            @Override
            protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
                if (name.startsWith("org.dyn4j.")) {
                    throw new ClassNotFoundException(name);
                }
                return super.loadClass(name, resolve);
            }
        }) {
            Class.forName(SimRobot.class.getName(), true, loader);
            fail("the simulated robot should not initialise without the rigid-body engine");
        } catch (ExceptionInInitializerError | NoClassDefFoundError | ClassNotFoundException expected) {
        }
    }

    private double[] at(double tilt, double[] local) {
        return Points.array(blue.at(tilt, Points.vec(local)));
    }

    private double[][] at(double tilt, double[][] ring) {
        double[][] out = new double[ring.length][];
        for (int i = 0; i < ring.length; i++) {
            out[i] = at(tilt, ring[i]);
        }
        return out;
    }

    private double[] direction(double tilt, double[] local) {
        return Points.array(blue.direction(tilt, Points.vec(local)));
    }

    private double[] inBlue(double[] local) {
        return at(hives.tilt("Blue"), local);
    }

    private static void assertVector(double[] expected, double[] actual) {
        assertVector("", expected, actual);
    }

    private static void assertVector(String message, double[] expected, double[] actual) {
        assertVector(message, expected, actual, DELTA);
    }

    private static void assertVector(String message, double[] expected, double[] actual, double delta) {
        for (int axis = 0; axis < 3; axis++) {
            assertEquals(
                    message + ": expected " + java.util.Arrays.toString(expected) + " but was "
                            + java.util.Arrays.toString(actual),
                    expected[axis],
                    actual[axis],
                    delta);
        }
    }

    static double[] centreOf(double[][] ring) {
        double[] out = new double[3];
        for (double[] corner : ring) {
            for (int axis = 0; axis < 3; axis++) {
                out[axis] += corner[axis] / ring.length;
            }
        }
        return out;
    }

    static double side(double[] normal, double[] on, double[] point) {
        return dot(normal, sub(point, on));
    }

    static double[] along(double[] from, double[] direction, double distance) {
        return new double[] {
            from[0] + direction[0] * distance, from[1] + direction[1] * distance, from[2] + direction[2] * distance
        };
    }

    static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    static double[] scale(double[] a, double by) {
        return new double[] {a[0] * by, a[1] * by, a[2] * by};
    }

    static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    static double[] unit(double[] a) {
        return scale(a, 1 / Math.sqrt(dot(a, a)));
    }

    static URL[] classpath() throws Exception {
        List<URL> urls = new ArrayList<>();
        for (String entry : System.getProperty("java.class.path").split(File.pathSeparator)) {
            urls.add(Paths.get(entry).toUri().toURL());
        }
        return urls.toArray(new URL[0]);
    }
}
