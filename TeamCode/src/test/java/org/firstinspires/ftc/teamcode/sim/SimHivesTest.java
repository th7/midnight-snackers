package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class SimHivesTest {
    private static final double DELTA = 0.001;
    private static final double BOUNCE = 0.3;
    private static final double ROLL_OUT = 20;
    private static final double POLLEN_RADIUS = 1.4;
    private static final double NECTAR_RADIUS = 2.5;

    private final SimField field = SimPlacement.FIELD;
    private final SimHives hives = new SimHives(field, BOUNCE, ROLL_OUT);

    private Object pollenIn(SimField.Cell cell) {
        Object ball = new Object();
        hives.put(ball, SimField.POLLEN, POLLEN_RADIUS, cell);
        return ball;
    }

    private Object nectarIn(SimField.Cell cell) {
        Object ball = new Object();
        hives.put(ball, SimField.NECTAR, NECTAR_RADIUS, cell);
        return ball;
    }

    @Test
    public void aHiveStartsLeaningTheWayTheFieldIsSetUp() {
        assertEquals(field.hive("Blue Hive <1>").tilt, hives.tilt("Blue"), DELTA);
        assertEquals(30, Math.abs(hives.tilt("Blue")), DELTA);
        assertEquals(30, Math.abs(hives.tilt("Red")), DELTA);
        assertNotEquals(
                "the two hives lean opposite ways", Math.signum(hives.tilt("Blue")), Math.signum(hives.tilt("Red")));
    }

    @Test
    public void oneCellOfEachHiveIsUpturnedAndItIsTheOneThatCanBeScoredIn() {
        for (String alliance : List.of("Blue", "Red")) {
            SimField.Cell up = hives.upturnedCell(alliance);
            assertTrue(hives.upturned(up));
            for (SimField.Cell cell : hives.hiveOf(alliance).cells) {
                assertEquals(cell.name + " upturned?", cell == up, hives.upturned(cell));
            }
        }
    }

    @Test
    public void aNectarIsAFifthOfAHiveAndAPollenAnEighth() {
        SimField.Cell blue = hives.upturnedCell("Blue");

        nectarIn(blue);
        assertEquals(0.2, hives.load("Blue"), DELTA);

        pollenIn(blue);
        assertEquals(0.2 + 0.125, hives.load("Blue"), DELTA);
    }

    @Test
    public void fiveNectarFillAHiveAndItTips() {
        SimField.Cell blue = hives.upturnedCell("Blue");
        double leaning = hives.tilt("Blue");
        for (int i = 0; i < 4; i++) {
            nectarIn(blue);
        }

        assertEquals("four fifths is not full", leaning, hives.tilt("Blue"), DELTA);
        assertEquals(0, hives.turn().size());

        nectarIn(blue);
        List<SimHives.LeftACell> left = hives.turn();

        assertEquals("the fifth fills it and it tips", -leaning, hives.tilt("Blue"), DELTA);
        assertEquals("everything in it falls out", 5, left.size());
        assertEquals("and the hive is empty", 0, hives.scored("Blue"));
    }

    @Test
    public void eightPollenFillAHiveAndItTips() {
        SimField.Cell blue = hives.upturnedCell("Blue");
        double leaning = hives.tilt("Blue");
        for (int i = 0; i < 7; i++) {
            pollenIn(blue);
        }
        hives.turn();

        assertEquals("seven eighths is not full", leaning, hives.tilt("Blue"), DELTA);
        assertEquals(0.875, hives.load("Blue"), DELTA);

        pollenIn(blue);
        hives.turn();

        assertEquals(-leaning, hives.tilt("Blue"), DELTA);
    }

    @Test
    public void tippingEmptiesTheHiveAndTheBallsLeaveThroughTheMouth() {
        SimField.Cell blue = hives.upturnedCell("Blue");
        for (int i = 0; i < 5; i++) {
            nectarIn(blue);
        }

        List<SimHives.LeftACell> left = hives.turn();

        assertEquals(5, left.size());
        for (SimHives.LeftACell ball : left) {
            assertFalse("it is no longer the hive's", hives.holds(ball.ball));
            double speed = Math.sqrt(ball.velocity[0] * ball.velocity[0]
                    + ball.velocity[1] * ball.velocity[1]
                    + ball.velocity[2] * ball.velocity[2]);
            assertEquals("it rolls out at the roll-out speed", ROLL_OUT, speed, DELTA);
        }
    }

    @Test
    public void tippingTheOtherHiveDoesNotEmptyThisOne() {
        SimField.Cell blue = hives.upturnedCell("Blue");
        SimField.Cell red = hives.upturnedCell("Red");
        pollenIn(blue);
        for (int i = 0; i < 5; i++) {
            nectarIn(red);
        }

        hives.turn();

        assertEquals(1, hives.scored("Blue"));
        assertEquals(0, hives.scored("Red"));
    }

    @Test
    public void ballsRestSideBySideOnTheFloorOfTheCellRatherThanInsideOneAnother() {
        SimField.Cell blue = hives.upturnedCell("Blue");
        Object first = pollenIn(blue);
        Object second = pollenIn(blue);

        double[] a = hives.restingPlace(first);
        double[] b = hives.restingPlace(second);

        double apart = Math.sqrt(
                (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
        assertTrue("two pollen " + apart + " apart", apart >= 2 * POLLEN_RADIUS - DELTA);
    }

    @Test
    public void aFlightThatCrossesTheMouthGoingInScores() {
        SimField.Cell blue = hives.upturnedCell("Blue");
        double[][] mouth = blue.mouthAt(hives.tilt("Blue"));
        double[] normal = blue.mouthNormalAt(hives.tilt("Blue"));
        double[] middle = middleOf(mouth);
        double[] from = along(middle, normal, 2);
        double[] to = along(middle, normal, -2);

        SimHives.Met met = hives.met(from, to, new double[] {-normal[0], -normal[1], -normal[2]});

        assertTrue(met.met());
        assertTrue("it went in", met.scored());
        assertEquals(blue, met.scoredIn);
    }

    @Test
    public void aFlightThatCrossesTheMouthFromBehindMeetsTheHiveAndScoresNothing() {
        SimField.Cell blue = hives.upturnedCell("Blue");
        double[][] mouth = blue.mouthAt(hives.tilt("Blue"));
        double[] normal = blue.mouthNormalAt(hives.tilt("Blue"));
        double[] middle = middleOf(mouth);
        double[] from = along(middle, normal, -2);
        double[] to = along(middle, normal, 2);

        SimHives.Met met = hives.met(from, to, new double[] {normal[0], normal[1], normal[2]});

        assertTrue("it met the hive", met.met());
        assertFalse(met.scored());
        assertFalse(met.bounced());
    }

    @Test
    public void aFlightThatMissesEveryHiveMeetsNothing() {
        SimHives.Met met = hives.met(new double[] {0, 0, 0.5}, new double[] {1, 0, 0.5}, new double[] {10, 0, 0});

        assertFalse(met.met());
        assertFalse(met.scored());
        assertFalse(met.bounced());
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
                    .getConstructor(Class.forName(SimField.class.getName(), true, loader), double.class, double.class)
                    .newInstance(theirField, BOUNCE, ROLL_OUT);

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

    private static double[] middleOf(double[][] ring) {
        double[] out = new double[3];
        for (double[] corner : ring) {
            for (int axis = 0; axis < 3; axis++) {
                out[axis] += corner[axis] / ring.length;
            }
        }
        return out;
    }

    private static double[] along(double[] from, double[] direction, double distance) {
        return new double[] {
            from[0] + direction[0] * distance, from[1] + direction[1] * distance, from[2] + direction[2] * distance
        };
    }

    private static URL[] classpath() throws Exception {
        List<URL> urls = new ArrayList<>();
        for (String entry : System.getProperty("java.class.path").split(File.pathSeparator)) {
            urls.add(Paths.get(entry).toUri().toURL());
        }
        return urls.toArray(new URL[0]);
    }
}
