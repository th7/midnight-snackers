package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class SimFlightTest {
    private static final double DELTA = 0.001;
    private static final double POLLEN = 1.39;
    private static final double NECTAR = 1.8;
    private static final double CONTACT = 0.1;
    private static final double STEP = 0.005;

    private final Field field = SimPlacement.FIELD;
    private final SimHives hives = new SimHives(field);
    private final SimFlight flight = new SimFlight(field, hives);
    private final Field.Hive blue = hives.hiveOf("Blue");
    private final List<SimFlight.Landing> landed = new ArrayList<>();

    private void fly(double seconds) {
        for (long i = Math.round(seconds / STEP); i > 0; i--) {
            landed.addAll(flight.step(STEP));
        }
    }

    private Object drop(Field.Kind kind, double radius, double[] at) {
        Object ball = new Object();
        flight.add(ball, kind, radius, at, new double[3]);
        return ball;
    }

    private double[] aboveTheMouthOf(Field.Cell cell, double height) {
        double[] mouth = Points.array(cell.mouthCentreAt(hives.tilt(cell.alliance())));
        return new double[] {mouth[0], mouth[1], mouth[2] + height};
    }

    @Test
    public void aBallDroppedIntoTheMouthOfAnUpturnedCellRollsDownToItsBackAndStaysThere() {
        Field.Cell cell = hives.upturnedCell("Blue");
        double tilt = hives.tilt("Blue");
        Object ball = drop(Field.Kind.POLLEN, POLLEN, aboveTheMouthOf(cell, 10));

        fly(3.0);

        assertTrue("it never reached the floor", landed.isEmpty() && flight.holds(ball));
        double[] at = flight.at(ball);
        assertSame("it is in the cell", cell, hives.cellHolding(at));
        assertEquals("and scores", 1, flight.scored("Blue"));
        double[][] back = at(tilt, Points.arrays(cell.back()));
        assertEquals(
                "against its back", POLLEN, Math.abs(SimHivesTest.side(Points.normal(back), back[0], at)), CONTACT);
        double[][] floor = at(tilt, floorOf(cell));
        assertEquals("on its floor", POLLEN, Math.abs(SimHivesTest.side(Points.normal(floor), floor[0], at)), CONTACT);

        fly(2.0);

        assertArrayEquals("where it stays", at, flight.at(ball), 0);
    }

    @Test
    public void aBallFallingBesideACellIsKeptItsRadiusClearOfTheWallAndFallsOnPastIt() {
        double[] onTheWall = Points.array(blue.at(hives.tilt("Blue"), new Vec3(15.45, 10.03, 2.8)));
        double wall = onTheWall[1];
        drop(Field.Kind.POLLEN, POLLEN, new double[] {onTheWall[0], wall + 0.7, onTheWall[2] + 15});

        fly(3.0);

        assertEquals("it reached the floor", 1, landed.size());
        assertFalse(landed.get(0).out);
        double clear = landed.get(0).at[1] - wall;
        assertTrue("no nearer the wall than its radius: " + clear, clear >= POLLEN - CONTACT);
    }

    @Test
    public void aBallThrownHardAtTheSideOfAHiveBouncesBackOffIt() {
        double[] onTheWall = Points.array(blue.at(hives.tilt("Blue"), new Vec3(15.45, 10.03, 2.8)));
        double thrownFrom = onTheWall[1] + 6;
        Object ball = new Object();
        flight.add(
                ball, Field.Kind.POLLEN, POLLEN, new double[] {onTheWall[0], thrownFrom, onTheWall[2]}, new double[] {
                    0, -200, 0
                });

        fly(2.0);

        assertEquals("it came down", 1, landed.size());
        assertTrue("back past where it was thrown from: y=" + landed.get(0).at[1], landed.get(0).at[1] > thrownFrom);
    }

    @Test
    public void aBallThrownHarderThanAnyLaunchAtACellsWallDoesNotGoThroughIt() {
        Field.Cell cell = hives.upturnedCell("Blue");
        double[] onTheWall = Points.array(blue.at(hives.tilt("Blue"), new Vec3(15.45, 10.03, 2.8)));
        double wall = onTheWall[1];
        Object ball = new Object();
        flight.add(ball, Field.Kind.POLLEN, POLLEN, new double[] {onTheWall[0], wall + 6, onTheWall[2]}, new double[] {
            0, -400, 0
        });

        for (int i = 0; i < 400 && flight.holds(ball); i++) {
            fly(STEP);
            if (flight.holds(ball)) {
                assertTrue("never in the cell", hives.cellHolding(flight.at(ball)) != cell);
            }
        }

        assertEquals("it came down", 1, landed.size());
        double clear = landed.get(0).at[1] - wall;
        assertTrue("it came down on the side it was thrown from: " + clear, clear >= POLLEN - CONTACT);
    }

    @Test
    public void ballsDroppedIntoACellOneAfterAnotherComeToRestInItApartFromOneAnother() {
        Field.Cell cell = hives.upturnedCell("Blue");
        double leaning = hives.tilt("Blue");
        List<Object> balls = new ArrayList<>();
        List<Double> radii = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            boolean nectar = i % 2 == 0;
            double radius = nectar ? NECTAR : POLLEN;
            double[] above = aboveTheMouthOf(cell, 10);
            above[1] += (i - 2.5) * 1.5;
            balls.add(drop(nectar ? Field.Kind.NECTAR : Field.Kind.POLLEN, radius, above));
            radii.add(radius);
            fly(0.4);
        }

        fly(3.0);

        assertTrue("none got away", landed.isEmpty());
        assertEquals("three nectar and three pollen are not quite full", leaning, hives.tilt("Blue"), 0);
        assertEquals(6, flight.scored("Blue"));
        for (int i = 0; i < balls.size(); i++) {
            double[] a = flight.at(balls.get(i));
            assertSame("ball " + i + " is in the cell", cell, hives.cellHolding(a));
            for (int j = 0; j < i; j++) {
                double[] b = flight.at(balls.get(j));
                double apart = Math.sqrt(SimHivesTest.dot(SimHivesTest.sub(a, b), SimHivesTest.sub(a, b)));
                assertTrue(
                        "balls " + i + " and " + j + " are " + apart + " apart",
                        apart >= radii.get(i) + radii.get(j) - CONTACT);
            }
        }
    }

    @Test
    public void aBallPutInACellRestsInItBesideWhatIsAlreadyThere() {
        Field.Cell cell = hives.upturnedCell("Blue");
        Object first = new Object();
        Object second = new Object();

        flight.putIn(first, Field.Kind.NECTAR, NECTAR, cell);
        flight.putIn(second, Field.Kind.POLLEN, POLLEN, cell);

        assertSame(cell, hives.cellHolding(flight.at(first)));
        assertSame(cell, hives.cellHolding(flight.at(second)));
        double[] between = SimHivesTest.sub(flight.at(first), flight.at(second));
        assertTrue("side by side", Math.sqrt(SimHivesTest.dot(between, between)) >= NECTAR + POLLEN - DELTA);
        assertEquals("and a nectar and a pollen are that much of the hive", 0.2 + 0.125, flight.load("Blue"), DELTA);
    }

    @Test
    public void aHiveThatIsFullTipsAndWhatWasInItRollsOutThroughTheMouthAndLandsBelowIt() {
        Field.Cell cell = hives.upturnedCell("Blue");
        double leaning = hives.tilt("Blue");
        double mouthSide = Math.signum(Points.array(cell.mouthCentreAt(leaning))[0]);
        for (int i = 0; i < 5; i++) {
            flight.putIn(new Object(), Field.Kind.NECTAR, NECTAR, cell);
        }

        fly(0.05);

        assertTrue("five nectar fill it and it has begun to tip", hives.tipping(blue));
        assertEquals("with them still in it", 5, flight.scored("Blue"));

        fly(4.0);

        assertEquals("it tipped over", -leaning, hives.tilt("Blue"), DELTA);
        assertEquals("and is empty", 0, flight.scored("Blue"));
        assertEquals("what was in it all landed", 5, landed.size());
        for (SimFlight.Landing landing : landed) {
            assertFalse(landing.out);
            assertEquals("on the floor", NECTAR, landing.at[2], DELTA);
            assertTrue(
                    "at the end of the hive whose mouth it rolled out of: x=" + landing.at[0],
                    landing.at[0] * mouthSide > 0);
        }
    }

    @Test
    public void aBallThatHitsTheFloorFastBouncesAndOneThatComesDownSlowlyLandsWithTheSpeedItHadAlongIt() {
        drop(Field.Kind.POLLEN, POLLEN, new double[] {0, -60, 30});
        Object slow = new Object();
        flight.add(slow, Field.Kind.POLLEN, POLLEN, new double[] {10, -60, POLLEN + 0.2}, new double[] {30, 5, 0});

        fly(0.1);

        assertEquals("the slow one landed", 1, landed.size());
        assertSame(slow, landed.get(0).ball);
        assertEquals(30, landed.get(0).velocity[0], DELTA);
        assertEquals(5, landed.get(0).velocity[1], DELTA);

        fly(1.0);

        assertEquals("the fast one bounced first, then landed", 2, landed.size());
        assertEquals(POLLEN, landed.get(1).at[2], DELTA);
    }

    @Test
    public void aBallThatMeetsTheFieldWallBelowItsTopBouncesBackAndOneThatClearsItIsOut() {
        double half = SimPlacement.FIELD_SIZE_IN / 2;
        Object low = new Object();
        flight.add(low, Field.Kind.POLLEN, POLLEN, new double[] {half - 10, 30, 3}, new double[] {200, 0, 0});
        Object high = new Object();
        flight.add(
                high,
                Field.Kind.POLLEN,
                POLLEN,
                new double[] {half - 10, -30, SimPlacement.WALL_HEIGHT_IN + 20},
                new double[] {200, 0, 0});

        fly(2.0);

        assertEquals(2, landed.size());
        for (SimFlight.Landing landing : landed) {
            if (landing.ball == low) {
                assertFalse("the low one stays in", landing.out);
                assertTrue("inside the wall: x=" + landing.at[0], landing.at[0] <= half - POLLEN + DELTA);
            } else {
                assertTrue("the high one is out", landing.out);
            }
        }
    }

    @Test
    public void flightIsGeometryAndNeedsNeitherTheRigidBodyEngineNorTheSimulatedRobot() throws Exception {
        List<String> forbidden = List.of("org.dyn4j.", SimRobot.class.getName());
        try (URLClassLoader loader = new URLClassLoader(SimHivesTest.classpath(), null) {
            @Override
            protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
                for (String banned : forbidden) {
                    if (name.startsWith(banned)) {
                        throw new ClassNotFoundException(
                                name + " must not be needed to fly a ball; that is the seam this test holds");
                    }
                }
                return super.loadClass(name, resolve);
            }
        }) {
            Class<?> fieldClass = Class.forName(Field.class.getName(), true, loader);
            Class<?> kindClass = Class.forName(Field.Kind.class.getName(), true, loader);
            Class<?> hivesClass = Class.forName(SimHives.class.getName(), true, loader);
            Class<?> flightClass = Class.forName(SimFlight.class.getName(), true, loader);
            Object theirField = Class.forName(SimPlacement.class.getName(), true, loader)
                    .getField("FIELD")
                    .get(null);
            Object theirHives = hivesClass.getConstructor(fieldClass).newInstance(theirField);
            Object theirFlight =
                    flightClass.getConstructor(fieldClass, hivesClass).newInstance(theirField, theirHives);
            Object ball = new Object();
            flightClass
                    .getMethod("add", Object.class, kindClass, double.class, double[].class, double[].class)
                    .invoke(
                            theirFlight,
                            ball,
                            kindClass.getField("POLLEN").get(null),
                            POLLEN,
                            new double[] {0, -60, 30},
                            new double[3]);

            for (int i = 0; i < 10; i++) {
                flightClass.getMethod("step", double.class).invoke(theirFlight, STEP);
            }

            double[] at = (double[]) flightClass.getMethod("at", Object.class).invoke(theirFlight, ball);
            assertTrue("it fell with no engine on the classpath it could reach: z=" + at[2], at[2] < 30);
        }
    }

    private double[][] at(double tilt, double[][] ring) {
        double[][] out = new double[ring.length][];
        for (int i = 0; i < ring.length; i++) {
            out[i] = Points.array(blue.at(tilt, Points.vec(ring[i])));
        }
        return out;
    }

    private static double[][] floorOf(Field.Cell cell) {
        double[][] lowest = null;
        double depth = Double.MAX_VALUE;
        for (double[][] wall : Points.arrays(cell.walls())) {
            double z = SimHivesTest.centreOf(wall)[2];
            if (z < depth) {
                depth = z;
                lowest = wall;
            }
        }
        return lowest;
    }
}
