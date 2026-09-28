package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Flight;
import org.firstinspires.ftc.teamcode.simcore.Hives;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Ring;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class FlightTest {
    private static final double DELTA = 0.001;
    private static final double POLLEN_RADIUS = 1.39;
    private static final double NECTAR_RADIUS = 1.8;
    private static final Length POLLEN = Valid.value(Length.of(POLLEN_RADIUS));
    private static final Length NECTAR = Valid.value(Length.of(NECTAR_RADIUS));
    private static final double CONTACT = 0.1;
    private static final double STEP = 0.005;

    private final Field field = SimPlacement.FIELD;
    private Flight<Object> flight = Flight.over(Hives.of(field));
    private final Field.Hive blue = flight.hives().hiveOf("Blue").orElseThrow();
    private final List<Flight.Landing<Object>> landed = new ArrayList<>();

    private Hives hives() {
        return flight.hives();
    }

    private double tilt(Field.Hive hive) {
        return hives().tilt(hive);
    }

    private Field.Cell upturned(Field.Hive hive) {
        return hives().upturnedCell(hive).orElseThrow();
    }

    private void fly(double seconds) {
        for (long i = Math.round(seconds / STEP); i > 0; i--) {
            Flight.Stepped<Object> stepped = flight.step(Valid.value(Seconds.of(STEP)));
            flight = stepped.flight();
            landed.addAll(stepped.landings());
        }
    }

    private Object throwIt(Field.Kind kind, Length radius, Vec3 at, Vec3 velocity) {
        Object ball = new Object();
        flight = flight.with(ball, kind, radius, at, velocity);
        return ball;
    }

    private Object drop(Field.Kind kind, Length radius, Vec3 at) {
        return throwIt(kind, radius, at, Vec3.zero());
    }

    private Object putIn(Field.Kind kind, Length radius, Field.Cell cell) {
        Object ball = new Object();
        flight = Valid.value(flight.within(ball, kind, radius, cell));
        return ball;
    }

    private Vec3 at(Object ball) {
        return flight.at(ball).orElseThrow();
    }

    private Vec3 aboveTheMouthOf(Field.Cell cell, double height) {
        Vec3 mouth = cell.mouthCentreAt(tilt(cell.hive()));
        return new Vec3(mouth.x(), mouth.y(), mouth.z() + height);
    }

    private Vec3 onBluesWall() {
        return blue.at(tilt(blue), new Vec3(15.45, 10.03, 2.8));
    }

    private static double side(Ring<Vec3> ring, Vec3 point) {
        double[][] corners = Points.arrays(ring);
        return point.minus(Points.vec(corners[0])).dot(Points.vec(Points.normal(corners)));
    }

    @Test
    public void aBallDroppedIntoTheMouthOfAnUpturnedCellRollsDownToItsBackAndStaysThere() {
        Field.Cell cell = upturned(blue);
        double tilt = tilt(blue);
        Object ball = drop(Field.Kind.POLLEN, POLLEN, aboveTheMouthOf(cell, 10));

        fly(3.0);

        assertTrue("it never reached the floor", landed.isEmpty() && flight.holds(ball));
        Vec3 at = at(ball);
        assertEquals("it is in the cell", Optional.of(cell), hives().cellHolding(at));
        assertEquals("and scores", 1, flight.scored("Blue"));
        assertEquals("against its back", POLLEN_RADIUS, Math.abs(side(blue.at(tilt, cell.back()), at)), CONTACT);
        assertEquals("on its floor", POLLEN_RADIUS, Math.abs(side(blue.at(tilt, floorOf(cell)), at)), CONTACT);

        fly(2.0);

        assertEquals("where it stays", at, at(ball));
    }

    @Test
    public void aBallFallingBesideACellIsKeptItsRadiusClearOfTheWallAndFallsOnPastIt() {
        Vec3 onTheWall = onBluesWall();
        double wall = onTheWall.y();
        drop(Field.Kind.POLLEN, POLLEN, new Vec3(onTheWall.x(), wall + 0.7, onTheWall.z() + 15));

        fly(3.0);

        assertEquals("it reached the floor", 1, landed.size());
        assertTrue(landed.get(0) instanceof Flight.OnTheFloor);
        double clear = landed.get(0).at().y() - wall;
        assertTrue("no nearer the wall than its radius: " + clear, clear >= POLLEN_RADIUS - CONTACT);
    }

    @Test
    public void aBallThrownHardAtTheSideOfAHiveBouncesBackOffIt() {
        Vec3 onTheWall = onBluesWall();
        double thrownFrom = onTheWall.y() + 6;
        throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(onTheWall.x(), thrownFrom, onTheWall.z()), new Vec3(0, -200, 0));

        fly(2.0);

        assertEquals("it came down", 1, landed.size());
        assertTrue(
                "back past where it was thrown from: y=" + landed.get(0).at().y(),
                landed.get(0).at().y() > thrownFrom);
    }

    @Test
    public void aBallThrownHarderThanAnyLaunchAtACellsWallDoesNotGoThroughIt() {
        Field.Cell cell = upturned(blue);
        Vec3 onTheWall = onBluesWall();
        double wall = onTheWall.y();
        Object ball = throwIt(
                Field.Kind.POLLEN, POLLEN, new Vec3(onTheWall.x(), wall + 6, onTheWall.z()), new Vec3(0, -400, 0));

        for (int i = 0; i < 400 && flight.holds(ball); i++) {
            fly(STEP);
            if (flight.holds(ball)) {
                assertTrue("never in the cell", !hives().cellHolding(at(ball)).equals(Optional.of(cell)));
            }
        }

        assertEquals("it came down", 1, landed.size());
        double clear = landed.get(0).at().y() - wall;
        assertTrue("it came down on the side it was thrown from: " + clear, clear >= POLLEN_RADIUS - CONTACT);
    }

    @Test
    public void ballsDroppedIntoACellOneAfterAnotherComeToRestInItApartFromOneAnother() {
        Field.Cell cell = upturned(blue);
        double leaning = tilt(blue);
        List<Object> balls = new ArrayList<>();
        List<Double> radii = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            boolean nectar = i % 2 == 0;
            Vec3 above = aboveTheMouthOf(cell, 10);
            balls.add(drop(
                    nectar ? Field.Kind.NECTAR : Field.Kind.POLLEN,
                    nectar ? NECTAR : POLLEN,
                    new Vec3(above.x(), above.y() + (i - 2.5) * 1.5, above.z())));
            radii.add(nectar ? NECTAR_RADIUS : POLLEN_RADIUS);
            fly(0.4);
        }

        fly(3.0);

        assertTrue("none got away", landed.isEmpty());
        assertEquals("three nectar and three pollen are not quite full", leaning, tilt(blue), 0);
        assertEquals(6, flight.scored("Blue"));
        for (int i = 0; i < balls.size(); i++) {
            Vec3 a = at(balls.get(i));
            assertEquals("ball " + i + " is in the cell", Optional.of(cell), hives().cellHolding(a));
            for (int j = 0; j < i; j++) {
                double apart = a.minus(at(balls.get(j))).length();
                assertTrue(
                        "balls " + i + " and " + j + " are " + apart + " apart",
                        apart >= radii.get(i) + radii.get(j) - CONTACT);
            }
        }
    }

    @Test
    public void aBallPutInACellRestsInItBesideWhatIsAlreadyThere() {
        Field.Cell cell = upturned(blue);

        Object first = putIn(Field.Kind.NECTAR, NECTAR, cell);
        Object second = putIn(Field.Kind.POLLEN, POLLEN, cell);

        assertEquals(Optional.of(cell), hives().cellHolding(at(first)));
        assertEquals(Optional.of(cell), hives().cellHolding(at(second)));
        assertTrue("side by side", at(first).minus(at(second)).length() >= NECTAR_RADIUS + POLLEN_RADIUS - DELTA);
        assertEquals(
                "and a nectar and a pollen are that much of the hive",
                0.2 + 0.125,
                flight.load("Blue").orElseThrow(),
                DELTA);
    }

    @Test
    public void aCellWithNoRoomLeftRefusesAnotherBallByName() {
        Field.Cell cell = upturned(blue);
        String refused = "";
        for (int i = 0; i < 100 && refused.isEmpty(); i++) {
            Object ball = new Object();
            refused = flight.within(ball, Field.Kind.NECTAR, NECTAR, cell)
                    .fold(
                            next -> {
                                flight = next;
                                return "";
                            },
                            rule -> rule);
        }

        assertTrue(refused, refused.contains("no room left in " + cell.name() + " for another Nectar"));
    }

    @Test
    public void aHiveThatIsFullTipsAndWhatWasInItRollsOutThroughTheMouthAndLandsBelowIt() {
        Field.Cell cell = upturned(blue);
        double leaning = tilt(blue);
        double mouthSide = Math.signum(cell.mouthCentreAt(leaning).x());
        for (int i = 0; i < 5; i++) {
            putIn(Field.Kind.NECTAR, NECTAR, cell);
        }

        fly(0.05);

        assertTrue("five nectar fill it and it has begun to tip", hives().tipping(blue));
        assertEquals("with them still in it", 5, flight.scored("Blue"));

        fly(4.0);

        assertEquals("it tipped over", -leaning, tilt(blue), DELTA);
        assertEquals("and is empty", 0, flight.scored("Blue"));
        assertEquals("what was in it all landed", 5, landed.size());
        for (Flight.Landing<Object> landing : landed) {
            assertTrue(landing instanceof Flight.OnTheFloor);
            assertEquals("on the floor", NECTAR_RADIUS, landing.at().z(), DELTA);
            assertTrue(
                    "at the end of the hive whose mouth it rolled out of: x="
                            + landing.at().x(),
                    landing.at().x() * mouthSide > 0);
        }
    }

    @Test
    public void aBallThatHitsTheFloorFastBouncesAndOneThatComesDownSlowlyLandsWithTheSpeedItHadAlongIt() {
        drop(Field.Kind.POLLEN, POLLEN, new Vec3(0, -60, 30));
        Object slow = throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(10, -60, POLLEN_RADIUS + 0.2), new Vec3(30, 5, 0));

        fly(0.1);

        assertEquals("the slow one landed", 1, landed.size());
        assertSame(slow, landed.get(0).ball());
        Flight.OnTheFloor<Object> landing = (Flight.OnTheFloor<Object>) landed.get(0);
        assertEquals(30, landing.velocity().x(), DELTA);
        assertEquals(5, landing.velocity().y(), DELTA);

        fly(1.0);

        assertEquals("the fast one bounced first, then landed", 2, landed.size());
        assertEquals(POLLEN_RADIUS, landed.get(1).at().z(), DELTA);
    }

    @Test
    public void aBallThatMeetsTheFieldWallBelowItsTopBouncesBackAndOneThatClearsItIsOut() {
        double half = SimPlacement.FIELD_SIZE_IN / 2;
        Object low = throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(half - 10, 30, 3), new Vec3(200, 0, 0));
        throwIt(
                Field.Kind.POLLEN,
                POLLEN,
                new Vec3(half - 10, -30, SimPlacement.WALL_HEIGHT_IN + 20),
                new Vec3(200, 0, 0));

        fly(2.0);

        assertEquals(2, landed.size());
        for (Flight.Landing<Object> landing : landed) {
            if (landing.ball() == low) {
                assertTrue("the low one stays in", landing instanceof Flight.OnTheFloor);
                assertTrue(
                        "inside the wall: x=" + landing.at().x(), landing.at().x() <= half - POLLEN_RADIUS + DELTA);
            } else {
                assertTrue("the high one is out", landing instanceof Flight.OutOfTheField);
            }
        }
    }

    @Test
    public void aFlightIsAValueSoSteppingItLeavesTheOneSteppedAsItWas() {
        Object ball = drop(Field.Kind.POLLEN, POLLEN, new Vec3(0, -60, 30));
        Flight<Object> before = flight;

        fly(0.1);

        assertEquals(new Vec3(0, -60, 30), before.at(ball).orElseThrow());
        assertFalse(at(ball).equals(before.at(ball).orElseThrow()));
    }

    private static Ring<Vec3> floorOf(Field.Cell cell) {
        Ring<Vec3> lowest = cell.walls().get(0);
        double depth = Double.MAX_VALUE;
        for (Ring<Vec3> wall : cell.walls()) {
            double z = 0;
            for (Vec3 corner : wall.all()) {
                z += corner.z() / wall.size();
            }
            if (z < depth) {
                depth = z;
                lowest = wall;
            }
        }
        return lowest;
    }
}
