package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Flight;
import org.firstinspires.ftc.teamcode.simcore.Flywheel;
import org.firstinspires.ftc.teamcode.simcore.Hives;
import org.firstinspires.ftc.teamcode.simcore.Launcher;
import org.firstinspires.ftc.teamcode.simcore.Lean;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Ring;
import org.firstinspires.ftc.teamcode.simcore.Rolling;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Shot;
import org.firstinspires.ftc.teamcode.simcore.Stack;
import org.firstinspires.ftc.teamcode.simcore.Surface;
import org.firstinspires.ftc.teamcode.simcore.Tilt;
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
    private static final double POLLEN_INERTIA = 0.634;

    private final Field field = SimPlacement.FIELD;
    private Flight<Object> flight = Valid.value(Flight.<Object>over(field));
    private final Field.Hive blue = flight.hives().hiveOf("Blue").orElseThrow();
    private final List<Flight.Landing<Object>> landed = new ArrayList<>();
    private double flown = 0;

    private Hives hives() {
        return flight.hives();
    }

    private double tilt(Field.Hive hive) {
        return hives().tilt(hive);
    }

    private Field.Cell upturned(Field.Hive hive) {
        return hives().upturnedCell(hive).orElseThrow();
    }

    private static Flight.Tick tick(double seconds) {
        return Valid.value(Flight.Tick.of(Valid.value(Seconds.of(seconds))));
    }

    private List<Flight.Landing<Object>> fly(double seconds) {
        List<Flight.Landing<Object>> now = new ArrayList<>();
        for (long i = Math.round(seconds / STEP); i > 0; i--) {
            now.addAll(once(STEP));
        }
        return now;
    }

    private List<Flight.Landing<Object>> once(double seconds) {
        Flight.Stepped<Object> stepped = Valid.value(flight.step(tick(seconds)));
        flight = stepped.flight();
        landed.addAll(stepped.landings());
        flown += seconds;
        return stepped.landings();
    }

    private Object throwIt(Field.Kind kind, Length radius, Vec3 at, Vec3 velocity) {
        Object ball = new Object();
        flight = Valid.value(flight.with(ball, kind, radius, at, velocity));
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

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted", rule -> rule);
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
    public void aBallOverlappingTheOutsideOfACellsWallIsPutItsRadiusClearOfItAndFallsOnPastIt() {
        Vec3 onTheWall = onBluesWall();
        double wall = onTheWall.y();
        drop(Field.Kind.POLLEN, POLLEN, new Vec3(onTheWall.x(), wall + 0.7, onTheWall.z()));

        fly(Falling.settledAfter(onTheWall.z()));

        assertEquals("it reached the floor", 1, landed.size());
        assertTrue(landed.get(0) instanceof Flight.OnTheFloor);
        double clear = landed.get(0).at().y() - wall;
        assertTrue("no nearer the wall than its radius: " + clear, clear >= POLLEN_RADIUS - CONTACT);
    }

    @Test
    public void aBallThrownHardAtTheSideOfAHiveBouncesBackOffItAndComesDownOnTheSideItWasThrownFrom() {
        Vec3 onTheWall = onBluesWall();
        double thrownFrom = onTheWall.y() + 6;
        throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(onTheWall.x(), thrownFrom, onTheWall.z()), new Vec3(0, -150, 0));

        fly(Falling.settledAfter(onTheWall.z()));

        assertEquals("it came down", 1, landed.size());
        assertTrue("on the field: " + landed.get(0), landed.get(0) instanceof Flight.OnTheFloor);
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

        double until = Falling.settledAfter(onTheWall.z());
        while (flown < until && flight.holds(ball)) {
            once(STEP);
            if (flight.holds(ball)) {
                assertTrue("never in the cell", !hives().cellHolding(at(ball)).equals(Optional.of(cell)));
            }
        }

        assertEquals("it is out of flight", 1, landed.size());
        double clear = landed.get(0).at().y() - wall;
        assertTrue(
                "on the side it was thrown from, on the field or beyond its wall: " + clear,
                clear >= POLLEN_RADIUS - CONTACT);
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
    public void aBallPutInACellComesToRestThereAndThenCostsNothingToFly() {
        Object ball = putIn(Field.Kind.POLLEN, POLLEN, upturned(blue));
        Vec3 put = at(ball);

        fly(1.0);
        long rested = flight.work();
        fly(1.0);

        assertEquals("it rests where it was put", 0, at(ball).minus(put).length(), CONTACT);
        assertEquals("and costs nothing once asleep", rested, flight.work());
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

        fly(Lean.TIP_SECONDS + Falling.settledAfter(cell.mouthCentreAt(leaning).z()));

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
    public void aFlightsHivesBeginAtRestOnItsOwnClockAndATipItBeginsRunsFromItsStart() {
        Hives atRest = Valid.value(Hives.of(field));
        assertEquals(atRest.tilts(), hives().tilts());
        assertFalse(hives().tipping(blue));
        double leaning = tilt(blue);
        fly(0.5);
        Field.Cell cell = upturned(blue);
        for (int i = 0; i < 5; i++) {
            putIn(Field.Kind.NECTAR, NECTAR, cell);
        }

        once(0.01);

        Lean aHundredthIn = new Lean.Tipping(
                Valid.value(Tilt.of(leaning)), Valid.value(Tilt.of(-leaning)), Valid.value(Seconds.of(0.01)));
        assertEquals("a hundredth of a second into its tip", aHundredthIn.degrees(), tilt(blue), 1e-9);
    }

    @Test
    public void aTipBegunAtAnyStepOnTheGridEndsExactlyTipSecondsOfStepsLater() {
        double oneStep = Flight.STEP_NANOS / Flight.NANOS_PER_SECOND;
        long stepsOfATip = Math.round(Lean.TIP_SECONDS / oneStep);
        Flight<Object> fresh = flight;
        for (long began : List.of(0L, 45L, 46L, 66L, 187L, 1235L)) {
            flight = fresh;
            once(began * oneStep);
            Field.Cell cell = upturned(blue);
            for (int i = 0; i < 5; i++) {
                putIn(Field.Kind.NECTAR, NECTAR, cell);
            }
            assertTrue("begun at step " + began, hives().tipping(blue));

            for (long left = stepsOfATip - 1; left > 0; left -= Math.min(left, 2000)) {
                once(Math.min(left, 2000) * oneStep);
            }
            assertTrue("still tipping a step short of its end, begun at step " + began, hives().tipping(blue));
            once(oneStep);

            assertFalse("over, begun at step " + began + ": " + hives().lean(blue), hives().tipping(blue));
        }
    }

    @Test
    public void aBallCrossingTheMouthInTheAirDoesNotTipTheHiveThoughItsLoadReadsFull() {
        Field.Cell cell = upturned(blue);
        double leaning = tilt(blue);
        for (int i = 0; i < 4; i++) {
            putIn(Field.Kind.NECTAR, NECTAR, cell);
        }
        Object falling = drop(Field.Kind.NECTAR, NECTAR, aboveTheMouthOf(cell, 3));

        boolean seenInIt = false;
        while (!seenInIt && flown < 1) {
            once(0.001);
            seenInIt = hives().cellHolding(at(falling)).equals(Optional.of(cell));
        }

        assertTrue("it went in through the mouth", seenInIt);
        assertTrue(
                "so the cell reads full: " + flight.load("Blue"),
                flight.load("Blue").orElseThrow() >= 1);
        assertEquals("but it has met nothing in there yet, so the hive does not tip", leaning, tilt(blue), 0);
        assertFalse(hives().tipping(blue));

        while (!hives().tipping(blue) && flown < 2) {
            once(STEP);
        }

        assertTrue("once it presses on what is in the cell the hive tips", hives().tipping(blue));
    }

    @Test
    public void aBallDroppedStraightDownBouncesOffTheFloorAsTheFloorsRestitutionSaysBeforeItLands() {
        double height = 30;
        Object ball = drop(Field.Kind.POLLEN, POLLEN, new Vec3(0, -60, height));
        double before = 0;
        boolean bounced = false;
        double approach = 0, rebound = 0;

        while (!bounced && flown < 1) {
            once(Flight.STEP_NANOS / Flight.NANOS_PER_SECOND);
            double vz = flight.velocity(ball).orElseThrow().z();
            if (before < 0 && vz > 0) {
                bounced = true;
                approach = -before;
                rebound = vz;
            }
            before = vz;
        }

        assertTrue("it bounced", bounced);
        assertTrue("with the ball still in the air", flight.holds(ball) && landed.isEmpty());
        assertEquals(
                "as hard as the floor gives back at " + approach + " in/s",
                flight.surfaces().restitution(Surface.FLOOR, approach),
                rebound / approach,
                0.02);

        fly(Falling.settledAfter(height));

        assertEquals("then it lands", 1, landed.size());
        assertEquals("on the floor", POLLEN_RADIUS, landed.get(0).at().z(), DELTA);
    }

    @Test
    public void aBallThatComesDownSlowlyAndSlidingLandsOnlyOnceTheFloorHasItRollingAndAtTheSpeedItRollsAt() {
        Object slow = throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(10, -60, POLLEN_RADIUS + 0.2), new Vec3(30, 5, 0));

        fly(1.0);

        assertEquals("it landed", 1, landed.size());
        Flight.OnTheFloor<Object> landing = (Flight.OnTheFloor<Object>) landed.get(0);
        assertTrue(landing.ball() == slow);
        double rolling = Math.hypot(30, 5) / (1 + POLLEN_INERTIA);
        double speed = Math.hypot(landing.velocity().x(), landing.velocity().y());
        assertEquals("rolling, no faster than a slide turned into rolling leaves it", rolling, speed, 0.05 * rolling);
        assertTrue("and slower for the floor it rolled over: " + speed, speed < rolling);
        assertEquals(
                "the way it was going",
                5.0 / 30,
                landing.velocity().y() / landing.velocity().x(),
                0.01);
        assertEquals(POLLEN_RADIUS, landing.at().z(), DELTA);
    }

    @Test
    public void aBallThatComesToRestAgainstAWallOrAnObstacleOnTheFloorLandsThere() {
        double half = SimPlacement.FIELD_SIZE_IN / 2;
        drop(Field.Kind.POLLEN, POLLEN, new Vec3(half - POLLEN_RADIUS, -40, 6));
        drop(Field.Kind.POLLEN, POLLEN, new Vec3(0, -24.73 - POLLEN_RADIUS, 6));

        fly(Falling.settledAfter(6) + 1);

        assertEquals("both landed", 2, landed.size());
        assertEquals(
                "against the far wall", half - POLLEN_RADIUS, landed.get(0).at().x(), CONTACT);
        assertEquals(
                "against the foot bar",
                -24.73 - POLLEN_RADIUS,
                landed.get(1).at().y(),
                CONTACT);
    }

    @Test
    public void twoBallsThatComeToRestSideBySideOnTheFloorLandTogether() {
        drop(Field.Kind.POLLEN, POLLEN, new Vec3(-30, -50, 4));
        drop(Field.Kind.POLLEN, POLLEN, new Vec3(-30 + 2 * POLLEN_RADIUS, -50, 4));

        List<Flight.Landing<Object>> together = List.of();
        while (landed.isEmpty() && flown < Falling.settledAfter(4) + 1) {
            together = once(STEP);
        }

        assertEquals("they landed in the same step", 2, together.size());
        assertEquals(
                "touching",
                2 * POLLEN_RADIUS,
                together.get(0).at().minus(together.get(1).at()).length(),
                CONTACT);
    }

    @Test
    public void aBallThrownAtAFlowersPipeBouncesOffIt() {
        Field.Flower flower = field.flowers().get(0);
        Vec3 pipe = new Vec3(66.36, 21.72, 12);
        double face = lowestXOf(flower.name(), "Flower HIPS Pipe");
        double from = pipe.x() - 10;
        Object ball = throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(from, pipe.y(), pipe.z()), new Vec3(150, 0, 0));

        double farthest = from;
        while (flight.holds(ball) && flown < Falling.settledAfter(pipe.z()) + 1) {
            once(STEP);
            farthest = flight.holds(ball) ? Math.max(farthest, at(ball).x()) : farthest;
        }

        assertTrue(
                "it reached the pipe's face and went no further: x=" + farthest,
                farthest > face - POLLEN_RADIUS - 0.5 && farthest <= face - POLLEN_RADIUS + 0.05);
        assertEquals(1, landed.size());
        assertTrue(
                "and came back off it: x=" + landed.get(0).at().x(),
                landed.get(0).at().x() < from);
    }

    @Test
    public void aPollenThatComesToRestOnAFlowersBasePlateLandsThere() {
        Field.Flower flower = field.flowers().get(0);
        double x = flower.axis().x(), y = flower.axis().y(), under = 2.3;
        Flight<Object> fresh = flight;
        for (Vec3 spot : List.of(
                new Vec3(x, y, under),
                new Vec3(x - 1.0, y + 0.5, under),
                new Vec3(x - flower.bore() - 0.15, y, under))) {
            flight = fresh;
            landed.clear();
            Object ball = drop(Field.Kind.POLLEN, POLLEN, spot);

            fly(Falling.settledAfter(under) + 1);

            assertEquals("let go at " + spot, 1, landed.size());
            assertTrue(landed.get(0) instanceof Flight.OnTheFloor);
            assertTrue(landed.get(0).ball() == ball);
            assertEquals(
                    "where it came to rest, let go at " + spot,
                    spot.x(),
                    landed.get(0).at().x(),
                    0.2);
            assertEquals(spot.y(), landed.get(0).at().y(), 0.2);
            assertEquals(POLLEN_RADIUS, landed.get(0).at().z(), 0);
        }
    }

    private static final double TOUCHING_WITHIN = 0.01;

    private static Stack<Object> threeStandingOnTheNestedPollen(Field.Flower flower) {
        Stack<Object> stack = Stack.in(flower);
        for (int above = 1; above <= 3; above++) {
            stack = stack.with(new Object(), POLLEN, (2 * above + 1) * POLLEN_RADIUS);
        }
        return stack;
    }

    private static List<Rolling<Object>> nestedIn(Field.Flower flower) {
        return List.of(new Rolling<>(new Object(), flower.axis(), POLLEN));
    }

    private static List<Vec3[]> rollsOntoTheBasePlateOf(Field.Flower flower) {
        double x = flower.axis().x(), y = flower.axis().y();
        return List.of(
                new Vec3[] {new Vec3(x - 7, y, POLLEN_RADIUS), new Vec3(50, 0, 0)},
                new Vec3[] {new Vec3(x, y + 7, POLLEN_RADIUS), new Vec3(0, -50, 0)});
    }

    private double nearestTo(Vec3 point, Object ball) {
        double nearest = Double.POSITIVE_INFINITY;
        for (double rolled = 0; flight.holds(ball) && rolled < 5; rolled += STEP) {
            once(STEP);
            if (flight.holds(ball)) {
                Vec3 apart = at(ball).minus(point);
                nearest = Math.min(nearest, Math.sqrt(apart.dot(apart)));
            }
        }
        return nearest;
    }

    @Test
    public void aPollenRolledOntoAFlowersBasePlateMeetsTheStackStandingInItsBoreAndLandsClearOfIt() {
        Field.Flower flower = field.flowers().get(0);
        double x = flower.axis().x(), y = flower.axis().y();
        Vec3 nested = new Vec3(x, y, highestOf(flower.name(), "Flower Layer X") + POLLEN_RADIUS);
        Flight<Object> amidTheStack =
                Valid.value(flight.amid(List.of(threeStandingOnTheNestedPollen(flower)), nestedIn(flower)));
        for (Vec3[] roll : rollsOntoTheBasePlateOf(flower)) {
            flight = amidTheStack;
            landed.clear();
            Object ball = throwIt(Field.Kind.POLLEN, POLLEN, roll[0], roll[1]);

            double nearest = nearestTo(nested, ball);

            String rolled = "rolled from " + roll[0] + " at " + roll[1];
            assertTrue(
                    rolled + ": it came within " + nearest + " in of the nested pollen's centre",
                    nearest >= 2 * POLLEN_RADIUS - TOUCHING_WITHIN);
            assertEquals(rolled, 1, landed.size());
            assertTrue(rolled, landed.get(0) instanceof Flight.OnTheFloor);
            double clear =
                    Math.hypot(landed.get(0).at().x() - x, landed.get(0).at().y() - y);
            assertTrue(
                    rolled + ": it landed " + clear + " in from the axis",
                    clear >= 2 * POLLEN_RADIUS - TOUCHING_WITHIN);
        }
    }

    @Test
    public void aPollenSetDownOnABasePlateAgainstTheSideOfAStackLandsThereTouchingIt() {
        Field.Flower flower = field.flowers().get(0);
        double x = flower.axis().x(), y = flower.axis().y();
        double plate = highestOf(flower.name(), "Flower Layer X");
        flight = Valid.value(flight.amid(List.of(threeStandingOnTheNestedPollen(flower)), nestedIn(flower)));
        Object ball = drop(Field.Kind.POLLEN, POLLEN, new Vec3(x, y - 2 * POLLEN_RADIUS, plate + POLLEN_RADIUS));

        fly(1.0);

        assertEquals("it landed", 1, landed.size());
        assertTrue(landed.get(0) instanceof Flight.OnTheFloor);
        assertTrue(landed.get(0).ball() == ball);
        assertEquals(
                "against the nested pollen",
                2 * POLLEN_RADIUS,
                Math.hypot(landed.get(0).at().x() - x, landed.get(0).at().y() - y),
                TOUCHING_WITHIN);
    }

    @Test
    public void aStackStandsInItsBoreThroughAHivesTip() {
        Field.Flower flower = field.flowers().get(0);
        Vec3 nested = new Vec3(
                flower.axis().x(), flower.axis().y(), highestOf(flower.name(), "Flower Layer X") + POLLEN_RADIUS);
        flight = Valid.value(flight.amid(List.of(threeStandingOnTheNestedPollen(flower)), nestedIn(flower)));
        for (int i = 0; i < 5; i++) {
            putIn(Field.Kind.NECTAR, NECTAR, upturned(blue));
        }
        fly(0.05);
        assertTrue("the hive has begun to tip", hives().tipping(blue));
        Vec3[] roll = rollsOntoTheBasePlateOf(flower).get(0);
        Object ball = throwIt(Field.Kind.POLLEN, POLLEN, roll[0], roll[1]);

        double nearest = nearestTo(nested, ball);

        assertTrue(
                "it came within " + nearest + " in of the nested pollen's centre",
                nearest >= 2 * POLLEN_RADIUS - TOUCHING_WITHIN);
    }

    @Test
    public void aBallTheFloorNoLongerHasInABoreNoLongerStandsThereInFlight() {
        Field.Flower flower = field.flowers().get(0);
        Vec3 nested = new Vec3(
                flower.axis().x(), flower.axis().y(), highestOf(flower.name(), "Flower Layer X") + POLLEN_RADIUS);
        flight = Valid.value(flight.amid(List.of(threeStandingOnTheNestedPollen(flower)), nestedIn(flower)));
        flight = Valid.value(flight.amid(List.of(Stack.in(flower)), List.of()));
        Vec3[] roll = rollsOntoTheBasePlateOf(flower).get(0);
        Object ball = throwIt(Field.Kind.POLLEN, POLLEN, roll[0], roll[1]);

        double nearest = nearestTo(nested, ball);

        assertTrue("it rolled in over the empty nest: " + nearest, nearest < POLLEN_RADIUS);
    }

    @Test
    public void aBallLetGoOnAFlowersTopComesToRestThereAboveItsBoreAndStaysInFlight() {
        Field.Flower flower = field.flowers().get(0);
        double top = highestOf(flower.name(), "Flower Layer C");
        double clearOfTheBackstop = lowestXOf(flower.name(), "Flower Backstop") - POLLEN_RADIUS - 0.2;
        Object ball = drop(
                Field.Kind.POLLEN,
                POLLEN,
                new Vec3(clearOfTheBackstop, flower.axis().y(), top + POLLEN_RADIUS + 0.5));

        fly(1.0);

        assertTrue("it is still in flight", flight.holds(ball) && landed.isEmpty());
        assertEquals("resting on the flower's top", top + POLLEN_RADIUS, at(ball).z(), CONTACT);
        assertTrue("which closes its bore", at(ball).z() - POLLEN_RADIUS > flower.lip());
        long resting = flight.work();
        fly(1.0);
        assertEquals("at rest", resting, flight.work());
    }

    private double highestOf(String group, String name) {
        double highest = Double.NEGATIVE_INFINITY;
        for (Field.Element element : field.elements()) {
            if (element.group().equals(group) && element.name().equals(name)) {
                for (Vec3 vertex : element.vertices()) {
                    highest = Math.max(highest, vertex.z());
                }
            }
        }
        return highest;
    }

    private double lowestXOf(String group, String name) {
        double lowest = Double.POSITIVE_INFINITY;
        for (Field.Element element : field.elements()) {
            if (element.group().equals(group) && element.name().equals(name)) {
                for (Vec3 vertex : element.vertices()) {
                    lowest = Math.min(lowest, vertex.x());
                }
            }
        }
        return lowest;
    }

    @Test
    public void aBallThatMeetsTheFieldWallBelowItsTopBouncesBackAndOneThatClearsItIsOutBeyondItsOuterFace() {
        double half = SimPlacement.FIELD_SIZE_IN / 2;
        Object low = throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(half - 10, 30, 3), new Vec3(200, 0, 0));
        throwIt(
                Field.Kind.POLLEN,
                POLLEN,
                new Vec3(half - 10, -30, SimPlacement.WALL_HEIGHT_IN + 20),
                new Vec3(200, 0, 0));

        fly(Falling.settledAfter(SimPlacement.WALL_HEIGHT_IN + 20));

        assertEquals(2, landed.size());
        for (Flight.Landing<Object> landing : landed) {
            if (landing.ball() == low) {
                assertTrue("the low one stays in", landing instanceof Flight.OnTheFloor);
                assertTrue(
                        "inside the wall: x=" + landing.at().x(), landing.at().x() <= half - POLLEN_RADIUS + DELTA);
            } else {
                assertTrue("the high one is out", landing instanceof Flight.OutOfTheField);
                assertTrue(
                        "beyond the wall's outer face: x=" + landing.at().x(),
                        landing.at().x() > half + 1 && landing.at().x() < half + 1 + 0.5);
                assertEquals(POLLEN_RADIUS, landing.at().z(), 0);
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

    @Test
    public void aTickOfFlightIsAtMostAQuarterOfASecond() {
        String longer = rejection(Flight.Tick.of(Valid.value(Seconds.of(Math.nextUp(Flight.Tick.MOST_SECONDS)))));

        assertTrue(longer, longer.contains("a tick of flight is at most 0.25 s"));
        assertEquals(
                Flight.Tick.MOST_SECONDS,
                tick(Flight.Tick.MOST_SECONDS).seconds().value(),
                0);
    }

    @Test
    public void aBallComesIntoFlightAtAFinitePlaceMovingNoFasterThanAThousandInchesASecond() {
        Vec3 at = new Vec3(0, 0, 20);
        List<String> said = List.of(
                rejection(flight.with(new Object(), Field.Kind.POLLEN, POLLEN, at, new Vec3(1000.001, 0, 0))),
                rejection(flight.with(new Object(), Field.Kind.POLLEN, POLLEN, at, new Vec3(Double.NaN, 0, 0))));
        String nowhere = rejection(
                flight.with(new Object(), Field.Kind.POLLEN, POLLEN, new Vec3(0, Double.NaN, 1), Vec3.zero()));

        for (String rule : said) {
            assertTrue(rule, rule.contains("a ball comes into flight at most 1000.0 in/s"));
        }
        assertTrue(nowhere, nowhere.contains("a body is at a finite place"));
        assertEquals(
                "accepted", rejection(flight.with(new Object(), Field.Kind.POLLEN, POLLEN, at, new Vec3(0, 0, 1000))));
    }

    @Test
    public void flyingInFourTicksOfFiveMillisecondsIsFlyingInOneOfTwentyToTheBit() {
        Flight<Object> start = flight;
        Object ball = drop(Field.Kind.POLLEN, POLLEN, aboveTheMouthOf(upturned(blue), 10));
        Flight<Object> fine = flight, coarse = flight;
        for (int i = 0; i < 100; i++) {
            for (int j = 0; j < 4; j++) {
                fine = Valid.value(fine.step(tick(0.005))).flight();
            }
            coarse = Valid.value(coarse.step(tick(0.02))).flight();
        }

        assertTrue(start != fine);
        assertEquals(fine.at(ball), coarse.at(ball));
        assertEquals(fine.velocity(ball), coarse.velocity(ball));
        assertEquals(fine.work(), coarse.work());
    }

    private record Flown(
            List<Flight.Landing<Object>> landings, List<Optional<Vec3>> at, List<Optional<Vec3>> velocity, long work) {}

    private record Ticked(Flown flown, long mostInATick) {}

    private static Ticked flownIn(double tick, Flight<Object> start, List<Object> balls, double seconds) {
        Flight<Object> flying = start;
        List<Flight.Landing<Object>> landings = new ArrayList<>();
        long most = 0;
        for (long i = Math.round(seconds / tick); i > 0; i--) {
            long before = flying.work();
            Flight.Stepped<Object> stepped = Valid.value(flying.step(tick(tick)));
            flying = stepped.flight();
            landings.addAll(stepped.landings());
            most = Math.max(most, flying.work() - before);
        }
        List<Optional<Vec3>> at = new ArrayList<>(), velocity = new ArrayList<>();
        for (Object ball : balls) {
            at.add(flying.at(ball));
            velocity.add(flying.velocity(ball));
        }
        return new Ticked(new Flown(landings, at, velocity, flying.work()), most);
    }

    private Flight<Object> crowd(List<Object> balls) {
        Flight<Object> crowd = flight;
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 4; j++) {
                Object ball = new Object();
                balls.add(ball);
                crowd = Valid.value(crowd.with(
                        ball, Field.Kind.POLLEN, POLLEN, new Vec3(-10 + 2.9 * i, -50 + 2.9 * j, 3), Vec3.zero()));
            }
        }
        Object thrown = new Object();
        balls.add(thrown);
        return Valid.value(crowd.with(thrown, Field.Kind.POLLEN, POLLEN, new Vec3(-30, -47, 3), new Vec3(400, 0, 0)));
    }

    private Flight<Object> heap(List<Object> balls) {
        Flight<Object> heap = flight;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    Object ball = new Object();
                    balls.add(ball);
                    heap = Valid.value(heap.with(
                            ball,
                            Field.Kind.POLLEN,
                            POLLEN,
                            new Vec3(-30 + 2.9 * i, -50 + 2.9 * j, 10 + 2.9 * k),
                            new Vec3(0, 0, -150)));
                }
            }
        }
        return heap;
    }

    @Test
    public void aCrowdAndAHeapThatSpendTheirAllowanceFlyTheSameToTheBitInTicksOfAnyLength() {
        List<Object> crowded = new ArrayList<>(), heaped = new ArrayList<>();
        Flight<Object> crowd = crowd(crowded), heap = heap(heaped);

        Ticked crowdInWindows = flownIn(WINDOW_SECONDS, crowd, crowded, 4),
                heapInWindows = flownIn(WINDOW_SECONDS, heap, heaped, 1);

        assertTrue(
                "the crowd spends a window's allowance: " + crowdInWindows.mostInATick(),
                crowdInWindows.mostInATick() > WINDOW_WORK);
        assertTrue("and so does the heap: " + heapInWindows.mostInATick(), heapInWindows.mostInATick() > WINDOW_WORK);
        assertEquals("what the crowd cost", CROWD_WORK, crowdInWindows.flown().work());
        assertEquals("what the heap cost", HEAP_WORK, heapInWindows.flown().work());
        for (double tick : List.of(0.001, 0.0025, 0.02)) {
            assertEquals(
                    "the crowd in ticks of " + tick + " s",
                    crowdInWindows.flown(),
                    flownIn(tick, crowd, crowded, 4).flown());
            assertEquals(
                    "the heap in ticks of " + tick + " s",
                    heapInWindows.flown(),
                    flownIn(tick, heap, heaped, 1).flown());
        }
    }

    @Test
    public void aWindowOfWorkIsFiveMillisecondsOfStepsAllowedWhatAShotMayDoInThatTime() {
        assertEquals(0.005, Flight.WINDOW_STEPS * Flight.STEP_NANOS / Flight.NANOS_PER_SECOND, 0);
        assertEquals(0.005, Flight.WINDOW_SECONDS, 0);
        assertEquals(Shot.MOST_WORK_PER_SECOND * Flight.WINDOW_SECONDS, Flight.WINDOW_WORK, 0);
    }

    @Test
    public void aStepThatBeginsWithItsWindowsAllowanceSpentLocatesNoContactAndWhatAHeapCostsIsPinned() {
        List<Object> balls = new ArrayList<>();
        flight = heap(balls);
        double oneStep = Flight.STEP_NANOS / Flight.NANOS_PER_SECOND;
        long windowBegan = flight.work();
        int unlocated = 0;

        for (long step = 0; step < Math.round(1 / oneStep); step++) {
            if (step % Flight.WINDOW_STEPS == 0) {
                windowBegan = flight.work();
            }
            long inFlight = balls.stream().filter(flight::holds).count();
            long before = flight.work();
            once(oneStep);
            if (before - windowBegan >= WINDOW_WORK) {
                unlocated++;
                long cost = flight.work() - before;
                assertTrue(
                        "step " + step + " began with its window's allowance spent and cost " + cost
                                + ", as much as locating a contact among " + inFlight + " balls would",
                        cost < Shot.BISECTIONS * inFlight);
            }
        }

        assertTrue("steps that began with their window's allowance spent: " + unlocated, unlocated > 0);
        assertEquals("what the heap cost", HEAP_WORK, flight.work());
    }

    private static final double WINDOW_SECONDS = Flight.WINDOW_SECONDS;
    private static final long WINDOW_WORK = Flight.WINDOW_WORK;
    private static final long CROWD_WORK = 538639;
    private static final long HEAP_WORK = 1437391;

    @Test
    public void aFlightWithNothingAwakeJumpsToWhereFlyingThereStepByStepWouldLeaveIt() {
        Flight<Object> waited = flight, stepped = flight;
        for (int i = 0; i < 2; i++) {
            waited = Valid.value(waited.step(tick(0.25))).flight();
        }
        for (int i = 0; i < 500; i++) {
            stepped = Valid.value(stepped.step(tick(0.001))).flight();
        }
        Object ball = new Object();
        Vec3 from = aboveTheMouthOf(upturned(blue), 10);
        waited = Valid.value(waited.with(ball, Field.Kind.POLLEN, POLLEN, from, Vec3.zero()));
        stepped = Valid.value(stepped.with(ball, Field.Kind.POLLEN, POLLEN, from, Vec3.zero()));
        for (int i = 0; i < 100; i++) {
            waited = Valid.value(waited.step(tick(0.01))).flight();
            stepped = Valid.value(stepped.step(tick(0.01))).flight();
        }

        assertEquals(stepped.at(ball), waited.at(ball));
        assertEquals(stepped.work(), waited.work());
    }

    @Test
    public void ballsSetDownAtRestInACellCostNothingToFlyUntilSomethingReachesThem() {
        Field.Cell cell = upturned(blue);
        for (Field.Piece piece : field.cellPieces()) {
            if (piece.place() instanceof Field.Place.InCell in && in.cell().equals(cell)) {
                flight = Valid.value(flight.restingAt(
                        new Object(), piece.kind(), Valid.value(Length.of(piece.radius())), piece.at()));
            }
        }
        long setDown = flight.work();

        fly(2.0);

        assertEquals(3, flight.scored("Blue"));
        assertEquals(0.6, flight.load("Blue").orElseThrow(), DELTA);
        assertEquals("nothing moved, so nothing was worked out", setDown, flight.work());
    }

    @Test
    public void aBallSetDownWhereItOverlapsTheCellIsSetDownClearOfIt() {
        Field.Cell cell = upturned(blue);
        Field.Piece piece = field.cellPieces().stream()
                .filter(one -> one.place() instanceof Field.Place.InCell in
                        && in.cell().equals(cell))
                .findFirst()
                .orElseThrow();
        Object ball = new Object();

        flight = Valid.value(flight.restingAt(ball, piece.kind(), Valid.value(Length.of(piece.radius())), piece.at()));

        double fromTheBack = Math.abs(side(blue.at(tilt(blue), cell.back()), at(ball)));
        assertTrue("clear of the back: " + fromTheBack, fromTheBack >= piece.radius() - 1e-6);
        assertEquals(Optional.of(cell), hives().cellHolding(at(ball)));
    }

    @Test
    public void whatALaunchIntoACellAndAFiveNectarTipCostIsPinned() {
        Field.Cell cell = upturned(blue);
        Vec3 mouth = cell.mouthCentreAt(tilt(blue));
        Vec3 from = new Vec3(mouth.x() + 40, mouth.y(), 14);
        double seconds = 0.6, rise = mouth.z() + 1 - from.z();
        throwIt(
                Field.Kind.POLLEN,
                POLLEN,
                from,
                new Vec3((mouth.x() - from.x()) / seconds, 0, rise / seconds + Flight.GRAVITY_IN_PER_S2 * seconds / 2));
        fly(3.0);
        long launched = flight.work();
        for (int i = 0; i < 5; i++) {
            putIn(Field.Kind.NECTAR, NECTAR, upturned(hives().hiveOf("Red").orElseThrow()));
        }
        fly(4);

        assertEquals("a launch into the cell", LAUNCH_WORK, launched);
        assertEquals("then a five-nectar tip", TIP_WORK, flight.work() - launched);
    }

    private static final String AIR_SOURCE = "a ball the robot carries goes through the still air at the robot's"
            + " velocity as well as its own, so the air slows it more than it slows the same throw from a still"
            + " robot, and by the 50 ms after the feed at which the throws are compared that has cost it less than"
            + " 1 % of the robot's speed; in a vacuum the two differ by rounding alone (LauncherTest)";
    private static final double AIR_TAKES_OF_THE_ROBOTS_SPEED = 0.01;
    private static final String ROUNDING_SOURCE = "a turned frame throws as an unturned one does, turned, but for"
            + " where the two put the ball, which differs by the rounding of a place on the field, an ulp of 100 in;"
            + " the squeeze, ringing at about 1,800 a second, the root of its 80 N/mm over the pollen's mass, turns"
            + " that into a velocity at each of the 600 steps from the first course to the comparison";
    private static final double ROUNDING_WITHIN = Math.ulp(100.0) * 1800 * 600;
    private static final long LAUNCH_WORK = 136054;
    private static final long TIP_WORK = 1251376;

    private static final String TICKS_PER_TURN_SOURCE = "doc/plans/runtime-flight.md, stage 4, Calibration: the"
            + " flywheel's ticks per turn is expected near 24.2, a 1.16:1 step-up over a 28-tick motor, until the"
            + " calibration builds it in";
    private static final double TICKS_PER_TURN = 24.2;
    private static final double CLOSE_LAUNCH = 1050;
    private static final String ROBOT_TOP_SOURCE =
            "doc/plans/reginald.md: Reginald stands 9.084 in high after the trim";
    private static final double ROBOT_TOP_IN = 9.084;
    private static final double THROWN_ABOVE_IN = 30;
    private static final double ROBOT_X = -30;
    private static final double ROBOT_Y = -45;
    private static final double ROBOT_YAW = 0;
    private static final double AFTER_THE_FEED_SECONDS = 0.05;

    private static Launcher.Frame frame(double x, double y, double yaw) {
        return Valid.value(Launcher.Frame.of(x, y, yaw));
    }

    private static Launcher.Frame resting(double seconds) {
        return frame(ROBOT_X, ROBOT_Y, ROBOT_YAW);
    }

    private static Flywheel.AngularSpeed wheel(double ticksPerSecond) {
        assertEquals(TICKS_PER_TURN_SOURCE, 24.2, TICKS_PER_TURN, 0);
        return Flywheel.AngularSpeed.from(ticksPerSecond, Valid.value(Flywheel.TicksPerTurn.of(TICKS_PER_TURN)));
    }

    private void carryALauncher(DoubleFunction<Launcher.Frame> frameAt) {
        flown = 0;
        flight = Valid.value(Flight.<Object>carryingALauncher(field, frameAt.apply(flown)));
    }

    private Object feed() {
        Object ball = new Object();
        flight = Valid.value(flight.fed(ball, Field.Kind.POLLEN, POLLEN)).fold(next -> next, why -> {
            throw new AssertionError("the entry was clear, so the ball goes in, but the feed waited: " + why);
        });
        return ball;
    }

    private static String waiting(Checked<Flight.Feeding<Object>> feeding) {
        return feeding.fold(
                fed -> fed.fold(
                        next -> {
                            throw new AssertionError("fed while its entry was not clear");
                        },
                        why -> why),
                rule -> {
                    throw new AssertionError("refused, where a feed whose entry is not clear waits: " + rule);
                });
    }

    private void carry(double seconds, DoubleFunction<Launcher.Frame> frameAt, Flywheel.AngularSpeed wheel) {
        for (long i = Math.round(seconds / STEP); i > 0; i--) {
            flown += STEP;
            Flight.Stepped<Object> stepped = Valid.value(flight.step(tick(STEP), frameAt.apply(flown), wheel));
            flight = stepped.flight();
            landed.addAll(stepped.landings());
        }
    }

    private double highestWithin(double seconds, Object ball, DoubleFunction<Launcher.Frame> frameAt) {
        double highest = Double.NEGATIVE_INFINITY;
        for (long i = Math.round(seconds / STEP); i > 0 && flight.holds(ball); i--) {
            carry(STEP, frameAt, wheel(CLOSE_LAUNCH));
            highest = flight.holds(ball) ? Math.max(highest, at(ball).z()) : highest;
        }
        return highest;
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual, double within) {
        assertEquals(what + ": x of " + actual, expected.x(), actual.x(), within);
        assertEquals(what + ": y of " + actual, expected.y(), actual.y(), within);
        assertEquals(what + ": z of " + actual, expected.z(), actual.z(), within);
    }

    private static Vec3 turned(Vec3 v, double radians) {
        double cos = Math.cos(radians), sin = Math.sin(radians);
        return new Vec3(cos * v.x() - sin * v.y(), sin * v.x() + cos * v.y(), v.z());
    }

    @Test
    public void aFlightWithNoRobotHasNoLauncherToStepFeedOrCarryAndOneCarryingALauncherStepsOnlyWithItsFrameAndWheel() {
        Launcher.Frame at = resting(0);

        List<String> robotless = List.of(
                rejection(flight.fed(new Object(), Field.Kind.POLLEN, POLLEN)),
                rejection(flight.step(tick(STEP), at, wheel(0))),
                rejection(flight.carriedTo(at)));
        String alone = rejection(
                Valid.value(Flight.<Object>carryingALauncher(field, at)).step(tick(STEP)));

        for (String rule : robotless) {
            assertTrue(rule, rule.contains("a flight with no robot has no launcher"));
        }
        assertTrue(
                alone,
                alone.contains(
                        "a flight that carries a launcher steps with the frame it ends at and its" + " wheel's speed"));
    }

    @Test
    public void aLauncherIsFedOnlyAPollenOfItsSizeARefusalNamingTheRuleAndAFeedWaitsWhileItsEntryIsNotClearSayingSo() {
        carryALauncher(FlightTest::resting);
        Flight<Object> empty = flight;
        Object first = feed();

        String crowded = waiting(flight.fed(new Object(), Field.Kind.POLLEN, POLLEN));
        carry(1.0, FlightTest::resting, wheel(0));
        String stillCrowded = waiting(flight.fed(new Object(), Field.Kind.POLLEN, POLLEN));
        String nectar = rejection(empty.fed(new Object(), Field.Kind.NECTAR, NECTAR));
        String bigger = rejection(empty.fed(new Object(), Field.Kind.POLLEN, NECTAR));

        assertTrue(crowded, crowded.contains("the launcher's entry is not clear"));
        assertTrue(stillCrowded, stillCrowded.contains("the launcher's entry is not clear"));
        assertTrue(nectar, nectar.contains("the launcher is fed a pollen of 1.39 in radius"));
        assertTrue(bigger, bigger.contains("the launcher is fed a pollen of 1.39 in radius"));
        assertTrue("the fed ball is in flight", flight.holds(first) && landed.isEmpty());
        assertTrue(
                ROBOT_TOP_SOURCE + ": inside the robot and off the floor at " + at(first),
                at(first).z() - POLLEN_RADIUS > 0 && at(first).z() + POLLEN_RADIUS < ROBOT_TOP_IN);
    }

    @Test
    public void withBothGatesOpenBallsAreFedOneAtATimeAsTheEntryClearsAndEachIsThrownWithNoTwoEverOverlapping() {
        carryALauncher(FlightTest::resting);
        List<Object> fed = new ArrayList<>();
        List<Double> highest = new ArrayList<>();
        int refusals = 0;
        double nearest = Double.POSITIVE_INFINITY;

        for (int tick = 0; tick < 400; tick++) {
            if (fed.size() < 4) {
                Object ball = new Object();
                String refused = Valid.value(flight.fed(ball, Field.Kind.POLLEN, POLLEN))
                        .fold(
                                next -> {
                                    flight = next;
                                    return "";
                                },
                                why -> why);
                if (refused.isEmpty()) {
                    fed.add(ball);
                    highest.add(Double.NEGATIVE_INFINITY);
                } else {
                    assertTrue(refused, refused.contains("the launcher's entry is not clear"));
                    refusals++;
                }
            }
            carry(STEP, FlightTest::resting, wheel(CLOSE_LAUNCH));
            for (int i = 0; i < fed.size(); i++) {
                if (!flight.holds(fed.get(i))) {
                    continue;
                }
                highest.set(i, Math.max(highest.get(i), at(fed.get(i)).z()));
                for (int j = 0; j < i; j++) {
                    if (flight.holds(fed.get(j))) {
                        nearest = Math.min(
                                nearest, at(fed.get(i)).minus(at(fed.get(j))).length());
                    }
                }
            }
        }

        assertEquals("all four were fed", 4, fed.size());
        assertTrue("each only once the entry was clear: refused " + refusals + " times", refusals > 0);
        assertTrue("no two ever overlapped: " + nearest + " apart at the nearest", nearest >= 2 * POLLEN_RADIUS);
        for (int i = 0; i < fed.size(); i++) {
            assertTrue("ball " + i + " was thrown, up to " + highest.get(i), highest.get(i) > THROWN_ABOVE_IN);
        }
    }

    @Test
    public void
            aBallInTheLauncherStaysWhereItWasThroughAHivesTipAndAChangeInWhatStandsInTheBoresAndIsThrownOnceSpunUp() {
        carryALauncher(FlightTest::resting);
        Object ball = feed();
        carry(1.0, FlightTest::resting, wheel(0));
        Vec3 held = at(ball);
        for (int i = 0; i < 5; i++) {
            putIn(Field.Kind.NECTAR, NECTAR, upturned(blue));
        }

        carry(STEP, FlightTest::resting, wheel(0));
        assertTrue("the hive has begun to tip", hives().tipping(blue));
        carry(Lean.TIP_SECONDS, FlightTest::resting, wheel(0));
        assertFalse("and is over", hives().tipping(blue));
        Vec3 throughTheTip = at(ball);
        Field.Flower flower = field.flowers().get(0);
        flight = Valid.value(flight.amid(List.of(threeStandingOnTheNestedPollen(flower)), nestedIn(flower)));
        Vec3 amidTheStack = at(ball);
        carry(0.1, FlightTest::resting, wheel(0));

        assertEquals("through the tip", held, throughTheTip);
        assertEquals("amid what stands in the bores", held, amidTheStack);
        assertEquals("and on", held, at(ball));
        assertTrue("thrown once the wheel spins up", highestWithin(0.5, ball, FlightTest::resting) > THROWN_ABOVE_IN);
    }

    @Test
    public void
            aRestingRobotsLauncherLetsAFedBallFallAsleepAgainstItsStopWithItsWheelStoppedAndTheFlightThenIdlesAtNoWork() {
        carryALauncher(FlightTest::resting);
        Object ball = feed();

        carry(1.5, FlightTest::resting, wheel(0));
        Vec3 held = at(ball);
        long rested = flight.work();
        carry(1.0, FlightTest::resting, wheel(0));

        assertTrue(
                ROBOT_TOP_SOURCE + ": inside the robot and off the floor at " + held,
                held.z() - POLLEN_RADIUS > 0 && held.z() + POLLEN_RADIUS < ROBOT_TOP_IN);
        assertEquals("where it stays", held, at(ball));
        assertEquals("and costs nothing asleep", rested, flight.work());
    }

    @Test
    public void aLauncherMovingAndSpinningWhereNothingIsNearItWakesNothingInTheHivesAndCostsTheFlightNoWork() {
        DoubleFunction<Launcher.Frame> driving =
                t -> frame(ROBOT_X + 20 * Math.sin(t), ROBOT_Y + 10 * (1 - Math.cos(t)), ROBOT_YAW + 0.8 * t);
        carryALauncher(driving);
        List<Object> resting = new ArrayList<>();
        for (Field.Piece piece : field.cellPieces()) {
            Object ball = new Object();
            resting.add(ball);
            flight = Valid.value(
                    flight.restingAt(ball, piece.kind(), Valid.value(Length.of(piece.radius())), piece.at()));
        }
        List<Vec3> setDownAt = new ArrayList<>();
        for (Object ball : resting) {
            setDownAt.add(at(ball));
        }
        long setDown = flight.work();

        carry(2.0, driving, wheel(CLOSE_LAUNCH));

        assertEquals("nothing moved, so nothing was worked out", setDown, flight.work());
        for (int i = 0; i < resting.size(); i++) {
            assertEquals("ball " + i + " lies where it was set down", setDownAt.get(i), at(resting.get(i)));
        }
        assertEquals(3, flight.scored("Blue"));
        Vec3 fedAt = at(feed());
        double fromTheRobot = Math.hypot(
                fedAt.x() - (ROBOT_X + 20 * Math.sin(flown)), fedAt.y() - (ROBOT_Y + 10 * (1 - Math.cos(flown))));
        assertTrue("the launcher went with the robot: fed " + fromTheRobot + " in from it", fromTheRobot < 2);
    }

    @Test
    public void whileItsRobotRestsAFlightCarryingALauncherFliesTheSameToTheBitItsWorkIncludedHoweverItsTimeIsCut() {
        carryALauncher(FlightTest::resting);
        Object fed = feed();
        Object dropped = drop(Field.Kind.POLLEN, POLLEN, aboveTheMouthOf(upturned(blue), 10));
        Flight<Object> fine = flight, coarse = flight;
        Launcher.Frame at = resting(0);

        for (int i = 0; i < 25; i++) {
            for (int j = 0; j < 4; j++) {
                fine = Valid.value(fine.step(tick(0.005), resting(0), wheel(CLOSE_LAUNCH)))
                        .flight();
            }
            coarse = Valid.value(coarse.step(tick(0.02), at, wheel(CLOSE_LAUNCH)))
                    .flight();
        }

        for (Object ball : List.of(fed, dropped)) {
            assertEquals(fine.at(ball), coarse.at(ball));
            assertEquals(fine.velocity(ball), coarse.velocity(ball));
        }
        assertEquals(fine.work(), coarse.work());
        assertTrue(
                "the fed ball was thrown: " + fine.at(fed),
                fine.at(fed).orElseThrow().z() > ROBOT_TOP_IN);
    }

    private Vec3 thrownBy(DoubleFunction<Launcher.Frame> frameAt) {
        carryALauncher(frameAt);
        carry(2 * STEP, frameAt, wheel(CLOSE_LAUNCH));
        Object ball = feed();
        carry(AFTER_THE_FEED_SECONDS, frameAt, wheel(CLOSE_LAUNCH));
        assertTrue("thrown above the robot: " + at(ball), at(ball).z() - POLLEN_RADIUS > ROBOT_TOP_IN);
        return flight.velocity(ball).orElseThrow();
    }

    @Test
    public void theRobotsVelocityCarriesIntoTheThrow() {
        Vec3 robot = new Vec3(0.6, -0.8, 0).times(180);
        Vec3 still = thrownBy(FlightTest::resting);

        Vec3 moving = thrownBy(t -> frame(ROBOT_X + robot.x() * t, ROBOT_Y + robot.y() * t, ROBOT_YAW));

        double off = moving.minus(still.plus(robot)).length();
        assertTrue(
                AIR_SOURCE + ": the still throw " + still + " with the robot's " + robot + " added is " + off
                        + " in/s from " + moving,
                off <= AIR_TAKES_OF_THE_ROBOTS_SPEED * robot.length());
    }

    @Test
    public void turningTheFramesYawByAnAngleTurnsTheThrowsDirectionByAsMuchAboutTheUpright() {
        Vec3 facing = thrownBy(FlightTest::resting);
        for (double yaw : new double[] {0.7, -2.5, Math.PI}) {
            Vec3 turned = thrownBy(t -> frame(ROBOT_X, ROBOT_Y, ROBOT_YAW + yaw));

            assertVec(ROUNDING_SOURCE + ", turned by " + yaw, turned(facing, yaw), turned, ROUNDING_WITHIN);
        }
    }

    @Test
    public void aTeleportCarriesTheLauncherAndTheBallsInItAtOnceHoweverFarThoughAStepThatFarIsRefusedForItsSweep() {
        double toX = 30, toY = -45, toYaw = Math.PI / 2;
        Launcher.Frame to = frame(toX, toY, toYaw);
        carryALauncher(FlightTest::resting);
        Object lying = new Object();
        Field.Piece piece = field.cellPieces().get(0);
        flight = Valid.value(flight.restingAt(lying, piece.kind(), Valid.value(Length.of(piece.radius())), piece.at()));
        Object ball = feed();
        carry(1.0, FlightTest::resting, wheel(0));
        Vec3 held = at(ball), elsewhere = at(lying);

        String stepped = rejection(flight.step(tick(STEP), to, wheel(0)));
        flight = Valid.value(flight.carriedTo(to));

        assertTrue(stepped, stepped.contains("a launcher's frame sweeps its faces at most 1000.0 in/s"));
        Vec3 expected =
                new Vec3(toX, toY, 0).plus(turned(held.minus(new Vec3(ROBOT_X, ROBOT_Y, 0)), toYaw - ROBOT_YAW));
        assertVec("carried with the launcher", expected, at(ball), 1e-9);
        assertEquals("what is not in the launcher stays where it was", elsewhere, at(lying));
        DoubleFunction<Launcher.Frame> there = t -> frame(toX, toY, toYaw);
        carry(STEP, there, wheel(0));
        assertVec("and lies there in it", expected, at(ball), 1e-9);
        assertTrue("thrown from there once the wheel spins up", highestWithin(0.5, ball, there) > THROWN_ABOVE_IN);
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
