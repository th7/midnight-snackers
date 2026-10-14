package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Flight;
import org.firstinspires.ftc.teamcode.simcore.Hives;
import org.firstinspires.ftc.teamcode.simcore.Lean;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Ring;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Shot;
import org.firstinspires.ftc.teamcode.simcore.Surface;
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
    private static final double BOUNCES_FOR_EACH_IN_PER_S_OF_ITS_FIRST_IMPACT = 1 / 129.0;
    private static final double ROLLS_OUT_AND_LANDS_SECONDS = 0.5;

    private final Field field = SimPlacement.FIELD;
    private Flight<Object> flight = Valid.value(Hives.of(field).then(Flight::<Object>over));
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

    private static double settledAfterFalling(double inches) {
        double g = Flight.GRAVITY_IN_PER_S2;
        return Math.sqrt(2 * inches / g)
                + Math.sqrt(2 * g * inches) * BOUNCES_FOR_EACH_IN_PER_S_OF_ITS_FIRST_IMPACT
                + ROLLS_OUT_AND_LANDS_SECONDS;
    }

    private List<Flight.Landing<Object>> fly(double seconds) {
        List<Flight.Landing<Object>> now = new ArrayList<>();
        for (long i = Math.round(seconds / STEP); i > 0; i--) {
            now.addAll(once(STEP));
        }
        return now;
    }

    private List<Flight.Landing<Object>> once(double seconds) {
        Flight.Stepped<Object> stepped = flight.step(tick(seconds));
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

        fly(settledAfterFalling(onTheWall.z()));

        assertEquals("it reached the floor", 1, landed.size());
        assertTrue(landed.get(0) instanceof Flight.OnTheFloor);
        double clear = landed.get(0).at().y() - wall;
        assertTrue("no nearer the wall than its radius: " + clear, clear >= POLLEN_RADIUS - CONTACT);
    }

    @Test
    public void aBallThrownHardAtTheSideOfAHiveBouncesBackOffItAndComesDownOnTheSideItWasThrownFrom() {
        Vec3 onTheWall = onBluesWall();
        double thrownFrom = onTheWall.y() + 6;
        throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(onTheWall.x(), thrownFrom, onTheWall.z()), new Vec3(0, -200, 0));

        fly(settledAfterFalling(onTheWall.z()));

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

        double until = settledAfterFalling(onTheWall.z());
        while (flown < until && flight.holds(ball)) {
            once(STEP);
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

        fly(Lean.TIP_SECONDS + settledAfterFalling(cell.mouthCentreAt(leaning).z()));

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

        fly(1.0);

        assertTrue("once it presses on what is in the cell the hive tips", tilt(blue) != leaning);
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

        fly(settledAfterFalling(height));

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

        fly(settledAfterFalling(6) + 1);

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
        while (landed.isEmpty() && flown < settledAfterFalling(4) + 1) {
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
        Vec3 pipe = new Vec3(66.36, 21.72, 12);
        double from = pipe.x() - 10;
        throwIt(Field.Kind.POLLEN, POLLEN, new Vec3(from, pipe.y(), pipe.z()), new Vec3(150, 0, 0));

        fly(settledAfterFalling(pipe.z()) + 1);

        assertEquals(1, landed.size());
        assertTrue(
                "it came back off the pipe: x=" + landed.get(0).at().x(),
                landed.get(0).at().x() < from);
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

        fly(settledAfterFalling(SimPlacement.WALL_HEIGHT_IN + 20));

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
                fine = fine.step(tick(0.005)).flight();
            }
            coarse = coarse.step(tick(0.02)).flight();
        }

        assertTrue(start != fine);
        assertEquals(fine.at(ball), coarse.at(ball));
        assertEquals(fine.velocity(ball), coarse.velocity(ball));
        assertEquals(fine.work(), coarse.work());
    }

    @Test
    public void aFlightWithNothingAwakeJumpsToWhereFlyingThereStepByStepWouldLeaveIt() {
        Flight<Object> waited = flight, stepped = flight;
        for (int i = 0; i < 2; i++) {
            waited = waited.step(tick(0.25)).flight();
        }
        for (int i = 0; i < 500; i++) {
            stepped = stepped.step(tick(0.001)).flight();
        }
        Object ball = new Object();
        Vec3 from = aboveTheMouthOf(upturned(blue), 10);
        waited = Valid.value(waited.with(ball, Field.Kind.POLLEN, POLLEN, from, Vec3.zero()));
        stepped = Valid.value(stepped.with(ball, Field.Kind.POLLEN, POLLEN, from, Vec3.zero()));
        for (int i = 0; i < 100; i++) {
            waited = waited.step(tick(0.01)).flight();
            stepped = stepped.step(tick(0.01)).flight();
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
        fly(4.0);

        assertEquals("a launch into the cell", LAUNCH_WORK, launched);
        assertEquals("then a five-nectar tip", TIP_WORK, flight.work() - launched);
    }

    @Test
    public void noTickOfAFiveNectarTipDoesMoreWorkThanItIsAllowedAndThePlainStepsOfItsBalls() {
        Field.Cell cell = upturned(blue);
        for (int i = 0; i < 5; i++) {
            putIn(Field.Kind.NECTAR, NECTAR, cell);
        }
        double plainStepOfABall = 16, steps = STEP / (Flight.STEP_NANOS / Flight.NANOS_PER_SECOND);
        long most = 0;

        for (int i = 0; i < 4 / STEP; i++) {
            long before = flight.work();
            once(STEP);
            most = Math.max(most, flight.work() - before);
        }

        assertTrue("it tipped", hives().tilt(blue) != blue.tilt().degrees());
        assertTrue(
                "the most a tick did was " + most,
                most <= Shot.MOST_WORK_PER_SECOND * STEP + steps * 5 * plainStepOfABall);
    }

    private static final long LAUNCH_WORK = 136055;
    private static final long TIP_WORK = 1251366;

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
