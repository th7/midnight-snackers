package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class LauncherCourseTest {
    private static final double WITHIN = 1e-12;
    private static final double BEGINS = 0.1;
    private static final double ENDS = 0.105;
    private static final double STEP_SECONDS = ENDS - BEGINS;
    private static final double STEP_ON_WHICH_TIMES_SUBTRACT_EXACTLY_SECONDS = 1.0 / 8192;
    private static final long STEPS_ACROSS = 41;

    private record Between(Launcher.Frame from, GridTime began, Launcher.Frame to, GridTime ends) {}

    private static Launcher.Frame frame(double x, double y, double yaw) {
        return Valid.value(Launcher.Frame.of(x, y, yaw));
    }

    private static Launcher.Course course(Launcher.Frame from, double began, Launcher.Frame to, double ends) {
        return Valid.value(Launcher.Course.of(from, Valid.time(began), to, Valid.time(ends)));
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    private static Launcher.Placed placed(Launcher.Course course, double seconds) {
        return course.at(Valid.time(seconds));
    }

    private static void assertPlaced(String what, double x, double y, double yaw, Launcher.Placed placed) {
        assertEquals(what + ": x of " + placed, x, placed.origin().x(), WITHIN);
        assertEquals(what + ": y of " + placed, y, placed.origin().y(), WITHIN);
        assertEquals(what + ": on the floor, " + placed, 0, placed.origin().z(), 0);
        assertEquals(what + ": facing", StrictMath.cos(yaw), StrictMath.cos(placed.yaw()), WITHIN);
        assertEquals(what + ": facing", StrictMath.sin(yaw), StrictMath.sin(placed.yaw()), WITHIN);
    }

    private static void assertExactly(String what, Launcher.Frame frame, Launcher.Placed placed) {
        assertEquals(what, new Vec3(frame.x(), frame.y(), 0), placed.origin());
        assertEquals(what, Double.doubleToRawLongBits(frame.yaw()), Double.doubleToRawLongBits(placed.yaw()));
    }

    private static void assertExactly(String what, Launcher.Placed expected, Launcher.Placed placed) {
        assertEquals(what, expected.origin(), placed.origin());
        assertEquals(what, Double.doubleToRawLongBits(expected.yaw()), Double.doubleToRawLongBits(placed.yaw()));
    }

    private static boolean isExactly(Launcher.Frame frame, double x, double y, double yaw) {
        return Double.doubleToRawLongBits(frame.x()) == Double.doubleToRawLongBits(x)
                && Double.doubleToRawLongBits(frame.y()) == Double.doubleToRawLongBits(y)
                && Double.doubleToRawLongBits(frame.yaw()) == Double.doubleToRawLongBits(yaw);
    }

    private static boolean carriedAcrossLands(
            Launcher.Frame frame, double seconds, Launcher.Course course, Launcher.Frame other) {
        return isExactly(
                other,
                frame.x() + seconds * course.velocity().x(),
                frame.y() + seconds * course.velocity().y(),
                frame.yaw() + seconds * course.turning());
    }

    private static List<Launcher.Frame[]> framesThatCarryingEitherAcrossMisses() {
        return List.of(
                new Launcher.Frame[] {frame(1.377, 0, 0), frame(4.997, 0, 0)},
                new Launcher.Frame[] {frame(34.92, -1.76, -2.95), frame(34.55, 0.11, -3.25)});
    }

    @Test
    public void aLaunchersFrameIsAPlaceOnTheFieldAndAFiniteYawKeptWithinHalfATurnEitherWay() {
        Launcher.Frame frame = frame(10, -20, 0.3);
        assertEquals(10, frame.x(), 0);
        assertEquals(-20, frame.y(), 0);
        assertEquals(0.3, frame.yaw(), 0);
        assertEquals(-StrictMath.PI / 2, frame(0, 0, 1.5 * StrictMath.PI).yaw(), WITHIN);
        assertEquals(StrictMath.PI - 0.1, frame(0, 0, -StrictMath.PI - 0.1).yaw(), WITHIN);
        assertEquals(0.25, frame(0, 0, 0.25 + 40 * StrictMath.PI).yaw(), 1e-9);
        double most = Launcher.Frame.MOST_FROM_CENTRE_IN;
        assertEquals(Launcher.Frame.PLACE_SOURCE, 10000, most, 0);
        assertEquals(most, frame(most, -most, 0).x(), 0);
        for (double[] beyond : new double[][] {
            {StrictMath.nextUp(most), 0},
            {0, -StrictMath.nextUp(most)},
            {Double.NaN, 0},
            {0, Double.POSITIVE_INFINITY}
        }) {
            assertEquals(
                    "a launcher's frame is within " + most + " in of the field's centre either way, not at ("
                            + beyond[0] + ", " + beyond[1] + ")",
                    rejection(Launcher.Frame.of(beyond[0], beyond[1], 0)));
        }
        for (double yaw : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertEquals("a launcher's frame faces a finite yaw, not " + yaw, rejection(Launcher.Frame.of(0, 0, yaw)));
        }
    }

    @Test
    public void aCoursePlacesItsFrameOnTheStraightLineThroughTheFramesItBeginsAndEndsAtAndOnPastThemNeverClamped() {
        Launcher.Frame from = frame(10, 20, 0.2), to = frame(12, 19, 0.3);
        Launcher.Course course = course(from, BEGINS, to, ENDS);

        assertExactly("as it begins", from, placed(course, BEGINS));
        assertExactly("as it ends", to, placed(course, ENDS));
        for (double part : new double[] {0.25, 0.5, 0.75, 2, 30, -1, -7}) {
            assertPlaced(
                    part + " of the way",
                    10 + 2 * part,
                    20 - part,
                    0.2 + 0.1 * part,
                    placed(course, BEGINS + part * STEP_SECONDS));
        }
        assertEquals(
                new Vec3(2 / STEP_SECONDS, -1 / STEP_SECONDS, 0).x(),
                course.velocity().x(),
                1e-9);
        assertEquals(-1 / STEP_SECONDS, course.velocity().y(), 1e-9);
        assertEquals(0, course.velocity().z(), 0);
        assertEquals(0.1 / STEP_SECONDS, course.turning(), 1e-9);
    }

    @Test
    public void aFrameWhoseYawCrossesHalfATurnTurnsTheShortWayAsAFrameThatDoesNotCrossItTurns() {
        double half = StrictMath.PI;
        Launcher.Course crossing = course(frame(0, 0, half - 0.01), BEGINS, frame(0, 0, -half + 0.01), ENDS);
        Launcher.Course quarter = course(frame(0, 0, half / 2 - 0.01), BEGINS, frame(0, 0, half / 2 + 0.01), ENDS);

        assertEquals(0.02 / STEP_SECONDS, crossing.turning(), 1e-9);
        assertEquals(quarter.turning(), crossing.turning(), 1e-9);
        for (double part : new double[] {0.25, 0.5, 0.75, 3, -2}) {
            double seconds = BEGINS + part * STEP_SECONDS;
            assertPlaced(
                    part + " of the way across",
                    0,
                    0,
                    quarter.at(Valid.time(seconds)).yaw() + half / 2,
                    placed(crossing, seconds));
        }
    }

    @Test
    public void aCourseOfNoTimeLeavesTheFrameWhereItEndsAndAtRestWhereverItIsPlaced() {
        Launcher.Frame from = frame(10, 20, 0.2), to = frame(-3, 4, -1);
        Launcher.Course course = course(from, ENDS, to, ENDS);

        assertEquals(Vec3.zero(), course.velocity());
        assertEquals(0, course.turning(), 0);
        for (double seconds : new double[] {0, BEGINS, ENDS, 0.2, 7}) {
            assertExactly("at " + seconds, to, placed(course, seconds));
        }
    }

    @Test
    public void aCourseFromAFrameToTheSameFrameIsExactlyStillWhereverItIsPlaced() {
        Launcher.Frame still = frame(-31.7, 12.25, 2.9);
        Launcher.Course course = course(still, BEGINS, frame(-31.7, 12.25, 2.9), ENDS);

        assertEquals(Vec3.zero(), course.velocity());
        assertEquals(0, Double.doubleToRawLongBits(course.turning()));
        for (double seconds : new double[] {0, 0.0371, BEGINS, 0.1023, 0.1025, ENDS, 0.31, 250}) {
            assertExactly("at " + seconds, still, placed(course, seconds));
        }
        Bodies.Step step = Valid.value(Flight.stepping());
        Launcher.Course instant = Valid.value(Launcher.Course.of(
                still,
                GridTime.at(0, step),
                frame(-31.7, 12.25, 2.9),
                GridTime.partWay(0, Valid.value(GridTime.PartOfAStep.of(1e-306)), step)));
        assertEquals(Vec3.zero(), instant.velocity());
        assertEquals(0, Double.doubleToRawLongBits(instant.turning()));
        for (Launcher.Course stillCourse : List.of(course, instant)) {
            for (GridTime time : List.of(
                    GridTime.at(1, step),
                    Valid.time(0.31),
                    Valid.time(-250),
                    GridTime.at(Long.MAX_VALUE, step),
                    GridTime.at(Long.MIN_VALUE, step))) {
                assertExactly("over " + stillCourse + ", at " + time, still, stillCourse.at(time));
            }
        }
    }

    @Test
    public void aCourseCarriesItsFrameAtItsBoundedVelocityAndTurnSoItIsFiniteWhereverItIsPlacedHoweverShort() {
        Bodies.Step step = Valid.value(Flight.stepping());
        GridTime began = GridTime.at(0, step);
        GridTime soon = GridTime.partWay(0, Valid.value(GridTime.PartOfAStep.of(1e-310)), step);
        Launcher.Course sliding = Valid.value(Launcher.Course.of(frame(0, 0, 0), began, frame(1e-312, 0, 0), soon));
        Launcher.Course turning = Valid.value(Launcher.Course.of(frame(0, 0, 0), began, frame(0, 0, 1e-314), soon));

        assertEquals(100, sliding.velocity().x(), 1e-6);
        assertEquals(1, turning.turning(), 1e-6);
        for (double seconds : new double[] {1, -1, 0.3, 250}) {
            assertPlaced("sliding, at " + seconds, sliding.velocity().x() * seconds, 0, 0, placed(sliding, seconds));
            assertPlaced("turning, at " + seconds, 0, 0, turning.turning() * seconds, placed(turning, seconds));
        }
        Launcher.Course fastest = course(
                frame(-10000, 10000, StrictMath.PI),
                BEGINS,
                frame(-10000 + 0.99 * Flight.MOST_SPEED_IN_PER_S * STEP_SECONDS, 10000, StrictMath.PI),
                ENDS);
        for (Launcher.Course course : List.of(sliding, turning, fastest)) {
            for (long steps : new long[] {Long.MAX_VALUE, Long.MIN_VALUE}) {
                Launcher.Placed placed = course.at(GridTime.at(steps, step));
                assertTrue(
                        "over " + course + ", at step " + steps + ": " + placed,
                        placed.origin().isFinite() && Double.isFinite(placed.yaw()));
            }
        }
    }

    @Test
    public void aMovingCourseIsExactlyEachOfItsFramesAtItsTimeWhereCarryingEitherAcrossWouldMiss() {
        double across = Valid.time(ENDS).seconds() - Valid.time(BEGINS).seconds();
        for (Launcher.Frame[] frames : framesThatCarryingEitherAcrossMisses()) {
            Launcher.Frame from = frames[0], to = frames[1];
            Launcher.Course course = course(from, BEGINS, to, ENDS);

            assertTrue(
                    "carried across " + course + " from either end at its velocity and turn, it misses the other",
                    !carriedAcrossLands(from, across, course, to) && !carriedAcrossLands(to, -across, course, from));
            assertTrue(course.moving());
            assertExactly("as " + course + " begins", from, placed(course, BEGINS));
            assertExactly("as " + course + " ends", to, placed(course, ENDS));
        }
    }

    @Test
    public void aCourseRunTheOtherWayIsPlacedExactlyAsItIsRunForwardEitherSideOfItsMiddle() {
        Bodies.Step step = Valid.value(
                Seconds.of(STEP_ON_WHICH_TIMES_SUBTRACT_EXACTLY_SECONDS).then(Bodies.Step::of));
        GridTime began = GridTime.at(0, step);
        GridTime ends = GridTime.at(STEPS_ACROSS, step);
        for (Launcher.Frame[] frames : framesThatCarryingEitherAcrossMisses()) {
            Launcher.Course forward = Valid.value(Launcher.Course.of(frames[0], began, frames[1], ends));
            Launcher.Course backward = Valid.value(Launcher.Course.of(frames[1], began, frames[0], ends));
            for (long steps = -STEPS_ACROSS; 2 * steps < STEPS_ACROSS; steps++) {
                GridTime early = GridTime.at(steps, step);
                GridTime late = GridTime.at(STEPS_ACROSS - steps, step);
                assertExactly(
                        forward + " at " + early + " against " + backward + " at " + late,
                        forward.at(early),
                        backward.at(late));
                assertExactly(
                        backward + " at " + early + " against " + forward + " at " + late,
                        backward.at(early),
                        forward.at(late));
            }
        }
    }

    @Test
    public void aCourseThatDoesNotMoveThoughItsFramesDifferLeavesTheFrameExactlyWhereItEndsWhereverItIsPlaced() {
        Bodies.Step step = Valid.value(Flight.stepping());
        GridTime began = GridTime.at(0, step);
        GridTime twoSecondsOn = GridTime.at(20000, step);
        Launcher.Frame origin = frame(0, 0, 0);
        for (Between between : List.of(
                new Between(origin, began, frame(Double.MIN_VALUE, 0, 0), twoSecondsOn),
                new Between(origin, began, frame(0, Double.MIN_VALUE, 0), twoSecondsOn),
                new Between(origin, began, frame(0, 0, Double.MIN_VALUE), twoSecondsOn),
                new Between(origin, began, frame(1e-310, 0, 0), GridTime.at(Long.MAX_VALUE, step)),
                new Between(
                        frame(0, 0, StrictMath.PI),
                        Valid.time(BEGINS),
                        frame(0, 0, -StrictMath.PI),
                        Valid.time(ENDS)))) {
            Launcher.Frame from = between.from(), to = between.to();
            Launcher.Course course = Valid.value(Launcher.Course.of(from, between.began(), to, between.ends()));

            assertTrue("the frames of " + course + " differ", !isExactly(from, to.x(), to.y(), to.yaw()));
            assertTrue(course + " does not move", !course.moving());
            assertEquals(Vec3.zero(), course.velocity());
            assertEquals(0, course.turning(), 0);
            for (GridTime time : List.of(
                    began,
                    twoSecondsOn,
                    Valid.time(BEGINS),
                    Valid.time(ENDS),
                    Valid.time(-250),
                    Valid.time(250),
                    GridTime.at(Long.MAX_VALUE, step),
                    GridTime.at(Long.MIN_VALUE, step))) {
                assertExactly("over " + course + ", at " + time, to, course.at(time));
            }
        }
    }

    @Test
    public void aCourseThatEndsBeforeItBeginsOrWhoseFacesSweepFasterThanAFlightTakesABallIsRefusedNamingTheRule() {
        Launcher.Frame at = frame(0, 0, 0);
        assertTrue(rejection(Launcher.Course.of(at, Valid.time(ENDS), at, Valid.time(BEGINS)))
                .startsWith("a launcher's course ends no earlier than it begins, not "));
        double most = Flight.MOST_SPEED_IN_PER_S;
        assertEquals(Launcher.Course.SWEEP_SOURCE, 1000, most, 0);
        double farthest = Launcher.FARTHEST_IN;
        double fastestTurn = most / farthest;
        String rule = "a launcher's frame sweeps its faces at most " + most + " in/s, as fast as a flight takes a"
                + " ball, not ";
        assertTrue(rejection(Launcher.Course.of(
                        at, Valid.time(BEGINS), frame(1.01 * most * STEP_SECONDS, 0, 0), Valid.time(ENDS)))
                .startsWith(rule));
        assertTrue(rejection(Launcher.Course.of(
                        at, Valid.time(BEGINS), frame(0, 0, 1.01 * fastestTurn * STEP_SECONDS), Valid.time(ENDS)))
                .startsWith(rule));
        assertTrue(rejection(Launcher.Course.of(
                        at,
                        Valid.time(BEGINS),
                        frame(0.5 * most * STEP_SECONDS, 0, 0.51 * fastestTurn * STEP_SECONDS),
                        Valid.time(ENDS)))
                .startsWith(rule));
        Bodies.Step step = Valid.value(Flight.stepping());
        GridTime began = GridTime.at(1000, step);
        GridTime soon = GridTime.partWay(1000, Valid.value(GridTime.PartOfAStep.of(1e-12)), step);
        assertTrue(rejection(Launcher.Course.of(at, began, frame(1e-9, 0, 0), soon))
                .startsWith(rule));
        assertTrue(rejection(Launcher.Course.of(at, began, frame(0, 0, 1e-9), soon))
                .startsWith(rule));
        assertEquals(
                Vec3.zero(),
                Valid.value(Launcher.Course.of(at, began, frame(0, 0, 0), soon)).velocity());
        course(at, BEGINS, frame(0.99 * most * STEP_SECONDS, 0, 0), ENDS);
        course(at, BEGINS, frame(0, 0, 0.99 * fastestTurn * STEP_SECONDS), ENDS);
        course(frame(0, 0, StrictMath.PI - 0.01), BEGINS, frame(0, 0, -StrictMath.PI + 0.01), ENDS);
    }
}
