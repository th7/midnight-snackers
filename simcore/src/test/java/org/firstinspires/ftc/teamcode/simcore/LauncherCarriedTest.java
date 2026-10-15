package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class LauncherCarriedTest {
    private static final double TICKS_PER_TURN = 24.2;
    private static final double LAUNCH = 1050;
    private static final double WITHIN = 1e-9;
    private static final Length POLLEN = Valid.length(Launcher.POLLEN_RADIUS_IN);

    private static Launcher.Faces faces() {
        return Valid.value(Launcher.Faces.built());
    }

    private static Flywheel.AngularSpeed wheel(double ticksPerSecond) {
        return Flywheel.AngularSpeed.from(ticksPerSecond, Valid.value(Flywheel.TicksPerTurn.of(TICKS_PER_TURN)));
    }

    private static Launcher.Frame frame(double x, double y, double yaw) {
        return Valid.value(Launcher.Frame.of(x, y, yaw));
    }

    private static Launcher standing(Launcher.Frame at, double seconds) {
        return Valid.value(Launcher.standing(faces(), at, Valid.time(seconds)));
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    private static void assertAt(String what, Launcher.Frame frame, Launcher launcher) {
        Launcher.Placed placed = launcher.where();
        assertEquals(what, new Vec3(frame.x(), frame.y(), 0), placed.origin());
        assertEquals(what, Double.doubleToRawLongBits(frame.yaw()), Double.doubleToRawLongBits(placed.yaw()));
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual) {
        assertEquals(what + ": x of " + actual, expected.x(), actual.x(), WITHIN);
        assertEquals(what + ": y of " + actual, expected.y(), actual.y(), WITHIN);
        assertEquals(what + ": z of " + actual, expected.z(), actual.z(), WITHIN);
    }

    private static Vec3 turned(Vec3 v, double radians) {
        double cos = StrictMath.cos(radians), sin = StrictMath.sin(radians);
        return new Vec3(cos * v.x() - sin * v.y(), sin * v.x() + cos * v.y(), v.z());
    }

    @Test
    public void aLauncherStandingAtAFrameIsThereWhereverItIsPlacedWithNothingMovingAndItsWheelStopped() {
        Launcher.Frame at = frame(-30, -45, 2.5);
        Launcher standing = standing(at, 0);

        for (double t : new double[] {0, 0.0001, 0.37, 12}) {
            Launcher then = standing.at(Valid.time(t));
            assertAt("at " + t, at, then);
            assertTrue("at " + t, !then.moving());
            assertEquals(0, then.sweep(), 0);
        }
        assertEquals(wheel(0), Flywheel.AngularSpeed.stopped());
        Vec3 entry = standing.entry(POLLEN);
        assertTrue(!standing.squeezing(entry, POLLEN).isEmpty());
        for (Surroundings.Overlap overlap : standing.squeezing(entry, POLLEN)) {
            assertEquals(Vec3.zero(), overlap.support().velocityAt(entry));
            assertEquals(Vec3.zero(), overlap.support().spin());
        }
    }

    @Test
    public void eachLauncherOnwardBeginsExactlyAtTheFrameAndTimeTheOneBeforeItEndedAtAndEndsAtTheFrameItIsGiven() {
        Launcher.Frame first = frame(-30, -45, 3.1), second = frame(-29.4, -45.3, -3.12);
        Launcher.Frame third = frame(-28.9, -45.5, -3.05);
        Launcher standing = standing(first, 0.1);

        Launcher onward = Valid.value(standing.onward(second, Valid.time(0.105), wheel(LAUNCH)));
        Launcher further = Valid.value(onward.onward(third, Valid.time(0.1083), wheel(-LAUNCH)));

        assertAt("as the first course begins", first, onward.at(Valid.time(0.1)));
        assertAt("as it ends", second, onward.at(Valid.time(0.105)));
        assertAt("as the next begins", second, further.at(Valid.time(0.105)));
        assertAt("as it ends", third, further.at(Valid.time(0.1083)));
        assertTrue(onward.moving() && further.moving());
        assertTrue(
                "turning the short way across half a turn: " + onward.sweep(),
                onward.sweep() < 0.6 / 0.005 + 0.1 / 0.005 * Launcher.FARTHEST_IN);
        Launcher stillAndStopped = Valid.value(standing.onward(first, Valid.time(0.105), wheel(0)));
        Launcher stillAndSpinning = Valid.value(standing.onward(first, Valid.time(0.105), wheel(LAUNCH)));
        assertTrue(!stillAndStopped.moving());
        assertTrue(stillAndSpinning.moving());
        assertEquals(0, stillAndSpinning.sweep(), 0);
        String tooFast = rejection(standing.onward(frame(50, -45, 3.1), Valid.time(0.1001), wheel(0)));
        assertTrue(tooFast, tooFast.contains("a launcher's frame sweeps its faces at most 1000.0 in/s"));
        String backwards = rejection(standing.onward(second, Valid.time(0.09), wheel(0)));
        assertTrue(backwards, backwards.contains("a launcher's course ends no earlier than it begins"));
    }

    @Test
    public void aLauncherCarriedToAFrameIsThereAtOnceAtRestHoweverFarWithItsWheelAsItWasAndTheNextCourseBeginsThere() {
        Launcher.Frame from = frame(-30, -45, 0), far = frame(60, 50, -2);
        Launcher spinning =
                Valid.value(standing(from, 0).onward(frame(-29.5, -45, 0.01), Valid.time(0.005), wheel(LAUNCH)));

        Launcher carried = Valid.value(spinning.carriedTo(far));
        Launcher stopped = Valid.value(standing(from, 0).carriedTo(far));

        for (double t : new double[] {0, 0.005, 0.0051, 3}) {
            assertAt("at " + t, far, carried.at(Valid.time(t)));
            assertAt("at " + t, far, stopped.at(Valid.time(t)));
            assertEquals(0, carried.at(Valid.time(t)).sweep(), 0);
        }
        assertTrue("its wheel spins on", carried.moving());
        assertTrue(!stopped.moving());
        Launcher next = Valid.value(carried.onward(frame(60.5, 50, -2), Valid.time(0.01), wheel(LAUNCH)));
        assertAt("the next course begins there", far, next.at(Valid.time(0.005)));
        String sooner = rejection(carried.onward(far, Valid.time(0.004), wheel(0)));
        assertTrue(
                "and when it was carried there: " + sooner,
                sooner.contains("a launcher's course ends no earlier than it begins"));
    }

    @Test
    public void aLauncherStandsAsAnotherOnlyWhereNeitherMovesBothStandAtOneFrameAndTheirWheelsTurnAlike() {
        Launcher standing = standing(frame(-30, -45, 1), 0);

        Launcher again = Valid.value(standing.onward(frame(-30, -45, 1), Valid.time(0.005), wheel(0)));
        Launcher spinning = Valid.value(standing.onward(frame(-30, -45, 1), Valid.time(0.005), wheel(LAUNCH)));
        Launcher spinningOn = Valid.value(spinning.onward(frame(-30, -45, 1), Valid.time(0.01), wheel(LAUNCH)));
        Launcher moved = Valid.value(standing.onward(frame(-29.9, -45, 1), Valid.time(0.005), wheel(0)));
        Launcher stoppedThere = Valid.value(moved.onward(frame(-29.9, -45, 1), Valid.time(0.01), wheel(0)));
        Launcher carried = Valid.value(standing.carriedTo(frame(-29.9, -45, 1)));

        assertTrue(again.standsAs(standing) && standing.standsAs(again));
        assertTrue(spinningOn.standsAs(spinning));
        assertTrue(!spinning.standsAs(standing) && !standing.standsAs(spinning));
        assertTrue(!moved.standsAs(standing) && !stoppedThere.standsAs(moved));
        assertTrue(!carried.standsAs(standing));
        assertTrue(Valid.value(stoppedThere.onward(frame(-29.9, -45, 1), Valid.time(0.02), wheel(0)))
                .standsAs(stoppedThere));
        assertEquals(frame(-30, -45, 1), frame(-30, -45, 1));
        assertEquals(frame(-30, -45, 1).hashCode(), frame(-30, -45, 1).hashCode());
        assertNotEquals(frame(-30, -45, 1), frame(-30, -45, StrictMath.nextUp(1.0)));
        assertNotEquals(frame(-30, -45, 1), frame(-30, StrictMath.nextDown(-45.0), 1));
    }

    @Test
    public void aLauncherHoldsABallItSqueezesOrTouchesAndNoneClearOfIt() {
        Launcher standing = standing(frame(-30, -45, 0.4), 0);
        Vec3 entry = standing.entry(POLLEN);
        Vec3 hub = new Vec3(-30, -45, 0).plus(turned(new Vec3(-13 / 25.4, 0, 180 / 25.4), 0.4));
        double out = Launcher.GROOVE_BOTTOM_MM / 25.4 + Launcher.POLLEN_RADIUS_IN;

        Vec3 inTheHood = hub.plus(turned(new Vec3(-out, 0, 0), 0.4));
        Vec3 pastTheHood = hub.plus(turned(
                new Vec3(-StrictMath.cos(40 * StrictMath.PI / 180), 0, StrictMath.sin(40 * StrictMath.PI / 180))
                        .times(out + 0.5),
                0.4));

        assertTrue("at the entry", standing.holds(entry, POLLEN));
        assertTrue("in the hood", standing.holds(inTheHood, POLLEN));
        assertTrue(!standing.squeezing(inTheHood, POLLEN).isEmpty());
        assertTrue("past the hood's upper end", !standing.holds(pastTheHood, POLLEN));
        assertTrue("above the robot", !standing.holds(entry.plus(new Vec3(0, 0, 10)), POLLEN));
    }

    @Test
    public void
            aBallCarriedFromOneLauncherToAnotherLiesInItAsItLayInTheFirstAndMovesThroughItAsItMovedThroughTheFirst() {
        Launcher.Frame began = frame(-30, -45, 0.3), ends = frame(-29.5, -45.2, 0.35);
        Launcher.Course course = Valid.value(Launcher.Course.of(began, Valid.time(0), ends, Valid.time(0.005)));
        Launcher moving = Valid.value(standing(began, 0).onward(ends, Valid.time(0.005), wheel(LAUNCH)))
                .at(Valid.time(0.004));
        Launcher there = Valid.value(moving.carriedTo(frame(60, 50, -2)));
        Vec3 turning = new Vec3(0, 0, course.turning());

        Bodies.Motion fed = Valid.value(moving.carried(
                new Bodies.Seen(
                        "fed",
                        moving.entry(POLLEN),
                        moving.feed(POLLEN),
                        turning,
                        Turn.identity(),
                        Bodies.Life.AWAKE,
                        0),
                there));

        assertVec("the entry is carried to the entry", there.entry(POLLEN), fed.at());
        assertVec("the feed is carried to the feed", there.feed(POLLEN), fed.velocity());
        assertVec("turning with the frame is carried to standing still", Vec3.zero(), fed.spin());

        Launcher.Placed from = moving.where(), to = there.where();
        Vec3 at = moving.entry(POLLEN).plus(new Vec3(0.3, -0.2, 0.1));
        Vec3 velocity = new Vec3(40, -70, 120), spin = new Vec3(4, -50, 6);
        Bodies.Motion carried = Valid.value(moving.carried(
                new Bodies.Seen("ball", at, velocity, spin, Turn.identity(), Bodies.Life.AWAKE, 0), there));
        Vec3 local = turned(at.minus(from.origin()), -from.yaw());
        Vec3 relative = velocity.minus(course.velocity().plus(turning.cross(at.minus(from.origin()))));
        assertVec("where", to.origin().plus(turned(local, to.yaw())), carried.at());
        assertVec("how fast", turned(turned(relative, -from.yaw()), to.yaw()), carried.velocity());
        assertVec("how it spins", turned(turned(spin.minus(turning), -from.yaw()), to.yaw()), carried.spin());
    }
}
