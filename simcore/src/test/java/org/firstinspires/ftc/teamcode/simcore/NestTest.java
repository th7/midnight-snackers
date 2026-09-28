package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class NestTest {
    private static final Field.Flower FLOWER =
            Valid.value(Field.Flower.of("Flower", new Vec2(10, 0), 2, 2.4, 4.25, 0.35));
    private static final Length POLLEN = Valid.length(2.5);
    private static final Seconds STEP = Valid.value(Seconds.of(0.005));
    private static final double RESTING = 0;
    private static final double ROLLING = 1;
    private static final double DELTA = 1e-9;

    private static Rolling<String> rolling(String name, double x, double y) {
        return new Rolling<>(name, new Vec2(x, y), POLLEN);
    }

    private static double weight(int balls) {
        return balls * Nest.BALL_MASS_KG * Flight.GRAVITY_IN_PER_S2 * Length.METRES_PER_INCH;
    }

    @Test
    public void theNestHoldsTheBallInTheBoreNearestItsMiddle() {
        assertEquals(
                Optional.of("nearer"),
                Nest.nested(
                        FLOWER,
                        List.of(rolling("farther", 11, 0), rolling("nearer", 10, 0.5), rolling("outside", 10, 5))));
        assertEquals(
                Optional.of("first"), Nest.nested(FLOWER, List.of(rolling("first", 11, 0), rolling("second", 9, 0))));
        assertEquals(Optional.empty(), Nest.nested(FLOWER, List.of(rolling("outside", 10, 5))));
        assertEquals(Optional.empty(), Nest.nested(FLOWER, List.<Rolling<String>>of()));
    }

    @Test
    public void theLoadOnTheNestIsTheBallInItAndEveryBallStandingOnIt() {
        assertEquals(weight(1), Nest.load(0), DELTA);
        assertEquals(weight(4), Nest.load(3), DELTA);
    }

    @Test
    public void toLeaveTheNestABallRollsUpOverItsRingCarryingTheLoad() {
        double r = POLLEN.inches(), n = FLOWER.nest();

        assertEquals(weight(4) * Math.sqrt(2 * r * n - n * n) / (r - n), Nest.overTheRing(FLOWER, POLLEN, 3), DELTA);
        assertTrue(
                "a shallower ring is less to climb",
                Nest.overTheRing(Valid.value(Field.Flower.of("Shallow", new Vec2(10, 0), 2, 2.4, 4.25, 0.1)), POLLEN, 3)
                        < Nest.overTheRing(FLOWER, POLLEN, 3));
    }

    @Test
    public void theNestPushesItsBallTowardTheAxisHardestAtTheRimAndNotAtAllInTheMiddle() {
        double full = Nest.overTheRing(FLOWER, POLLEN, 3);

        Vec2 atTheRim = Nest.hold(FLOWER, new Vec2(12, 0), POLLEN, 3).orElse(new Vec2(0, 0));
        assertEquals(-full, atTheRim.x(), DELTA);
        assertEquals(0, atTheRim.y(), DELTA);

        Vec2 halfway = Nest.hold(FLOWER, new Vec2(10, -1), POLLEN, 3).orElse(new Vec2(0, 0));
        assertEquals(0, halfway.x(), DELTA);
        assertEquals(full / 2, halfway.y(), DELTA);

        Vec2 beyond = Nest.hold(FLOWER, new Vec2(13, 0), POLLEN, 3).orElse(new Vec2(0, 0));
        assertEquals(-full, beyond.x(), DELTA);

        assertEquals(
                "in the middle to within the contact tolerance, so a ball rolled back rests",
                Optional.empty(),
                Nest.hold(FLOWER, new Vec2(10.01, 0.01), POLLEN, 3));
    }

    @Test
    public void theWeightOnItDragsItToAStopRatherThanLettingItRollAbout() {
        double massKg = 0.5;

        assertEquals(
                "rolling, the load's friction",
                Nest.FRICTION * weight(4),
                Nest.drag(3, massKg, 1, STEP).orElse(0.0),
                DELTA);
        assertEquals(
                "creeping, only what stops it in the step",
                massKg * 0.001 / STEP.value(),
                Nest.drag(3, massKg, 0.001, STEP).orElse(0.0),
                DELTA);
        assertEquals("still, nothing", Optional.empty(), Nest.drag(3, massKg, 0, STEP));
    }

    @Test
    public void aBallInTheNestIsHeldThere() {
        Seat.Next<String> next = Seat.<String>seated().next(Optional.of("ball"), false, RESTING);

        assertEquals(Seat.<String>seated(), next.seat());
        assertEquals(Optional.of("ball"), next.held());
    }

    @Test
    public void aBallTheRobotPushesOffItsSeatIsOutOfTheNestUntilItComesToRest() {
        Seat.Next<String> pushed = Seat.<String>seated().next(Optional.of("ball"), true, ROLLING);
        assertEquals(new Seat.Unseated<>("ball"), pushed.seat());
        assertEquals(Optional.empty(), pushed.held());

        Seat.Next<String> rollingOn = pushed.seat().next(Optional.of("ball"), false, ROLLING);
        assertEquals(new Seat.Unseated<>("ball"), rollingOn.seat());
        assertEquals(Optional.empty(), rollingOn.held());

        Seat.Next<String> atRest = rollingOn.seat().next(Optional.of("ball"), false, RESTING);
        assertEquals(Seat.<String>seated(), atRest.seat());
        assertEquals(Optional.of("ball"), atRest.held());
    }

    @Test
    public void aBallPushedButNotMovedIsStillHeld() {
        Seat.Next<String> next = Seat.<String>seated().next(Optional.of("ball"), true, RESTING);

        assertEquals(Seat.<String>seated(), next.seat());
        assertEquals(Optional.of("ball"), next.held());
    }

    @Test
    public void aBallRollingInTheNestThatNobodyPushedIsHeld() {
        Seat.Next<String> next = Seat.<String>seated().next(Optional.of("ball"), false, ROLLING);

        assertEquals(Optional.of("ball"), next.held());
    }

    @Test
    public void anotherBallInTheNestIsHeldWhateverTheBallPushedOffDid() {
        Seat<String> off = new Seat.Unseated<>("pushed");

        Seat.Next<String> another = off.next(Optional.of("another"), false, ROLLING);
        assertEquals(Seat.<String>seated(), another.seat());
        assertEquals(Optional.of("another"), another.held());

        Seat.Next<String> empty = off.next(Optional.empty(), false, RESTING);
        assertEquals(Seat.<String>seated(), empty.seat());
        assertEquals(Optional.empty(), empty.held());
    }

    @Test
    public void aBallIsAtRestBelowTheRestSpeed() {
        double rest = Flight.REST_SPEED_IN_PER_S * Length.METRES_PER_INCH;
        Seat<String> off = new Seat.Unseated<>("ball");

        assertEquals(
                Optional.of("ball"), off.next(Optional.of("ball"), false, rest).held());
        assertEquals(
                Optional.empty(),
                off.next(Optional.of("ball"), false, rest * 1.01).held());
    }
}
