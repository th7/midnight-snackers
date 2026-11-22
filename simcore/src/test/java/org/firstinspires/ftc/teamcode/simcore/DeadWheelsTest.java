package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DeadWheelsTest {
    private static final double IN_PER_TICK = 0.0005;
    private static final double PAR_Y_TICKS = 600;
    private static final double PERP_X_TICKS = 5000;
    private static final DeadWheels AT_REST = Valid.value(DeadWheels.of(IN_PER_TICK, PAR_Y_TICKS, PERP_X_TICKS));
    private static final Twist STILL = new Twist(new Vec2(0, 0), 0);

    private static final double DELTA = 1e-9;

    @Test
    public void drivingAheadTurnsTheParallelWheelByTheDistanceInTicks() {
        DeadWheels.Counts counts = AT_REST.moved(new Twist(new Vec2(1, 0), 0)).counts(STILL);

        assertEquals(2000, counts.par().ticks(), DELTA);
        assertEquals(0, counts.perp().ticks(), DELTA);
    }

    @Test
    public void strafingLeftTurnsThePerpendicularWheelByTheDistanceInTicks() {
        DeadWheels.Counts counts = AT_REST.moved(new Twist(new Vec2(0, 1), 0)).counts(STILL);

        assertEquals(0, counts.par().ticks(), DELTA);
        assertEquals(2000, counts.perp().ticks(), DELTA);
    }

    @Test
    public void turningInPlaceTurnsEachWheelByItsOffset() {
        DeadWheels.Counts counts = AT_REST.moved(new Twist(new Vec2(0, 0), 0.1)).counts(STILL);

        assertEquals(60, counts.par().ticks(), DELTA);
        assertEquals(500, counts.perp().ticks(), DELTA);
    }

    @Test
    public void reginaldsDeadWheelsCountATurnInPlaceAsTheyDidBeforeTheRobotWasTurnedAround() {
        DeadWheels.Counts counts = Valid.value(DeadWheels.reginalds(IN_PER_TICK))
                .moved(new Twist(new Vec2(0, 0), 0.1))
                .counts(STILL);

        assertEquals(
                "610.3 ticks off the centre, counting back",
                -61.03360642343223,
                counts.par().ticks(),
                DELTA);
        assertEquals(
                "5186.6 ticks off the centre, counting back",
                -518.6563208836955,
                counts.perp().ticks(),
                DELTA);
    }

    @Test
    public void reginaldsDeadWheelsCountDrivingAheadAsTheyCountedDrivingBackBeforeTheRobotWasTurnedAround() {
        DeadWheels.Counts counts = Valid.value(DeadWheels.reginalds(IN_PER_TICK))
                .moved(new Twist(new Vec2(1, 1), 0))
                .counts(STILL);

        assertEquals(2000, counts.par().ticks(), DELTA);
        assertEquals(2000, counts.perp().ticks(), DELTA);
    }

    @Test
    public void movementsAddUpToFractionsOfATickThatTheHubRoundsWhenItReports() {
        Twist aLittle = new Twist(new Vec2(0.0003, 0), 0);
        DeadWheels wheels = AT_REST;
        for (int i = 0; i < 10; i++) {
            wheels = wheels.moved(aLittle);
        }

        assertEquals(
                "six tenths of a tick, ten times", 6, wheels.counts(STILL).par().ticks(), DELTA);
        assertEquals(0.6, AT_REST.moved(aLittle).counts(STILL).par().ticks(), DELTA);
    }

    @Test
    public void theVelocityIsTheMotionInTicksASecondAsTheWheelsSeeIt() {
        DeadWheels.Counts counts = AT_REST.counts(new Twist(new Vec2(10, 5), 0.5));

        assertEquals(10 / 0.0005 + 600 * 0.5, counts.par().ticksPerSecond(), DELTA);
        assertEquals(5 / 0.0005 + 5000 * 0.5, counts.perp().ticksPerSecond(), DELTA);
    }

    @Test
    public void deadWheelsHaveAPositiveDistanceATickAndFiniteOffsets() {
        assertTrue(DeadWheels.of(0, PAR_Y_TICKS, PERP_X_TICKS) instanceof Checked.Rejected);
        assertTrue(DeadWheels.of(Double.NaN, PAR_Y_TICKS, PERP_X_TICKS) instanceof Checked.Rejected);
        assertTrue(DeadWheels.of(IN_PER_TICK, Double.POSITIVE_INFINITY, PERP_X_TICKS) instanceof Checked.Rejected);
        assertTrue(DeadWheels.of(IN_PER_TICK, PAR_Y_TICKS, Double.NaN) instanceof Checked.Rejected);
    }
}
