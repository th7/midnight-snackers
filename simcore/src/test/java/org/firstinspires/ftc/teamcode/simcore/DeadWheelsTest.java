package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DeadWheelsTest {
    private static final double IN_PER_TICK = 0.0005;
    private static final double PAR_Y_IN = -0.3;
    private static final double PERP_X_IN = 2.5;
    private static final DeadWheels AT_REST = Valid.value(DeadWheels.at(IN_PER_TICK, PAR_Y_IN, PERP_X_IN));
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
    public void turningInPlaceRollsEachWheelAsFarAsItSitsFromTheCentre() {
        DeadWheels.Counts counts = AT_REST.moved(new Twist(new Vec2(0, 0), 0.1)).counts(STILL);

        assertEquals(
                "0.3 in right of the centre, rolling ahead as the robot turns left",
                60,
                counts.par().ticks(),
                DELTA);
        assertEquals(
                "2.5 in ahead of the centre, rolling left", 500, counts.perp().ticks(), DELTA);
    }

    @Test
    public void reginaldsDeadWheelsWhereTheyNowSitCountATurnInPlace() {
        DeadWheels reginalds = Valid.value(DeadWheels.at(0.0005433029, -0.25, -3.5));

        DeadWheels.Counts counts =
                reginalds.moved(new Twist(new Vec2(0, 0), 0.1)).counts(STILL);

        assertEquals(
                "a quarter inch right of the centre",
                46.0148473347004,
                counts.par().ticks(),
                1e-9);
        assertEquals(
                "three and a half inches behind it",
                -644.2078626858056,
                counts.perp().ticks(),
                1e-9);
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

        assertEquals(10 / 0.0005 + 0.3 / 0.0005 * 0.5, counts.par().ticksPerSecond(), DELTA);
        assertEquals(5 / 0.0005 + 2.5 / 0.0005 * 0.5, counts.perp().ticksPerSecond(), DELTA);
    }

    @Test
    public void deadWheelsHaveAPositiveDistanceATickAndSitAFiniteWayFromTheCentre() {
        assertTrue(DeadWheels.at(0, PAR_Y_IN, PERP_X_IN) instanceof Checked.Rejected);
        assertTrue(DeadWheels.at(Double.NaN, PAR_Y_IN, PERP_X_IN) instanceof Checked.Rejected);
        assertTrue(DeadWheels.at(IN_PER_TICK, Double.POSITIVE_INFINITY, PERP_X_IN) instanceof Checked.Rejected);
        assertTrue(DeadWheels.at(IN_PER_TICK, PAR_Y_IN, Double.NaN) instanceof Checked.Rejected);
    }
}
