package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DriveEncodersTest {
    private static final double IN_PER_TICK = 0.0005;
    private static final double SIDE_OFFSET_TICKS = 10000;
    private static final Sides<Sense> MOUNTED = new Sides<>(Sense.REVERSE, Sense.FORWARD);
    private static final Sides<Sense> AS_MOUNTED = MOUNTED;
    private static final Sides<Sense> NEITHER_REVERSED = new Sides<>(Sense.FORWARD, Sense.FORWARD);
    private static final DriveEncoders AT_REST = Valid.value(DriveEncoders.of(MOUNTED, IN_PER_TICK, SIDE_OFFSET_TICKS));
    private static final Twist STILL = new Twist(new Vec2(0, 0), 0);

    @Test
    public void drivingAheadCountsBothSidesAheadByTheDistanceWhenEachMotorRunsTheWayItIsMounted() {
        Sides<Encoder> read = AT_REST.moved(new Twist(new Vec2(1, 0), 0)).read(STILL, AS_MOUNTED);

        assertEquals(2000, read.left().position());
        assertEquals(2000, read.right().position());
    }

    @Test
    public void drivingSidewaysTurnsNeitherSide() {
        Sides<Encoder> read = AT_REST.moved(new Twist(new Vec2(0, 1), 0)).read(STILL, AS_MOUNTED);

        assertEquals(0, read.left().position());
        assertEquals(0, read.right().position());
    }

    @Test
    public void turningCounterclockwiseTurnsTheLeftSideBackAndTheRightAheadByTheirOffset() {
        Sides<Encoder> read = AT_REST.moved(new Twist(new Vec2(0, 0), 0.1)).read(STILL, AS_MOUNTED);

        assertEquals(-1000, read.left().position());
        assertEquals(1000, read.right().position());
    }

    @Test
    public void aMotorNotReversedToMatchItsMountingCountsItsSideBackwards() {
        Sides<Encoder> read =
                AT_REST.moved(new Twist(new Vec2(1, 0), 0)).read(new Twist(new Vec2(1, 0), 0), NEITHER_REVERSED);

        assertEquals(-2000, read.left().position());
        assertEquals(-2000, read.left().velocity(), 0);
        assertEquals(2000, read.right().position());
        assertEquals(2000, read.right().velocity(), 0);
    }

    @Test
    public void movementsAddUpAndThePositionIsInWholeTicks() {
        Twist aLittle = new Twist(new Vec2(0.0003, 0), 0);
        DriveEncoders encoders = AT_REST;
        for (int i = 0; i < 10; i++) {
            encoders = encoders.moved(aLittle);
        }

        assertEquals(
                "six tenths of a tick, ten times",
                6,
                encoders.read(STILL, AS_MOUNTED).right().position());
        assertEquals(
                "the first is not a whole tick",
                1,
                AT_REST.moved(aLittle).read(STILL, AS_MOUNTED).right().position());
    }

    @Test
    public void theVelocityIsEachSidesSpeedInTicksASecondAsTheHubReportsIt() {
        Sides<Encoder> read = AT_REST.read(new Twist(new Vec2(10, 5), 0.5), AS_MOUNTED);

        assertEquals(10 / IN_PER_TICK - SIDE_OFFSET_TICKS * 0.5, read.left().velocity(), 10);
        assertEquals(10 / IN_PER_TICK + SIDE_OFFSET_TICKS * 0.5, read.right().velocity(), 10);
        assertEquals(0, Math.IEEEremainder(read.left().velocity(), Encoder.HUB_VELOCITY_STEP_TICKS_PER_S), 0);
        assertEquals(0, Math.IEEEremainder(read.right().velocity(), Encoder.HUB_VELOCITY_STEP_TICKS_PER_S), 0);
    }

    @Test
    public void driveEncodersHaveAPositiveDistanceATickAndAFiniteOffset() {
        assertTrue(DriveEncoders.of(MOUNTED, 0, SIDE_OFFSET_TICKS) instanceof Checked.Rejected);
        assertTrue(DriveEncoders.of(MOUNTED, Double.NaN, SIDE_OFFSET_TICKS) instanceof Checked.Rejected);
        assertTrue(DriveEncoders.of(MOUNTED, IN_PER_TICK, Double.POSITIVE_INFINITY) instanceof Checked.Rejected);
        assertTrue(DriveEncoders.of(MOUNTED, IN_PER_TICK, Double.NaN) instanceof Checked.Rejected);
    }
}
