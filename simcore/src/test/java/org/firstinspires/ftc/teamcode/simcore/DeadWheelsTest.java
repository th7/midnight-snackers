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

    @Test
    public void drivingAheadTurnsTheParallelWheelByTheDistanceInTicks() {
        DeadWheels.Reading reading = AT_REST.moved(new Twist(new Vec2(1, 0), 0)).read(STILL);

        assertEquals(2000, reading.par().position());
        assertEquals(0, reading.perp().position());
    }

    @Test
    public void strafingTurnsThePerpendicularWheelWhichIsWiredTheOtherWayRound() {
        DeadWheels.Reading reading = AT_REST.moved(new Twist(new Vec2(0, 1), 0)).read(STILL);

        assertEquals(0, reading.par().position());
        assertEquals(-2000, reading.perp().position());
    }

    @Test
    public void turningInPlaceTurnsEachWheelByItsOffset() {
        DeadWheels.Reading reading =
                AT_REST.moved(new Twist(new Vec2(0, 0), 0.1)).read(STILL);

        assertEquals(60, reading.par().position());
        assertEquals(-500, reading.perp().position());
    }

    @Test
    public void movementsAddUpAndThePositionIsInWholeTicks() {
        Twist aLittle = new Twist(new Vec2(0.0003, 0), 0);
        DeadWheels wheels = AT_REST;
        for (int i = 0; i < 10; i++) {
            wheels = wheels.moved(aLittle);
        }

        assertEquals(
                "six tenths of a tick, ten times", 6, wheels.read(STILL).par().position());
        assertEquals(
                "the first is not a whole tick",
                1,
                AT_REST.moved(aLittle).read(STILL).par().position());
    }

    @Test
    public void theVelocityIsTheMotionInTicksASecondAsTheWheelsSeeIt() {
        DeadWheels.Reading reading = AT_REST.read(new Twist(new Vec2(10, 5), 0.5));

        assertEquals(10 / IN_PER_TICK + PAR_Y_TICKS * 0.5, reading.par().velocity(), 20);
        assertEquals(-(5 / IN_PER_TICK + PERP_X_TICKS * 0.5), reading.perp().velocity(), 20);
    }

    @Test
    public void theVelocityIsReadToTheNearestTwentyTicksASecondAsTheHubReportsIt() {
        assertEquals(
                20,
                AT_REST.read(new Twist(new Vec2(29 * IN_PER_TICK, 0), 0)).par().velocity(),
                0);
        assertEquals(
                40,
                AT_REST.read(new Twist(new Vec2(31 * IN_PER_TICK, 0), 0)).par().velocity(),
                0);
        assertEquals(
                0,
                AT_REST.read(new Twist(new Vec2(9 * IN_PER_TICK, 0), 0)).par().velocity(),
                0);
        for (double ticks = -1000; ticks <= 1000; ticks += 7.3) {
            double read = AT_REST.read(new Twist(new Vec2(ticks * IN_PER_TICK, 0), 0))
                    .par()
                    .velocity();
            assertTrue(ticks + " read as " + read, Math.abs(read - ticks) <= 10 + 1e-9);
            assertEquals(ticks + " read as " + read, 0, Math.IEEEremainder(read, 20), 0);
        }
    }

    @Test
    public void deadWheelsHaveAPositiveDistanceATickAndFiniteOffsets() {
        assertTrue(DeadWheels.of(0, PAR_Y_TICKS, PERP_X_TICKS) instanceof Checked.Rejected);
        assertTrue(DeadWheels.of(Double.NaN, PAR_Y_TICKS, PERP_X_TICKS) instanceof Checked.Rejected);
        assertTrue(DeadWheels.of(IN_PER_TICK, Double.POSITIVE_INFINITY, PERP_X_TICKS) instanceof Checked.Rejected);
        assertTrue(DeadWheels.of(IN_PER_TICK, PAR_Y_TICKS, Double.NaN) instanceof Checked.Rejected);
    }
}
