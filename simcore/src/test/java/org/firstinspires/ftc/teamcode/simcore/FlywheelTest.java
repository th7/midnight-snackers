package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class FlywheelTest {
    private static final double[] TICKS_PER_TURN = {18, 24.2, 10000};

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    private static Flywheel.TicksPerTurn perTurn(double ticks) {
        return Valid.value(Flywheel.TicksPerTurn.of(ticks));
    }

    private static double turning(double commandedTicksPerSecond, double ticksPerTurn) {
        return Flywheel.AngularSpeed.from(commandedTicksPerSecond, perTurn(ticksPerTurn))
                .radiansPerSecond();
    }

    @Test
    public void theFlywheelsMotorTurnsAtMostItsFreeSpeedInEncoderTicks() {
        assertEquals(Flywheel.MOTOR, 6000, Flywheel.MOTOR_RPM, 0);
        assertEquals(Flywheel.MOTOR, 28, Flywheel.MOTOR_TICKS_PER_TURN, 0);
        assertEquals(2800, Flywheel.TOP_TICKS_PER_SECOND, 0);
    }

    @Test
    public void aFlywheelTurnsAsCommandedUpToItsTopSpeedEitherWayAndNotAtAllForACommandThatIsNotANumber() {
        assertEquals(1050, Flywheel.turnsAt(1050), 0);
        assertEquals(-1050, Flywheel.turnsAt(-1050), 0);
        assertEquals(Flywheel.TOP_TICKS_PER_SECOND, Flywheel.turnsAt(10000), 0);
        assertEquals(-Flywheel.TOP_TICKS_PER_SECOND, Flywheel.turnsAt(Double.NEGATIVE_INFINITY), 0);
        assertEquals(0, Flywheel.turnsAt(Double.NaN), 0);
    }

    @Test
    public void aFlywheelTurnsOnceForEighteenToTenThousandTicksOfItsMotorAndAnyOtherCountIsRefusedNamingTheRule() {
        assertEquals(Flywheel.TicksPerTurn.RANGE_SOURCE, 18, Flywheel.TicksPerTurn.LEAST, 0);
        assertEquals(Flywheel.TicksPerTurn.RANGE_SOURCE, 10000, Flywheel.TicksPerTurn.MOST, 0);
        for (double ticks : new double[] {18, 24.2, 28, 10000}) {
            assertEquals(ticks, perTurn(ticks).value(), 0);
        }
        for (double ticks : new double[] {
            StrictMath.nextDown(18.0),
            StrictMath.nextUp(10000.0),
            0,
            -24.2,
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY
        }) {
            assertEquals(
                    "a flywheel turns once for 18.0 to 10000.0 ticks of its motor, gearing included, not " + ticks,
                    rejection(Flywheel.TicksPerTurn.of(ticks)));
        }
    }

    @Test
    public void aFlywheelTurnsAtItsCommandTimesAWholeTurnOverItsTicksPerTurnInRadiansASecondSignedAsTheCommandIs() {
        for (double ticksPerTurn : TICKS_PER_TURN) {
            assertEquals(1050 * 2 * StrictMath.PI / ticksPerTurn, turning(1050, ticksPerTurn), 0);
            assertEquals(-1050 * 2 * StrictMath.PI / ticksPerTurn, turning(-1050, ticksPerTurn), 0);
            assertEquals(1, StrictMath.signum(turning(0.5, ticksPerTurn)), 0);
            assertEquals(-1, StrictMath.signum(turning(-0.5, ticksPerTurn)), 0);
        }
    }

    @Test
    public void aFlywheelCommandedPastItsMotorsTopSpeedTurnsAtItAndOneCommandedNoNumberDoesNotTurn() {
        for (double ticksPerTurn : TICKS_PER_TURN) {
            double top = Flywheel.TOP_TICKS_PER_SECOND * 2 * StrictMath.PI / ticksPerTurn;
            assertEquals(top, turning(10000, ticksPerTurn), 0);
            assertEquals(-top, turning(Double.NEGATIVE_INFINITY, ticksPerTurn), 0);
            assertEquals(0, turning(Double.NaN, ticksPerTurn), 0);
            assertEquals(0, turning(0, ticksPerTurn), 0);
        }
    }
}
