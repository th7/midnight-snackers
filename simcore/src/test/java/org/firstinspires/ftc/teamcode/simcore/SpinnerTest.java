package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SpinnerTest {
    private static final double SDK_LOWER_US = 600;
    private static final double SDK_UPPER_US = 2400;

    private static ServoSignal at(double position, double lowerUs, double upperUs) {
        return Valid.value(ServoSignal.of(true, position, lowerUs, upperUs));
    }

    private static ServoSignal at(double position) {
        return at(position, SDK_LOWER_US, SDK_UPPER_US);
    }

    @Test
    public void aPositionBecomesThePulseItsRangeMakes() {
        assertEquals(2400, ((ServoSignal.Pulse) at(1)).microseconds(), 1e-9);
        assertEquals(1500, ((ServoSignal.Pulse) at(0.5)).microseconds(), 1e-9);
        assertEquals(500, ((ServoSignal.Pulse) at(0, 500, 2500)).microseconds(), 1e-9);
    }

    @Test
    public void aServoNotDrivenGetsNoPulse() {
        assertEquals(ServoSignal.none(), Valid.value(ServoSignal.of(false, 1, SDK_LOWER_US, SDK_UPPER_US)));
    }

    @Test
    public void aPositionOutsideZeroToOneOrARangeBackwardsIsNoPulse() {
        assertEquals(
                "a servo position is from 0 to 1, not 1.5",
                ServoSignal.of(true, 1.5, SDK_LOWER_US, SDK_UPPER_US).fold(found -> "found " + found, rule -> rule));
        assertEquals(
                "a pulse range is finite microseconds from its lower to its upper, not 2400 to 600",
                ServoSignal.of(true, 0.5, 2400, 600).fold(found -> "found " + found, rule -> rule));
    }

    @Test
    public void fullPowerForwardThroughTheSdksRangeTurnsClockwiseAtNinetyPercent() {
        Spinner.Turning turning = Spinner.turning(at(1), false);

        assertEquals(Spinner.Way.CLOCKWISE, turning.way());
        assertEquals(0.9, turning.ofFull(), 1e-9);
    }

    @Test
    public void belowFifteenHundredMicrosecondsItTurnsCounterclockwise() {
        Spinner.Turning turning = Spinner.turning(at(0), false);

        assertEquals(Spinner.Way.COUNTERCLOCKWISE, turning.way());
        assertEquals(0.9, turning.ofFull(), 1e-9);
    }

    @Test
    public void aRangeOfFiveHundredToTwentyFiveHundredGivesFullSpeed() {
        assertEquals(1, Spinner.turning(at(1, 500, 2500), false).ofFull(), 1e-9);
        assertEquals(1, Spinner.turning(at(1, 400, 2600), false).ofFull(), 1e-9);
    }

    @Test
    public void atFifteenHundredMicrosecondsOrWithNoPulseItIsStill() {
        assertEquals(Spinner.Turning.still(), Spinner.turning(at(0.5), false));
        assertEquals(Spinner.Turning.still(), Spinner.turning(ServoSignal.none(), false));
    }

    @Test
    public void aReversedSpinnerTurnsTheOtherWay() {
        assertEquals(Spinner.Way.COUNTERCLOCKWISE, Spinner.turning(at(1), true).way());
        assertEquals(Spinner.Way.CLOCKWISE, Spinner.turning(at(0), true).way());
    }
}
