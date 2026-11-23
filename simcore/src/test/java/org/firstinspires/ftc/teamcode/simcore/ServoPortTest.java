package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ServoPortTest {
    private static final PulseRange SDK_RANGE = range(600, 2400);

    private static PulseRange range(double lowerUs, double upperUs) {
        return Valid.value(PulseRange.of(lowerUs, upperUs));
    }

    private static ServoPort positionedAt(double position, PulseRange range) {
        return Valid.value(ServoPort.unwritten().positioned(position, range));
    }

    private static double signalled(ServoPort port) {
        return ((ServoSignal.Pulse) port.signal()).microseconds();
    }

    private static String refusal(Checked<ServoPort> port) {
        return port.fold(accepted -> "accepted " + accepted.pulseWidth(), rule -> rule);
    }

    @Test
    public void aPulseWidthWrittenIsThePulseTheServoGets() {
        ServoPort port = Valid.value(ServoPort.unwritten().pulsed(2400));

        assertEquals(2400, signalled(port), 0);
        assertEquals(2400, port.pulseWidth(), 0);
    }

    @Test
    public void aPulseWidthIsSentInWholeMicrosecondsAsTheSdkTruncatesIt() {
        assertEquals(1499, Valid.value(ServoPort.unwritten().pulsed(1499.9)).pulseWidth(), 0);
    }

    @Test
    public void aPulseWidthOutsideOneToSixtyFiveThousandFiveHundredThirtyFiveIsRefusedAsTheHubCommandRefusesIt() {
        assertEquals("illegal pulse width: 0", refusal(ServoPort.unwritten().pulsed(0.9)));
        assertEquals("illegal pulse width: 65536", refusal(ServoPort.unwritten().pulsed(65536)));
        assertEquals("illegal pulse width: 0", refusal(ServoPort.unwritten().pulsed(Double.NaN)));
        assertEquals(65535, Valid.value(ServoPort.unwritten().pulsed(65535)).pulseWidth(), 0);
        assertEquals(1, Valid.value(ServoPort.unwritten().pulsed(1)).pulseWidth(), 0);
    }

    @Test
    public void aPositionIsSentAsThePulseItsRangeMakesInWholeMicroseconds() {
        assertEquals(2400, positionedAt(1, SDK_RANGE).pulseWidth(), 0);
        assertEquals(870, positionedAt(0.15, SDK_RANGE).pulseWidth(), 0);
        assertEquals(500, positionedAt(0, range(500, 2500)).pulseWidth(), 0);
        assertEquals(1, positionedAt(0, range(0, 2500)).pulseWidth(), 0);
        assertEquals(2400, signalled(positionedAt(1, SDK_RANGE)), 0);
    }

    @Test
    public void aPositionOutsideZeroToOneOrARangeBackwardsIsRefused() {
        assertEquals(
                "a servo position is from 0 to 1, not 1.5",
                refusal(ServoPort.unwritten().positioned(1.5, SDK_RANGE)));
        assertEquals(
                "a pulse range is finite microseconds from its lower to its upper, not 2400 to 600",
                PulseRange.of(2400, 600).fold(found -> "found " + found, rule -> rule));
        assertEquals(
                "a pulse range is finite microseconds from its lower to its upper, not -1 to 600",
                PulseRange.of(-1, 600).fold(found -> "found " + found, rule -> rule));
    }

    @Test
    public void theSamePositionAgainSendsNothingSoARangeChangedSinceWaitsForTheNextPositionThatDiffers() {
        ServoPort full = positionedAt(1, SDK_RANGE);

        assertEquals(2400, Valid.value(full.positioned(1, range(500, 2500))).pulseWidth(), 0);
        assertEquals(2000, Valid.value(full.positioned(0.75, range(500, 2500))).pulseWidth(), 0);
    }

    @Test
    public void aPulseWidthReplacesThePositionsPulseAndThePositionAgainIsSentAfterIt() {
        ServoPort pulsed = Valid.value(positionedAt(1, SDK_RANGE).pulsed(1000));

        assertEquals(1000, pulsed.pulseWidth(), 0);
        assertEquals(2400, Valid.value(pulsed.positioned(1, SDK_RANGE)).pulseWidth(), 0);
    }

    @Test
    public void thePositionReadIsTheLastWrittenOrNoneWrittenZero() {
        ServoPort quarter = positionedAt(0.25, SDK_RANGE);

        assertEquals(0.25, quarter.read(range(500, 2500)).position(), 0);
        assertEquals(quarter, quarter.read(range(500, 2500)).port());
        assertEquals(0, ServoPort.unwritten().read(SDK_RANGE).position(), 0);
    }

    @Test
    public void afterAPulseWidthThePositionReadIsThePulseThroughTheRangeKeptAsTheLastWritten() {
        ServoPort pulsed = Valid.value(ServoPort.unwritten().pulsed(2400));
        ServoPort.PositionRead read = pulsed.read(SDK_RANGE);

        assertEquals(1, read.position(), 0);
        assertEquals(
                2400, Valid.value(read.port().positioned(1, range(500, 2500))).pulseWidth(), 0);
        assertEquals(
                0.5,
                Valid.value(ServoPort.unwritten().pulsed(1500)).read(SDK_RANGE).position(),
                1e-12);
        assertEquals(
                1,
                Valid.value(ServoPort.unwritten().pulsed(3000)).read(SDK_RANGE).position(),
                0);
        assertEquals(
                0,
                Valid.value(ServoPort.unwritten().pulsed(100)).read(SDK_RANGE).position(),
                0);
    }

    @Test
    public void aForgottenPositionIsSentAgainEvenWhenItIsTheSame() {
        ServoPort forgotten = positionedAt(1, SDK_RANGE).forgotten();

        assertEquals(2400, forgotten.pulseWidth(), 0);
        assertEquals(
                2500, Valid.value(forgotten.positioned(1, range(500, 2500))).pulseWidth(), 0);
        assertEquals(ServoPort.unwritten(), ServoPort.unwritten().forgotten());
    }

    @Test
    public void aDisabledPortSendsNoPulseUntilAPulseIsSentAgain() {
        ServoPort disabled = positionedAt(1, SDK_RANGE).enabled(false);

        assertEquals(ServoSignal.none(), disabled.signal());
        assertFalse(disabled.enabled());
        assertEquals(2400, disabled.pulseWidth(), 0);
        assertEquals(
                ServoSignal.none(),
                Valid.value(disabled.positioned(1, SDK_RANGE)).signal());
        assertEquals(1500, signalled(Valid.value(disabled.positioned(0.5, SDK_RANGE))), 0);
        assertEquals(1000, signalled(Valid.value(disabled.pulsed(1000))), 0);
        assertEquals(2400, signalled(disabled.enabled(true)), 0);
    }

    @Test
    public void aPortNobodyWroteSendsNoPulseEnabledOrNot() {
        assertTrue(ServoPort.unwritten().enabled());
        assertEquals(ServoSignal.none(), ServoPort.unwritten().signal());
        assertEquals(ServoSignal.none(), ServoPort.unwritten().enabled(true).signal());
    }
}
