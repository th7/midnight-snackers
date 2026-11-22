package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HubPortTest {
    private static final double DELTA = 1e-9;
    private static final Seconds STEP = Valid.value(Seconds.of(0.005));
    private static final String SDK_DIRECTION =
            "the SDK's DcMotorImpl.getOperationalDirection flips FORWARD for a motor type declared CCW";
    private static final String SDK_LYNX =
            "the SDK's LynxDcMotorController (Hardware 11.2.1): RUN_USING_ENCODER targets sign x |power| x the"
                    + " type's achievable ticks/s, a written velocity is rounded and clipped to 32767, and a mode"
                    + " change reissues the target as the power it stands for";

    private static HubPort.Wrote wrote(Sense direction, RunMode mode, Written written) {
        return new HubPort.Wrote(MotorType.ULTRAPLANETARY_HD_HEX, direction, mode, written);
    }

    private static HubPort.Driven driven(HubPort.Wrote wrote, HubPort.State state, Count measured) {
        return Valid.value(HubPort.drive(wrote, state, measured, STEP));
    }

    private static double power(HubPort.Wrote wrote, double measuredTicksPerSecond) {
        return driven(wrote, HubPort.State.fresh(), new Count(0, measuredTicksPerSecond))
                .power()
                .value();
    }

    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    @Test
    public void aClockwiseTypeKeepsTheDirectionTheCodeSetAndACounterclockwiseOneFlipsIt() {
        assertEquals(SDK_DIRECTION, Sense.FORWARD, HubPort.operational(Turning.CLOCKWISE, Sense.FORWARD));
        assertEquals(SDK_DIRECTION, Sense.REVERSE, HubPort.operational(Turning.CLOCKWISE, Sense.REVERSE));
        assertEquals(SDK_DIRECTION, Sense.REVERSE, HubPort.operational(Turning.COUNTERCLOCKWISE, Sense.FORWARD));
        assertEquals(SDK_DIRECTION, Sense.FORWARD, HubPort.operational(Turning.COUNTERCLOCKWISE, Sense.REVERSE));
    }

    @Test
    public void withoutTheEncoderTheWrittenPowerReachesTheMotorInItsOperationalDirection() {
        assertEquals(0.5, power(wrote(Sense.FORWARD, RunMode.RUN_WITHOUT_ENCODER, new Written.Powered(0.5)), 0), 0);
        assertEquals(-0.5, power(wrote(Sense.REVERSE, RunMode.RUN_WITHOUT_ENCODER, new Written.Powered(0.5)), 0), 0);
        assertEquals(1, power(wrote(Sense.FORWARD, RunMode.RUN_WITHOUT_ENCODER, new Written.Powered(1.7)), 0), 0);
    }

    @Test
    public void withoutTheEncoderAVelocityLeftFromBeforeIsThePowerItStandsFor() {
        assertEquals(
                SDK_LYNX,
                0.5,
                power(wrote(Sense.FORWARD, RunMode.RUN_WITHOUT_ENCODER, new Written.AtVelocity(1190)), 0),
                DELTA);
        assertEquals(
                SDK_LYNX,
                -1,
                power(wrote(Sense.FORWARD, RunMode.RUN_WITHOUT_ENCODER, new Written.AtVelocity(-5000)), 0),
                DELTA);
    }

    @Test
    public void usingTheEncoderAPowerIsATargetOfThatShareOfTheTypesAchievableSpeed() {
        HubPort.Wrote full = wrote(Sense.FORWARD, RunMode.RUN_USING_ENCODER, new Written.Powered(1));
        HubPort.Wrote fullReversed = wrote(Sense.REVERSE, RunMode.RUN_USING_ENCODER, new Written.Powered(1));

        assertEquals(SDK_LYNX, 0, power(full, 2380), 0);
        assertTrue(SDK_LYNX, power(full, 2360) > 0);
        assertTrue(SDK_LYNX, power(full, 2400) < 0);
        assertEquals(SDK_LYNX, 0, power(fullReversed, -2380), 0);
        assertTrue(SDK_LYNX, power(fullReversed, -2360) < 0);
    }

    @Test
    public void usingTheEncoderAWrittenVelocityIsRoundedAndClippedToWhatTheHubCarries() {
        HubPort.Wrote rounded = wrote(Sense.FORWARD, RunMode.RUN_USING_ENCODER, new Written.AtVelocity(1000.4));
        HubPort.Wrote beyond = wrote(Sense.FORWARD, RunMode.RUN_USING_ENCODER, new Written.AtVelocity(40000));

        assertEquals(SDK_LYNX, 0, power(rounded, 1000), 0);
        assertTrue(SDK_LYNX, power(beyond, 32760) > 0);
        assertTrue(SDK_LYNX, power(beyond, 32780) < 0);
    }

    @Test
    public void theHubMeasuresVelocityInItsOwnTwentyTickSteps() {
        HubPort.Wrote holding = wrote(Sense.FORWARD, RunMode.RUN_USING_ENCODER, new Written.AtVelocity(1000));

        assertEquals(0, power(holding, 1009), 0);
        assertTrue(power(holding, 1011) < 0);
    }

    @Test
    public void runToPositionIsRefusedNamingItself() {
        assertRejected(
                "RUN_TO_POSITION",
                HubPort.drive(
                        wrote(Sense.FORWARD, RunMode.RUN_TO_POSITION, new Written.Powered(0.5)),
                        HubPort.State.fresh(),
                        Count.none(),
                        STEP));
    }

    @Test
    public void stopAndResetHoldsTheMotorStillAndCountsFromZeroAfter() {
        HubPort.Wrote resetting = wrote(Sense.FORWARD, RunMode.STOP_AND_RESET_ENCODER, new Written.Powered(0.5));
        HubPort.Wrote running = wrote(Sense.FORWARD, RunMode.RUN_WITHOUT_ENCODER, new Written.Powered(0.5));

        HubPort.Driven reset = driven(resetting, HubPort.State.fresh(), new Count(700, 0));

        assertEquals(0, reset.power().value(), 0);
        assertEquals(new Encoder(0, 0), HubPort.reports(resetting, reset.next(), new Count(700, 0)));
        assertEquals(new Encoder(50, 100), HubPort.reports(running, reset.next(), new Count(750, 100)));
    }

    @Test
    public void theHubReportsAReversedMotorsCountNegatedInTwentyTickSteps() {
        HubPort.Wrote forward = wrote(Sense.FORWARD, RunMode.RUN_WITHOUT_ENCODER, new Written.Powered(0));
        HubPort.Wrote reversed = wrote(Sense.REVERSE, RunMode.RUN_WITHOUT_ENCODER, new Written.Powered(0));

        assertEquals(new Encoder(124, 980), HubPort.reports(forward, HubPort.State.fresh(), new Count(123.6, 987)));
        assertEquals(new Encoder(-124, -980), HubPort.reports(reversed, HubPort.State.fresh(), new Count(123.6, 987)));
    }

    @Test
    public void theHubReportsVelocityToTheNearestTwentyTicksASecond() {
        HubPort.Wrote forward = wrote(Sense.FORWARD, RunMode.RUN_WITHOUT_ENCODER, new Written.Powered(0));

        assertEquals(
                20,
                HubPort.reports(forward, HubPort.State.fresh(), new Count(0, 29))
                        .velocity(),
                0);
        assertEquals(
                40,
                HubPort.reports(forward, HubPort.State.fresh(), new Count(0, 31))
                        .velocity(),
                0);
        assertEquals(
                0,
                HubPort.reports(forward, HubPort.State.fresh(), new Count(0, 9)).velocity(),
                0);
        for (double ticks = -1000; ticks <= 1000; ticks += 7.3) {
            double read = HubPort.reports(forward, HubPort.State.fresh(), new Count(0, ticks))
                    .velocity();
            assertTrue(ticks + " read as " + read, Math.abs(read - ticks) <= 10 + 1e-9);
            assertEquals(ticks + " read as " + read, 0, Math.IEEEremainder(read, 20), 0);
        }
    }
}
