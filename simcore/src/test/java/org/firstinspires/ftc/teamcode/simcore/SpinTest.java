package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SpinTest {
    private static final double DELTA = 1e-6;
    private static final String REV_HD_HEX =
            "REV's HD Hex Motor spins 6000 rpm free at 12 V; the SDK's UltraPlanetary type counts 560 ticks"
                    + " a turn at 300 rpm, 2800 ticks/s";

    private static Spin after(double seconds, double power, double volts) {
        Spin spin = Spin.still();
        Seconds step = Valid.value(Seconds.of(0.001));
        for (int i = 0; i < Math.round(seconds / 0.001); i++) {
            spin = spin.next(MotorType.ULTRAPLANETARY_HD_HEX, Power.clamped(power), volts, step);
        }
        return spin;
    }

    @Test
    public void anUltraPlanetaryHdHexSpinsTwentyEightHundredTicksASecondFreeAtFullPowerOnTwelveVolts() {
        assertEquals(REV_HD_HEX, 2800, MotorType.ULTRAPLANETARY_HD_HEX.freeTicksPerSecond(), 0);
        assertEquals(REV_HD_HEX, 2800, after(3, 1, 12).ticksPerSecond(), 0.01);
    }

    @Test
    public void itSpinsInProportionToPowerAndVolts() {
        assertEquals(1400, after(3, 0.5, 12).ticksPerSecond(), 0.01);
        assertEquals(-1400, after(3, -1, 6).ticksPerSecond(), 0.01);
    }

    @Test
    public void itTakesItsSpinUpTimeToComeMostOfTheWay() {
        assertEquals(0.1, Spin.SPIN_UP_SECONDS, 0);
        assertEquals(2800 * (1 - Math.exp(-1)), after(0.1, 1, 12).ticksPerSecond(), 0.01);
    }

    @Test
    public void itCountsTheTicksItTurns() {
        Spin spin = after(3, 1, 12);
        Spin later = spin.next(MotorType.ULTRAPLANETARY_HD_HEX, Power.clamped(1), 12, Valid.value(Seconds.of(0.5)));

        assertEquals(1400, later.count().ticks() - spin.count().ticks(), DELTA);
        assertEquals(2800, later.count().ticksPerSecond(), DELTA);
    }
}
