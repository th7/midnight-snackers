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

    private static final Feedforward A_FLYWHEEL = Valid.value(Feedforward.of(0.5, 0.005, 0.005));

    private static Spin driven(Spin from, double seconds, double power, double volts, ZeroPower atZero) {
        Spin spin = from;
        Seconds step = Valid.value(Seconds.of(0.005));
        for (int i = 0; i < Math.round(seconds / 0.005); i++) {
            spin = spin.next(A_FLYWHEEL, Power.clamped(power), volts, atZero, step);
        }
        return spin;
    }

    @Test
    public void aMotorDrivenByItsFeedforwardComesToWhatItsVoltsLessFrictionTurnItAt() {
        assertEquals(2300, driven(Spin.still(), 30, 1, 12, ZeroPower.BRAKE).ticksPerSecond(), 0.01);
        assertEquals(-1100, driven(Spin.still(), 30, -0.5, 12, ZeroPower.BRAKE).ticksPerSecond(), 0.01);
    }

    @Test
    public void itComesMostOfTheWayInItsKaOverItsKv() {
        assertEquals(1453.88, driven(Spin.still(), 1, 1, 12, ZeroPower.BRAKE).ticksPerSecond(), 0.01);
    }

    @Test
    public void itDoesNotStartWhileItsVoltsAreNoMoreThanItsFriction() {
        assertEquals(Spin.still(), driven(Spin.still(), 1, 0.04, 12, ZeroPower.BRAKE));
    }

    @Test
    public void atZeroPowerFloatingItCoastsDownOnFrictionAlone() {
        Spin spinning = new Spin(0, 2300);

        assertEquals(1300, driven(spinning, 10, 0, 12, ZeroPower.FLOAT).ticksPerSecond(), 0.01);
    }

    @Test
    public void atZeroPowerBrakingItsBackEmfStopsItSooner() {
        Spin spinning = new Spin(0, 2300);

        assertEquals(782.91, driven(spinning, 1, 0, 12, ZeroPower.BRAKE).ticksPerSecond(), 0.01);
        assertEquals(782.91, driven(spinning, 1, 0, 12, ZeroPower.UNKNOWN).ticksPerSecond(), 0.01);
    }

    @Test
    public void frictionStopsItAndNeverTurnsItBackward() {
        Spin spinning = new Spin(0, 2300);

        assertEquals(0, driven(spinning, 30, 0, 12, ZeroPower.FLOAT).ticksPerSecond(), 0);
        assertEquals(0, driven(spinning, 30, 0, 12, ZeroPower.BRAKE).ticksPerSecond(), 0);
    }

    @Test
    public void drivenTheOtherWayItStopsAndSpinsUpTheOtherWay() {
        Spin spinning = new Spin(0, 2300);

        assertEquals(-2300, driven(spinning, 30, -1, 12, ZeroPower.BRAKE).ticksPerSecond(), 0.01);
    }

    @Test
    public void itCountsTheTicksItTurnsUnderItsFeedforward() {
        Spin spinning = new Spin(100, 2300);
        Spin later = spinning.next(A_FLYWHEEL, Power.clamped(1), 12, ZeroPower.BRAKE, Valid.value(Seconds.of(0.5)));

        assertEquals(1250, later.count().ticks(), DELTA);
        assertEquals(2300, later.ticksPerSecond(), DELTA);
    }

    @Test
    public void inOneLongStepItStopsWhereFrictionStopsIt() {
        Spin spinning = new Spin(0, 2300);
        Spin later = spinning.next(A_FLYWHEEL, Power.none(), 12, ZeroPower.FLOAT, Valid.value(Seconds.of(30)));

        assertEquals(26450, later.count().ticks(), 1e-6);
        assertEquals(0, later.ticksPerSecond(), 0);
    }
}
