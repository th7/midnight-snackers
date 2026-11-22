package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SpindexerServosTest {
    private static final String SERVO_SOURCE = "doc/plans/spindexer-sim.md, What the tab does: the REV Smart Robot"
            + " Servo's 0.14 s per 60 degrees and 13.5 kg cm at 6 V, scaled to the Hub's 5 V: 1.0 rev/s unloaded"
            + " and 1.1 N m at stall turning continuously, 0.168 s per 60 degrees held at an angle";
    private static final double ROUNDING = 1e-9;

    private final Spindexer.Spinner spinner = Spindexer.Spinner.revSmartRobotServo();

    private final Spindexer.Ramp ramp = Spindexer.Ramp.revSmartRobotServo();

    private static Spindexer.RampAngle degrees(double degrees) {
        return Valid.value(Spindexer.RampAngle.ofDegrees(degrees));
    }

    private Spindexer.RampAngle slewed(double fromDegrees, double toDegrees, double loadNewtonMetres, double seconds) {
        return ramp.next(
                degrees(fromDegrees),
                degrees(toDegrees),
                Valid.value(Spindexer.Torque.ofNewtonMetres(loadNewtonMetres)),
                Valid.value(Seconds.of(seconds)));
    }

    private Spindexer.Spinner.Turning after(double power, double loadNewtonMetres) {
        return spinner.next(
                Spindexer.Spinner.Turning.still(),
                Valid.value(Power.of(power)),
                Valid.value(Spindexer.Torque.ofNewtonMetres(loadNewtonMetres)),
                Valid.value(Seconds.of(0.01)));
    }

    @Test
    public void unloadedAtFullPowerTheSpinnerTurnsOneRevolutionASecondAndAtHalfPowerHalfOfOne() {
        assertEquals(1.0, after(1, 0).revolutionsPerSecond(), ROUNDING);
        assertEquals(0.5, after(0.5, 0).revolutionsPerSecond(), ROUNDING);
    }

    @Test
    public void loadedWithHalfItsStallTorqueItTurnsAtHalfSpeedAndAtItsStallTorqueOrMoreItStops() {
        assertEquals(0.5, after(1, 0.55).revolutionsPerSecond(), ROUNDING);
        assertEquals(0, after(1, 1.1).revolutionsPerSecond(), ROUNDING);
        assertEquals(0, after(1, 3).revolutionsPerSecond(), ROUNDING);
    }

    @Test
    public void atNegativePowerItTurnsTheOtherWayAndItsLoadStillHoldsItBack() {
        assertEquals(-1.0, after(-1, 0).revolutionsPerSecond(), ROUNDING);
        assertEquals(-0.5, after(-1, 0.55).revolutionsPerSecond(), ROUNDING);
        assertEquals("a hundredth of a turn back", -0.0628319, after(-1, 0).radians(), 1e-7);
    }

    @Test
    public void theRampSlewsFrom0To35DegreesIn0Point098SecondsAndStaysAt35WhateverTheLoad() {
        assertEquals(32.142857, slewed(0, 35, 0, 0.09).degrees(), 1e-6);
        assertEquals(35, slewed(0, 35, 0, 0.098).degrees(), ROUNDING);
        assertEquals(35, slewed(0, 35, 0, 0.5).degrees(), ROUNDING);
        assertEquals(35, slewed(35, 35, 5, 1).degrees(), ROUNDING);
        assertEquals(35, slewed(35, 35, -5, 1).degrees(), ROUNDING);
        assertEquals(
                "and back down the same way", 2.857143, slewed(35, 0, 0, 0.09).degrees(), 1e-6);
    }

    @Test
    public void aCommandBeyondTheServosRangeIsRefusedNamingTheRange() {
        assertRejected("from -1 to 1", Power.of(1.01));
        assertRejected("from -1 to 1", Power.of(-1.01));
        assertRejected("from -1 to 1", Power.of(Double.NaN));
        assertRejected("from 0 to 60", Spindexer.RampAngle.ofDegrees(60.5));
        assertRejected("from 0 to 60", Spindexer.RampAngle.ofDegrees(-1));
    }

    @Test
    public void aServoIsSetWithinTheTabsRangesAndRefusedBeyondThemNamingTheRange() {
        Spindexer.Spinner faster = Valid.value(Spindexer.Spinner.of(2, 1.1));
        Spindexer.Ramp slower = Valid.value(Spindexer.Ramp.of(0.336));

        assertEquals(
                2,
                faster.next(
                                Spindexer.Spinner.Turning.still(),
                                Valid.value(Power.of(1)),
                                Valid.value(Spindexer.Torque.ofNewtonMetres(0)),
                                Valid.value(Seconds.of(0.01)))
                        .revolutionsPerSecond(),
                ROUNDING);
        assertEquals(
                17.5,
                slower.next(
                                degrees(0),
                                degrees(35),
                                Valid.value(Spindexer.Torque.ofNewtonMetres(0)),
                                Valid.value(Seconds.of(0.098)))
                        .degrees(),
                ROUNDING);
        assertRejected("from 0.2 to 3", Spindexer.Spinner.of(3.5, 1.1));
        assertRejected("from 0.2 to 3", Spindexer.Spinner.of(0.1, 1.1));
        assertRejected("from 0.1 to 5", Spindexer.Spinner.of(1, 5.5));
        assertRejected("from 0.05 to 1", Spindexer.Ramp.of(1.5));
        assertRejected("from 0.05 to 1", Spindexer.Ramp.of(Double.NaN));
    }

    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }
}
