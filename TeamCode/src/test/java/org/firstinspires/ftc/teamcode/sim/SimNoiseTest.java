package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.simcore.Noise;
import org.firstinspires.ftc.teamcode.simcore.PerWheel;
import org.firstinspires.ftc.teamcode.simcore.Power;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.junit.Test;

public class SimNoiseTest {
    @Test
    public void noNoiseIsTheTunedModelAtTheSimulatorsBatteryAndLoop() {
        Noise none = SimNoise.NONE;

        for (Noise.Motor motor : none.motors().inOrder()) {
            assertEquals(Noise.Motor.tuned(), motor);
        }
        assertEquals(
                SimRobot.BATTERY_VOLTS,
                none.battery().volts(Valid.value(Seconds.of(1000)), PerWheel.all(Power.clamped(1))),
                0);
        assertEquals(Double.POSITIVE_INFINITY, none.traction().inPerS2(), 0);
        assertEquals(SimRunner.LOOP_SECONDS, none.loop().period().value(), 0);
        assertEquals(0, none.loop().spread(), 0);
        assertEquals(0, none.loop().hiccupChance(), 0);
    }

    @Test
    public void namesTheRobotInALineForAFailureToQuote() {
        String line = SimNoise.described(SimNoise.NONE.withBattery(Valid.value(Noise.Battery.of(13.8, 0.2, 0.004))));

        assertTrue(line, line.startsWith("seed 0:"));
        assertTrue(
                line,
                line.contains("lf kSVolts x1.000 kVVoltSecondsPerTick x1.000 kAVoltSecondsSquaredPerTick x1.000"));
        assertTrue("ASCII, so any log carries it: " + line, line.chars().allMatch(c -> c < 128));
        assertTrue(line, line.contains("battery 13.80 V sag 0.20 V/power drain 0.0040 V/s"));
        assertTrue(line, line.contains("traction Infinity g"));
        assertTrue(line, line.contains("loop 20 ms"));
        String seven = SimNoise.described(Noise.seeded(7));
        assertTrue(seven, seven.startsWith("seed 7:"));
        assertTrue(seven, seven.contains("set down +-0.50 in +-2.0 deg; loop 33 ms spread 0.15 hiccups 2%"));
    }
}
