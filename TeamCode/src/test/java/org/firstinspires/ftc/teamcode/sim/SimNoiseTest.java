package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.HashSet;
import java.util.Set;
import org.junit.Test;

public class SimNoiseTest {
    private static final Pose2d POSE = new Pose2d(-56, -12, 0.3);

    @Test
    public void noNoiseIsTheTunedModelExactly() {
        SimNoise none = SimNoise.NONE;

        for (int wheel = 0; wheel < 4; wheel++) {
            assertEquals(1, none.motor(wheel).kSVolts, 0);
            assertEquals(1, none.motor(wheel).kVVoltSecondsPerTick, 0);
            assertEquals(1, none.motor(wheel).kAVoltSecondsSquaredPerTick, 0);
        }
        assertEquals(SimRobot.BATTERY_VOLTS, none.batteryVolts(1000, 4), 0);
        assertEquals(Double.POSITIVE_INFINITY, none.tractionInPerS2, 0);
        assertEquals(POSE, none.placed(POSE));
        for (int i = 0; i < 100; i++) {
            assertEquals(SimRunner.LOOP_SECONDS, none.nextLoopSeconds(), 0);
        }
    }

    @Test
    public void theSameSeedDrawsTheSameRun() {
        SimNoise first = SimNoise.seeded(42);
        SimNoise second = SimNoise.seeded(42);

        for (int wheel = 0; wheel < 4; wheel++) {
            assertEquals(first.motor(wheel).kSVolts, second.motor(wheel).kSVolts, 0);
            assertEquals(first.motor(wheel).kVVoltSecondsPerTick, second.motor(wheel).kVVoltSecondsPerTick, 0);
            assertEquals(
                    first.motor(wheel).kAVoltSecondsSquaredPerTick, second.motor(wheel).kAVoltSecondsSquaredPerTick, 0);
        }
        assertEquals(first.freshVolts, second.freshVolts, 0);
        assertEquals(first.tractionInPerS2, second.tractionInPerS2, 0);
        assertEquals(first.placed(POSE), second.placed(POSE));
        for (int i = 0; i < 100; i++) {
            assertEquals(first.nextLoopSeconds(), second.nextLoopSeconds(), 0);
        }
    }

    @Test
    public void differentSeedsDrawDifferentRuns() {
        SimNoise first = SimNoise.seeded(1);
        SimNoise second = SimNoise.seeded(2);

        assertNotEquals(first.motor(0).kVVoltSecondsPerTick, second.motor(0).kVVoltSecondsPerTick, 0);
        assertNotEquals(first.freshVolts, second.freshVolts, 0);
        assertNotEquals(first.tractionInPerS2, second.tractionInPerS2, 0);
    }

    @Test
    public void everyMotorIsWithinTheSpreadOfItsTuning() {
        for (long seed = 0; seed < 200; seed++) {
            SimNoise noise = SimNoise.seeded(seed);
            for (int wheel = 0; wheel < 4; wheel++) {
                SimNoise.Motor motor = noise.motor(wheel);
                assertEquals("seed " + seed + " kSVolts", 1, motor.kSVolts, SimNoise.MOTOR_SPREAD);
                assertEquals(
                        "seed " + seed + " kVVoltSecondsPerTick", 1, motor.kVVoltSecondsPerTick, SimNoise.MOTOR_SPREAD);
                assertEquals(
                        "seed " + seed + " kAVoltSecondsSquaredPerTick",
                        1,
                        motor.kAVoltSecondsSquaredPerTick,
                        SimNoise.MOTOR_SPREAD);
            }
        }
    }

    @Test
    public void theMotorsDifferFromEachOtherNotJustFromTheTuning() {
        SimNoise noise = SimNoise.seeded(3);

        Set<Double> kVs = new HashSet<>();
        for (int wheel = 0; wheel < 4; wheel++) {
            kVs.add(noise.motor(wheel).kVVoltSecondsPerTick);
        }
        assertEquals("four wheels, four kVs", 4, kVs.size());
    }

    @Test
    public void theBatteryStartsSomewhereBetweenFlatAndFreshAndSagsUnderLoadAndDrainsWithTime() {
        for (long seed = 0; seed < 200; seed++) {
            double fresh = SimNoise.seeded(seed).freshVolts;
            assertTrue(
                    "seed " + seed + ": " + fresh,
                    fresh >= SimNoise.FLATTEST_VOLTS && fresh <= SimNoise.FRESHEST_VOLTS);
        }
        SimNoise noise = SimNoise.NONE.withBattery(13.8, 0.2, 0.004);

        assertEquals(13.8, noise.batteryVolts(0, 0), 1e-9);
        assertEquals("all four motors at full power", 13.8 - 0.8, noise.batteryVolts(0, 4), 1e-9);
        assertEquals("a hundred seconds in", 13.8 - 0.4, noise.batteryVolts(100, 0), 1e-9);
        assertEquals(13.8 - 0.8 - 0.4, noise.batteryVolts(100, 4), 1e-9);
    }

    @Test
    public void tractionIsWithinWhatTheTilesGive() {
        for (long seed = 0; seed < 200; seed++) {
            double g = SimNoise.seeded(seed).tractionInPerS2 / SimNoise.GRAVITY_IN_PER_S2;
            assertTrue(
                    "seed " + seed + ": " + g + " g", g >= SimNoise.LEAST_TRACTION_G && g <= SimNoise.MOST_TRACTION_G);
        }
    }

    @Test
    public void aPlacedRobotIsNearThePoseNotOnIt() {
        SimNoise noise = SimNoise.seeded(5);

        Set<String> seen = new HashSet<>();
        for (int i = 0; i < 1000; i++) {
            Pose2d placed = noise.placed(POSE);
            seen.add(placed.toString());
            assertEquals(POSE.position.x, placed.position.x, 5 * SimNoise.PLACEMENT_INCHES);
            assertEquals(POSE.position.y, placed.position.y, 5 * SimNoise.PLACEMENT_INCHES);
            assertEquals(POSE.heading.toDouble(), placed.heading.toDouble(), 5 * SimNoise.PLACEMENT_RADIANS);
        }
        assertTrue("a thousand placements, " + seen.size() + " distinct", seen.size() > 900);
    }

    @Test
    public void theLoopRunsAboutThirtyHertzWithTheOddHiccupAndNeverImpossiblyFast() {
        SimNoise noise = SimNoise.seeded(6);

        int draws = 10_000;
        double total = 0;
        int hiccups = 0;
        Set<Double> seen = new HashSet<>();
        for (int i = 0; i < draws; i++) {
            double period = noise.nextLoopSeconds();
            seen.add(period);
            assertTrue("period " + period, period >= SimNoise.LEAST_LOOP_SECONDS);
            assertTrue("period " + period, period <= SimNoise.LONGEST_HICCUP_SECONDS);
            total += period;
            if (period >= SimNoise.SHORTEST_HICCUP_SECONDS) {
                hiccups++;
            }
        }
        assertEquals("mean period", SimNoise.LOOP_SECONDS, total / draws, 0.006);
        assertEquals("hiccups", SimNoise.HICCUP_CHANCE, (double) hiccups / draws, 0.01);
        assertTrue("the periods vary: " + seen.size() + " distinct", seen.size() > draws / 2);
    }

    @Test
    public void namesTheRobotInALineForAFailureToQuote() {
        String line = SimNoise.NONE.withBattery(13.8, 0.2, 0.004).toString();

        assertTrue(line, line.startsWith("seed 0:"));
        assertTrue(
                line,
                line.contains("lf kSVolts x1.000 kVVoltSecondsPerTick x1.000 kAVoltSecondsSquaredPerTick x1.000"));
        assertTrue("ASCII, so any log carries it: " + line, line.chars().allMatch(c -> c < 128));
        assertTrue(line, line.contains("battery 13.80 V sag 0.20 V/power drain 0.0040 V/s"));
        assertTrue(line, line.contains("traction Infinity g"));
        assertTrue(line, line.contains("loop 20 ms"));
        assertTrue(SimNoise.seeded(7).toString(), SimNoise.seeded(7).toString().startsWith("seed 7:"));
    }

    @Test
    public void withersKeepTheSeedAndReplaceOneThing() {
        SimNoise noise = SimNoise.seeded(9);

        SimNoise weaker = noise.withMotor(SimNoise.RIGHT_FRONT, new SimNoise.Motor(1, 1.2, 1));

        assertEquals(9, weaker.seed);
        assertEquals(1.2, weaker.motor(SimNoise.RIGHT_FRONT).kVVoltSecondsPerTick, 0);
        assertEquals(
                noise.motor(SimNoise.LEFT_FRONT).kVVoltSecondsPerTick,
                weaker.motor(SimNoise.LEFT_FRONT).kVVoltSecondsPerTick,
                0);
        assertEquals(noise.freshVolts, weaker.freshVolts, 0);
        assertEquals(noise.tractionInPerS2, weaker.tractionInPerS2, 0);
        assertEquals(
                "the original is unchanged",
                noise.motor(SimNoise.RIGHT_FRONT).kVVoltSecondsPerTick,
                SimNoise.seeded(9).motor(SimNoise.RIGHT_FRONT).kVVoltSecondsPerTick,
                0);
    }
}
