package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.Test;

public class NoiseTest {
    private static final double EXACT_VOLTS = 12.5;
    private static final double EXACT_LOOP_SECONDS = 0.02;

    private static final Noise EXACT = Noise.exact(
            Valid.value(Noise.Battery.of(EXACT_VOLTS, 0, 0)), Valid.value(Noise.Loop.of(EXACT_LOOP_SECONDS, 0, 0)));

    private static final PerWheel<Power> STILL = PerWheel.all(Power.none());
    private static final PerWheel<Power> FULL = PerWheel.all(Power.clamped(1));

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    @Test
    public void anExactRobotIsTheTunedModelExactlyAndDrawsNothing() {
        for (Noise.Motor motor : EXACT.motors().inOrder()) {
            assertEquals(1, motor.kSVolts(), 0);
            assertEquals(1, motor.kVVoltSecondsPerTick(), 0);
            assertEquals(1, motor.kAVoltSecondsSquaredPerTick(), 0);
        }
        assertEquals(EXACT_VOLTS, EXACT.battery().volts(seconds(1000), FULL), 0);
        assertEquals(Double.POSITIVE_INFINITY, EXACT.traction().inPerS2(), 0);

        Draws draws = EXACT.draws();
        Drawn<Noise.Nudge> setDown = EXACT.setDown(draws);
        assertEquals(new Noise.Nudge.None(), setDown.value());
        assertSame("set down without a draw", draws, setDown.next());
        for (int i = 0; i < 100; i++) {
            Drawn<Seconds> loop = EXACT.nextLoop(draws);
            assertEquals(EXACT_LOOP_SECONDS, loop.value().value(), 0);
            assertSame("a loop without a draw", draws, loop.next());
        }
    }

    @Test
    public void theSameSeedDrawsTheSameRobot() {
        Noise first = Noise.seeded(42);
        Noise second = Noise.seeded(42);

        for (Wheel wheel : Wheel.values()) {
            Noise.Motor one = first.motors().of(wheel), other = second.motors().of(wheel);
            assertEquals(one.kSVolts(), other.kSVolts(), 0);
            assertEquals(one.kVVoltSecondsPerTick(), other.kVVoltSecondsPerTick(), 0);
            assertEquals(one.kAVoltSecondsSquaredPerTick(), other.kAVoltSecondsSquaredPerTick(), 0);
        }
        assertEquals(first.battery().freshVolts(), second.battery().freshVolts(), 0);
        assertEquals(first.traction().inPerS2(), second.traction().inPerS2(), 0);
        assertEquals(
                first.setDown(first.draws()).value(),
                second.setDown(second.draws()).value());
        Draws one = first.draws(), other = second.draws();
        for (int i = 0; i < 100; i++) {
            Drawn<Seconds> a = first.nextLoop(one), b = second.nextLoop(other);
            assertEquals(a.value(), b.value());
            one = a.next();
            other = b.next();
        }
    }

    @Test
    public void differentSeedsDrawDifferentRobots() {
        Noise first = Noise.seeded(1);
        Noise second = Noise.seeded(2);

        assertNotEquals(
                first.motors().leftFront().kVVoltSecondsPerTick(),
                second.motors().leftFront().kVVoltSecondsPerTick(),
                0);
        assertNotEquals(first.battery().freshVolts(), second.battery().freshVolts(), 0);
        assertNotEquals(first.traction().inPerS2(), second.traction().inPerS2(), 0);
    }

    @Test
    public void everyMotorIsWithinTheSpreadOfItsTuning() {
        for (long seed = 0; seed < 200; seed++) {
            for (Noise.Motor motor : Noise.seeded(seed).motors().inOrder()) {
                assertEquals("seed " + seed + " kSVolts", 1, motor.kSVolts(), Noise.MOTOR_SPREAD);
                assertEquals(
                        "seed " + seed + " kVVoltSecondsPerTick", 1, motor.kVVoltSecondsPerTick(), Noise.MOTOR_SPREAD);
                assertEquals(
                        "seed " + seed + " kAVoltSecondsSquaredPerTick",
                        1,
                        motor.kAVoltSecondsSquaredPerTick(),
                        Noise.MOTOR_SPREAD);
            }
        }
    }

    @Test
    public void theMotorsDifferFromEachOtherNotJustFromTheTuning() {
        Set<Double> kVs = new HashSet<>();
        for (Noise.Motor motor : Noise.seeded(3).motors().inOrder()) {
            kVs.add(motor.kVVoltSecondsPerTick());
        }

        assertEquals("four wheels, four kVs", 4, kVs.size());
    }

    @Test
    public void theBatteryStartsSomewhereBetweenFlatAndFreshAndSagsUnderLoadAndDrainsWithTime() {
        for (long seed = 0; seed < 200; seed++) {
            double fresh = Noise.seeded(seed).battery().freshVolts();
            assertTrue("seed " + seed + ": " + fresh, fresh >= Noise.FLATTEST_VOLTS && fresh <= Noise.FRESHEST_VOLTS);
        }
        Noise.Battery battery = Valid.value(Noise.Battery.of(13.8, 0.2, 0.004));

        assertEquals(13.8, battery.volts(Seconds.zero(), STILL), 1e-9);
        assertEquals("all four motors at full power", 13.8 - 0.8, battery.volts(Seconds.zero(), FULL), 1e-9);
        assertEquals(
                "one at full power, one at full reverse",
                13.8 - 0.4,
                battery.volts(
                        Seconds.zero(),
                        STILL.with(Wheel.LEFT_FRONT, Power.clamped(1)).with(Wheel.RIGHT_BACK, Power.clamped(-1))),
                1e-9);
        assertEquals("a hundred seconds in", 13.8 - 0.4, battery.volts(seconds(100), STILL), 1e-9);
        assertEquals(13.8 - 0.8 - 0.4, battery.volts(seconds(100), FULL), 1e-9);
    }

    @Test
    public void tractionIsWithinWhatTheTilesGive() {
        for (long seed = 0; seed < 200; seed++) {
            double g = Noise.seeded(seed).traction().inPerS2() / Flight.GRAVITY_IN_PER_S2;
            assertTrue("seed " + seed + ": " + g + " g", g >= Noise.LEAST_TRACTION_G && g <= Noise.MOST_TRACTION_G);
        }
    }

    @Test
    public void aRobotSetDownLandsNearWhereItWasPutNotOnIt() {
        Noise noise = Noise.seeded(5);
        Draws draws = noise.draws();

        Set<Noise.Nudge> seen = new HashSet<>();
        for (int i = 0; i < 1000; i++) {
            Drawn<Noise.Nudge> drawn = noise.setDown(draws);
            draws = drawn.next();
            assertTrue(drawn.value() instanceof Noise.Nudge.By);
            Noise.Nudge.By by = (Noise.Nudge.By) drawn.value();
            seen.add(by);
            assertEquals(0, by.offset().x(), 5 * Noise.SET_DOWN_INCHES);
            assertEquals(0, by.offset().y(), 5 * Noise.SET_DOWN_INCHES);
            assertEquals(0, by.radians(), 5 * Noise.SET_DOWN_RADIANS);
        }
        assertTrue("a thousand set downs, " + seen.size() + " distinct", seen.size() > 900);
    }

    @Test
    public void theLoopRunsAboutThirtyHertzWithTheOddHiccupAndNeverImpossiblyFast() {
        Noise noise = Noise.seeded(6);
        Draws draws = noise.draws();

        int loops = 10_000;
        double total = 0;
        int hiccups = 0;
        Set<Double> seen = new HashSet<>();
        for (int i = 0; i < loops; i++) {
            Drawn<Seconds> drawn = noise.nextLoop(draws);
            draws = drawn.next();
            double period = drawn.value().value();
            seen.add(period);
            assertTrue("period " + period, period >= Noise.LEAST_LOOP_SECONDS);
            assertTrue("period " + period, period <= Noise.LONGEST_HICCUP_SECONDS);
            total += period;
            if (period >= Noise.SHORTEST_HICCUP_SECONDS) {
                hiccups++;
            }
        }
        assertEquals("mean period", Noise.LOOP_SECONDS, total / loops, 0.006);
        assertEquals("hiccups", Noise.HICCUP_CHANCE, (double) hiccups / loops, 0.01);
        assertTrue("the periods vary: " + seen.size() + " distinct", seen.size() > loops / 2);
    }

    @Test
    public void withersKeepTheSeedAndReplaceOneThing() {
        Noise noise = Noise.seeded(9);
        Noise.Motor weaker = Valid.value(Noise.Motor.of(1, 1.2, 1));

        Noise changed = noise.withMotor(Wheel.RIGHT_FRONT, weaker);

        assertEquals(9, changed.seed());
        assertSame(weaker, changed.motors().rightFront());
        assertSame(noise.motors().leftFront(), changed.motors().leftFront());
        assertSame(noise.battery(), changed.battery());
        assertSame(noise.traction(), changed.traction());
        assertEquals(
                "the original is unchanged",
                Noise.seeded(9).motors().rightFront().kVVoltSecondsPerTick(),
                noise.motors().rightFront().kVVoltSecondsPerTick(),
                0);
        assertEquals(
                List.of(weaker, weaker, weaker, weaker),
                noise.withMotors(weaker).motors().inOrder());
        Traction slippery = Valid.value(Traction.of(40));
        assertSame(slippery, noise.withTraction(slippery).traction());
        assertEquals(
                "a robot's draws start from its seed, whatever else changed",
                noise.draws().nextDouble().value(),
                changed.withLoop(noise.loop())
                        .withHand(noise.hand())
                        .draws()
                        .nextDouble()
                        .value(),
                0);
    }

    @Test
    public void whatANoiseIsMadeOfIsChecked() {
        assertRejected("positive", Noise.Motor.of(0, 1, 1));
        assertRejected("positive", Noise.Motor.of(1, Double.NaN, 1));
        assertRejected("finite", Noise.Motor.of(1, 1, Double.POSITIVE_INFINITY));
        assertRejected("positive", Noise.Battery.of(0, 0, 0));
        assertRejected("not below zero", Noise.Battery.of(12, -0.1, 0));
        assertRejected("not below zero", Noise.Battery.of(12, 0, Double.NaN));
        assertRejected("not below zero", Noise.Hand.of(-1, 0));
        assertRejected("finite", Noise.Hand.of(0, Double.POSITIVE_INFINITY));
        assertRejected("positive", Noise.Loop.of(0, 0, 0));
        assertRejected("not below zero", Noise.Loop.of(0.02, -1, 0));
        assertRejected("chance", Noise.Loop.of(0.02, 0, 1.5));
        assertRejected("chance", Noise.Loop.of(0.02, 0, Double.NaN));
        assertRejected("positive", Traction.of(0));
        assertRejected("positive", Traction.of(Double.NaN));
        assertEquals(
                Double.POSITIVE_INFINITY,
                Valid.value(Traction.of(Double.POSITIVE_INFINITY)).inPerS2(),
                0);
    }
}
