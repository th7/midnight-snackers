package org.firstinspires.ftc.teamcode.simcore;

import java.util.function.Function;

public final class Noise {
    public static final double MOTOR_SPREAD = 0.1;
    public static final double FRESHEST_VOLTS = 13.8;
    public static final double FLATTEST_VOLTS = 12.0;
    public static final double SAG_VOLTS_PER_POWER = 0.2;
    public static final double DRAIN_VOLTS_PER_SECOND = 0.004;
    public static final double LEAST_TRACTION_G = 0.3;
    public static final double MOST_TRACTION_G = 0.6;
    public static final double SET_DOWN_INCHES = 0.5;
    public static final double SET_DOWN_RADIANS = Math.toRadians(2);
    public static final double LOOP_SECONDS = 0.033;
    public static final double LOOP_SPREAD = 0.15;
    public static final double LEAST_LOOP_SECONDS = 0.015;
    public static final double HICCUP_CHANCE = 0.02;
    public static final double SHORTEST_HICCUP_SECONDS = 0.08;
    public static final double LONGEST_HICCUP_SECONDS = 0.2;

    private final long seed;
    private final PerWheel<Motor> motors;
    private final Battery battery;
    private final Traction traction;
    private final Hand hand;
    private final Loop loop;

    private Noise(long seed, PerWheel<Motor> motors, Battery battery, Traction traction, Hand hand, Loop loop) {
        this.seed = seed;
        this.motors = motors;
        this.battery = battery;
        this.traction = traction;
        this.hand = hand;
        this.loop = loop;
    }

    public static Noise exact(Battery battery, Loop loop) {
        return new Noise(0, PerWheel.all(Motor.tuned()), battery, Traction.unlimited(), new Hand(0, 0), loop);
    }

    public static Noise seeded(long seed) {
        Drawing drawing = new Drawing(Draws.seeded(seed));
        Motor leftFront = drawing.motor();
        Motor rightFront = drawing.motor();
        Motor leftBack = drawing.motor();
        Motor rightBack = drawing.motor();
        double freshVolts = FLATTEST_VOLTS + drawing.nextDouble() * (FRESHEST_VOLTS - FLATTEST_VOLTS);
        double traction = (LEAST_TRACTION_G + drawing.nextDouble() * (MOST_TRACTION_G - LEAST_TRACTION_G))
                * Flight.GRAVITY_IN_PER_S2;
        return new Noise(
                seed,
                new PerWheel<>(leftFront, rightFront, leftBack, rightBack),
                new Battery(freshVolts, SAG_VOLTS_PER_POWER, DRAIN_VOLTS_PER_SECOND),
                Traction.of(traction).orElse(Traction.unlimited()),
                new Hand(SET_DOWN_INCHES, SET_DOWN_RADIANS),
                new Loop(Seconds.of(LOOP_SECONDS).orElse(Seconds.zero()), LOOP_SPREAD, HICCUP_CHANCE));
    }

    public long seed() {
        return seed;
    }

    public PerWheel<Motor> motors() {
        return motors;
    }

    public Battery battery() {
        return battery;
    }

    public Traction traction() {
        return traction;
    }

    public Hand hand() {
        return hand;
    }

    public Loop loop() {
        return loop;
    }

    public Noise withMotor(Wheel wheel, Motor motor) {
        return new Noise(seed, motors.with(wheel, motor), battery, traction, hand, loop);
    }

    public Noise withMotors(Motor motor) {
        return new Noise(seed, PerWheel.all(motor), battery, traction, hand, loop);
    }

    public Noise withBattery(Battery battery) {
        return new Noise(seed, motors, battery, traction, hand, loop);
    }

    public Noise withTraction(Traction traction) {
        return new Noise(seed, motors, battery, traction, hand, loop);
    }

    public Noise withHand(Hand hand) {
        return new Noise(seed, motors, battery, traction, hand, loop);
    }

    public Noise withLoop(Loop loop) {
        return new Noise(seed, motors, battery, traction, hand, loop);
    }

    public Draws draws() {
        return Draws.seeded(seed);
    }

    public Drawn<Nudge> setDown(Draws draws) {
        if (hand.inches == 0 && hand.radians == 0) {
            return new Drawn<>(new Nudge.None(), draws);
        }
        Drawing drawing = new Drawing(draws);
        double x = drawing.nextGaussian() * hand.inches;
        double y = drawing.nextGaussian() * hand.inches;
        double radians = drawing.nextGaussian() * hand.radians;
        return drawing.drawn(new Nudge.By(new Vec2(x, y), radians));
    }

    public Drawn<Seconds> nextLoop(Draws draws) {
        if (loop.spread == 0 && loop.hiccupChance == 0) {
            return new Drawn<>(loop.period, draws);
        }
        Drawing drawing = new Drawing(draws);
        if (drawing.nextDouble() < loop.hiccupChance) {
            double hiccup =
                    SHORTEST_HICCUP_SECONDS + drawing.nextDouble() * (LONGEST_HICCUP_SECONDS - SHORTEST_HICCUP_SECONDS);
            return drawing.drawn(Seconds.of(hiccup).orElse(loop.period));
        }
        double period = loop.period.value() * Math.exp(drawing.nextGaussian() * loop.spread);
        double bounded = Math.max(LEAST_LOOP_SECONDS, Math.min(SHORTEST_HICCUP_SECONDS, period));
        return drawing.drawn(Seconds.of(bounded).orElse(loop.period));
    }

    public sealed interface Nudge permits Nudge.None, Nudge.By {
        <R> R fold(R asPut, Function<? super By, ? extends R> by);

        record None() implements Nudge {
            @Override
            public <R> R fold(R asPut, Function<? super By, ? extends R> by) {
                return asPut;
            }
        }

        record By(Vec2 offset, double radians) implements Nudge {
            @Override
            public <R> R fold(R asPut, Function<? super By, ? extends R> by) {
                return by.apply(this);
            }
        }
    }

    public static final class Motor {
        private final double kSVolts;
        private final double kVVoltSecondsPerTick;
        private final double kAVoltSecondsSquaredPerTick;

        private Motor(double kSVolts, double kVVoltSecondsPerTick, double kAVoltSecondsSquaredPerTick) {
            this.kSVolts = kSVolts;
            this.kVVoltSecondsPerTick = kVVoltSecondsPerTick;
            this.kAVoltSecondsSquaredPerTick = kAVoltSecondsSquaredPerTick;
        }

        public static Motor tuned() {
            return new Motor(1, 1, 1);
        }

        public static Checked<Motor> of(
                double kSVolts, double kVVoltSecondsPerTick, double kAVoltSecondsSquaredPerTick) {
            if (!(positiveAndFinite(kSVolts)
                    && positiveAndFinite(kVVoltSecondsPerTick)
                    && positiveAndFinite(kAVoltSecondsSquaredPerTick))) {
                return Checked.rejected("a motor's noise scales its kS, kV and kA each by a positive, finite factor,"
                        + " not " + kSVolts + ", " + kVVoltSecondsPerTick + " and " + kAVoltSecondsSquaredPerTick);
            }
            return Checked.ok(new Motor(kSVolts, kVVoltSecondsPerTick, kAVoltSecondsSquaredPerTick));
        }

        public double kSVolts() {
            return kSVolts;
        }

        public double kVVoltSecondsPerTick() {
            return kVVoltSecondsPerTick;
        }

        public double kAVoltSecondsSquaredPerTick() {
            return kAVoltSecondsSquaredPerTick;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Motor that
                    && Double.compare(kSVolts, that.kSVolts) == 0
                    && Double.compare(kVVoltSecondsPerTick, that.kVVoltSecondsPerTick) == 0
                    && Double.compare(kAVoltSecondsSquaredPerTick, that.kAVoltSecondsSquaredPerTick) == 0;
        }

        @Override
        public int hashCode() {
            return 31 * (31 * Double.hashCode(kSVolts) + Double.hashCode(kVVoltSecondsPerTick))
                    + Double.hashCode(kAVoltSecondsSquaredPerTick);
        }

        @Override
        public String toString() {
            return "kS x" + kSVolts + ", kV x" + kVVoltSecondsPerTick + ", kA x" + kAVoltSecondsSquaredPerTick;
        }
    }

    public static final class Battery {
        private final double freshVolts;
        private final double sagVoltsPerPower;
        private final double drainVoltsPerSecond;

        private Battery(double freshVolts, double sagVoltsPerPower, double drainVoltsPerSecond) {
            this.freshVolts = freshVolts;
            this.sagVoltsPerPower = sagVoltsPerPower;
            this.drainVoltsPerSecond = drainVoltsPerSecond;
        }

        public static Checked<Battery> of(double freshVolts, double sagVoltsPerPower, double drainVoltsPerSecond) {
            if (!positiveAndFinite(freshVolts)) {
                return Checked.rejected("a battery's fresh voltage is positive and finite, not " + freshVolts);
            }
            if (!(finiteAndNotBelowZero(sagVoltsPerPower) && finiteAndNotBelowZero(drainVoltsPerSecond))) {
                return Checked.rejected("a battery's sag and drain are finite and not below zero, not "
                        + sagVoltsPerPower + " and " + drainVoltsPerSecond);
            }
            return Checked.ok(new Battery(freshVolts, sagVoltsPerPower, drainVoltsPerSecond));
        }

        public double freshVolts() {
            return freshVolts;
        }

        public double sagVoltsPerPower() {
            return sagVoltsPerPower;
        }

        public double drainVoltsPerSecond() {
            return drainVoltsPerSecond;
        }

        public double volts(Seconds elapsed, PerWheel<Power> drive) {
            double totalPower = Math.abs(drive.leftFront().value())
                    + Math.abs(drive.rightFront().value())
                    + Math.abs(drive.leftBack().value())
                    + Math.abs(drive.rightBack().value());
            return freshVolts - drainVoltsPerSecond * elapsed.value() - sagVoltsPerPower * totalPower;
        }
    }

    public static final class Hand {
        private final double inches;
        private final double radians;

        private Hand(double inches, double radians) {
            this.inches = inches;
            this.radians = radians;
        }

        public static Checked<Hand> of(double inches, double radians) {
            if (!(finiteAndNotBelowZero(inches) && finiteAndNotBelowZero(radians))) {
                return Checked.rejected("a hand sets a robot down off by a distance and an angle that are finite and"
                        + " not below zero, not " + inches + " in and " + radians + " rad");
            }
            return Checked.ok(new Hand(inches, radians));
        }

        public double inches() {
            return inches;
        }

        public double radians() {
            return radians;
        }
    }

    public static final class Loop {
        private final Seconds period;
        private final double spread;
        private final double hiccupChance;

        private Loop(Seconds period, double spread, double hiccupChance) {
            this.period = period;
            this.spread = spread;
            this.hiccupChance = hiccupChance;
        }

        public static Checked<Loop> of(double seconds, double spread, double hiccupChance) {
            if (!positiveAndFinite(seconds)) {
                return Checked.rejected("a loop's period is a positive, finite time, not " + seconds);
            }
            if (!finiteAndNotBelowZero(spread)) {
                return Checked.rejected("a loop's spread is finite and not below zero, not " + spread);
            }
            if (!(hiccupChance >= 0 && hiccupChance <= 1)) {
                return Checked.rejected("a hiccup's chance is between 0 and 1, not " + hiccupChance);
            }
            return Seconds.of(seconds).map(period -> new Loop(period, spread, hiccupChance));
        }

        public Seconds period() {
            return period;
        }

        public double spread() {
            return spread;
        }

        public double hiccupChance() {
            return hiccupChance;
        }
    }

    private static boolean positiveAndFinite(double value) {
        return value > 0 && Double.isFinite(value);
    }

    private static boolean finiteAndNotBelowZero(double value) {
        return value >= 0 && Double.isFinite(value);
    }

    private static final class Drawing {
        private Draws draws;

        Drawing(Draws draws) {
            this.draws = draws;
        }

        double nextDouble() {
            Drawn<Double> drawn = draws.nextDouble();
            draws = drawn.next();
            return drawn.value();
        }

        double nextGaussian() {
            Drawn<Double> drawn = draws.nextGaussian();
            draws = drawn.next();
            return drawn.value();
        }

        Motor motor() {
            double kSVolts = spread();
            double kVVoltSecondsPerTick = spread();
            double kAVoltSecondsSquaredPerTick = spread();
            return new Motor(kSVolts, kVVoltSecondsPerTick, kAVoltSecondsSquaredPerTick);
        }

        double spread() {
            return 1 + (2 * nextDouble() - 1) * MOTOR_SPREAD;
        }

        <T> Drawn<T> drawn(T value) {
            return new Drawn<>(value, draws);
        }
    }
}
