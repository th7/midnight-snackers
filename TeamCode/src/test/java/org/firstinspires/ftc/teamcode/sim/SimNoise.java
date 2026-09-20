package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.Random;

public final class SimNoise {
    public static final double MOTOR_SPREAD = 0.1;

    public static final double FRESHEST_VOLTS = 13.8;

    public static final double FLATTEST_VOLTS = 12.0;

    public static final double SAG_VOLTS_PER_POWER = 0.2;

    public static final double DRAIN_VOLTS_PER_SECOND = 0.004;

    public static final double GRAVITY_IN_PER_S2 = 386.09;

    public static final double LEAST_TRACTION_G = 0.3;

    public static final double MOST_TRACTION_G = 0.6;

    public static final double PLACEMENT_INCHES = 0.5;

    public static final double PLACEMENT_RADIANS = Math.toRadians(2);

    public static final double LOOP_SECONDS = 0.033;

    public static final double LOOP_SPREAD = 0.15;

    public static final double LEAST_LOOP_SECONDS = 0.015;

    public static final double HICCUP_CHANCE = 0.02;

    public static final double SHORTEST_HICCUP_SECONDS = 0.08;
    public static final double LONGEST_HICCUP_SECONDS = 0.2;

    public static final int LEFT_FRONT = 0, RIGHT_FRONT = 1, LEFT_BACK = 2, RIGHT_BACK = 3;

    public static final class Motor {
        public final double kSVolts, kVVoltSecondsPerTick, kAVoltSecondsSquaredPerTick;

        public Motor(double kSVolts, double kVVoltSecondsPerTick, double kAVoltSecondsSquaredPerTick) {
            this.kSVolts = kSVolts;
            this.kVVoltSecondsPerTick = kVVoltSecondsPerTick;
            this.kAVoltSecondsSquaredPerTick = kAVoltSecondsSquaredPerTick;
        }
    }

    public static final Motor TUNED = new Motor(1, 1, 1);

    public static final SimNoise NONE = new SimNoise(
            0,
            new Motor[] {TUNED, TUNED, TUNED, TUNED},
            new Battery(SimRobot.BATTERY_VOLTS, 0, 0),
            Double.POSITIVE_INFINITY,
            new Placement(0, 0),
            new Loop(SimRunner.LOOP_SECONDS, 0, 0));

    public static final class Battery {
        public final double freshVolts;
        public final double sagVoltsPerPower;
        public final double drainVoltsPerSecond;

        public Battery(double freshVolts, double sagVoltsPerPower, double drainVoltsPerSecond) {
            this.freshVolts = freshVolts;
            this.sagVoltsPerPower = sagVoltsPerPower;
            this.drainVoltsPerSecond = drainVoltsPerSecond;
        }
    }

    public static final class Placement {
        public final double inches;
        public final double radians;

        public Placement(double inches, double radians) {
            this.inches = inches;
            this.radians = radians;
        }
    }

    public static final class Loop {
        public final double seconds;
        public final double spread;
        public final double hiccupChance;

        public Loop(double seconds, double spread, double hiccupChance) {
            this.seconds = seconds;
            this.spread = spread;
            this.hiccupChance = hiccupChance;
        }
    }

    public final long seed;
    private final Motor[] motors;

    public final Battery battery;

    public final double tractionInPerS2;

    public final Placement placement;

    public final Loop loop;

    private final Random draws;

    private SimNoise(
            long seed, Motor[] motors, Battery battery, double tractionInPerS2, Placement placement, Loop loop) {
        this.seed = seed;
        this.motors = motors.clone();
        this.battery = battery;
        this.tractionInPerS2 = tractionInPerS2;
        this.placement = placement;
        this.loop = loop;
        this.draws = new Random(seed);
    }

    public static SimNoise seeded(long seed) {
        Random random = new Random(seed);
        Motor[] motors = new Motor[4];
        for (int wheel = 0; wheel < motors.length; wheel++) {
            motors[wheel] = new Motor(spread(random), spread(random), spread(random));
        }
        double freshVolts = FLATTEST_VOLTS + random.nextDouble() * (FRESHEST_VOLTS - FLATTEST_VOLTS);
        double traction =
                (LEAST_TRACTION_G + random.nextDouble() * (MOST_TRACTION_G - LEAST_TRACTION_G)) * GRAVITY_IN_PER_S2;
        return new SimNoise(
                seed,
                motors,
                new Battery(freshVolts, SAG_VOLTS_PER_POWER, DRAIN_VOLTS_PER_SECOND),
                traction,
                new Placement(PLACEMENT_INCHES, PLACEMENT_RADIANS),
                new Loop(LOOP_SECONDS, LOOP_SPREAD, HICCUP_CHANCE));
    }

    private static double spread(Random random) {
        return 1 + (2 * random.nextDouble() - 1) * MOTOR_SPREAD;
    }

    public Motor motor(int wheel) {
        return motors[wheel];
    }

    public SimNoise withMotor(int wheel, Motor motor) {
        Motor[] replaced = motors.clone();
        replaced[wheel] = motor;
        return new SimNoise(seed, replaced, battery, tractionInPerS2, placement, loop);
    }

    public SimNoise withMotors(Motor motor) {
        return new SimNoise(seed, new Motor[] {motor, motor, motor, motor}, battery, tractionInPerS2, placement, loop);
    }

    public SimNoise withBattery(Battery battery) {
        return new SimNoise(seed, motors, battery, tractionInPerS2, placement, loop);
    }

    public SimNoise withTraction(double inPerS2) {
        return new SimNoise(seed, motors, battery, inPerS2, placement, loop);
    }

    public SimNoise withPlacement(Placement placement) {
        return new SimNoise(seed, motors, battery, tractionInPerS2, placement, loop);
    }

    public SimNoise withLoop(Loop loop) {
        return new SimNoise(seed, motors, battery, tractionInPerS2, placement, loop);
    }

    public double batteryVolts(double seconds, double totalPower) {
        return battery.freshVolts - battery.drainVoltsPerSecond * seconds - battery.sagVoltsPerPower * totalPower;
    }

    public Pose2d placed(Pose2d pose) {
        if (placement.inches == 0 && placement.radians == 0) {
            return pose;
        }
        return new Pose2d(
                pose.position.x + draws.nextGaussian() * placement.inches,
                pose.position.y + draws.nextGaussian() * placement.inches,
                pose.heading.toDouble() + draws.nextGaussian() * placement.radians);
    }

    @Override
    public String toString() {
        StringBuilder motors = new StringBuilder();
        String[] names = {"lf", "rf", "lb", "rb"};
        for (int wheel = 0; wheel < 4; wheel++) {
            Motor motor = this.motors[wheel];
            motors.append(String.format(
                    " %s kSVolts x%.3f kVVoltSecondsPerTick x%.3f kAVoltSecondsSquaredPerTick x%.3f",
                    names[wheel], motor.kSVolts, motor.kVVoltSecondsPerTick, motor.kAVoltSecondsSquaredPerTick));
        }
        return String.format(
                "seed %d:%s; battery %.2f V sag %.2f V/power drain %.4f V/s; traction %.2f g;"
                        + " placement +-%.2f in +-%.1f deg; loop %.0f ms spread %.2f hiccups %.0f%%",
                seed,
                motors,
                battery.freshVolts,
                battery.sagVoltsPerPower,
                battery.drainVoltsPerSecond,
                tractionInPerS2 / GRAVITY_IN_PER_S2,
                placement.inches,
                Math.toDegrees(placement.radians),
                loop.seconds * 1000,
                loop.spread,
                loop.hiccupChance * 100);
    }

    public double nextLoopSeconds() {
        if (loop.spread == 0 && loop.hiccupChance == 0) {
            return loop.seconds;
        }
        if (draws.nextDouble() < loop.hiccupChance) {
            return SHORTEST_HICCUP_SECONDS + draws.nextDouble() * (LONGEST_HICCUP_SECONDS - SHORTEST_HICCUP_SECONDS);
        }
        double period = loop.seconds * Math.exp(draws.nextGaussian() * loop.spread);
        return Math.max(LEAST_LOOP_SECONDS, Math.min(SHORTEST_HICCUP_SECONDS, period));
    }
}
