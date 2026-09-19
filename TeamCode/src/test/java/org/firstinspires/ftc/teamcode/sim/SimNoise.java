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
            SimRobot.BATTERY_VOLTS,
            0,
            0,
            Double.POSITIVE_INFINITY,
            0,
            0,
            SimRunner.LOOP_SECONDS,
            0,
            0);

    public final long seed;
    private final Motor[] motors;

    public final double freshVolts;

    public final double sagVoltsPerPower;
    public final double drainVoltsPerSecond;

    public final double tractionInPerS2;

    public final double placementInches;

    public final double placementRadians;

    public final double loopSeconds;

    public final double loopSpread;
    public final double hiccupChance;

    private final Random draws;

    private SimNoise(
            long seed,
            Motor[] motors,
            double freshVolts,
            double sagVoltsPerPower,
            double drainVoltsPerSecond,
            double tractionInPerS2,
            double placementInches,
            double placementRadians,
            double loopSeconds,
            double loopSpread,
            double hiccupChance) {
        this.seed = seed;
        this.motors = motors.clone();
        this.freshVolts = freshVolts;
        this.sagVoltsPerPower = sagVoltsPerPower;
        this.drainVoltsPerSecond = drainVoltsPerSecond;
        this.tractionInPerS2 = tractionInPerS2;
        this.placementInches = placementInches;
        this.placementRadians = placementRadians;
        this.loopSeconds = loopSeconds;
        this.loopSpread = loopSpread;
        this.hiccupChance = hiccupChance;
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
                freshVolts,
                SAG_VOLTS_PER_POWER,
                DRAIN_VOLTS_PER_SECOND,
                traction,
                PLACEMENT_INCHES,
                PLACEMENT_RADIANS,
                LOOP_SECONDS,
                LOOP_SPREAD,
                HICCUP_CHANCE);
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
        return new SimNoise(
                seed,
                replaced,
                freshVolts,
                sagVoltsPerPower,
                drainVoltsPerSecond,
                tractionInPerS2,
                placementInches,
                placementRadians,
                loopSeconds,
                loopSpread,
                hiccupChance);
    }

    public SimNoise withMotors(Motor motor) {
        return new SimNoise(
                seed,
                new Motor[] {motor, motor, motor, motor},
                freshVolts,
                sagVoltsPerPower,
                drainVoltsPerSecond,
                tractionInPerS2,
                placementInches,
                placementRadians,
                loopSeconds,
                loopSpread,
                hiccupChance);
    }

    public SimNoise withBattery(double freshVolts, double sagVoltsPerPower, double drainVoltsPerSecond) {
        return new SimNoise(
                seed,
                motors,
                freshVolts,
                sagVoltsPerPower,
                drainVoltsPerSecond,
                tractionInPerS2,
                placementInches,
                placementRadians,
                loopSeconds,
                loopSpread,
                hiccupChance);
    }

    public SimNoise withTraction(double inPerS2) {
        return new SimNoise(
                seed,
                motors,
                freshVolts,
                sagVoltsPerPower,
                drainVoltsPerSecond,
                inPerS2,
                placementInches,
                placementRadians,
                loopSeconds,
                loopSpread,
                hiccupChance);
    }

    public SimNoise withPlacement(double inches, double radians) {
        return new SimNoise(
                seed,
                motors,
                freshVolts,
                sagVoltsPerPower,
                drainVoltsPerSecond,
                tractionInPerS2,
                inches,
                radians,
                loopSeconds,
                loopSpread,
                hiccupChance);
    }

    public SimNoise withLoop(double seconds, double spread, double hiccupChance) {
        return new SimNoise(
                seed,
                motors,
                freshVolts,
                sagVoltsPerPower,
                drainVoltsPerSecond,
                tractionInPerS2,
                placementInches,
                placementRadians,
                seconds,
                spread,
                hiccupChance);
    }

    public double batteryVolts(double seconds, double totalPower) {
        return freshVolts - drainVoltsPerSecond * seconds - sagVoltsPerPower * totalPower;
    }

    public Pose2d placed(Pose2d pose) {
        if (placementInches == 0 && placementRadians == 0) {
            return pose;
        }
        return new Pose2d(
                pose.position.x + draws.nextGaussian() * placementInches,
                pose.position.y + draws.nextGaussian() * placementInches,
                pose.heading.toDouble() + draws.nextGaussian() * placementRadians);
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
                freshVolts,
                sagVoltsPerPower,
                drainVoltsPerSecond,
                tractionInPerS2 / GRAVITY_IN_PER_S2,
                placementInches,
                Math.toDegrees(placementRadians),
                loopSeconds * 1000,
                loopSpread,
                hiccupChance * 100);
    }

    public double nextLoopSeconds() {
        if (loopSpread == 0 && hiccupChance == 0) {
            return loopSeconds;
        }
        if (draws.nextDouble() < hiccupChance) {
            return SHORTEST_HICCUP_SECONDS + draws.nextDouble() * (LONGEST_HICCUP_SECONDS - SHORTEST_HICCUP_SECONDS);
        }
        double period = loopSeconds * Math.exp(draws.nextGaussian() * loopSpread);
        return Math.max(LEAST_LOOP_SECONDS, Math.min(SHORTEST_HICCUP_SECONDS, period));
    }
}
