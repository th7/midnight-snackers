package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.Random;

/**
 * How a run's robot differs from the tuned model: the ways a real robot is not the one Road Runner
 * was tuned on, drawn once per run from a seed, so a run with a seed comes out the same every time
 * and two seeds are two robots. The noise is in the mechanisms only. What the sensors read is
 * still exactly what the robot did, so a difference between where the robot is and where its
 * localizer believes it is comes from the robot code, never from here.
 * <ul>
 * <li>Each drive motor's kS, kV and kA are within {@link #MOTOR_SPREAD} of the tuned values, each
 *     its own way, so no two wheels are quite alike and the robot drifts under equal power.
 * <li>The battery starts anywhere from flat to fresh, sags in proportion to the drive power
 *     commanded, and drains as the run goes on. The sensor reads it exactly, and the motors get
 *     what it reads.
 * <li>The floor gives only so much traction: a wheel that asks for more acceleration than that,
 *     driving or braking, slips, and the robot gets only that much.
 * <li>A robot set down is near the pose, not on it, as a hand puts it.
 * <li>The op mode's loop runs at about thirty hertz, not exactly, with the odd long loop.
 * </ul>
 * {@link #NONE} is the tuned model exactly, and the loop at {@link SimRunner#LOOP_SECONDS}: what
 * every run had before there was noise. The {@code with} methods replace one thing and keep the
 * rest, for a test that wants a particular robot rather than a drawn one.
 */
public final class SimNoise {
    /** Each drive motor's kS, kV and kA are within this much of the tuned values, either way. */
    public static final double MOTOR_SPREAD = 0.1;
    /** A battery straight off the charger reads this. */
    public static final double FRESHEST_VOLTS = 13.8;
    /** A battery a run might still start on. */
    public static final double FLATTEST_VOLTS = 12.0;
    /** The battery sags this many volts for each unit of drive power commanded, summed over the four motors. */
    public static final double SAG_VOLTS_PER_POWER = 0.2;
    /** The battery loses this many volts for each second of the run. */
    public static final double DRAIN_VOLTS_PER_SECOND = 0.004;

    public static final double GRAVITY_IN_PER_S2 = 386.09;
    /** The floor holds a wheel to at least this much acceleration, in g: mecanum rollers on a worn tile. */
    public static final double LEAST_TRACTION_G = 0.3;
    /** And at most this much: rollers on a fresh tile. */
    public static final double MOST_TRACTION_G = 0.6;
    /** A hand sets the robot down within about this much of the pose, in each of x and y. */
    public static final double PLACEMENT_INCHES = 0.5;
    /** And turned within about this much. */
    public static final double PLACEMENT_RADIANS = Math.toRadians(2);
    /** The robot's loop takes about this long: thirty hertz. */
    public static final double LOOP_SECONDS = 0.033;
    /** How much the loop period varies, as the standard deviation of its logarithm. */
    public static final double LOOP_SPREAD = 0.15;
    /** No loop is shorter than this. */
    public static final double LEAST_LOOP_SECONDS = 0.015;
    /** This often a loop is a hiccup: a bulk read, a telemetry flush or the camera getting in the way. */
    public static final double HICCUP_CHANCE = 0.02;

    public static final double SHORTEST_HICCUP_SECONDS = 0.08;
    public static final double LONGEST_HICCUP_SECONDS = 0.2;

    /** The drive motors, in the order the recording keeps their powers. */
    public static final int LEFT_FRONT = 0, RIGHT_FRONT = 1, LEFT_BACK = 2, RIGHT_BACK = 3;

    /** One drive motor as a factor on each of its tuned constants: 1 is the tuning. */
    public static final class Motor {
        public final double kS, kV, kA;

        public Motor(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
        }
    }

    public static final Motor TUNED = new Motor(1, 1, 1);

    /** The tuned model exactly, on the noise-free battery, with the loop at {@link SimRunner#LOOP_SECONDS}. */
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
    /** What the battery reads with nothing running, at the start of the run. */
    public final double freshVolts;

    public final double sagVoltsPerPower;
    public final double drainVoltsPerSecond;
    /** The most a wheel can accelerate the robot, driving or braking, in inches per second squared. */
    public final double tractionInPerS2;
    /** How far from the pose a hand sets the robot down, as a standard deviation. */
    public final double placementInches;

    public final double placementRadians;
    /** The loop period's median, the spread of its logarithm, and how often a loop is a hiccup. */
    public final double loopSeconds;

    public final double loopSpread;
    public final double hiccupChance;
    /** The draws made as the run goes: each loop's period, and where the robot is set down. */
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

    /** A robot drawn from the seed: the same robot for the same seed, every time. */
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

    /** The drive motor at {@code wheel}, one of {@link #LEFT_FRONT} to {@link #RIGHT_BACK}. */
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

    /** All four drive motors alike. */
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

    /**
     * What the battery reads {@code seconds} into the run with {@code totalPower} of drive power
     * commanded (the four motors' powers, each between 0 and 1, summed).
     */
    public double batteryVolts(double seconds, double totalPower) {
        return freshVolts - drainVoltsPerSecond * seconds - sagVoltsPerPower * totalPower;
    }

    /** Where a hand sets the robot down, asked to put it at {@code pose}. */
    public Pose2d placed(Pose2d pose) {
        if (placementInches == 0 && placementRadians == 0) {
            return pose;
        }
        return new Pose2d(
                pose.position.x + draws.nextGaussian() * placementInches,
                pose.position.y + draws.nextGaussian() * placementInches,
                pose.heading.toDouble() + draws.nextGaussian() * placementRadians);
    }

    /** The robot in a line, for a failure to name and a log to carry: the seed and everything drawn from it, in ASCII. */
    @Override
    public String toString() {
        StringBuilder motors = new StringBuilder();
        String[] names = {"lf", "rf", "lb", "rb"};
        for (int wheel = 0; wheel < 4; wheel++) {
            Motor motor = this.motors[wheel];
            motors.append(String.format(" %s kS x%.3f kV x%.3f kA x%.3f", names[wheel], motor.kS, motor.kV, motor.kA));
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

    /** How long the next op mode loop takes: how far the world moves before the loop after it. */
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
