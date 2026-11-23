package org.firstinspires.ftc.teamcode.simcore;

public final class Flywheels {
    static final double TURNING_FRACTION_OF_FREE = 0.05;

    private static final double SECONDS_PER_MINUTE = 60;

    public enum Side {
        FRONT(Mechanism.FLYWHEEL_FRONT, -1),
        BACK(Mechanism.FLYWHEEL_BACK, 1);

        private final Mechanism mechanism;
        private final double launchingCounterclockwise;

        Side(Mechanism mechanism, double launchingCounterclockwise) {
            this.mechanism = mechanism;
            this.launchingCounterclockwise = launchingCounterclockwise;
        }

        public Mechanism mechanism() {
            return mechanism;
        }
    }

    public enum Way {
        LAUNCHING,
        STILL,
        WRONG_WAY
    }

    public record Turning(Way way, double rpm, double surfaceInPerS) {
        public static Turning still() {
            return new Turning(Way.STILL, 0, 0);
        }
    }

    private final Feedforward front;
    private final Feedforward back;
    private final Flywheel.TicksPerTurn ticksPerTurn;
    private final double frontDiameterInches;
    private final double backDiameterInches;

    private Flywheels(
            Feedforward front,
            Feedforward back,
            Flywheel.TicksPerTurn ticksPerTurn,
            double frontDiameterInches,
            double backDiameterInches) {
        this.front = front;
        this.back = back;
        this.ticksPerTurn = ticksPerTurn;
        this.frontDiameterInches = frontDiameterInches;
        this.backDiameterInches = backDiameterInches;
    }

    public static Checked<Flywheels> of(
            Feedforward front,
            Feedforward back,
            Flywheel.TicksPerTurn ticksPerTurn,
            double frontDiameterInches,
            double backDiameterInches) {
        for (double diameter : new double[] {frontDiameterInches, backDiameterInches}) {
            if (!(diameter > 0 && Double.isFinite(diameter))) {
                return Checked.rejected(
                        "a flywheel's diameter is a positive, finite number of inches, not " + diameter);
            }
        }
        return Checked.ok(new Flywheels(front, back, ticksPerTurn, frontDiameterInches, backDiameterInches));
    }

    public Feedforward motor(Side side) {
        return side == Side.FRONT ? front : back;
    }

    public double diameterInches(Side side) {
        return side == Side.FRONT ? frontDiameterInches : backDiameterInches;
    }

    public Turning turning(Side side, double motorTicksPerSecond, MotorType type, boolean reversed) {
        double counterclockwise = reversed ? -motorTicksPerSecond : motorTicksPerSecond;
        double launching = counterclockwise * side.launchingCounterclockwise;
        double rpm = launching / ticksPerTurn.value() * SECONDS_PER_MINUTE;
        double surface = StrictMath.PI * diameterInches(side) * rpm / SECONDS_PER_MINUTE;
        double turningAtLeast = type.freeTicksPerSecond() * TURNING_FRACTION_OF_FREE;
        if (launching > turningAtLeast) {
            return new Turning(Way.LAUNCHING, rpm, surface);
        }
        if (launching < -turningAtLeast) {
            return new Turning(Way.WRONG_WAY, rpm, surface);
        }
        return new Turning(Way.STILL, rpm, surface);
    }

    public static double mismatch(Turning front, Turning back) {
        return front.surfaceInPerS() - back.surfaceInPerS();
    }

    @Override
    public String toString() {
        return "flywheels turned by front " + front + " and back " + back + ", " + ticksPerTurn + ", "
                + frontDiameterInches + " in and " + backDiameterInches + " in across";
    }
}
