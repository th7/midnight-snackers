package org.firstinspires.ftc.teamcode.simcore;

public final class Flywheel {
    public static final String MOTOR = "goBILDA 5203-2402-0001, 1:1: 6,000 rpm with no load at 12 V, an encoder of 28"
            + " counts a turn of its output shaft,"
            + " https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/";
    public static final double MOTOR_RPM = 6000;
    public static final double MOTOR_TICKS_PER_TURN = 28;
    public static final double TOP_TICKS_PER_SECOND = MOTOR_RPM / 60 * MOTOR_TICKS_PER_TURN;
    public static final double WHOLE_TURN = 2 * StrictMath.PI;

    public static final class TicksPerTurn {
        public static final String RANGE_SOURCE = "doc/plans/runtime-flight.md, stage 4: the flywheel's ticks per turn"
                + " runs from 18, at which the fastest throw off the fastest robot is still one a flight takes, to"
                + " 10000";
        public static final double LEAST = 18;
        public static final double MOST = 10000;

        private final double value;

        private TicksPerTurn(double value) {
            this.value = value;
        }

        public static Checked<TicksPerTurn> of(double ticks) {
            if (!(ticks >= LEAST && ticks <= MOST)) {
                return Checked.rejected("a flywheel turns once for " + LEAST + " to " + MOST
                        + " ticks of its motor, gearing included, not " + ticks);
            }
            return Checked.ok(new TicksPerTurn(ticks));
        }

        public double value() {
            return value;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof TicksPerTurn that && Double.compare(value, that.value) == 0;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(value);
        }

        @Override
        public String toString() {
            return value + " ticks a turn";
        }
    }

    public static final class AngularSpeed {
        private final double radiansPerSecond;

        private AngularSpeed(double radiansPerSecond) {
            this.radiansPerSecond = radiansPerSecond;
        }

        public static AngularSpeed from(double commandedTicksPerSecond, TicksPerTurn ticksPerTurn) {
            return new AngularSpeed(turnsAt(commandedTicksPerSecond) * WHOLE_TURN / ticksPerTurn.value());
        }

        static AngularSpeed stopped() {
            return new AngularSpeed(0);
        }

        public double radiansPerSecond() {
            return radiansPerSecond;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof AngularSpeed that && Double.compare(radiansPerSecond, that.radiansPerSecond) == 0;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(radiansPerSecond);
        }

        @Override
        public String toString() {
            return radiansPerSecond + " rad/s";
        }
    }

    private Flywheel() {}

    public static double turnsAt(double commandedTicksPerSecond) {
        if (Double.isNaN(commandedTicksPerSecond)) {
            return 0;
        }
        return StrictMath.max(-TOP_TICKS_PER_SECOND, StrictMath.min(TOP_TICKS_PER_SECOND, commandedTicksPerSecond));
    }
}
