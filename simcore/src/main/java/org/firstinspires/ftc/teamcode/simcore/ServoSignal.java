package org.firstinspires.ftc.teamcode.simcore;

public sealed interface ServoSignal permits ServoSignal.None, ServoSignal.Pulse {
    static ServoSignal none() {
        return new None();
    }

    static Checked<ServoSignal> of(boolean driven, double position, double lowerUs, double upperUs) {
        if (!(position >= 0 && position <= 1)) {
            return Checked.rejected("a servo position is from 0 to 1, not " + position);
        }
        if (!(lowerUs >= 0 && lowerUs <= upperUs && Double.isFinite(upperUs))) {
            return Checked.rejected("a pulse range is finite microseconds from its lower to its upper, not "
                    + Pulse.said(lowerUs) + " to " + Pulse.said(upperUs));
        }
        if (!driven) {
            return Checked.ok(none());
        }
        return Checked.ok(new Pulse(lowerUs + position * (upperUs - lowerUs)));
    }

    record None() implements ServoSignal {}

    final class Pulse implements ServoSignal {
        private final double microseconds;

        private Pulse(double microseconds) {
            this.microseconds = microseconds;
        }

        static String said(double microseconds) {
            return microseconds == StrictMath.rint(microseconds) && StrictMath.abs(microseconds) < 1e15
                    ? Long.toString((long) microseconds)
                    : Double.toString(microseconds);
        }

        public double microseconds() {
            return microseconds;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Pulse that && Double.compare(microseconds, that.microseconds) == 0;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(microseconds);
        }

        @Override
        public String toString() {
            return microseconds + " us";
        }
    }
}
