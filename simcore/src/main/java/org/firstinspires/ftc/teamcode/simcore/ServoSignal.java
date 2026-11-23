package org.firstinspires.ftc.teamcode.simcore;

public sealed interface ServoSignal permits ServoSignal.None, ServoSignal.Pulse {
    static ServoSignal none() {
        return new None();
    }

    record None() implements ServoSignal {}

    final class Pulse implements ServoSignal {
        private final double microseconds;

        Pulse(double microseconds) {
            this.microseconds = microseconds;
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
