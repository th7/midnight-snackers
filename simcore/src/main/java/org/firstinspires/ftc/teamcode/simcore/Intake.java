package org.firstinspires.ftc.teamcode.simcore;

public final class Intake {
    public static final int HOLDS = 2;

    static final double TURNING_FRACTION_OF_FREE = 0.05;

    public enum Way {
        IN,
        OUT,
        STILL
    }

    public static final class Held {
        private final int count;

        private Held(int count) {
            this.count = count;
        }

        public static Held none() {
            return new Held(0);
        }

        public Held plus(int more) {
            if (more <= 0) {
                return this;
            }
            return new Held(more >= HOLDS - count ? HOLDS : count + more);
        }

        public Held less(int fewer) {
            if (fewer <= 0) {
                return this;
            }
            return new Held(fewer >= count ? 0 : count - fewer);
        }

        public int count() {
            return count;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Held that && count == that.count;
        }

        @Override
        public int hashCode() {
            return count;
        }

        @Override
        public String toString() {
            return count + " held";
        }
    }

    public record Taken(Held held, int taken) {}

    private Intake() {}

    public static Way way(double motorTicksPerSecond, MotorType type, boolean reversed) {
        double inward = reversed ? -motorTicksPerSecond : motorTicksPerSecond;
        double turning = type.freeTicksPerSecond() * TURNING_FRACTION_OF_FREE;
        if (inward > turning) {
            return Way.IN;
        }
        if (inward < -turning) {
            return Way.OUT;
        }
        return Way.STILL;
    }

    public static Taken take(Way way, Held held, int touching) {
        if (way != Way.IN) {
            return new Taken(held, 0);
        }
        Held now = held.plus(touching);
        return new Taken(now, now.count() - held.count());
    }
}
