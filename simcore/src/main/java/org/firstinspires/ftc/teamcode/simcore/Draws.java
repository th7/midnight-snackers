package org.firstinspires.ftc.teamcode.simcore;

public final class Draws {
    private static final long MULTIPLIER = 0x5DEECE66DL;
    private static final long ADDEND = 0xBL;
    private static final long MASK = (1L << 48) - 1;
    private static final int STATE_BITS = 48;
    private static final int HIGH_BITS = 26;
    private static final int LOW_BITS = 27;
    private static final double DOUBLE_UNIT = 0x1.0p-53;
    private static final int MOST_TRIES_FOR_A_PAIR = 64;

    private final long state;
    private final Spare spare;

    private sealed interface Spare permits NoSpare, SpareGaussian {}

    private record NoSpare() implements Spare {}

    private record SpareGaussian(double value) implements Spare {}

    private Draws(long state, Spare spare) {
        this.state = state;
        this.spare = spare;
    }

    public static Draws seeded(long seed) {
        return new Draws((seed ^ MULTIPLIER) & MASK, new NoSpare());
    }

    public Drawn<Double> nextDouble() {
        long high = advanced(state);
        long low = advanced(high);
        double value = ((bits(high, HIGH_BITS) << LOW_BITS) + bits(low, LOW_BITS)) * DOUBLE_UNIT;
        return new Drawn<>(value, new Draws(low, spare));
    }

    public Drawn<Double> nextGaussian() {
        if (spare instanceof SpareGaussian held) {
            return new Drawn<>(held.value(), new Draws(state, new NoSpare()));
        }
        Draws draws = this;
        for (int tries = 0; tries < MOST_TRIES_FOR_A_PAIR; tries++) {
            Drawn<Double> first = draws.nextDouble();
            Drawn<Double> second = first.next().nextDouble();
            draws = second.next();
            double v1 = 2 * first.value() - 1;
            double v2 = 2 * second.value() - 1;
            double s = v1 * v1 + v2 * v2;
            if (s < 1 && s != 0) {
                double multiplier = StrictMath.sqrt(-2 * StrictMath.log(s) / s);
                return new Drawn<>(v1 * multiplier, new Draws(draws.state, new SpareGaussian(v2 * multiplier)));
            }
        }
        return new Drawn<>(0.0, draws);
    }

    private static long advanced(long state) {
        return (state * MULTIPLIER + ADDEND) & MASK;
    }

    private static long bits(long state, int count) {
        return state >>> (STATE_BITS - count);
    }
}
