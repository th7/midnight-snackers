package org.firstinspires.ftc.teamcode.simcore;

public final class Seconds {
    private final double value;

    private Seconds(double value) {
        this.value = value;
    }

    public static Seconds zero() {
        return new Seconds(0);
    }

    public static Checked<Seconds> of(double seconds) {
        if (!(seconds >= 0 && Double.isFinite(seconds))) {
            return Checked.rejected("a time is a finite number of seconds, not below zero: " + seconds);
        }
        return Checked.ok(new Seconds(seconds));
    }

    public double value() {
        return value;
    }

    public Seconds plus(Seconds other) {
        return new Seconds(Math.min(Double.MAX_VALUE, value + other.value));
    }

    public Seconds dividedInto(int parts) {
        return new Seconds(value / Math.max(1, parts));
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Seconds that && Double.compare(value, that.value) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(value);
    }

    @Override
    public String toString() {
        return value + " s";
    }
}
