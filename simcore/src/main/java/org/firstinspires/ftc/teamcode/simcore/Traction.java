package org.firstinspires.ftc.teamcode.simcore;

public final class Traction {
    private final double inPerS2;

    private Traction(double inPerS2) {
        this.inPerS2 = inPerS2;
    }

    public static Checked<Traction> of(double inPerS2) {
        if (!(inPerS2 > 0)) {
            return Checked.rejected("traction is a positive acceleration in inches a second squared, or Infinity for"
                    + " a floor that never lets a wheel slip, not " + inPerS2);
        }
        return Checked.ok(new Traction(inPerS2));
    }

    public static Traction unlimited() {
        return new Traction(Double.POSITIVE_INFINITY);
    }

    public double inPerS2() {
        return inPerS2;
    }

    public double gives(double asked) {
        return Math.max(-inPerS2, Math.min(inPerS2, asked));
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Traction that && Double.compare(inPerS2, that.inPerS2) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(inPerS2);
    }

    @Override
    public String toString() {
        return inPerS2 + " in/s^2";
    }
}
