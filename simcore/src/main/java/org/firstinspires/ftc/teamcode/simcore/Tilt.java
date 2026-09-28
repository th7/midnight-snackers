package org.firstinspires.ftc.teamcode.simcore;

public final class Tilt {
    public static final double MOST_DEGREES = 90;

    private final double degrees;

    private Tilt(double degrees) {
        this.degrees = degrees;
    }

    public static Checked<Tilt> of(double degrees) {
        if (!(Math.abs(degrees) <= MOST_DEGREES)) {
            return Checked.rejected("a tilt is within " + MOST_DEGREES + " degrees of level, not " + degrees);
        }
        return Checked.ok(new Tilt(degrees));
    }

    public double degrees() {
        return degrees;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Tilt that && Double.compare(degrees, that.degrees) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(degrees);
    }

    @Override
    public String toString() {
        return degrees + " degrees";
    }
}
