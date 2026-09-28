package org.firstinspires.ftc.teamcode.simcore;

public final class Length {
    public static final double METRES_PER_INCH = 0.0254;

    private final double inches;

    private Length(double inches) {
        this.inches = inches;
    }

    public static Checked<Length> of(double inches) {
        if (!(inches > 0 && Double.isFinite(inches))) {
            return Checked.rejected("a length is a positive, finite number of inches, not " + inches);
        }
        return Checked.ok(new Length(inches));
    }

    public double inches() {
        return inches;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Length that && Double.compare(inches, that.inches) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(inches);
    }

    @Override
    public String toString() {
        return inches + " in";
    }
}
