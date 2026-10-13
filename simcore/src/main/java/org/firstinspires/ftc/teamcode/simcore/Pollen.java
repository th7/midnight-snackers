package org.firstinspires.ftc.teamcode.simcore;

public final class Pollen {
    public static final double THIN_SHELL = 2.0 / 3;
    public static final double GRAMS_PER_KILOGRAM = 1000;

    private final double radius;
    private final double kilograms;
    private final double inertiaFactor;

    private Pollen(double radius, double kilograms, double inertiaFactor) {
        this.radius = radius;
        this.kilograms = kilograms;
        this.inertiaFactor = inertiaFactor;
    }

    public static Checked<Pollen> of(double diameterInches, double grams, double inertiaFactor) {
        if (!(diameterInches > 0 && Double.isFinite(diameterInches))) {
            return Checked.rejected(
                    "a pollen's diameter is a positive, finite number of inches, not " + diameterInches);
        }
        if (!(grams > 0 && Double.isFinite(grams))) {
            return Checked.rejected("a pollen weighs a positive, finite number of grams, not " + grams);
        }
        if (!(inertiaFactor > 0 && inertiaFactor <= THIN_SHELL)) {
            return Checked.rejected("a sphere's moment of inertia is more than 0 and at most 2/3 of m r^2, the thin"
                    + " shell's, so its inertia factor is not " + inertiaFactor);
        }
        return Checked.ok(new Pollen(diameterInches / 2, grams / GRAMS_PER_KILOGRAM, inertiaFactor));
    }

    public double radius() {
        return radius;
    }

    public double kilograms() {
        return kilograms;
    }

    public double inertiaFactor() {
        return inertiaFactor;
    }

    public double radiusMetres() {
        return radius * Length.METRES_PER_INCH;
    }

    public double areaSquareMetres() {
        double r = radiusMetres();
        return StrictMath.PI * r * r;
    }

    public double volumeCubicMetres() {
        double r = radiusMetres();
        return 4.0 / 3 * StrictMath.PI * r * r * r;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Pollen that
                && Double.compare(radius, that.radius) == 0
                && Double.compare(kilograms, that.kilograms) == 0
                && Double.compare(inertiaFactor, that.inertiaFactor) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(radius) * 31 * 31 + Double.hashCode(kilograms) * 31 + Double.hashCode(inertiaFactor);
    }

    @Override
    public String toString() {
        return "a pollen " + radius * 2 + " in across, " + kilograms * GRAMS_PER_KILOGRAM + " g, inertia "
                + inertiaFactor + " m r^2";
    }
}
