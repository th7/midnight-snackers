package org.firstinspires.ftc.teamcode.simcore;

public final class Heading {
    private static final double UNIT_TOLERANCE = 1e-9;

    private final double cos;
    private final double sin;

    private Heading(double cos, double sin) {
        this.cos = cos;
        this.sin = sin;
    }

    public static Checked<Heading> of(double cos, double sin) {
        if (!(Math.abs(Math.hypot(cos, sin) - 1) <= UNIT_TOLERANCE)) {
            return Checked.rejected("a heading is a unit vector, not (" + cos + ", " + sin + ")");
        }
        return Checked.ok(new Heading(cos, sin));
    }

    public static Checked<Heading> ofRadians(double radians) {
        if (!Double.isFinite(radians)) {
            return Checked.rejected("a heading is a finite angle, not " + radians);
        }
        return of(Math.cos(radians), Math.sin(radians));
    }

    public double cos() {
        return cos;
    }

    public double sin() {
        return sin;
    }

    public Vec2 onTheRobot(Vec2 onTheField) {
        return new Vec2(cos * onTheField.x() + sin * onTheField.y(), -sin * onTheField.x() + cos * onTheField.y());
    }

    public Vec2 onTheField(Vec2 onTheRobot) {
        return new Vec2(cos * onTheRobot.x() - sin * onTheRobot.y(), sin * onTheRobot.x() + cos * onTheRobot.y());
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Heading that
                && Double.compare(cos, that.cos) == 0
                && Double.compare(sin, that.sin) == 0;
    }

    @Override
    public int hashCode() {
        return 31 * Double.hashCode(cos) + Double.hashCode(sin);
    }

    @Override
    public String toString() {
        return "(" + cos + ", " + sin + ")";
    }
}
