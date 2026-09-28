package org.firstinspires.ftc.teamcode.simcore;

public final class Pose {
    private final Vec2 position;
    private final Heading heading;

    private Pose(Vec2 position, Heading heading) {
        this.position = position;
        this.heading = heading;
    }

    public static Checked<Pose> of(double x, double y, Heading heading) {
        if (!(Double.isFinite(x) && Double.isFinite(y))) {
            return Checked.rejected("a pose is at a finite position, not (" + x + ", " + y + ")");
        }
        return Checked.ok(new Pose(new Vec2(x, y), heading));
    }

    public Vec2 position() {
        return position;
    }

    public Heading heading() {
        return heading;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Pose that && position.equals(that.position) && heading.equals(that.heading);
    }

    @Override
    public int hashCode() {
        return 31 * position.hashCode() + heading.hashCode();
    }

    @Override
    public String toString() {
        return position + " facing " + heading;
    }
}
