package org.firstinspires.ftc.teamcode.simcore;

public final class Bounce {
    private final double restitution;
    private final double friction;
    private final double grip;

    private Bounce(double restitution, double friction, double grip) {
        this.restitution = restitution;
        this.friction = friction;
        this.grip = grip;
    }

    public static Checked<Bounce> of(double restitution, double friction, double grip) {
        if (!(restitution >= 0 && restitution <= 1)) {
            return Checked.rejected("a ball keeps between none and all of the speed it meets a surface at, so a"
                    + " bounce's restitution is between 0 and 1, not " + restitution);
        }
        if (!(friction >= 0 && Double.isFinite(friction))) {
            return Checked.rejected("a surface's friction is a finite number not below 0, not " + friction);
        }
        if (!(grip >= 0 && grip <= 1)) {
            return Checked.rejected("a bounce turns back between none and all of the ball's slip, so its grip is"
                    + " between 0 and 1, not " + grip);
        }
        return Checked.ok(new Bounce(restitution, friction, grip));
    }

    public double restitution() {
        return restitution;
    }

    public double friction() {
        return friction;
    }

    public double grip() {
        return grip;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Bounce that
                && Double.compare(restitution, that.restitution) == 0
                && Double.compare(friction, that.friction) == 0
                && Double.compare(grip, that.grip) == 0;
    }

    @Override
    public int hashCode() {
        return (Double.hashCode(restitution) * 31 + Double.hashCode(friction)) * 31 + Double.hashCode(grip);
    }

    @Override
    public String toString() {
        return "a bounce keeping " + restitution + " of the approach, friction " + friction + ", grip " + grip;
    }
}
