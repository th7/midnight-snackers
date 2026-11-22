package org.firstinspires.ftc.teamcode.simcore;

public final class DeadWheels {
    public static final int PAR_RAW_SIGN = 1;
    public static final int PERP_RAW_SIGN = 1;

    private final double inPerTick;
    private final double parYInches;
    private final double perpXInches;
    private final double parTicks;
    private final double perpTicks;

    private DeadWheels(double inPerTick, double parYInches, double perpXInches, double parTicks, double perpTicks) {
        this.inPerTick = inPerTick;
        this.parYInches = parYInches;
        this.perpXInches = perpXInches;
        this.parTicks = parTicks;
        this.perpTicks = perpTicks;
    }

    public record Counts(Count par, Count perp) {}

    public static Checked<DeadWheels> at(double inPerTick, double parYInches, double perpXInches) {
        if (!(inPerTick > 0 && Double.isFinite(inPerTick))) {
            return Checked.rejected("a dead wheel turns a positive, finite distance a tick, not " + inPerTick + " in");
        }
        if (!(Double.isFinite(parYInches) && Double.isFinite(perpXInches))) {
            return Checked.rejected("the dead wheels sit a finite distance from the centre, not " + parYInches
                    + " and " + perpXInches + " in");
        }
        return Checked.ok(new DeadWheels(inPerTick, parYInches, perpXInches, 0, 0));
    }

    public DeadWheels moved(Twist moved) {
        return new DeadWheels(
                inPerTick,
                parYInches,
                perpXInches,
                parTicks + parTicks(moved),
                perpTicks + perpTicks(moved));
    }

    public Counts counts(Twist perSecond) {
        double parVelocity = parTicks(perSecond);
        double perpVelocity = perpTicks(perSecond);
        return new Counts(
                new Count(PAR_RAW_SIGN * parTicks, PAR_RAW_SIGN * parVelocity),
                new Count(PERP_RAW_SIGN * perpTicks, PERP_RAW_SIGN * perpVelocity));
    }

    private double parTicks(Twist moved) {
        return (moved.line().x() - parYInches * moved.angle()) / inPerTick;
    }

    private double perpTicks(Twist moved) {
        return (moved.line().y() + perpXInches * moved.angle()) / inPerTick;
    }
}
