package org.firstinspires.ftc.teamcode.simcore;

public final class DeadWheels {
    public static final int PAR_RAW_SIGN = 1;
    public static final int PERP_RAW_SIGN = -1;

    private final double inPerTick;
    private final double parYTicks;
    private final double perpXTicks;
    private final double parTicks;
    private final double perpTicks;

    private DeadWheels(double inPerTick, double parYTicks, double perpXTicks, double parTicks, double perpTicks) {
        this.inPerTick = inPerTick;
        this.parYTicks = parYTicks;
        this.perpXTicks = perpXTicks;
        this.parTicks = parTicks;
        this.perpTicks = perpTicks;
    }

    public record Reading(Encoder par, Encoder perp) {}

    public static Checked<DeadWheels> of(double inPerTick, double parYTicks, double perpXTicks) {
        if (!(inPerTick > 0 && Double.isFinite(inPerTick))) {
            return Checked.rejected("a dead wheel turns a positive, finite distance a tick, not " + inPerTick + " in");
        }
        if (!(Double.isFinite(parYTicks) && Double.isFinite(perpXTicks))) {
            return Checked.rejected("the dead wheels are a finite number of ticks off the centre, not " + parYTicks
                    + " and " + perpXTicks);
        }
        return Checked.ok(new DeadWheels(inPerTick, parYTicks, perpXTicks, 0, 0));
    }

    public DeadWheels moved(Twist moved) {
        return new DeadWheels(
                inPerTick,
                parYTicks,
                perpXTicks,
                parTicks + (moved.line().x() / inPerTick + parYTicks * moved.angle()),
                perpTicks + (moved.line().y() / inPerTick + perpXTicks * moved.angle()));
    }

    public Reading read(Twist perSecond) {
        double parVelocity = perSecond.line().x() / inPerTick + parYTicks * perSecond.angle();
        double perpVelocity = perSecond.line().y() / inPerTick + perpXTicks * perSecond.angle();
        return new Reading(
                Encoder.asTheHubReports(PAR_RAW_SIGN * parTicks, PAR_RAW_SIGN * parVelocity),
                Encoder.asTheHubReports(PERP_RAW_SIGN * perpTicks, PERP_RAW_SIGN * perpVelocity));
    }
}
