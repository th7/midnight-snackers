package org.firstinspires.ftc.teamcode.simcore;

public final class DeadWheels {
    public static final int PAR_RAW_SIGN = 1;
    public static final int PERP_RAW_SIGN = 1;
    public static final double PAR_Y_TICKS = -610.3360642343223;
    public static final double PERP_X_TICKS = -5186.563208836955;

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

    public static Checked<DeadWheels> reginalds(double inPerTick) {
        return of(inPerTick, PAR_Y_TICKS, PERP_X_TICKS);
    }

    public DeadWheels moved(Twist moved) {
        return new DeadWheels(
                inPerTick,
                parYTicks,
                perpXTicks,
                parTicks + (moved.line().x() / inPerTick + parYTicks * moved.angle()),
                perpTicks + (moved.line().y() / inPerTick + perpXTicks * moved.angle()));
    }

    public Reading read(Twist perSecond, Sense parPortsMotor, Sense perpPortsMotor) {
        double parVelocity = perSecond.line().x() / inPerTick + parYTicks * perSecond.angle();
        double perpVelocity = perSecond.line().y() / inPerTick + perpXTicks * perSecond.angle();
        return new Reading(
                Encoder.asTheHubReports(
                        parPortsMotor.of(PAR_RAW_SIGN * parTicks), parPortsMotor.of(PAR_RAW_SIGN * parVelocity)),
                Encoder.asTheHubReports(
                        perpPortsMotor.of(PERP_RAW_SIGN * perpTicks), perpPortsMotor.of(PERP_RAW_SIGN * perpVelocity)));
    }
}
