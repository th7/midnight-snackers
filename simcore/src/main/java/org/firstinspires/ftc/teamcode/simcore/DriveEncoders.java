package org.firstinspires.ftc.teamcode.simcore;

public final class DriveEncoders {
    private final Sides<Sense> mounted;
    private final double inPerTick;
    private final double sideOffsetTicks;
    private final Sides<Double> aheadTicks;

    private DriveEncoders(Sides<Sense> mounted, double inPerTick, double sideOffsetTicks, Sides<Double> aheadTicks) {
        this.mounted = mounted;
        this.inPerTick = inPerTick;
        this.sideOffsetTicks = sideOffsetTicks;
        this.aheadTicks = aheadTicks;
    }

    public static Checked<DriveEncoders> of(Sides<Sense> mounted, double inPerTick, double sideOffsetTicks) {
        if (!(inPerTick > 0 && Double.isFinite(inPerTick))) {
            return Checked.rejected(
                    "a drive motor turns its side a positive, finite distance a tick, not " + inPerTick + " in");
        }
        if (!Double.isFinite(sideOffsetTicks)) {
            return Checked.rejected("each side is a finite number of ticks off the centre, not " + sideOffsetTicks);
        }
        return Checked.ok(new DriveEncoders(mounted, inPerTick, sideOffsetTicks, new Sides<>(0.0, 0.0)));
    }

    public DriveEncoders moved(Twist moved) {
        Sides<Double> turned = aheadOf(moved);
        return new DriveEncoders(
                mounted,
                inPerTick,
                sideOffsetTicks,
                new Sides<>(aheadTicks.left() + turned.left(), aheadTicks.right() + turned.right()));
    }

    public Sides<Encoder> read(Twist perSecond, Sides<Sense> directions) {
        Sides<Double> speed = aheadOf(perSecond);
        return new Sides<>(
                asReported(mounted.left(), directions.left(), aheadTicks.left(), speed.left()),
                asReported(mounted.right(), directions.right(), aheadTicks.right(), speed.right()));
    }

    private Sides<Double> aheadOf(Twist twist) {
        double ahead = twist.line().x() / inPerTick;
        double turning = sideOffsetTicks * twist.angle();
        return new Sides<>(ahead - turning, ahead + turning);
    }

    private static Encoder asReported(Sense mounted, Sense direction, double ticks, double ticksPerSecond) {
        return Encoder.asTheHubReports(direction.of(mounted.of(ticks)), direction.of(mounted.of(ticksPerSecond)));
    }
}
