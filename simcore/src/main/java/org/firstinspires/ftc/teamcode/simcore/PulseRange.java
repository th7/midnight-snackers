package org.firstinspires.ftc.teamcode.simcore;

public final class PulseRange {
    private final double lowerUs;
    private final double upperUs;

    private PulseRange(double lowerUs, double upperUs) {
        this.lowerUs = lowerUs;
        this.upperUs = upperUs;
    }

    public static Checked<PulseRange> of(double lowerUs, double upperUs) {
        if (!(lowerUs >= 0 && lowerUs <= upperUs && Double.isFinite(upperUs))) {
            return Checked.rejected("a pulse range is finite microseconds from its lower to its upper, not "
                    + said(lowerUs) + " to " + said(upperUs));
        }
        return Checked.ok(new PulseRange(lowerUs, upperUs));
    }

    private static String said(double microseconds) {
        return microseconds == StrictMath.rint(microseconds) && StrictMath.abs(microseconds) < 1e15
                ? Long.toString((long) microseconds)
                : Double.toString(microseconds);
    }

    double lowerUs() {
        return lowerUs;
    }

    double upperUs() {
        return upperUs;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof PulseRange that
                && Double.compare(lowerUs, that.lowerUs) == 0
                && Double.compare(upperUs, that.upperUs) == 0;
    }

    @Override
    public int hashCode() {
        return 31 * Double.hashCode(lowerUs) + Double.hashCode(upperUs);
    }

    @Override
    public String toString() {
        return said(lowerUs) + " to " + said(upperUs) + " us";
    }
}
