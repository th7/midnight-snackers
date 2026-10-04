package org.firstinspires.ftc.teamcode.simcore;

public final class Turntable {
    public static final double TICKS_PER_SECOND_AT_FULL_POWER = 1700;

    private final int ticksPerRevolution;
    private final double ticksPerSecondAtFullPower;

    private Turntable(int ticksPerRevolution, double ticksPerSecondAtFullPower) {
        this.ticksPerRevolution = ticksPerRevolution;
        this.ticksPerSecondAtFullPower = ticksPerSecondAtFullPower;
    }

    public static Checked<Turntable> of(int ticksPerRevolution, Constants constants) {
        if (ticksPerRevolution <= 0) {
            return Checked.rejected(
                    "a turntable turns a positive number of ticks a revolution, not " + ticksPerRevolution);
        }
        return Checked.ok(new Turntable(ticksPerRevolution, constants.value(Constant.TURNTABLE_SPEED)));
    }

    public int turned(int ticks, Power power, Seconds dt) {
        return ticks + (int) Math.round(power.value() * ticksPerSecondAtFullPower * dt.value());
    }

    public double radians(int ticks) {
        return (double) ticks / ticksPerRevolution * 2 * Math.PI;
    }
}
