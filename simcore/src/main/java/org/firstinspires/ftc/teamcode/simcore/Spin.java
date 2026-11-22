package org.firstinspires.ftc.teamcode.simcore;

public record Spin(double ticks, double ticksPerSecond) {
    public static final double NOMINAL_VOLTS = 12;
    public static final double SPIN_UP_SECONDS = 0.1;

    public static Spin still() {
        return new Spin(0, 0);
    }

    public Spin next(MotorType type, Power power, double volts, Seconds dt) {
        double free = type.freeTicksPerSecond() * power.value() * volts / NOMINAL_VOLTS;
        double velocity = free + (ticksPerSecond - free) * StrictMath.exp(-dt.value() / SPIN_UP_SECONDS);
        double turned = free * dt.value()
                + (ticksPerSecond - free) * SPIN_UP_SECONDS * (1 - StrictMath.exp(-dt.value() / SPIN_UP_SECONDS));
        return new Spin(ticks + turned, velocity);
    }

    public Count count() {
        return new Count(ticks, ticksPerSecond);
    }
}
