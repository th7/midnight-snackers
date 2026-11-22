package org.firstinspires.ftc.teamcode.simcore;

public final class VelocityLoop {
    public static final double P_POWER_PER_TICK_PER_SECOND = 4e-4;
    public static final double I_POWER_PER_TICK = 4e-3;

    private final double integral;

    private VelocityLoop(double integral) {
        this.integral = integral;
    }

    public record Stepped(Power power, VelocityLoop loop) {}

    public static VelocityLoop idle() {
        return new VelocityLoop(0);
    }

    public Stepped toward(double targetTicksPerSecond, Count measured, Seconds dt) {
        double measuredTicksPerSecond = Encoder.asTheHubReports(measured.ticks(), measured.ticksPerSecond())
                .velocity();
        double error = targetTicksPerSecond - measuredTicksPerSecond;
        double wound =
                Power.clamped(integral + I_POWER_PER_TICK * error * dt.value()).value();
        return new Stepped(Power.clamped(P_POWER_PER_TICK_PER_SECOND * error + wound), new VelocityLoop(wound));
    }
}
