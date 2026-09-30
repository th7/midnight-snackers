package org.firstinspires.ftc.teamcode.simcore;

public record Encoder(int position, double velocity) {
    public static final double HUB_VELOCITY_STEP_TICKS_PER_S = 20;

    public static Encoder asTheHubReports(double ticks, double ticksPerSecond) {
        return new Encoder(
                (int) Math.round(ticks),
                Math.round(ticksPerSecond / HUB_VELOCITY_STEP_TICKS_PER_S) * HUB_VELOCITY_STEP_TICKS_PER_S);
    }
}
