package org.firstinspires.ftc.teamcode.simcore;

public record Count(double ticks, double ticksPerSecond) {
    public static Count none() {
        return new Count(0, 0);
    }
}
