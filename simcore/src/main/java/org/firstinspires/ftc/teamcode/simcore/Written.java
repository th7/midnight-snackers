package org.firstinspires.ftc.teamcode.simcore;

public sealed interface Written {
    record Powered(double power) implements Written {}

    record AtVelocity(double ticksPerSecond) implements Written {}
}
