package org.firstinspires.ftc.teamcode.simcore;

public record Sides<T>(T left, T right) {
    public PerWheel<T> wheels() {
        return new PerWheel<>(left, right, left, right);
    }
}
