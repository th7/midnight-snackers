package org.firstinspires.ftc.teamcode.simcore;

public record Vec2(double x, double y) {
    public Vec2 plus(Vec2 other) {
        return new Vec2(x + other.x, y + other.y);
    }

    public double dot(Vec2 other) {
        return x * other.x + y * other.y;
    }
}
