package org.firstinspires.ftc.teamcode.simcore;

public record Launch(Vec3 at, Vec3 velocity) {
    public static final double HEIGHT_IN = 14;
    public static final double AHEAD_IN = 6;
    public static final double ANGLE_RADIANS = Math.toRadians(70);
    public static final double IN_PER_S_PER_TICK_PER_S = 0.189;

    public static Launch from(
            Vec2 robotAt,
            double headingRadians,
            double turntableRadians,
            double flywheelTicksPerSecond,
            Vec2 robotVelocity,
            Constants constants) {
        double aim = headingRadians + turntableRadians;
        double speed = Math.abs(flywheelTicksPerSecond) * constants.value(Constant.LAUNCH_THROW);
        return new Launch(
                new Vec3(robotAt.x() + AHEAD_IN * Math.cos(aim), robotAt.y() + AHEAD_IN * Math.sin(aim), HEIGHT_IN),
                new Vec3(
                        robotVelocity.x() + speed * Math.cos(ANGLE_RADIANS) * Math.cos(aim),
                        robotVelocity.y() + speed * Math.cos(ANGLE_RADIANS) * Math.sin(aim),
                        speed * Math.sin(ANGLE_RADIANS)));
    }
}
