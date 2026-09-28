package org.firstinspires.ftc.teamcode.simcore;

public final class Chassis {
    public static final double SIZE_IN = 18;
    public static final double INTAKE_REACH_IN = 0.25;

    private Chassis() {}

    public static boolean againstTheFront(Heading heading, Vec2 offset, Length radius) {
        Vec2 seen = heading.onTheRobot(offset);
        double half = SIZE_IN / 2;
        return seen.x() > 0 && seen.x() - radius.inches() <= half + INTAKE_REACH_IN && Math.abs(seen.y()) <= half;
    }
}
