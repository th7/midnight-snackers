package org.firstinspires.ftc.teamcode.simcore;

public final class Launch {
    public enum Why {
        FRONT_WHEEL_STILL,
        FRONT_WHEEL_WRONG_WAY,
        BACK_WHEEL_STILL,
        BACK_WHEEL_WRONG_WAY,
        TOO_FAST
    }

    public record Robot(Vec2 at, Heading heading, Vec2 velocity, double turnRadiansPerSecond) {}

    public sealed interface Decision permits Launches, Stays {}

    public record Launches(WheelPair.Launched launched) implements Decision {}

    public record Stays(Why why) implements Decision {}

    private final Flywheels flywheels;
    private final Vec3 exitOnTheRobot;
    private final double pitchDegrees;
    private final double rollDegrees;

    Launch(Flywheels flywheels, Vec3 exitOnTheRobot, double pitchDegrees, double rollDegrees) {
        this.flywheels = flywheels;
        this.exitOnTheRobot = exitOnTheRobot;
        this.pitchDegrees = pitchDegrees;
        this.rollDegrees = rollDegrees;
    }

    public Checked<Decision> decide(Flywheels.Turning front, Flywheels.Turning back, Length pollenRadius, Robot robot) {
        if (front.way() == Flywheels.Way.STILL) {
            return Checked.ok(new Stays(Why.FRONT_WHEEL_STILL));
        }
        if (front.way() == Flywheels.Way.WRONG_WAY) {
            return Checked.ok(new Stays(Why.FRONT_WHEEL_WRONG_WAY));
        }
        if (back.way() == Flywheels.Way.STILL) {
            return Checked.ok(new Stays(Why.BACK_WHEEL_STILL));
        }
        if (back.way() == Flywheels.Way.WRONG_WAY) {
            return Checked.ok(new Stays(Why.BACK_WHEEL_WRONG_WAY));
        }
        Vec2 offset = robot.heading().onTheField(new Vec2(exitOnTheRobot.x(), exitOnTheRobot.y()));
        Vec3 exit = new Vec3(robot.at().x() + offset.x(), robot.at().y() + offset.y(), exitOnTheRobot.z());
        double turning = robot.turnRadiansPerSecond();
        Vec2 carried = new Vec2(
                robot.velocity().x() - turning * offset.y(), robot.velocity().y() + turning * offset.x());
        double yawDegrees = StrictMath.toDegrees(
                StrictMath.atan2(robot.heading().sin(), robot.heading().cos()));
        ShotSettings field = ShotSettings.defaults(Constants.defaults());
        return Ball.of(pollenRadius, field.value(ShotSetting.MASS), field.value(ShotSetting.INERTIA))
                .then(pollen -> WheelPair.Flywheel.of(flywheels.diameterInches(Flywheels.Side.BACK), back.rpm())
                        .then(one -> WheelPair.Flywheel.of(flywheels.diameterInches(Flywheels.Side.FRONT), front.rpm())
                                .then(two -> ShotSetup.wheels(
                                        field,
                                        pollen,
                                        exit,
                                        new ShotSetup.Angles(yawDegrees, pitchDegrees, rollDegrees),
                                        one,
                                        two,
                                        carried))))
                .map(pair -> leaving(pair.launch()));
    }

    private static Decision leaving(WheelPair.Launched launched) {
        Vec3 velocity = launched.velocity();
        if (!(StrictMath.sqrt(velocity.dot(velocity)) <= Flight.MOST_SPEED_IN_PER_S)) {
            return new Stays(Why.TOO_FAST);
        }
        return new Launches(launched);
    }

    @Override
    public String toString() {
        return "a launch by " + flywheels + " from " + exitOnTheRobot + " on the robot, pitched " + pitchDegrees
                + " deg and rolled " + rollDegrees + " deg";
    }
}
