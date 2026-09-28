package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.Chassis;
import org.firstinspires.ftc.teamcode.simcore.ConvexPolygon;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Heading;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Placed;
import org.firstinspires.ftc.teamcode.simcore.Placement;
import org.firstinspires.ftc.teamcode.simcore.Pose;

public final class SimPlacement {
    private static final SimField.Loaded SEASON = SimField.load();

    public static final Field FIELD = SEASON.field();

    public static final JsonObject FIELD_JSON = SEASON.page();

    public static final double FIELD_SIZE_IN = FIELD.size();

    public static final double WALL_HEIGHT_IN = FIELD.wallHeight();

    public static final double ROBOT_SIZE_IN = Chassis.SIZE_IN;

    private static final Placement PLACEMENT = new Placement(
            Valid.value(Length.of(FIELD_SIZE_IN)), Valid.value(Length.of(ROBOT_SIZE_IN)), obstaclesOf(FIELD));

    public static Pose2d onTheField(Pose2d candidate) {
        return applied(candidate, PLACEMENT.onTheField(poseOf(candidate)));
    }

    public static Pose2d clearOfTheObstacles(Pose2d candidate) {
        return applied(candidate, PLACEMENT.clearOfTheObstacles(poseOf(candidate)));
    }

    private static List<ConvexPolygon> obstaclesOf(Field field) {
        List<ConvexPolygon> obstacles = new ArrayList<>();
        for (Field.Obstacle obstacle : field.obstacles()) {
            obstacles.add(obstacle.footprint());
        }
        return obstacles;
    }

    private static Pose poseOf(Pose2d pose) {
        return Valid.value(Heading.of(pose.heading.real, pose.heading.imag)
                .then(heading -> Pose.of(pose.position.x, pose.position.y, heading)));
    }

    private static Pose2d applied(Pose2d candidate, Placed placed) {
        if (placed instanceof Placed.Moved moved) {
            return new Pose2d(new Vector2d(moved.to().x(), moved.to().y()), candidate.heading);
        }
        return candidate;
    }
}
