package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.simcore.ConvexPolygon;
import org.firstinspires.ftc.teamcode.simcore.Heading;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Placed;
import org.firstinspires.ftc.teamcode.simcore.Placement;
import org.firstinspires.ftc.teamcode.simcore.Pose;
import org.firstinspires.ftc.teamcode.simcore.Vec2;

public final class SimPlacement {
    public static final SimField FIELD = SimField.load();

    public static final double FIELD_SIZE_IN = FIELD.size;

    public static final double WALL_HEIGHT_IN = FIELD.wallHeight;

    public static final double ROBOT_SIZE_IN = 18;

    private static final Placement PLACEMENT = new Placement(
            Valid.value(Length.of(FIELD_SIZE_IN)), Valid.value(Length.of(ROBOT_SIZE_IN)), obstaclesOf(FIELD));

    public static Pose2d onTheField(Pose2d candidate) {
        return applied(candidate, PLACEMENT.onTheField(poseOf(candidate)));
    }

    public static Pose2d clearOfTheObstacles(Pose2d candidate) {
        return applied(candidate, PLACEMENT.clearOfTheObstacles(poseOf(candidate)));
    }

    private static List<ConvexPolygon> obstaclesOf(SimField field) {
        List<ConvexPolygon> obstacles = new ArrayList<>();
        for (SimField.Obstacle obstacle : field.obstacles) {
            List<Vec2> corners = new ArrayList<>();
            for (double[] corner : obstacle.footprint) {
                corners.add(new Vec2(corner[0], corner[1]));
            }
            obstacles.add(Valid.value(ConvexPolygon.of(corners)));
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
