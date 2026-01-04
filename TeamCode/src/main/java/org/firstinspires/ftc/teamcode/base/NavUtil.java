package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class NavUtil {
    public static double angleRadians(Vector2d from, Vector2d to) {
        return Math.atan2(to.y - from.y, to.x - from.x);
    }

    public static double distanceInches(Vector2d from, Vector2d to) {
        return Math.sqrt(Math.pow(to.y - from.y, 2) + Math.pow(to.x - from.x, 2));
    }

    public static Vector2d pointAtDistanceInDirection(Vector2d from, double distance, double directionRadians) {
        double newX = from.x + Math.cos(directionRadians) * distance;
        double newY = from.y + Math.sin(directionRadians) * distance;
        return new Vector2d(newX, newY);
    }

    public static Pose2d nearestPoseAtDistanceFromTarget(Vector2d from, Vector2d to, double distance) {
        double bearingToTarget = NavUtil.angleRadians(from, to);
        double distanceToTarget = NavUtil.distanceInches(from, to);
        double distanceError = distanceToTarget - distance;
        Vector2d position = NavUtil.pointAtDistanceInDirection(from, distanceError, bearingToTarget);
        return new Pose2d(position, bearingToTarget);
    }
}
