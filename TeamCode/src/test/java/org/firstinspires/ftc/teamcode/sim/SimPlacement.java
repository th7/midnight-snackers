package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class SimPlacement {
    public static final SimField FIELD = SimField.load();

    public static final double FIELD_SIZE_IN = FIELD.size;

    public static final double WALL_HEIGHT_IN = FIELD.wallHeight;

    public static final double ROBOT_SIZE_IN = 18;

    public static Pose2d onTheField(Pose2d candidate) {
        return insideTheWalls(clearOfTheObstacles(candidate));
    }

    public static Pose2d insideTheWalls(Pose2d candidate) {
        double reach = ROBOT_SIZE_IN / 2 * (Math.abs(candidate.heading.real) + Math.abs(candidate.heading.imag));
        double limit = FIELD_SIZE_IN / 2 - reach;
        double x = Math.max(-limit, Math.min(limit, candidate.position.x));
        double y = Math.max(-limit, Math.min(limit, candidate.position.y));
        if (x == candidate.position.x && y == candidate.position.y) {
            return candidate;
        }
        return new Pose2d(new Vector2d(x, y), candidate.heading);
    }

    public static Pose2d clearOfTheObstacles(Pose2d candidate) {
        Vector2d position = candidate.position;
        for (int pass = 0; pass < 2; pass++) {
            for (SimField.Obstacle obstacle : FIELD.obstacles) {
                Vector2d push = pushOutOf(obstacle.footprint, corners(position, candidate.heading));
                if (push != null) {
                    position = position.plus(push);
                }
            }
        }
        if (position == candidate.position) {
            return candidate;
        }
        return new Pose2d(position, candidate.heading);
    }

    private static double[][] corners(Vector2d position, Rotation2d heading) {
        double h = ROBOT_SIZE_IN / 2;
        double[][] corners = new double[4][];
        double[][] local = {{h, h}, {-h, h}, {-h, -h}, {h, -h}};
        for (int i = 0; i < 4; i++) {
            corners[i] = new double[] {
                position.x + local[i][0] * heading.real - local[i][1] * heading.imag,
                position.y + local[i][0] * heading.imag + local[i][1] * heading.real
            };
        }
        return corners;
    }

    private static Vector2d pushOutOf(double[][] obstacle, double[][] robot) {
        double leastOverlap = Double.POSITIVE_INFINITY;
        double[] leastAxis = null;
        for (double[][] polygon : new double[][][] {obstacle, robot}) {
            for (int i = 0; i < polygon.length; i++) {
                double[] a = polygon[i], b = polygon[(i + 1) % polygon.length];
                double length = Math.hypot(b[0] - a[0], b[1] - a[1]);
                if (length == 0) {
                    continue;
                }
                double[] axis = {(a[1] - b[1]) / length, (b[0] - a[0]) / length};
                double[] robotSpan = span(robot, axis), obstacleSpan = span(obstacle, axis);
                double overlap = Math.min(robotSpan[1] - obstacleSpan[0], obstacleSpan[1] - robotSpan[0]);
                if (overlap <= 0) {
                    return null;
                }
                if (overlap < leastOverlap) {
                    leastOverlap = overlap;
                    leastAxis = axis;
                }
            }
        }

        double[] robotCentre = centre(robot), obstacleCentre = centre(obstacle);
        double side = (robotCentre[0] - obstacleCentre[0]) * leastAxis[0]
                + (robotCentre[1] - obstacleCentre[1]) * leastAxis[1];
        double sign = side < 0 ? -1 : 1;
        return new Vector2d(sign * leastAxis[0] * leastOverlap, sign * leastAxis[1] * leastOverlap);
    }

    private static double[] span(double[][] polygon, double[] axis) {
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (double[] p : polygon) {
            double along = p[0] * axis[0] + p[1] * axis[1];
            min = Math.min(min, along);
            max = Math.max(max, along);
        }
        return new double[] {min, max};
    }

    private static double[] centre(double[][] polygon) {
        double x = 0, y = 0;
        for (double[] p : polygon) {
            x += p[0];
            y += p[1];
        }
        return new double[] {x / polygon.length, y / polygon.length};
    }
}
