package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Where the field lets a robot be. One question -- {@link #onTheField} -- over the season's field
 * and an eighteen-inch square: a pose beyond a wall or inside an obstacle comes back pushed
 * against it, at the same heading.
 *
 * <p>This is geometry, not physics. It has no world, no bodies, no time and no balls, so placing a
 * pose costs a polygon overlap rather than a rigid-body engine. It used to live inside
 * {@link SimRobot}, which meant that dragging the robot on the placement page, encoding a tick's
 * hive tilts and templating the replay page each loaded fifteen hundred lines of simulator and a
 * dozen dyn4j classes to answer a question none of them was asking. {@code SimPlacementTest.placingAPoseNeedsNeitherTheRigidBodyEngineNorTheSimulatedRobot}
 * holds that seam: it loads this class with dyn4j and {@link SimRobot} both forbidden, and
 * checks the same loader still refuses {@link SimRobot}, so the gate cannot pass by being toothless.
 */
public final class SimPlacement {
    /** The season's field: its walls, the elements the robot runs into, and the hives' cells. */
    public static final SimField FIELD = SimField.load();
    /** The field is a square of this many inches between the walls, centred on the origin. */
    public static final double FIELD_SIZE_IN = FIELD.size;
    /**
     * The walls are this many inches high. A driving robot never goes over them; a flying ball
     * that clears one is out of the field.
     */
    public static final double WALL_HEIGHT_IN = FIELD.wallHeight;
    /**
     * The robot is a cube of this many inches on a side, centred on its pose and standing on the
     * floor. Only its footprint collides, with the walls, the obstacles and the balls.
     */
    public static final double ROBOT_SIZE_IN = 18;

    /**
     * The pose the field allows: {@link #clearOfTheObstacles clear of the obstacles} and
     * {@link #insideTheWalls inside the walls}, in that order, so a robot pushed out of an
     * obstacle at the wall still ends inside the field.
     */
    public static Pose2d onTheField(Pose2d candidate) {
        return insideTheWalls(clearOfTheObstacles(candidate));
    }

    /**
     * The pose the walls allow: the same heading, and the position pushed back just far enough that
     * no corner of the robot's square is beyond a wall. A wall is a straight line, so the square
     * reaches it at half its side scaled by how far the heading is from square-on. Each axis is
     * clamped on its own, which is what lets the robot slide along a wall it drives into at an
     * angle.
     */
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

    /**
     * The pose the field elements allow: the same heading, and the position pushed the shortest
     * way out of any obstacle the robot's square overlaps. That push is along the face it hit, so
     * a robot driving into an obstacle at an angle slides along it. Two passes, so a push out of
     * one obstacle into its neighbour (a leg into its foot) is undone too.
     */
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

    /** The robot's footprint: its square's corners, counter-clockwise, at a position and heading. */
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

    /**
     * The shortest move that takes the robot's footprint out of a convex obstacle, or null when
     * they do not overlap: the separating axis theorem over both polygons' edge normals, keeping
     * the axis they overlap least along.
     */
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
        // Push the robot away from the obstacle, whichever way along the axis that is.
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
