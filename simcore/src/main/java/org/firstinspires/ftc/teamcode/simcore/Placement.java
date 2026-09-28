package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public final class Placement {
    private static final int PASSES = 2;

    private final double fieldSize;
    private final double robotSize;
    private final List<ConvexPolygon> obstacles;

    public Placement(Length field, Length robot, List<ConvexPolygon> obstacles) {
        this.fieldSize = field.inches();
        this.robotSize = robot.inches();
        this.obstacles = Collections.unmodifiableList(new ArrayList<>(obstacles));
    }

    public Placed onTheField(Pose candidate) {
        Placed clear = clearOfTheObstacles(candidate);
        return insideTheWalls(clear.from(candidate.position()), candidate.heading())
                .after(clear);
    }

    public Placed insideTheWalls(Vec2 position, Heading heading) {
        double reach = robotSize / 2 * (Math.abs(heading.cos()) + Math.abs(heading.sin()));
        double limit = fieldSize / 2 - reach;
        double x = Math.max(-limit, Math.min(limit, position.x()));
        double y = Math.max(-limit, Math.min(limit, position.y()));
        if (x == position.x() && y == position.y()) {
            return new Placed.AsGiven();
        }
        return new Placed.Moved(new Vec2(x, y));
    }

    public Placed clearOfTheObstacles(Pose candidate) {
        Vec2 position = candidate.position();
        Placed placed = new Placed.AsGiven();
        for (int pass = 0; pass < PASSES; pass++) {
            for (ConvexPolygon obstacle : obstacles) {
                Vec2 from = position;
                Optional<Vec2> pushed = pushOutOf(obstacle.ring(), corners(from, candidate.heading()))
                        .map(push -> from.plus(push));
                placed = pushed.<Placed>map(Placed.Moved::new).orElse(placed);
                position = pushed.orElse(position);
            }
        }
        return placed;
    }

    private Ring<Vec2> corners(Vec2 position, Heading heading) {
        double h = robotSize / 2;
        return Ring.of(
                corner(position, heading, h, h),
                List.of(
                        corner(position, heading, -h, h),
                        corner(position, heading, -h, -h),
                        corner(position, heading, h, -h)));
    }

    private static Vec2 corner(Vec2 position, Heading heading, double ahead, double left) {
        return new Vec2(
                position.x() + ahead * heading.cos() - left * heading.sin(),
                position.y() + ahead * heading.sin() + left * heading.cos());
    }

    private static Optional<Vec2> pushOutOf(Ring<Vec2> obstacle, Ring<Vec2> robot) {
        double leastOverlap = Double.POSITIVE_INFINITY;
        Optional<Vec2> leastAxis = Optional.empty();
        for (Ring<Vec2> polygon : List.of(obstacle, robot)) {
            for (Ring.Edge<Vec2> edge : polygon.edges()) {
                Vec2 a = edge.from(), b = edge.to();
                double length = Math.hypot(b.x() - a.x(), b.y() - a.y());
                if (length == 0) {
                    continue;
                }
                Vec2 axis = new Vec2((a.y() - b.y()) / length, (b.x() - a.x()) / length);
                Span robotSpan = Span.of(robot, axis), obstacleSpan = Span.of(obstacle, axis);
                double overlap = Math.min(robotSpan.max - obstacleSpan.min, obstacleSpan.max - robotSpan.min);
                if (overlap <= 0) {
                    return Optional.empty();
                }
                if (overlap < leastOverlap) {
                    leastOverlap = overlap;
                    leastAxis = Optional.of(axis);
                }
            }
        }
        Vec2 robotCentre = centre(robot), obstacleCentre = centre(obstacle);
        double least = leastOverlap;
        return leastAxis.map(axis -> {
            double side = (robotCentre.x() - obstacleCentre.x()) * axis.x()
                    + (robotCentre.y() - obstacleCentre.y()) * axis.y();
            double sign = side < 0 ? -1 : 1;
            return new Vec2(sign * axis.x() * least, sign * axis.y() * least);
        });
    }

    private record Span(double min, double max) {
        static Span of(Ring<Vec2> polygon, Vec2 axis) {
            double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
            for (Vec2 p : polygon.all()) {
                double along = p.x() * axis.x() + p.y() * axis.y();
                min = Math.min(min, along);
                max = Math.max(max, along);
            }
            return new Span(min, max);
        }
    }

    private static Vec2 centre(Ring<Vec2> polygon) {
        double x = 0, y = 0;
        for (Vec2 p : polygon.all()) {
            x += p.x();
            y += p.y();
        }
        return new Vec2(x / polygon.size(), y / polygon.size());
    }
}
