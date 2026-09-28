package org.firstinspires.ftc.teamcode.simcore;

import java.util.List;

public final class ConvexPolygon {
    private static final double TURNING_TOLERANCE = 1e-9;

    private final Ring<Vec2> corners;

    private ConvexPolygon(Ring<Vec2> corners) {
        this.corners = corners;
    }

    public static Checked<ConvexPolygon> of(List<Vec2> corners) {
        if (corners.size() < 3) {
            return Checked.rejected("a convex polygon has at least three corners, not " + corners);
        }
        for (Vec2 corner : corners) {
            if (!(Double.isFinite(corner.x()) && Double.isFinite(corner.y()))) {
                return Checked.rejected("a convex polygon's corners are finite, not " + corners);
            }
        }
        return Ring.of(corners).then(ConvexPolygon::turningLeftOnce);
    }

    private static Checked<ConvexPolygon> turningLeftOnce(Ring<Vec2> ring) {
        double turned = 0;
        List<Ring.Edge<Vec2>> edges = ring.edges();
        Ring.Edge<Vec2> previous = new Ring.Edge<>(ring.first(), ring.first());
        for (Ring.Edge<Vec2> edge : edges) {
            previous = edge;
        }
        for (Ring.Edge<Vec2> edge : edges) {
            double ax = previous.to().x() - previous.from().x(),
                    ay = previous.to().y() - previous.from().y();
            double bx = edge.to().x() - edge.from().x(),
                    by = edge.to().y() - edge.from().y();
            double cross = ax * by - ay * bx;
            if (!(cross > 0)) {
                return Checked.rejected(
                        "a convex polygon turns left at every corner, and " + ring + " does not at " + edge.from());
            }
            turned += Math.atan2(cross, ax * bx + ay * by);
            previous = edge;
        }
        if (!(Math.abs(turned - 2 * Math.PI) <= TURNING_TOLERANCE)) {
            return Checked.rejected("a convex polygon goes round once, and " + ring + " turns " + turned);
        }
        return Checked.ok(new ConvexPolygon(ring));
    }

    public List<Vec2> corners() {
        return corners.all();
    }

    Ring<Vec2> ring() {
        return corners;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof ConvexPolygon that && corners.equals(that.corners);
    }

    @Override
    public int hashCode() {
        return corners.hashCode();
    }

    @Override
    public String toString() {
        return corners.toString();
    }
}
