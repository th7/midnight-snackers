package org.firstinspires.ftc.teamcode.simcore;

final class Arc {
    static final double UNIT_WITHIN = 1e-9;
    static final double WHOLE_TURN = 2 * StrictMath.PI;

    sealed interface Span permits Span.Whole, Span.Part {
        static Span whole() {
            return new Whole();
        }

        static Checked<Span> of(double from, double width) {
            if (!Double.isFinite(from)) {
                return Checked.rejected("an arc's span starts at a finite angle, not " + from);
            }
            if (!(width > 0 && width < WHOLE_TURN)) {
                return Checked.rejected(
                        "an arc's span is more than 0 and less than a whole turn, 2 pi, wide, not " + width);
            }
            return Checked.ok(new Part(from, width));
        }

        final class Whole implements Span {
            private Whole() {}

            @Override
            public String toString() {
                return "the whole circle";
            }
        }

        final class Part implements Span {
            private final double from;
            private final double width;

            private Part(double from, double width) {
                this.from = from;
                this.width = width;
            }

            double from() {
                return from;
            }

            double width() {
                return width;
            }

            @Override
            public String toString() {
                return "from " + from + " over " + width;
            }
        }
    }

    record Nearest(Vec3 point, Vec3 normal, double distance) {}

    private record Way(double cos, double sin) {}

    private final Vec3 centre;
    private final Vec3 axis;
    private final Vec3 first;
    private final Vec3 second;
    private final double radius;
    private final Span span;

    private Arc(Vec3 centre, Vec3 axis, Vec3 first, double radius, Span span) {
        this.centre = centre;
        this.axis = axis;
        this.first = first;
        this.second = axis.cross(first);
        this.radius = radius;
        this.span = span;
    }

    static Checked<Arc> of(Vec3 centre, Vec3 axis, double radius, Span span) {
        if (!centre.isFinite()) {
            return Checked.rejected("an arc's centre is a finite point, not " + centre);
        }
        double squared = axis.dot(axis);
        if (!(StrictMath.abs(squared - 1) <= UNIT_WITHIN)) {
            return Checked.rejected("an arc's axis is a unit vector, not " + axis);
        }
        if (!(radius > 0 && Double.isFinite(radius))) {
            return Checked.rejected("an arc's radius is a positive, finite number of inches, not " + radius);
        }
        Vec3 unit = axis.times(1 / StrictMath.sqrt(squared));
        Vec3 level = new Vec3(0, 0, 1).cross(unit);
        double across = StrictMath.sqrt(level.dot(level));
        Vec3 first = across > 0 ? level.times(1 / across) : new Vec3(1, 0, 0);
        return Checked.ok(new Arc(centre, unit, first, radius, span));
    }

    Vec3 centre() {
        return centre;
    }

    Vec3 axis() {
        return axis;
    }

    double radius() {
        return radius;
    }

    Span span() {
        return span;
    }

    Vec3 at(double angle) {
        return pointAt(new Way(StrictMath.cos(angle), StrictMath.sin(angle)));
    }

    Nearest nearestTo(Vec3 point) {
        Vec3 offset = point.minus(centre);
        double along = offset.dot(first);
        double up = offset.dot(second);
        double out = StrictMath.sqrt(along * along + up * up);
        Way way = out > 0 ? within(new Way(along / out, up / out)) : start();
        Vec3 nearest = pointAt(way);
        Vec3 apart = point.minus(nearest);
        double distance = StrictMath.sqrt(apart.dot(apart));
        Vec3 normal = distance > 0 ? apart.times(1 / distance) : outward(way);
        return new Nearest(nearest, normal, distance);
    }

    private Way start() {
        if (span instanceof Span.Part part) {
            return new Way(StrictMath.cos(part.from()), StrictMath.sin(part.from()));
        }
        return new Way(1, 0);
    }

    private Way within(Way way) {
        if (!(span instanceof Span.Part part)) {
            return way;
        }
        double past = StrictMath.atan2(way.sin(), way.cos()) - part.from();
        double turned = past - WHOLE_TURN * StrictMath.floor(past / WHOLE_TURN);
        double round = turned < WHOLE_TURN ? turned : 0;
        if (round <= part.width()) {
            return way;
        }
        double end = part.from() + part.width();
        return round - part.width() < WHOLE_TURN - round ? new Way(StrictMath.cos(end), StrictMath.sin(end)) : start();
    }

    private Vec3 outward(Way way) {
        return first.times(way.cos()).plus(second.times(way.sin()));
    }

    private Vec3 pointAt(Way way) {
        return centre.plus(outward(way).times(radius));
    }

    @Override
    public String toString() {
        return "an arc of radius " + radius + " in about " + centre + " along " + axis + ", " + span;
    }
}
