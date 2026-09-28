package org.firstinspires.ftc.teamcode.simcore;

import java.util.List;

final class Valid {
    private Valid() {}

    static <T> T value(Checked<T> checked) {
        return checked.fold(value -> value, rule -> {
            throw new AssertionError(rule);
        });
    }

    static Length length(double inches) {
        return value(Length.of(inches));
    }

    static Heading heading(double radians) {
        return value(Heading.ofRadians(radians));
    }

    static Pose pose(double x, double y, double radians) {
        return value(Pose.of(x, y, heading(radians)));
    }

    static ConvexPolygon polygon(List<Vec2> corners) {
        return value(ConvexPolygon.of(corners));
    }
}
