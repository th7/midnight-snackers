package org.firstinspires.ftc.teamcode.simcore;

import java.util.List;
import java.util.Optional;

public final class Nest {
    public static final double BALL_MASS_KG = 0.5;
    public static final double FRICTION = 0.5;

    private static final double CONTACT_TOLERANCE_IN = 0.02;

    private Nest() {}

    public static <K> Optional<K> nested(Field.Flower flower, List<Rolling<K>> rolling) {
        Optional<K> nested = Optional.empty();
        double nearest = Double.MAX_VALUE;
        for (Rolling<K> ball : rolling) {
            double x = ball.at().x(), y = ball.at().y();
            if (!flower.standsIn(x, y)) {
                continue;
            }
            double out = Math.hypot(x - flower.axis().x(), y - flower.axis().y());
            if (out < nearest) {
                nearest = out;
                nested = Optional.of(ball.key());
            }
        }
        return nested;
    }

    public static double load(int standingOnIt) {
        return (1 + standingOnIt) * BALL_MASS_KG * Flight.GRAVITY_IN_PER_S2 * Length.METRES_PER_INCH;
    }

    public static double overTheRing(Field.Flower flower, Length radius, int standingOnIt) {
        double r = radius.inches();
        return load(standingOnIt)
                * Math.sqrt(2 * r * flower.nest() - flower.nest() * flower.nest())
                / (r - flower.nest());
    }

    public static Optional<Vec2> hold(Field.Flower flower, Vec2 ballAt, Length radius, int standingOnIt) {
        double toTheAxis = flower.axis().x() - ballAt.x();
        double acrossToIt = flower.axis().y() - ballAt.y();
        double out = Math.hypot(toTheAxis, acrossToIt);
        if (!(out > CONTACT_TOLERANCE_IN)) {
            return Optional.empty();
        }
        double hold = overTheRing(flower, radius, standingOnIt) * Math.min(1, out / flower.bore());
        return Optional.of(new Vec2(hold * toTheAxis / out, hold * acrossToIt / out));
    }

    public static Optional<Double> drag(int standingOnIt, double massKg, double speedMetresPerSecond, Seconds dt) {
        if (!(speedMetresPerSecond > 0)) {
            return Optional.empty();
        }
        return Optional.of(Math.min(FRICTION * load(standingOnIt), massKg * speedMetresPerSecond / dt.value()));
    }
}
