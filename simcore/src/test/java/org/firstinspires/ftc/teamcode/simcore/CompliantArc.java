package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;

final class CompliantArc implements Surroundings.Part {
    private record Spin(double starts, double rising, double top) {
        double at(double seconds) {
            if (!(seconds > starts)) {
                return 0;
            }
            return top * StrictMath.min(1, (seconds - starts) / rising);
        }
    }

    private final String id;
    private final Arc arc;
    private final Compliance compliance;
    private final Spin spin;
    private final double seconds;

    private CompliantArc(String id, Arc arc, Compliance compliance, Spin spin, double seconds) {
        this.id = id;
        this.arc = arc;
        this.compliance = compliance;
        this.spin = spin;
        this.seconds = seconds;
    }

    static CompliantArc still(String id, Arc arc, Compliance compliance) {
        return new CompliantArc(id, arc, compliance, new Spin(0, 1, 0), 0);
    }

    static CompliantArc spinning(
            String id, Vec3 centre, Vec3 axis, double radius, Compliance compliance, double radiansPerSecond) {
        return spinningUp(id, centre, axis, radius, compliance, Double.NEGATIVE_INFINITY, 1, radiansPerSecond);
    }

    static CompliantArc spinningUp(
            String id,
            Vec3 centre,
            Vec3 axis,
            double radius,
            Compliance compliance,
            double starts,
            double risingSeconds,
            double radiansPerSecond) {
        return new CompliantArc(
                id,
                Valid.value(Arc.of(centre, axis, radius, Arc.Span.whole())),
                compliance,
                new Spin(starts, risingSeconds, radiansPerSecond),
                0);
    }

    double radiansPerSecond() {
        return spin.at(seconds);
    }

    @Override
    public List<String> ids() {
        return List.of(id);
    }

    @Override
    public boolean reaches(Vec3 centre, double by) {
        return arc.nearestTo(centre).distance() <= by;
    }

    @Override
    public double sweep() {
        return 0;
    }

    @Override
    public boolean moving() {
        return radiansPerSecond() != 0;
    }

    @Override
    public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
        return List.of();
    }

    @Override
    public boolean squeezes() {
        return true;
    }

    @Override
    public List<Surroundings.Squeeze> squeezing(Vec3 centre, Length radius) {
        List<Surroundings.Squeeze> squeezes = new ArrayList<>();
        Arc.Nearest nearest = arc.nearestTo(centre);
        double depth = radius.inches() - nearest.distance();
        if (depth > 0) {
            Surroundings.Support turning = Valid.value(Surroundings.Support.of(
                    arc.centre(), Vec3.zero(), arc.axis().times(radiansPerSecond())));
            squeezes.add(Valid.value(
                    Surroundings.Squeeze.of(id, compliance, centre, radius, nearest.normal(), depth, turning)));
        }
        return squeezes;
    }

    @Override
    public Surroundings.Part at(GridTime time) {
        return new CompliantArc(id, arc, compliance, spin, time.seconds());
    }
}
