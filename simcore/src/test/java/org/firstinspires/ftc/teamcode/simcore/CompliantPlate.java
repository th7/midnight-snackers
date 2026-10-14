package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;

final class CompliantPlate implements Surroundings.Part {
    private final String id;
    private final Vec3 point;
    private final Vec3 normal;
    private final Compliance compliance;
    private final Vec3 sliding;

    private CompliantPlate(String id, Vec3 point, Vec3 normal, Compliance compliance, Vec3 sliding) {
        this.id = id;
        this.point = point;
        this.normal = normal;
        this.compliance = compliance;
        this.sliding = sliding.minus(normal.times(sliding.dot(normal)));
    }

    static CompliantPlate still(String id, Vec3 point, Vec3 normal, Compliance compliance) {
        return new CompliantPlate(id, point, normal, compliance, Vec3.zero());
    }

    static CompliantPlate sliding(String id, Vec3 point, Vec3 normal, Compliance compliance, Vec3 velocity) {
        return new CompliantPlate(id, point, normal, compliance, velocity);
    }

    private double above(Vec3 centre) {
        return centre.minus(point).dot(normal);
    }

    @Override
    public List<String> ids() {
        return List.of(id);
    }

    @Override
    public boolean reaches(Vec3 centre, double by) {
        return above(centre) <= by;
    }

    @Override
    public double sweep() {
        return 0;
    }

    @Override
    public boolean moving() {
        return sliding.dot(sliding) > 0;
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
        double depth = radius.inches() - above(centre);
        if (depth > 0) {
            squeezes.add(Valid.value(Surroundings.Squeeze.of(id, compliance, normal, depth, sliding)));
        }
        return squeezes;
    }

    @Override
    public Surroundings.Part at(GridTime time) {
        return this;
    }
}
