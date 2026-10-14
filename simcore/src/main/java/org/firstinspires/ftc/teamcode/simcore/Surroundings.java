package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

final class Surroundings {
    record Touch(String id, Surface surface, Vec3 normal, double gap, Vec3 velocity, Vec3 spin) {
        boolean moves() {
            return velocity.dot(velocity) > 0 || spin.dot(spin) > 0;
        }
    }

    interface Part {
        List<String> ids();

        boolean reaches(Vec3 centre, double by);

        double sweep();

        boolean moving();

        List<Touch> near(Vec3 centre, Length radius, double within);

        Part at(double seconds);
    }

    private final Colliders still;
    private final List<Part> parts;
    private final List<Part> moving;
    private final double sweep;

    private Surroundings(Colliders still, List<Part> parts) {
        this.still = still;
        this.parts = Collections.unmodifiableList(new ArrayList<>(parts));
        List<Part> moving = new ArrayList<>();
        double sweep = 0;
        for (Part part : parts) {
            if (part.moving()) {
                moving.add(part);
                sweep = StrictMath.max(sweep, sweepOf(part));
            }
        }
        this.moving = Collections.unmodifiableList(moving);
        this.sweep = sweep;
    }

    static Surroundings still(Colliders colliders) {
        return new Surroundings(colliders, List.of());
    }

    static Checked<Surroundings> of(Colliders still, List<Part> parts) {
        Set<String> named = new HashSet<>();
        for (Collider collider : still.all()) {
            named.add(collider.name());
        }
        for (Part part : parts) {
            for (String id : part.ids()) {
                if (!named.add(id)) {
                    return Checked.rejected("every collider has a name of its own, and '" + id + "' is not one");
                }
            }
        }
        return Checked.ok(new Surroundings(still, parts));
    }

    Surroundings at(double seconds) {
        if (parts.isEmpty()) {
            return this;
        }
        List<Part> then = new ArrayList<>();
        for (Part part : parts) {
            then.add(part.at(seconds));
        }
        return new Surroundings(still, then);
    }

    List<Touch> near(Vec3 centre, Length radius, double within) {
        List<Touch> touches = new ArrayList<>();
        for (Colliders.Near near : still.near(centre, radius, within)) {
            Collider collider = near.collider();
            touches.add(new Touch(
                    collider.name(), collider.surface(), near.normal(), near.gap(), Vec3.zero(), Vec3.zero()));
        }
        double reach = radius.inches() + within;
        for (Part part : parts) {
            if (part.reaches(centre, reach)) {
                touches.addAll(part.near(centre, radius, within));
            }
        }
        return touches;
    }

    List<Part> moving() {
        return moving;
    }

    double sweep() {
        return sweep;
    }

    double sweepOf(Part part) {
        double said = part.sweep();
        return said >= 0 ? said : Double.POSITIVE_INFINITY;
    }

    Optional<Colliders.Side> over(Vec3 centre, Length radius) {
        return still.over(centre, radius);
    }
}
