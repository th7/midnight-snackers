package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

final class Surroundings {
    static final class Touch {
        static final double UNIT_WITHIN = 1e-9;

        private final String id;
        private final Surface surface;
        private final Vec3 normal;
        private final double gap;
        private final Vec3 velocity;
        private final Vec3 spin;

        private Touch(String id, Surface surface, Vec3 normal, double gap, Vec3 velocity, Vec3 spin) {
            this.id = id;
            this.surface = surface;
            this.normal = normal;
            this.gap = gap;
            this.velocity = velocity;
            this.spin = spin;
        }

        static Checked<Touch> of(String id, Surface surface, Vec3 normal, double gap, Vec3 velocity, Vec3 spin) {
            if (!(StrictMath.abs(normal.dot(normal) - 1) <= UNIT_WITHIN
                    && Double.isFinite(gap)
                    && velocity.isFinite()
                    && spin.isFinite())) {
                return Checked.rejected(
                        "a touch has a unit normal and a finite gap, velocity and spin, not a normal of " + normal
                                + ", a gap of " + gap + ", a velocity of " + velocity + " and a spin of " + spin);
            }
            return Checked.ok(new Touch(id, surface, normal, gap, velocity, spin));
        }

        String id() {
            return id;
        }

        Surface surface() {
            return surface;
        }

        Vec3 normal() {
            return normal;
        }

        double gap() {
            return gap;
        }

        Vec3 velocity() {
            return velocity;
        }

        Vec3 spin() {
            return spin;
        }

        boolean moves() {
            return velocity.dot(velocity) > 0 || spin.dot(spin) > 0;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Touch that
                    && id.equals(that.id)
                    && surface == that.surface
                    && normal.equals(that.normal)
                    && Double.compare(gap, that.gap) == 0
                    && velocity.equals(that.velocity)
                    && spin.equals(that.spin);
        }

        @Override
        public int hashCode() {
            return Objects.hash(id, surface, normal, gap, velocity, spin);
        }

        @Override
        public String toString() {
            return "Touch[id=" + id + ", surface=" + surface + ", normal=" + normal + ", gap=" + gap + ", velocity="
                    + velocity + ", spin=" + spin + "]";
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
            Touch.of(collider.name(), collider.surface(), near.normal(), near.gap(), Vec3.zero(), Vec3.zero())
                    .fold(touch -> touches.add(touch), rule -> false);
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
