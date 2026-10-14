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

    static final class Support {
        private final Vec3 point;
        private final Vec3 velocity;
        private final Vec3 spin;

        private Support(Vec3 point, Vec3 velocity, Vec3 spin) {
            this.point = point;
            this.velocity = velocity;
            this.spin = spin;
        }

        static Checked<Support> of(Vec3 point, Vec3 velocity, Vec3 spin) {
            if (!(point.isFinite() && velocity.isFinite() && spin.isFinite())) {
                return Checked.rejected("a support moves as a point of it goes and as it spins, each finite, not a"
                        + " point at " + point + " going at " + velocity + " and a spin of " + spin);
            }
            return Checked.ok(new Support(point, velocity, spin));
        }

        static Support still() {
            return new Support(Vec3.zero(), Vec3.zero(), Vec3.zero());
        }

        Vec3 velocityAt(Vec3 where) {
            return velocity.plus(spin.cross(where.minus(point)));
        }

        Vec3 spin() {
            return spin;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Support that
                    && point.equals(that.point)
                    && velocity.equals(that.velocity)
                    && spin.equals(that.spin);
        }

        @Override
        public int hashCode() {
            return Objects.hash(point, velocity, spin);
        }

        @Override
        public String toString() {
            return "Support[point=" + point + ", velocity=" + velocity + ", spin=" + spin + "]";
        }
    }

    static final class Overlap {
        private final String id;
        private final Compliance compliance;
        private final Vec3 normal;
        private final double depth;
        private final Support support;

        private Overlap(String id, Compliance compliance, Vec3 normal, double depth, Support support) {
            this.id = id;
            this.compliance = compliance;
            this.normal = normal;
            this.depth = depth;
            this.support = support;
        }

        static Checked<Overlap> of(String id, Compliance compliance, Vec3 normal, double depth, Support support) {
            if (!(StrictMath.abs(normal.dot(normal) - 1) <= Touch.UNIT_WITHIN && depth > 0 && Double.isFinite(depth))) {
                return Checked.rejected("an overlap has a unit normal and a finite depth more than 0, not a normal of "
                        + normal + " and a depth of " + depth);
            }
            return Checked.ok(new Overlap(id, compliance, normal, depth, support));
        }

        String id() {
            return id;
        }

        Compliance compliance() {
            return compliance;
        }

        Vec3 normal() {
            return normal;
        }

        double depth() {
            return depth;
        }

        Support support() {
            return support;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Overlap that
                    && id.equals(that.id)
                    && compliance.equals(that.compliance)
                    && normal.equals(that.normal)
                    && Double.compare(depth, that.depth) == 0
                    && support.equals(that.support);
        }

        @Override
        public int hashCode() {
            return Objects.hash(id, compliance, normal, depth, support);
        }

        @Override
        public String toString() {
            return "Overlap[id=" + id + ", compliance=" + compliance + ", normal=" + normal + ", depth=" + depth
                    + ", support=" + support + "]";
        }
    }

    static final class Squeeze {
        private final String id;
        private final Compliance compliance;
        private final Vec3 normal;
        private final double depth;
        private final Vec3 arm;
        private final Vec3 velocity;
        private final Vec3 spin;

        private Squeeze(
                String id, Compliance compliance, Vec3 normal, double depth, Vec3 arm, Vec3 velocity, Vec3 spin) {
            this.id = id;
            this.compliance = compliance;
            this.normal = normal;
            this.depth = depth;
            this.arm = arm;
            this.velocity = velocity;
            this.spin = spin;
        }

        static Checked<Squeeze> of(Overlap overlap, Vec3 centre, Length radius) {
            Vec3 arm = overlap.normal().times(-(radius.inches() - overlap.depth() / 2));
            Vec3 meets = centre.plus(arm);
            Vec3 velocity = overlap.support().velocityAt(meets);
            if (!(meets.isFinite() && velocity.isFinite())) {
                return Checked.rejected("a squeeze meets the ball at a finite point and takes a finite velocity there,"
                        + " not a velocity of " + velocity + " at " + meets);
            }
            return Checked.ok(new Squeeze(
                    overlap.id(),
                    overlap.compliance(),
                    overlap.normal(),
                    overlap.depth(),
                    arm,
                    velocity,
                    overlap.support().spin()));
        }

        String id() {
            return id;
        }

        Compliance compliance() {
            return compliance;
        }

        Vec3 normal() {
            return normal;
        }

        double depth() {
            return depth;
        }

        Vec3 arm() {
            return arm;
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
            return other instanceof Squeeze that
                    && id.equals(that.id)
                    && compliance.equals(that.compliance)
                    && normal.equals(that.normal)
                    && Double.compare(depth, that.depth) == 0
                    && arm.equals(that.arm)
                    && velocity.equals(that.velocity)
                    && spin.equals(that.spin);
        }

        @Override
        public int hashCode() {
            return Objects.hash(id, compliance, normal, depth, arm, velocity, spin);
        }

        @Override
        public String toString() {
            return "Squeeze[id=" + id + ", compliance=" + compliance + ", normal=" + normal + ", depth=" + depth
                    + ", arm=" + arm + ", velocity=" + velocity + ", spin=" + spin + "]";
        }
    }

    interface Part {
        List<String> ids();

        boolean reaches(Vec3 centre, double by);

        double sweep();

        boolean moving();

        List<Touch> near(Vec3 centre, Length radius, double within);

        boolean squeezes();

        List<Overlap> squeezing(Vec3 centre, Length radius);

        Part at(GridTime time);
    }

    static final class Held {
        private final Part part;
        private final Set<String> ids;

        private Held(Part part, Set<String> ids) {
            this.part = part;
            this.ids = ids;
        }

        Part part() {
            return part;
        }

        boolean moving() {
            return part.moving();
        }

        boolean reaches(Vec3 centre, double by) {
            return part.reaches(centre, by);
        }

        double sweep() {
            double said = part.sweep();
            return said >= 0 ? said : Double.POSITIVE_INFINITY;
        }

        List<Touch> near(Vec3 centre, Length radius, double within) {
            List<Touch> kept = new ArrayList<>();
            for (Touch touch : part.near(centre, radius, within)) {
                if (touch.gap() <= within && ids.contains(touch.id())) {
                    kept.add(touch);
                }
            }
            return kept;
        }

        boolean squeezes() {
            return part.squeezes();
        }

        List<Squeeze> squeezing(Vec3 centre, Length radius) {
            List<Squeeze> kept = new ArrayList<>();
            for (Overlap overlap : part.squeezing(centre, radius)) {
                if (ids.contains(overlap.id())) {
                    Squeeze.of(overlap, centre, radius).fold(squeeze -> kept.add(squeeze), rule -> false);
                }
            }
            return kept;
        }

        Held at(GridTime time) {
            return new Held(part.at(time), ids);
        }
    }

    private final Colliders still;
    private final List<Held> parts;
    private final List<Held> moving;
    private final List<Held> compliant;
    private final double sweep;

    private Surroundings(Colliders still, List<Held> parts) {
        this.still = still;
        this.parts = Collections.unmodifiableList(new ArrayList<>(parts));
        List<Held> moving = new ArrayList<>();
        List<Held> compliant = new ArrayList<>();
        double sweep = 0;
        for (Held part : parts) {
            if (part.moving()) {
                moving.add(part);
                sweep = StrictMath.max(sweep, part.sweep());
            }
            if (part.squeezes()) {
                compliant.add(part);
            }
        }
        this.moving = Collections.unmodifiableList(moving);
        this.compliant = Collections.unmodifiableList(compliant);
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
        List<Held> held = new ArrayList<>();
        for (Part part : parts) {
            Set<String> ids = new HashSet<>();
            for (String id : part.ids()) {
                if (!named.add(id)) {
                    return Checked.rejected("every collider has a name of its own, and '" + id + "' is not one");
                }
                ids.add(id);
            }
            held.add(new Held(part, Collections.unmodifiableSet(ids)));
        }
        return Checked.ok(new Surroundings(still, held));
    }

    Surroundings at(GridTime time) {
        if (parts.isEmpty()) {
            return this;
        }
        List<Held> then = new ArrayList<>();
        for (Held part : parts) {
            then.add(part.at(time));
        }
        return new Surroundings(still, then);
    }

    Checked<Surroundings> replacing(Part part) {
        Set<String> names = new HashSet<>();
        for (String id : part.ids()) {
            if (!names.add(id)) {
                return Checked.rejected("every collider has a name of its own, and '" + id + "' is not one");
            }
        }
        List<Held> replaced = new ArrayList<>();
        boolean found = false;
        for (Held held : parts) {
            boolean same = held.ids.equals(names);
            found = found || same;
            replaced.add(same ? new Held(part, held.ids) : held);
        }
        if (!found) {
            return Checked.rejected("a part takes the place of one the surroundings hold under exactly its names, and "
                    + part.ids() + " are not the names of any");
        }
        return Checked.ok(new Surroundings(still, replaced));
    }

    List<Touch> near(Vec3 centre, Length radius, double within) {
        List<Touch> touches = new ArrayList<>();
        for (Colliders.Near near : still.near(centre, radius, within)) {
            Collider collider = near.collider();
            Touch.of(collider.name(), collider.surface(), near.normal(), near.gap(), Vec3.zero(), Vec3.zero())
                    .fold(touch -> touches.add(touch), rule -> false);
        }
        double reach = radius.inches() + within;
        for (Held part : parts) {
            if (part.reaches(centre, reach)) {
                touches.addAll(part.near(centre, radius, within));
            }
        }
        return touches;
    }

    List<Held> moving() {
        return moving;
    }

    boolean squeezes() {
        return !compliant.isEmpty();
    }

    boolean squeezeReaches(Vec3 centre, double by) {
        for (Held part : compliant) {
            if (part.reaches(centre, by)) {
                return true;
            }
        }
        return false;
    }

    List<Squeeze> squeezing(Vec3 centre, Length radius) {
        List<Squeeze> squeezes = new ArrayList<>();
        for (Held part : compliant) {
            if (part.reaches(centre, radius.inches())) {
                squeezes.addAll(part.squeezing(centre, radius));
            }
        }
        return squeezes;
    }

    List<Squeeze> squeezingAt(GridTime time, Vec3 centre, Length radius) {
        List<Squeeze> squeezes = new ArrayList<>();
        for (Held part : compliant) {
            Held then = part.at(time);
            if (then.reaches(centre, radius.inches())) {
                squeezes.addAll(then.squeezing(centre, radius));
            }
        }
        return squeezes;
    }

    int placements() {
        return parts.size();
    }

    double sweep() {
        return sweep;
    }

    Optional<Colliders.Side> over(Vec3 centre, Length radius) {
        return still.over(centre, radius);
    }
}
