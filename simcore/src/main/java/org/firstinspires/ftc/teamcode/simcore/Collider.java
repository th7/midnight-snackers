package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public final class Collider {
    public enum Kind {
        FLOOR,
        SOLID,
        PLATE;

        public String asked() {
            return name().toLowerCase(Locale.ROOT);
        }
    }

    static final class Face {
        private record Edge(Vec3 from, Vec3 along, double squared, Vec3 inward) {}

        private final Ring<Vec3> ring;
        private final List<Vec3> corners;
        private final Vec3 normal;
        private final Vec3 centre;
        private final List<Edge> edges;

        private Face(Ring<Vec3> ring, Vec3 normal) {
            this.ring = ring;
            this.corners = ring.all();
            this.normal = normal;
            this.centre = meanOf(corners);
            List<Edge> edges = new ArrayList<>();
            for (Ring.Edge<Vec3> edge : ring.edges()) {
                Vec3 along = edge.to().minus(edge.from());
                edges.add(new Edge(edge.from(), along, along.dot(along), normal.cross(along)));
            }
            this.edges = Collections.unmodifiableList(edges);
        }

        static Checked<Face> of(String name, Ring<Vec3> ring) {
            Vec3 newell = Vec3.newellNormalOf(ring);
            double length = lengthOf(newell);
            if (!(length > 0 && Double.isFinite(length))) {
                return Checked.rejected(name + "'s faces each have an area, and " + ring + " has none");
            }
            return new Face(ring, newell.times(1 / length)).flatAndConvex(name);
        }

        private Checked<Face> flatAndConvex(String name) {
            double tolerance = Colliders.CONVEX_TOLERANCE_IN;
            for (Vec3 corner : corners) {
                double off = above(corner);
                if (!(StrictMath.abs(off) <= tolerance)) {
                    return Checked.rejected(name + "'s faces are flat to within " + tolerance + " in, and " + ring
                            + " is not: " + corner + " is " + off + " in off it");
                }
            }
            for (Edge edge : edges) {
                double reach = lengthOf(edge.inward());
                if (!(reach > 0)) {
                    continue;
                }
                for (Vec3 corner : corners) {
                    double inside = corner.minus(edge.from()).dot(edge.inward()) / reach;
                    if (!(inside >= -tolerance)) {
                        return Checked.rejected(name + "'s faces are convex to within " + tolerance + " in, and "
                                + ring + " is not: " + corner + " is " + -inside + " in outside its edge from "
                                + edge.from());
                    }
                }
            }
            return Checked.ok(this);
        }

        Ring<Vec3> ring() {
            return ring;
        }

        List<Vec3> corners() {
            return corners;
        }

        Vec3 normal() {
            return normal;
        }

        Vec3 centre() {
            return centre;
        }

        double above(Vec3 point) {
            return (point.x() - centre.x()) * normal.x()
                    + (point.y() - centre.y()) * normal.y()
                    + (point.z() - centre.z()) * normal.z();
        }

        Face outwardFrom(Vec3 inside) {
            return above(inside) > 0 ? flipped() : this;
        }

        private Face flipped() {
            List<Vec3> rest = new ArrayList<>();
            boolean first = true;
            for (Vec3 corner : corners) {
                if (!first) {
                    rest.add(corner);
                }
                first = false;
            }
            Collections.reverse(rest);
            return new Face(Ring.of(ring.first(), rest), normal.negated());
        }

        boolean holds(Vec3 point) {
            for (Edge edge : edges) {
                Vec3 from = edge.from();
                Vec3 inward = edge.inward();
                double inside = (point.x() - from.x()) * inward.x()
                        + (point.y() - from.y()) * inward.y()
                        + (point.z() - from.z()) * inward.z();
                if (inside < 0) {
                    return false;
                }
            }
            return true;
        }

        Vec3 nearestTo(Vec3 point) {
            return holds(point) ? point.along(normal, -above(point)) : nearestOnRim(point);
        }

        Vec3 nearestOnRim(Vec3 point) {
            Vec3 nearest = ring.first();
            double closest = Double.POSITIVE_INFINITY;
            for (Edge edge : edges) {
                Vec3 onIt = edge.from();
                if (edge.squared() > 0) {
                    double t = point.minus(edge.from()).dot(edge.along()) / edge.squared();
                    onIt = edge.from().along(edge.along(), StrictMath.max(0, StrictMath.min(1, t)));
                }
                double squared = squaredDistance(point, onIt);
                if (squared < closest) {
                    closest = squared;
                    nearest = onIt;
                }
            }
            return nearest;
        }
    }

    private sealed interface Shape permits Floor, Solid, Plate {}

    private record Floor() implements Shape {}

    private record Solid(Face first, List<Face> faces) implements Shape {}

    private record Plate(Face face) implements Shape {}

    private record Join(Vec3 from, Vec3 to) {
        static Join of(Vec3 a, Vec3 b) {
            return before(a, b) ? new Join(a, b) : new Join(b, a);
        }

        private static boolean before(Vec3 a, Vec3 b) {
            int x = Double.compare(a.x(), b.x());
            if (x != 0) {
                return x < 0;
            }
            int y = Double.compare(a.y(), b.y());
            if (y != 0) {
                return y < 0;
            }
            return Double.compare(a.z(), b.z()) <= 0;
        }
    }

    private final String name;
    private final Surface surface;
    private final Shape shape;
    private final Vec3 least;
    private final Vec3 most;
    private final List<Ring<Vec3>> faces;

    private Collider(String name, Surface surface, Shape shape, Vec3 least, Vec3 most, List<Face> faces) {
        this.name = name;
        this.surface = surface;
        this.shape = shape;
        this.least = least;
        this.most = most;
        List<Ring<Vec3>> rings = new ArrayList<>();
        for (Face face : faces) {
            rings.add(face.ring());
        }
        this.faces = Collections.unmodifiableList(rings);
    }

    static Collider floor(String name) {
        return new Collider(
                name,
                Surface.FLOOR,
                new Floor(),
                new Vec3(-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE),
                new Vec3(Double.MAX_VALUE, Double.MAX_VALUE, 0),
                List.of());
    }

    static Checked<Collider> plate(String name, Surface surface, Ring<Vec3> corners) {
        return Face.of(name, corners)
                .map(face -> new Collider(
                        name,
                        surface,
                        new Plate(face),
                        leastOf(face.corners()),
                        mostOf(face.corners()),
                        List.of(face)));
    }

    static Checked<Collider> box(String name, Surface surface, Vec3 least, Vec3 most) {
        Vec3 a = least, g = most;
        Vec3 b = new Vec3(g.x(), a.y(), a.z()), c = new Vec3(g.x(), g.y(), a.z()), d = new Vec3(a.x(), g.y(), a.z());
        Vec3 e = new Vec3(a.x(), a.y(), g.z()), f = new Vec3(g.x(), a.y(), g.z()), h = new Vec3(a.x(), g.y(), g.z());
        return solid(
                name,
                surface,
                List.of(
                        Ring.of(a, List.of(d, c, b)),
                        Ring.of(e, List.of(f, g, h)),
                        Ring.of(a, List.of(b, f, e)),
                        Ring.of(c, List.of(d, h, g)),
                        Ring.of(b, List.of(c, g, f)),
                        Ring.of(a, List.of(e, h, d))));
    }

    static Checked<Collider> solid(String name, Surface surface, List<Ring<Vec3>> rings) {
        List<Face> given = new ArrayList<>();
        List<Vec3> corners = new ArrayList<>();
        for (Ring<Vec3> ring : rings) {
            Checked<Face> face = Face.of(name, ring);
            if (face instanceof Checked.Rejected<Face> rejected) {
                return Checked.rejected(rejected.rule());
            }
            face.fold(made -> given.add(made), rule -> false);
            corners.addAll(ring.all());
        }
        Vec3 middle = meanOf(corners);
        List<Face> faces = new ArrayList<>();
        for (Face face : given) {
            faces.add(face.outwardFrom(middle));
        }
        return closed(name, faces)
                .then(shut -> convex(name, shut, corners))
                .then(convex -> Ring.of(convex)
                        .fold(
                                ring -> Checked.ok(new Collider(
                                        name,
                                        surface,
                                        new Solid(ring.first(), Collections.unmodifiableList(convex)),
                                        leastOf(corners),
                                        mostOf(corners),
                                        convex)),
                                rule -> Checked.rejected(name + " is a solid of faces, and has none")));
    }

    private static Checked<List<Face>> closed(String name, List<Face> faces) {
        Map<Join, Integer> shared = new LinkedHashMap<>();
        for (Face face : faces) {
            for (Ring.Edge<Vec3> edge : face.ring().edges()) {
                Join join = Join.of(edge.from(), edge.to());
                shared.put(join, shared.getOrDefault(join, 0) + 1);
            }
        }
        for (Map.Entry<Join, Integer> join : shared.entrySet()) {
            if (join.getValue() != 2) {
                return Checked.rejected(name + " is closed, each of its edges shared by two of its faces, and its edge"
                        + " from " + join.getKey().from() + " to "
                        + join.getKey().to() + " is shared by "
                        + join.getValue());
            }
        }
        return Checked.ok(faces);
    }

    private static Checked<List<Face>> convex(String name, List<Face> faces, List<Vec3> corners) {
        double tolerance = Colliders.CONVEX_TOLERANCE_IN;
        for (Face face : faces) {
            for (Vec3 corner : corners) {
                double out = face.above(corner);
                if (!(out <= tolerance)) {
                    return Checked.rejected(name + " is convex to within " + tolerance + " in, and its corner " + corner
                            + " stands " + out + " in out from its face " + face.ring());
                }
            }
        }
        return Checked.ok(faces);
    }

    public String name() {
        return name;
    }

    public Surface surface() {
        return surface;
    }

    public Kind kind() {
        if (shape instanceof Solid) {
            return Kind.SOLID;
        }
        if (shape instanceof Plate) {
            return Kind.PLATE;
        }
        return Kind.FLOOR;
    }

    public Vec3 least() {
        return least;
    }

    public Vec3 most() {
        return most;
    }

    public List<Ring<Vec3>> faces() {
        return faces;
    }

    boolean reaches(Vec3 centre, double by) {
        return centre.x() >= least.x() - by
                && centre.x() <= most.x() + by
                && centre.y() >= least.y() - by
                && centre.y() <= most.y() + by
                && centre.z() >= least.z() - by
                && centre.z() <= most.z() + by;
    }

    Colliders.Near near(Vec3 centre, double radius) {
        if (shape instanceof Solid solid) {
            return nearSolid(solid, centre, radius);
        }
        if (shape instanceof Plate plate) {
            Face face = plate.face();
            Vec3 nearest = face.nearestTo(centre);
            return away(centre, nearest, squaredDistance(centre, nearest), face.normal(), radius);
        }
        return new Colliders.Near(this, new Vec3(0, 0, 1), centre.z() - radius);
    }

    private Colliders.Near nearSolid(Solid solid, Vec3 centre, double radius) {
        Face shallowest = solid.first();
        double highest = shallowest.above(centre);
        for (Face face : solid.faces()) {
            double above = face.above(centre);
            if (above > highest) {
                highest = above;
                shallowest = face;
            }
        }
        if (!(highest > 0)) {
            return new Colliders.Near(this, shallowest.normal(), highest - radius);
        }
        Vec3 nearest = centre;
        double closest = Double.POSITIVE_INFINITY;
        for (Face face : solid.faces()) {
            if (face.above(centre) > 0) {
                Vec3 onIt = face.nearestTo(centre);
                double squared = squaredDistance(centre, onIt);
                if (squared < closest) {
                    closest = squared;
                    nearest = onIt;
                }
            }
        }
        return away(centre, nearest, closest, shallowest.normal(), radius);
    }

    private Colliders.Near away(Vec3 centre, Vec3 nearest, double squared, Vec3 otherwise, double radius) {
        double distance = StrictMath.sqrt(squared);
        if (!(distance > 0)) {
            return new Colliders.Near(this, otherwise, -radius);
        }
        return new Colliders.Near(this, centre.minus(nearest).times(1 / distance), distance - radius);
    }

    static double lengthOf(Vec3 v) {
        return StrictMath.sqrt(v.dot(v));
    }

    static double squaredDistance(Vec3 a, Vec3 b) {
        double x = a.x() - b.x(), y = a.y() - b.y(), z = a.z() - b.z();
        return x * x + y * y + z * z;
    }

    static Vec3 meanOf(List<Vec3> points) {
        double x = 0, y = 0, z = 0;
        int count = 0;
        for (Vec3 point : points) {
            x += point.x();
            y += point.y();
            z += point.z();
            count++;
        }
        double each = 1.0 / StrictMath.max(1, count);
        return new Vec3(x * each, y * each, z * each);
    }

    private static Vec3 leastOf(List<Vec3> points) {
        double x = Double.POSITIVE_INFINITY, y = Double.POSITIVE_INFINITY, z = Double.POSITIVE_INFINITY;
        for (Vec3 point : points) {
            x = StrictMath.min(x, point.x());
            y = StrictMath.min(y, point.y());
            z = StrictMath.min(z, point.z());
        }
        return new Vec3(x, y, z);
    }

    private static Vec3 mostOf(List<Vec3> points) {
        double x = Double.NEGATIVE_INFINITY, y = Double.NEGATIVE_INFINITY, z = Double.NEGATIVE_INFINITY;
        for (Vec3 point : points) {
            x = StrictMath.max(x, point.x());
            y = StrictMath.max(y, point.y());
            z = StrictMath.max(z, point.z());
        }
        return new Vec3(x, y, z);
    }

    @Override
    public String toString() {
        return name;
    }
}
