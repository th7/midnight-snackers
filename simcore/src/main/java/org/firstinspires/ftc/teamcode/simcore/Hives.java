package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public final class Hives {
    public static final int FULL = 40;
    public static final int NECTAR_FILLS = FULL / 5;
    public static final int POLLEN_FILLS = FULL / 8;

    private static final double CONTACT_TOLERANCE_IN = 0.02;
    private static final double LEAST_EDGE_SQUARED = 1e-12;
    private static final double TOUCHING_IN = 1e-9;
    private static final double FLOOR_BAND_IN = 0.2;

    public record Touch(Vec3 normal, double depth, Vec3 velocity) {}

    private record Corner(Vec3 at, Vec2 flat) {}

    private static final class Face {
        final Ring<Corner> corners;
        final Vec3 normal;
        final Vec3 u;
        final Vec3 v;

        private Face(Ring<Corner> corners, Vec3 normal, Vec3 u, Vec3 v) {
            this.corners = corners;
            this.normal = normal;
            this.u = u;
            this.v = v;
        }

        static Face of(Ring<Vec3> ring) {
            Vec3 normal = normalOf(ring);
            Vec3 edge = Vec3.zero();
            boolean found = false;
            for (Ring.Edge<Vec3> next : ring.edges()) {
                if (!found) {
                    edge = next.to().minus(next.from());
                    found = !(edge.dot(edge) < LEAST_EDGE_SQUARED);
                }
            }
            Vec3 u = edge.unit();
            Vec3 v = normal.cross(u);
            Vec3 first = ring.first();
            return new Face(
                    ring.map(corner -> {
                        Vec3 from = corner.minus(first);
                        return new Corner(corner, new Vec2(from.dot(u), from.dot(v)));
                    }),
                    normal,
                    u,
                    v);
        }

        Face flipped() {
            return new Face(corners, normal.negated(), u, v);
        }

        Vec3 first() {
            return corners.first().at();
        }

        Vec3 nearestTo(Vec3 point) {
            Vec3 from = point.minus(first());
            double s = from.dot(u), t = from.dot(v);
            boolean inside = false;
            Corner previous = last(corners);
            for (Corner corner : corners.all()) {
                Vec2 a = corner.flat(), b = previous.flat();
                if ((a.y() > t) != (b.y() > t) && s < (b.x() - a.x()) * (t - a.y()) / (b.y() - a.y()) + a.x()) {
                    inside = !inside;
                }
                previous = corner;
            }
            if (inside) {
                return point.along(normal, -from.dot(normal));
            }
            Vec3 nearest = first();
            double closest = Double.MAX_VALUE;
            previous = last(corners);
            for (Corner corner : corners.all()) {
                Vec3 onEdge = nearestOnSegment(previous.at(), corner.at(), point);
                Vec3 gap = point.minus(onEdge);
                if (gap.dot(gap) < closest) {
                    closest = gap.dot(gap);
                    nearest = onEdge;
                }
                previous = corner;
            }
            return nearest;
        }
    }

    private static final class Solid {
        final Ring<Face> faces;
        final boolean closed;
        final Vec3 middle;
        final double reach;

        private Solid(Ring<Face> faces, boolean closed) {
            List<Vec3> corners = new ArrayList<>();
            for (Face face : faces.all()) {
                for (Corner corner : face.corners.all()) {
                    corners.add(corner.at());
                }
            }
            Vec3 middle = mean(corners);
            double reach = 0;
            for (Vec3 corner : corners) {
                reach = Math.max(reach, corner.minus(middle).length());
            }
            this.middle = middle;
            this.reach = reach;
            this.closed = closed;
            this.faces = closed
                    ? faces.map(face -> face.first().minus(middle).dot(face.normal) < 0 ? face.flipped() : face)
                    : faces;
        }

        static Solid plate(Ring<Vec3> corners) {
            return new Solid(Ring.of(Face.of(corners), List.of()), false);
        }

        static Optional<Solid> part(Field.Element part) {
            List<Face> faces = new ArrayList<>();
            for (Ring<Vec3> face : part.faces()) {
                faces.add(Face.of(face));
            }
            return Ring.of(faces).fold(ring -> Optional.of(new Solid(ring, true)), rule -> Optional.empty());
        }

        Optional<Vec3> outOfItFrom(Vec3 point) {
            if (!closed) {
                return Optional.empty();
            }
            Optional<Vec3> shallowest = Optional.empty();
            double least = Double.MAX_VALUE;
            for (Face face : faces.all()) {
                double under = -point.minus(face.first()).dot(face.normal);
                if (under < 0) {
                    return Optional.empty();
                }
                if (under < least) {
                    least = under;
                    shallowest = Optional.of(face.normal);
                }
            }
            return shallowest;
        }
    }

    private record Bound(Vec3 on, Vec3 inward) {
        static Bound of(Ring<Vec3> ring, Vec3 centre) {
            Vec3 on = ring.first();
            Vec3 normal = normalOf(ring);
            return new Bound(on, centre.minus(on).dot(normal) >= 0 ? normal : normal.times(-1));
        }
    }

    private record CellBounds(Field.Cell cell, List<Bound> bounds) {}

    private record Shape(List<Solid> solids, Vec3 least, Vec3 most, List<CellBounds> cells) {
        static Shape of(Field.Hive hive) {
            List<Solid> solids = new ArrayList<>();
            List<CellBounds> cells = new ArrayList<>();
            for (Field.Cell cell : hive.cells()) {
                for (Ring<Vec3> panel : cell.panels()) {
                    solids.add(Solid.plate(panel));
                }
                List<Ring<Vec3>> faces = new ArrayList<>(cell.panels());
                faces.add(cell.mouth());
                List<Bound> around = new ArrayList<>();
                for (Ring<Vec3> face : faces) {
                    around.add(Bound.of(face, cell.centre()));
                }
                cells.add(new CellBounds(cell, around));
            }
            for (Field.Element part : hive.parts()) {
                Solid.part(part).ifPresent(solid -> solids.add(solid));
            }
            double lx = Double.MAX_VALUE, ly = Double.MAX_VALUE, lz = Double.MAX_VALUE;
            double mx = -Double.MAX_VALUE, my = -Double.MAX_VALUE, mz = -Double.MAX_VALUE;
            for (Solid solid : solids) {
                for (Face face : solid.faces.all()) {
                    for (Corner corner : face.corners.all()) {
                        Vec3 at = corner.at();
                        lx = Math.min(lx, at.x());
                        ly = Math.min(ly, at.y());
                        lz = Math.min(lz, at.z());
                        mx = Math.max(mx, at.x());
                        my = Math.max(my, at.y());
                        mz = Math.max(mz, at.z());
                    }
                }
            }
            return new Shape(
                    Collections.unmodifiableList(solids),
                    new Vec3(lx, ly, lz),
                    new Vec3(mx, my, mz),
                    Collections.unmodifiableList(cells));
        }
    }

    private record Held(Field.Hive hive, Shape shape, Lean lean) {}

    private final Field field;
    private final List<Held> hives;

    private Hives(Field field, List<Held> hives) {
        this.field = field;
        this.hives = Collections.unmodifiableList(hives);
    }

    public static Hives of(Field field) {
        List<Held> hives = new ArrayList<>();
        for (Field.Hive hive : field.hives()) {
            hives.add(new Held(hive, Shape.of(hive), new Lean.Resting(hive.tilt())));
        }
        return new Hives(field, hives);
    }

    public Field field() {
        return field;
    }

    public Optional<Field.Hive> hiveOf(String alliance) {
        for (Held held : hives) {
            if (held.hive().alliance().equals(alliance)) {
                return Optional.of(held.hive());
            }
        }
        return Optional.empty();
    }

    public Lean lean(Field.Hive hive) {
        for (Held held : hives) {
            if (held.hive().equals(hive)) {
                return held.lean();
            }
        }
        return new Lean.Resting(hive.tilt());
    }

    public double tilt(Field.Hive hive) {
        return lean(hive).degrees();
    }

    public Map<String, Double> tilts() {
        Map<String, Double> out = new LinkedHashMap<>();
        for (Held held : hives) {
            out.put(held.hive().alliance(), held.lean().degrees());
        }
        return out;
    }

    public boolean tipping(Field.Hive hive) {
        return lean(hive) instanceof Lean.Tipping;
    }

    public Optional<Field.Cell> upturnedCell(String alliance) {
        return hiveOf(alliance).flatMap(this::upturnedCell);
    }

    public Optional<Field.Cell> upturnedCell(Field.Hive hive) {
        Lean lean = lean(hive);
        double tilt = lean.degrees();
        Optional<Tilt> tippingTo = lean.tippingTo();
        Optional<Field.Cell> up = Optional.empty();
        double highest = -Double.MAX_VALUE;
        for (Field.Cell cell : hive.cells()) {
            double facing = cell.mouthNormalAt(tilt).z();
            if (facing > highest
                    || (facing == highest
                            && tippingTo
                                    .map(to -> cell.upturnedAt(to.degrees()))
                                    .orElse(false))) {
                highest = facing;
                up = Optional.of(cell);
            }
        }
        return up;
    }

    public boolean upturned(Field.Cell cell) {
        return upturnedCell(cell.alliance()).map(up -> up.equals(cell)).orElse(false);
    }

    public static int fills(Field.Kind kind) {
        return kind == Field.Kind.NECTAR ? NECTAR_FILLS : POLLEN_FILLS;
    }

    public Hives tipped(Field.Hive hive) {
        List<Held> next = new ArrayList<>();
        for (Held held : hives) {
            next.add(
                    held.hive().equals(hive)
                            ? new Held(held.hive(), held.shape(), held.lean().tipped())
                            : held);
        }
        return new Hives(field, next);
    }

    public Hives after(Seconds seconds) {
        List<Held> next = new ArrayList<>();
        for (Held held : hives) {
            next.add(new Held(held.hive(), held.shape(), held.lean().after(seconds)));
        }
        return new Hives(field, next);
    }

    public List<Touch> touching(Vec3 centre, Length radius) {
        double r = radius.inches();
        List<Touch> out = new ArrayList<>();
        for (Held held : hives) {
            if (!near(held, centre, r + CONTACT_TOLERANCE_IN)) {
                continue;
            }
            Field.Hive hive = held.hive();
            double tilt = held.lean().degrees();
            Vec3 local = inHive(hive, tilt, centre);
            for (Solid solid : held.shape().solids()) {
                if (local.minus(solid.middle).length() > solid.reach + r + CONTACT_TOLERANCE_IN) {
                    continue;
                }
                Vec3 nearest = local;
                double closest = Double.MAX_VALUE;
                for (Face face : solid.faces.all()) {
                    Vec3 onIt = face.nearestTo(local);
                    double apart = local.minus(onIt).length();
                    if (apart < closest) {
                        closest = apart;
                        nearest = onIt;
                    }
                }
                Optional<Vec3> inside = solid.outOfItFrom(local);
                double depth;
                Vec3 normal;
                if (inside.isPresent()) {
                    normal = inside.orElse(Vec3.zero());
                    depth = r + closest;
                } else if (closest >= r + CONTACT_TOLERANCE_IN) {
                    continue;
                } else if (closest > TOUCHING_IN) {
                    normal = local.minus(nearest).times(1 / closest);
                    depth = r - closest;
                } else {
                    normal = solid.faces.first().normal;
                    depth = r;
                }
                Vec3 touched = hive.at(tilt, nearest);
                out.add(new Touch(hive.direction(tilt, normal), depth, motionOf(hive, held.lean(), touched)));
            }
        }
        return out;
    }

    boolean near(Field.Hive hive, Vec3 centre, double within) {
        for (Held held : hives) {
            if (held.hive().equals(hive)) {
                return near(held, centre, within);
            }
        }
        return false;
    }

    private static boolean near(Held held, Vec3 centre, double within) {
        Vec3 local = inHive(held.hive(), held.lean().degrees(), centre);
        Vec3 least = held.shape().least(), most = held.shape().most();
        return !(local.x() < least.x() - within
                || local.x() > most.x() + within
                || local.y() < least.y() - within
                || local.y() > most.y() + within
                || local.z() < least.z() - within
                || local.z() > most.z() + within);
    }

    public Optional<Field.Cell> cellHolding(Vec3 point) {
        for (Held held : hives) {
            for (CellBounds cell : held.shape().cells()) {
                Vec3 local = inHive(held.hive(), held.lean().degrees(), point);
                boolean inside = true;
                for (Bound bound : cell.bounds()) {
                    inside &= local.minus(bound.on()).dot(bound.inward()) >= 0;
                }
                if (inside) {
                    return Optional.of(cell.cell());
                }
            }
        }
        return Optional.empty();
    }

    public List<Vec3> restingSpots(Field.Cell cell, Length radius) {
        double r = radius.inches();
        double lowest = Double.MAX_VALUE;
        for (Vec3 corner : cell.back().all()) {
            lowest = Math.min(lowest, corner.z());
        }
        List<Vec3> floor = new ArrayList<>();
        for (Vec3 corner : cell.back().all()) {
            if (corner.z() <= lowest + FLOOR_BAND_IN) {
                floor.add(corner);
            }
        }
        floor.sort((a, b) -> Double.compare(a.y(), b.y()));
        Optional<Ring<Vec3>> sorted = Ring.of(floor).fold(Optional::of, rule -> Optional.empty());
        if (sorted.isEmpty()) {
            return List.of();
        }
        Ring<Vec3> across = sorted.orElse(Ring.of(Vec3.zero(), List.of()));
        Vec3 atTheBack = mean(floor);
        double width = last(across).y() - across.first().y();
        double toward = Math.signum(cell.mouthNormal().x());
        double deep = Math.abs(cell.mouthCentre().x() - atTheBack.x());
        int rows = Math.max(1, (int) (deep / (2 * r)));
        int perRow = Math.max(1, (int) (width / (2 * r)));
        double tilt = tilt(cell.hive());
        List<Vec3> spots = new ArrayList<>();
        for (int row = 0; row < rows; row++) {
            for (int slot = 0; slot < perRow; slot++) {
                spots.add(cell.hive()
                        .at(
                                tilt,
                                new Vec3(
                                        atTheBack.x() + toward * (r + row * 2 * r),
                                        atTheBack.y() + (slot - (perRow - 1) / 2.0) * 2 * r,
                                        atTheBack.z() + r)));
            }
        }
        return spots;
    }

    private static Vec3 motionOf(Field.Hive hive, Lean lean, Vec3 point) {
        if (!(lean instanceof Lean.Tipping)) {
            return Vec3.zero();
        }
        double radiansPerSecond = Math.toRadians(lean.degreesPerSecond());
        return new Vec3(
                -radiansPerSecond * (point.z() - hive.pivot().z()),
                0,
                radiansPerSecond * (point.x() - hive.pivot().x()));
    }

    private static Vec3 inHive(Field.Hive hive, double tilt, Vec3 point) {
        double c = Math.cos(Math.toRadians(tilt)), s = Math.sin(Math.toRadians(tilt));
        Vec3 pivot = hive.pivot();
        double dx = point.x() - pivot.x(), dy = point.y() - pivot.y(), dz = point.z() - pivot.z();
        return new Vec3(dx * c + dz * s, dy, -dx * s + dz * c);
    }

    static Vec3 normalOf(Ring<Vec3> ring) {
        Vec3 n = Vec3.newellNormalOf(ring);
        double length = Math.sqrt(n.x() * n.x() + n.y() * n.y() + n.z() * n.z());
        return new Vec3(n.x() / length, n.y() / length, n.z() / length);
    }

    private static Vec3 nearestOnSegment(Vec3 a, Vec3 b, Vec3 point) {
        Vec3 ab = b.minus(a);
        double squared = ab.dot(ab);
        double t = squared == 0 ? 0 : Math.max(0, Math.min(1, point.minus(a).dot(ab) / squared));
        return a.along(ab, t);
    }

    private static Vec3 mean(List<Vec3> points) {
        double x = 0, y = 0, z = 0;
        for (Vec3 point : points) {
            x += point.x() / points.size();
            y += point.y() / points.size();
            z += point.z() / points.size();
        }
        return new Vec3(x, y, z);
    }

    private static <T> T last(Ring<T> ring) {
        T last = ring.first();
        for (T element : ring.all()) {
            last = element;
        }
        return last;
    }
}
