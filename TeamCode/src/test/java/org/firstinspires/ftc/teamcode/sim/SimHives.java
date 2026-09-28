package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Ring;
import org.firstinspires.ftc.teamcode.simcore.Vec3;

public final class SimHives {
    public static final int FULL = 40;
    public static final int NECTAR_FILLS = FULL / 5;
    public static final int POLLEN_FILLS = FULL / 8;

    public static final double TIP_SECONDS = 1;

    private static final double CONTACT_TOLERANCE_IN = 0.02;

    public static final class Touch {
        public final double[] normal;

        public final double depth;

        public final double[] velocity;

        Touch(double[] normal, double depth, double[] velocity) {
            this.normal = normal;
            this.depth = depth;
            this.velocity = velocity;
        }
    }

    private static final class Face {
        final double[][] corners;
        final double[] normal;
        final double[] u;
        final double[] v;
        final double[][] flat;

        Face(double[][] corners) {
            this.corners = corners;
            this.normal = Points.normal(corners);
            double[] edge = sub(corners[1], corners[0]);
            for (int i = 1; dot(edge, edge) < 1e-12 && i < corners.length; i++) {
                edge = sub(corners[(i + 1) % corners.length], corners[i]);
            }
            this.u = unit(edge);
            this.v = cross(normal, u);
            this.flat = new double[corners.length][];
            for (int i = 0; i < corners.length; i++) {
                double[] from = sub(corners[i], corners[0]);
                flat[i] = new double[] {dot(from, u), dot(from, v)};
            }
        }

        double[] nearestTo(double[] point) {
            double[] from = sub(point, corners[0]);
            double s = dot(from, u), t = dot(from, v);
            boolean inside = false;
            for (int i = 0, j = flat.length - 1; i < flat.length; j = i++) {
                double[] a = flat[i], b = flat[j];
                if ((a[1] > t) != (b[1] > t) && s < (b[0] - a[0]) * (t - a[1]) / (b[1] - a[1]) + a[0]) {
                    inside = !inside;
                }
            }
            if (inside) {
                return along(point, normal, -dot(from, normal));
            }
            double[] nearest = null;
            double closest = Double.MAX_VALUE;
            for (int i = 0, j = corners.length - 1; i < corners.length; j = i++) {
                double[] onEdge = nearestOnSegment(corners[j], corners[i], point);
                double[] gap = sub(point, onEdge);
                if (dot(gap, gap) < closest) {
                    closest = dot(gap, gap);
                    nearest = onEdge;
                }
            }
            return nearest;
        }
    }

    private static final class Solid {
        final List<Face> faces;
        final boolean closed;
        final double[] middle;
        final double reach;

        Solid(List<Face> faces, boolean closed) {
            this.faces = faces;
            this.closed = closed;
            List<double[]> corners = new ArrayList<>();
            for (Face face : faces) {
                for (double[] corner : face.corners) {
                    corners.add(corner);
                }
            }
            this.middle = mean(corners);
            double reach = 0;
            for (double[] corner : corners) {
                reach = Math.max(reach, length(sub(corner, middle)));
            }
            this.reach = reach;
            if (closed) {
                for (Face face : faces) {
                    if (dot(sub(face.corners[0], middle), face.normal) < 0) {
                        for (int axis = 0; axis < 3; axis++) {
                            face.normal[axis] = -face.normal[axis];
                        }
                    }
                }
            }
        }

        static Solid plate(double[][] corners) {
            return new Solid(List.of(new Face(corners)), false);
        }

        static Solid part(Field.Element part) {
            List<Face> faces = new ArrayList<>();
            for (Ring<Vec3> ring : part.faces()) {
                faces.add(new Face(Points.arrays(ring)));
            }
            return new Solid(faces, true);
        }

        double[] outOfItFrom(double[] point) {
            if (closed) {
                Face shallowest = null;
                double least = Double.MAX_VALUE;
                for (Face face : faces) {
                    double under = -dot(sub(point, face.corners[0]), face.normal);
                    if (under < 0) {
                        return null;
                    }
                    if (under < least) {
                        least = under;
                        shallowest = face;
                    }
                }
                return shallowest.normal.clone();
            }
            return null;
        }
    }

    private static final class Bound {
        final double[] on;
        final double[] inward;

        Bound(double[][] ring, double[] centre) {
            this.on = ring[0];
            double[] normal = Points.normal(ring);
            this.inward = dot(sub(centre, on), normal) >= 0 ? normal : scale(normal, -1);
        }
    }

    private static final class Tip {
        final double from;
        final double to;
        double elapsed;

        Tip(double from, double to) {
            this.from = from;
            this.to = to;
        }

        double tilt() {
            return from + (to - from) * (1 - Math.cos(Math.PI * elapsed / TIP_SECONDS)) / 2;
        }

        double degreesPerSecond() {
            return (to - from) * Math.PI / (2 * TIP_SECONDS) * Math.sin(Math.PI * elapsed / TIP_SECONDS);
        }
    }

    private final Field field;
    private final Map<Field.Hive, Double> tilts = new LinkedHashMap<>();
    private final Map<Field.Hive, Tip> tipping = new LinkedHashMap<>();
    private final Map<Field.Hive, List<Solid>> solids = new LinkedHashMap<>();
    private final Map<Field.Hive, double[][]> extents = new LinkedHashMap<>();
    private final Map<Field.Cell, List<Bound>> bounds = new LinkedHashMap<>();

    public SimHives(Field field) {
        this.field = field;
        for (Field.Hive hive : field.hives()) {
            tilts.put(hive, hive.tilt().degrees());
            List<Solid> of = new ArrayList<>();
            for (Field.Cell cell : hive.cells()) {
                for (Ring<Vec3> panel : cell.panels()) {
                    of.add(Solid.plate(Points.arrays(panel)));
                }
                List<Bound> around = new ArrayList<>();
                List<Ring<Vec3>> faces = new ArrayList<>(cell.panels());
                faces.add(cell.mouth());
                for (Ring<Vec3> face : faces) {
                    around.add(new Bound(Points.arrays(face), Points.array(cell.centre())));
                }
                bounds.put(cell, around);
            }
            for (Field.Element part : hive.parts()) {
                of.add(Solid.part(part));
            }
            solids.put(hive, of);
            double[] least = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
            double[] most = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
            for (Solid solid : of) {
                for (Face face : solid.faces) {
                    for (double[] corner : face.corners) {
                        for (int axis = 0; axis < 3; axis++) {
                            least[axis] = Math.min(least[axis], corner[axis]);
                            most[axis] = Math.max(most[axis], corner[axis]);
                        }
                    }
                }
            }
            extents.put(hive, new double[][] {least, most});
        }
    }

    public Field.Hive hiveOf(String alliance) {
        for (Field.Hive hive : field.hives()) {
            if (hive.alliance().equals(alliance)) {
                return hive;
            }
        }
        throw new IllegalArgumentException("no hive for " + alliance);
    }

    public double tilt(String alliance) {
        return tilts.get(hiveOf(alliance));
    }

    public Map<String, Double> tilts() {
        Map<String, Double> out = new LinkedHashMap<>();
        for (Field.Hive hive : field.hives()) {
            out.put(hive.alliance(), tilts.get(hive));
        }
        return out;
    }

    public boolean tipping(Field.Hive hive) {
        return tipping.containsKey(hive);
    }

    public Field.Cell upturnedCell(String alliance) {
        Field.Hive hive = hiveOf(alliance);
        double tilt = tilts.get(hive);
        Tip tip = tipping.get(hive);
        Field.Cell up = null;
        double highest = -Double.MAX_VALUE;
        for (Field.Cell cell : hive.cells()) {
            double facing = cell.mouthNormalAt(tilt).z();
            if (facing > highest || (facing == highest && tip != null && cell.upturnedAt(tip.to))) {
                highest = facing;
                up = cell;
            }
        }
        return up;
    }

    public boolean upturned(Field.Cell cell) {
        return upturnedCell(cell.alliance()) == cell;
    }

    public static int fills(Field.Kind kind) {
        return kind == Field.Kind.NECTAR ? NECTAR_FILLS : POLLEN_FILLS;
    }

    public void tip(Field.Hive hive) {
        if (!tipping.containsKey(hive)) {
            tipping.put(hive, new Tip(tilts.get(hive), -tilts.get(hive)));
        }
    }

    public void advance(double seconds) {
        for (Field.Hive hive : field.hives()) {
            Tip tip = tipping.get(hive);
            if (tip == null) {
                continue;
            }
            tip.elapsed += seconds;
            if (tip.elapsed >= TIP_SECONDS) {
                tilts.put(hive, tip.to);
                tipping.remove(hive);
            } else {
                tilts.put(hive, tip.tilt());
            }
        }
    }

    public List<Touch> touching(double[] centre, double radius) {
        List<Touch> out = new ArrayList<>();
        for (Field.Hive hive : field.hives()) {
            if (!near(hive, centre, radius + CONTACT_TOLERANCE_IN)) {
                continue;
            }
            double tilt = tilts.get(hive);
            double[] local = inHive(hive, tilt, centre);
            for (Solid solid : solids.get(hive)) {
                if (length(sub(local, solid.middle)) > solid.reach + radius + CONTACT_TOLERANCE_IN) {
                    continue;
                }
                double[] nearest = null;
                double closest = Double.MAX_VALUE;
                for (Face face : solid.faces) {
                    double[] onIt = face.nearestTo(local);
                    double apart = length(sub(local, onIt));
                    if (apart < closest) {
                        closest = apart;
                        nearest = onIt;
                    }
                }
                double[] inside = solid.outOfItFrom(local);
                double depth;
                double[] normal;
                if (inside != null) {
                    normal = inside;
                    depth = radius + closest;
                } else if (closest >= radius + CONTACT_TOLERANCE_IN) {
                    continue;
                } else if (closest > 1e-9) {
                    normal = scale(sub(local, nearest), 1 / closest);
                    depth = radius - closest;
                } else {
                    normal = solid.faces.get(0).normal.clone();
                    depth = radius;
                }
                double[] touched = Points.array(hive.at(tilt, Points.vec(nearest)));
                out.add(new Touch(
                        Points.array(hive.direction(tilt, Points.vec(normal))), depth, motionOf(hive, touched)));
            }
        }
        return out;
    }

    public boolean near(Field.Hive hive, double[] centre, double within) {
        double[] local = inHive(hive, tilts.get(hive), centre);
        double[][] extent = extents.get(hive);
        for (int axis = 0; axis < 3; axis++) {
            if (local[axis] < extent[0][axis] - within || local[axis] > extent[1][axis] + within) {
                return false;
            }
        }
        return true;
    }

    public Field.Cell cellHolding(double[] point) {
        for (Field.Cell cell : field.cells()) {
            double[] local = inHive(cell.hive(), tilts.get(cell.hive()), point);
            boolean inside = true;
            for (Bound bound : bounds.get(cell)) {
                inside &= dot(sub(local, bound.on), bound.inward) >= 0;
            }
            if (inside) {
                return cell;
            }
        }
        return null;
    }

    public List<double[]> restingSpots(Field.Cell cell, double radius) {
        double[][] back = Points.arrays(cell.back());
        double lowest = Double.MAX_VALUE;
        for (double[] corner : back) {
            lowest = Math.min(lowest, corner[2]);
        }
        List<double[]> floor = new ArrayList<>();
        for (double[] corner : back) {
            if (corner[2] <= lowest + 0.2) {
                floor.add(corner);
            }
        }
        floor.sort((a, b) -> Double.compare(a[1], b[1]));
        double[] atTheBack = mean(floor);
        double width = floor.get(floor.size() - 1)[1] - floor.get(0)[1];
        double toward = Math.signum(cell.mouthNormal().x());
        double deep = Math.abs(cell.mouthCentre().x() - atTheBack[0]);
        int rows = Math.max(1, (int) (deep / (2 * radius)));
        int perRow = Math.max(1, (int) (width / (2 * radius)));
        double tilt = tilts.get(cell.hive());
        List<double[]> spots = new ArrayList<>();
        for (int row = 0; row < rows; row++) {
            for (int slot = 0; slot < perRow; slot++) {
                spots.add(Points.array(cell.hive()
                        .at(
                                tilt,
                                new Vec3(
                                        atTheBack[0] + toward * (radius + row * 2 * radius),
                                        atTheBack[1] + (slot - (perRow - 1) / 2.0) * 2 * radius,
                                        atTheBack[2] + radius))));
            }
        }
        return spots;
    }

    private double[] motionOf(Field.Hive hive, double[] point) {
        Tip tip = tipping.get(hive);
        if (tip == null) {
            return new double[3];
        }
        double radiansPerSecond = Math.toRadians(tip.degreesPerSecond());
        return new double[] {
            -radiansPerSecond * (point[2] - hive.pivot().z()),
            0,
            radiansPerSecond * (point[0] - hive.pivot().x())
        };
    }

    private static double[] inHive(Field.Hive hive, double tilt, double[] point) {
        double c = Math.cos(Math.toRadians(tilt)), s = Math.sin(Math.toRadians(tilt));
        Vec3 pivot = hive.pivot();
        double dx = point[0] - pivot.x(), dy = point[1] - pivot.y(), dz = point[2] - pivot.z();
        return new double[] {dx * c + dz * s, dy, -dx * s + dz * c};
    }

    private static double[] nearestOnSegment(double[] a, double[] b, double[] point) {
        double[] ab = sub(b, a);
        double squared = dot(ab, ab);
        double t = squared == 0 ? 0 : Math.max(0, Math.min(1, dot(sub(point, a), ab) / squared));
        return along(a, ab, t);
    }

    private static double[] mean(List<double[]> points) {
        double[] out = new double[3];
        for (double[] point : points) {
            for (int axis = 0; axis < 3; axis++) {
                out[axis] += point[axis] / points.size();
            }
        }
        return out;
    }

    static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    static double[] scale(double[] a, double by) {
        return new double[] {a[0] * by, a[1] * by, a[2] * by};
    }

    static double[] along(double[] from, double[] direction, double distance) {
        return new double[] {
            from[0] + direction[0] * distance, from[1] + direction[1] * distance, from[2] + direction[2] * distance
        };
    }

    static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    static double length(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    static double[] unit(double[] a) {
        return scale(a, 1 / length(a));
    }
}
