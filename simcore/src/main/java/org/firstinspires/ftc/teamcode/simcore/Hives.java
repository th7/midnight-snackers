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

    private static final double FLOOR_BAND_IN = 0.2;

    public record Touch(String id, Surface surface, Vec3 normal, double gap, Vec3 velocity, Vec3 spin) {}

    private record Turned(Vec3 pivot, double cos, double sin) {
        static Turned by(Vec3 pivot, double degrees) {
            double radians = StrictMath.toRadians(degrees);
            return new Turned(pivot, StrictMath.cos(radians), StrictMath.sin(radians));
        }

        Vec3 direction(Vec3 local) {
            return new Vec3(local.x() * cos - local.z() * sin, local.y(), local.x() * sin + local.z() * cos);
        }

        Vec3 at(Vec3 local) {
            Vec3 turned = direction(local);
            return new Vec3(pivot.x() + turned.x(), pivot.y() + turned.y(), pivot.z() + turned.z());
        }

        Vec3 inHive(Vec3 point) {
            double dx = point.x() - pivot.x(), dy = point.y() - pivot.y(), dz = point.z() - pivot.z();
            return new Vec3(dx * cos + dz * sin, dy, -dx * sin + dz * cos);
        }
    }

    private record Bound(Vec3 on, Vec3 inward) {
        static Bound of(Ring<Vec3> ring, Vec3 centre) {
            Vec3 on = ring.first();
            Vec3 normal = normalOf(ring);
            return new Bound(on, centre.minus(on).dot(normal) >= 0 ? normal : normal.negated());
        }
    }

    private record CellBounds(Field.Cell cell, List<Bound> bounds) {
        boolean holds(Vec3 local) {
            for (Bound bound : bounds) {
                if (local.minus(bound.on()).dot(bound.inward()) < 0) {
                    return false;
                }
            }
            return true;
        }
    }

    private record Shape(
            List<Collider> colliders,
            Vec3 least,
            Vec3 most,
            double reach,
            List<CellBounds> cells,
            Map<String, Field.Cell> panels) {
        static Shape of(Field.Hive hive, List<Collider> colliders, Map<String, Field.Cell> panels) {
            double lx = Double.POSITIVE_INFINITY, ly = Double.POSITIVE_INFINITY, lz = Double.POSITIVE_INFINITY;
            double mx = Double.NEGATIVE_INFINITY, my = Double.NEGATIVE_INFINITY, mz = Double.NEGATIVE_INFINITY;
            double reach = 0;
            for (Collider collider : colliders) {
                lx = StrictMath.min(lx, collider.least().x());
                ly = StrictMath.min(ly, collider.least().y());
                lz = StrictMath.min(lz, collider.least().z());
                mx = StrictMath.max(mx, collider.most().x());
                my = StrictMath.max(my, collider.most().y());
                mz = StrictMath.max(mz, collider.most().z());
                for (Ring<Vec3> face : collider.faces()) {
                    for (Vec3 corner : face.all()) {
                        reach = StrictMath.max(reach, StrictMath.sqrt(corner.dot(corner)));
                    }
                }
            }
            List<CellBounds> cells = new ArrayList<>();
            for (Field.Cell cell : hive.cells()) {
                List<Ring<Vec3>> faces = new ArrayList<>(cell.panels());
                faces.add(cell.mouth());
                List<Bound> around = new ArrayList<>();
                for (Ring<Vec3> face : faces) {
                    around.add(Bound.of(face, cell.centre()));
                }
                cells.add(new CellBounds(cell, Collections.unmodifiableList(around)));
            }
            return new Shape(
                    colliders,
                    new Vec3(lx, ly, lz),
                    new Vec3(mx, my, mz),
                    reach,
                    Collections.unmodifiableList(cells),
                    panels);
        }

        boolean reaches(Vec3 local, double by) {
            return local.x() >= least.x() - by
                    && local.x() <= most.x() + by
                    && local.y() >= least.y() - by
                    && local.y() <= most.y() + by
                    && local.z() >= least.z() - by
                    && local.z() <= most.z() + by;
        }
    }

    private static final class Held {
        private final Field.Hive hive;
        private final Shape shape;
        private final Lean lean;
        private final double began;
        private final Turned turned;
        private final Vec3 spin;

        private Held(Field.Hive hive, Shape shape, Lean lean, double began) {
            this.hive = hive;
            this.shape = shape;
            this.lean = lean;
            this.began = began;
            this.turned = Turned.by(hive.pivot(), lean.degrees());
            this.spin = lean instanceof Lean.Tipping
                    ? new Vec3(0, -StrictMath.toRadians(lean.degreesPerSecond()), 0)
                    : Vec3.zero();
        }

        Held at(double seconds) {
            if (!(lean instanceof Lean.Tipping tipping)) {
                return this;
            }
            Seconds elapsed = Seconds.of(seconds - began).orElse(Seconds.zero());
            return new Held(
                    hive, shape, new Lean.Tipping(tipping.from(), tipping.to(), elapsed).after(Seconds.zero()), began);
        }

        Held tipped(double seconds) {
            return lean instanceof Lean.Tipping ? this : new Held(hive, shape, lean.tipped(), seconds);
        }

        boolean moving() {
            return lean instanceof Lean.Tipping;
        }

        double sweep() {
            return moving() ? StrictMath.toRadians(lean.fastestDegreesPerSecond()) * shape.reach() : 0;
        }

        boolean reaches(Vec3 centre, double by) {
            return shape.reaches(turned.inHive(centre), by);
        }

        void near(Vec3 centre, double radius, double within, List<Touch> touches) {
            Vec3 local = turned.inHive(centre);
            double reach = radius + within;
            if (!shape.reaches(local, reach)) {
                return;
            }
            for (Collider collider : shape.colliders()) {
                if (!collider.reaches(local, reach)) {
                    continue;
                }
                Colliders.Near near = collider.near(local, radius);
                if (!(near.gap() <= within)) {
                    continue;
                }
                Vec3 normal = turned.direction(near.normal());
                if (!moving()) {
                    touches.add(new Touch(
                            collider.name(), collider.surface(), normal, near.gap(), Vec3.zero(), Vec3.zero()));
                    continue;
                }
                Vec3 contact = centre.along(normal, -(near.gap() + radius));
                touches.add(new Touch(
                        collider.name(),
                        collider.surface(),
                        normal,
                        near.gap(),
                        spin.cross(contact.minus(hive.pivot())),
                        spin));
            }
        }

        Optional<Field.Cell> cellHolding(Vec3 point) {
            Vec3 local = turned.inHive(point);
            for (CellBounds cell : shape.cells()) {
                if (cell.holds(local)) {
                    return Optional.of(cell.cell());
                }
            }
            return Optional.empty();
        }
    }

    private static final class AsAPart implements Surroundings.Part {
        private final Hives hives;

        private AsAPart(Hives hives) {
            this.hives = hives;
        }

        @Override
        public List<String> ids() {
            return hives.ids;
        }

        @Override
        public boolean reaches(Vec3 centre, double by) {
            for (Held held : hives.hives) {
                if (held.reaches(centre, by)) {
                    return true;
                }
            }
            return false;
        }

        @Override
        public double sweep() {
            double sweep = 0;
            for (Held held : hives.hives) {
                sweep = StrictMath.max(sweep, held.sweep());
            }
            return sweep;
        }

        @Override
        public boolean moving() {
            return hives.moving();
        }

        @Override
        public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
            List<Surroundings.Touch> touches = new ArrayList<>();
            for (Touch touch : hives.near(centre, radius, within)) {
                Surroundings.Touch.of(
                                touch.id(),
                                touch.surface(),
                                touch.normal(),
                                touch.gap(),
                                touch.velocity(),
                                touch.spin())
                        .fold(made -> touches.add(made), rule -> false);
            }
            return touches;
        }

        @Override
        public Surroundings.Part at(double seconds) {
            return new AsAPart(hives.at(seconds));
        }
    }

    private final Field field;
    private final List<Held> hives;
    private final List<String> ids;
    private final double now;

    private Hives(Field field, List<Held> hives, List<String> ids, double now) {
        this.field = field;
        this.hives = Collections.unmodifiableList(hives);
        this.ids = ids;
        this.now = now;
    }

    public static Checked<Hives> of(Field field) {
        return Colliders.inTheirHivesFrames(field).map(framed -> {
            List<Held> hives = new ArrayList<>();
            List<String> ids = new ArrayList<>();
            for (Colliders.Framed hive : framed) {
                hives.add(new Held(
                        hive.hive(),
                        Shape.of(hive.hive(), hive.colliders(), hive.panels()),
                        new Lean.Resting(hive.hive().tilt()),
                        0));
                for (Collider collider : hive.colliders()) {
                    ids.add(collider.name());
                }
            }
            return new Hives(field, hives, Collections.unmodifiableList(ids), 0);
        });
    }

    public Field field() {
        return field;
    }

    public Optional<Field.Hive> hiveOf(String alliance) {
        for (Held held : hives) {
            if (held.hive.alliance().equals(alliance)) {
                return Optional.of(held.hive);
            }
        }
        return Optional.empty();
    }

    private Optional<Held> held(Field.Hive hive) {
        for (Held held : hives) {
            if (held.hive.equals(hive)) {
                return Optional.of(held);
            }
        }
        return Optional.empty();
    }

    public Lean lean(Field.Hive hive) {
        return held(hive).map(held -> held.lean).orElse(new Lean.Resting(hive.tilt()));
    }

    public double tilt(Field.Hive hive) {
        return lean(hive).degrees();
    }

    public Map<String, Double> tilts() {
        Map<String, Double> out = new LinkedHashMap<>();
        for (Held held : hives) {
            out.put(held.hive.alliance(), held.lean.degrees());
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
        return held(hive).flatMap(Hives::upturnedCell);
    }

    private static Optional<Field.Cell> upturnedCell(Held held) {
        Optional<Turned> tippingTo = held.lean.tippingTo().map(to -> Turned.by(held.hive.pivot(), to.degrees()));
        Optional<Field.Cell> up = Optional.empty();
        double highest = Double.NEGATIVE_INFINITY;
        for (Field.Cell cell : held.hive.cells()) {
            double facing = held.turned.direction(cell.mouthNormal()).z();
            if (facing > highest
                    || (facing == highest
                            && tippingTo
                                    .map(to -> to.direction(cell.mouthNormal()).z() > 0)
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
            next.add(held.hive.equals(hive) ? held.tipped(now) : held);
        }
        return new Hives(field, next, ids, now);
    }

    public Hives after(Seconds seconds) {
        return at(now + seconds.value());
    }

    Hives at(double seconds) {
        List<Held> then = new ArrayList<>();
        for (Held held : hives) {
            then.add(held.at(seconds));
        }
        return new Hives(field, then, ids, seconds);
    }

    boolean moving() {
        for (Held held : hives) {
            if (held.moving()) {
                return true;
            }
        }
        return false;
    }

    Surroundings.Part part() {
        return new AsAPart(this);
    }

    Optional<Field.Cell> cellOf(String id) {
        for (Held held : hives) {
            Optional<Field.Cell> cell = Optional.ofNullable(held.shape.panels().get(id));
            if (cell.isPresent()) {
                return cell;
            }
        }
        return Optional.empty();
    }

    public List<Touch> near(Vec3 centre, Length radius, double within) {
        List<Touch> touches = new ArrayList<>();
        for (Held held : hives) {
            held.near(centre, radius.inches(), within, touches);
        }
        return touches;
    }

    public Optional<Field.Cell> cellHolding(Vec3 point) {
        for (Held held : hives) {
            Optional<Field.Cell> cell = held.cellHolding(point);
            if (cell.isPresent()) {
                return cell;
            }
        }
        return Optional.empty();
    }

    public List<Vec3> restingSpots(Field.Cell cell, Length radius) {
        return held(cell.hive())
                .map(held -> restingSpots(held.turned, cell, radius))
                .orElse(List.of());
    }

    private static List<Vec3> restingSpots(Turned turned, Field.Cell cell, Length radius) {
        double r = radius.inches();
        double lowest = Double.POSITIVE_INFINITY;
        for (Vec3 corner : cell.back().all()) {
            lowest = StrictMath.min(lowest, corner.z());
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
        double toward = StrictMath.signum(cell.mouthNormal().x());
        double deep = StrictMath.abs(cell.mouthCentre().x() - atTheBack.x());
        int rows = (int) StrictMath.max(1, StrictMath.floor(deep / (2 * r)));
        int perRow = (int) StrictMath.max(1, StrictMath.floor(width / (2 * r)));
        List<Vec3> spots = new ArrayList<>();
        for (int row = 0; row < rows; row++) {
            for (int slot = 0; slot < perRow; slot++) {
                spots.add(turned.at(new Vec3(
                        atTheBack.x() + toward * (r + row * 2 * r),
                        atTheBack.y() + (slot - (perRow - 1) / 2.0) * 2 * r,
                        atTheBack.z() + r)));
            }
        }
        return spots;
    }

    static Vec3 normalOf(Ring<Vec3> ring) {
        Vec3 n = Vec3.newellNormalOf(ring);
        double length = StrictMath.sqrt(n.x() * n.x() + n.y() * n.y() + n.z() * n.z());
        return new Vec3(n.x() / length, n.y() / length, n.z() / length);
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
