package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public final class Colliders {
    public static final double WALL_THICKNESS_IN = 1;
    public static final double CONVEX_TOLERANCE_IN = 0.02;

    public record Near(Collider collider, Vec3 normal, double gap) {}

    public enum Side {
        AUDIENCE("the audience's wall", -1, 0),
        FAR("the far wall", 1, 0),
        BLUE("blue's wall", 0, -1),
        RED("red's wall", 0, 1);

        private final String says;
        private final double x;
        private final double y;

        Side(String says, double x, double y) {
            this.says = says;
            this.x = x;
            this.y = y;
        }

        public String says() {
            return says;
        }

        private double beyond(Vec3 centre, double outerFace) {
            return centre.x() * x + centre.y() * y - outerFace;
        }
    }

    public record InCell(String cell, String alliance, boolean upturned) {}

    record Framed(Field.Hive hive, List<Collider> colliders, Map<String, Field.Cell> panels) {}

    public record Crossing(
            String cell,
            String alliance,
            boolean upturned,
            boolean into,
            double fraction,
            Vec3 at,
            double offset,
            double rim,
            double angleDegrees) {}

    public record Mouth(String cell, String alliance, boolean upturned, Vec3 centre, Vec3 normal, List<Vec3> corners) {
        public Mouth {
            corners = Collections.unmodifiableList(new ArrayList<>(corners));
        }
    }

    private record Basket(Mouth mouth, Collider.Face opening, List<Collider.Face> bounds) {
        boolean holds(Vec3 point) {
            for (Collider.Face bound : bounds) {
                if (bound.above(point) > 0) {
                    return false;
                }
            }
            return true;
        }
    }

    private record Bore(String flower, double x, double y, double radius, double top) {
        boolean holds(Vec3 point) {
            return StrictMath.hypot(point.x() - x, point.y() - y) <= radius && point.z() <= top;
        }
    }

    private record Tilted(Vec3 pivot, double cos, double sin) {
        static Tilted by(Vec3 pivot, double degrees) {
            double radians = StrictMath.toRadians(degrees);
            return new Tilted(pivot, StrictMath.cos(radians), StrictMath.sin(radians));
        }

        Vec3 at(Vec3 local) {
            Vec3 turned = new Vec3(local.x() * cos - local.z() * sin, local.y(), local.x() * sin + local.z() * cos);
            return new Vec3(pivot.x() + turned.x(), pivot.y() + turned.y(), pivot.z() + turned.z());
        }

        Ring<Vec3> ring(Ring<Vec3> local) {
            return local.map(this::at);
        }

        List<Ring<Vec3>> rings(List<Ring<Vec3>> local) {
            List<Ring<Vec3>> turned = new ArrayList<>();
            for (Ring<Vec3> ring : local) {
                turned.add(ring(ring));
            }
            return turned;
        }
    }

    private final double size;
    private final double wallHeight;
    private final List<Collider> all;
    private final Map<String, Double> tilts;
    private final List<Basket> baskets;
    private final List<Bore> bores;

    private Colliders(
            Field field, List<Collider> all, Map<String, Double> tilts, List<Basket> baskets, List<Bore> bores) {
        this.size = field.size();
        this.wallHeight = field.wallHeight();
        this.all = Collections.unmodifiableList(new ArrayList<>(all));
        this.tilts = Collections.unmodifiableMap(new LinkedHashMap<>(tilts));
        this.baskets = Collections.unmodifiableList(new ArrayList<>(baskets));
        this.bores = Collections.unmodifiableList(new ArrayList<>(bores));
    }

    public static Checked<Colliders> of(Field field, List<String> tippedAlliances) {
        List<String> alliances = new ArrayList<>();
        for (Field.Hive hive : field.hives()) {
            alliances.add(hive.alliance());
        }
        for (String alliance : tippedAlliances) {
            if (!alliances.contains(alliance)) {
                return Checked.rejected("only a hive the field has can be tipped, and it has hives for " + alliances
                        + ", not for " + alliance);
            }
        }
        Map<String, Integer> seen = new LinkedHashMap<>();
        List<Checked<Collider>> made = floorWallsAndElements(field, seen);
        Map<String, Double> tilts = new LinkedHashMap<>();
        List<Checked<Basket>> baskets = new ArrayList<>();
        for (Field.Hive hive : field.hives()) {
            double rest = hive.tilt().degrees();
            double tilt = tippedAlliances.contains(hive.alliance()) ? -rest : rest;
            tilts.put(hive.alliance(), tilt);
            Tilted turn = Tilted.by(hive.pivot(), tilt);
            for (Field.Element part : hive.parts()) {
                made.add(Collider.solid(numbered(part, seen), Surface.FRAME, turn.rings(part.faces())));
            }
            for (Field.Cell cell : hive.cells()) {
                for (Map.Entry<String, Ring<Vec3>> panel : panelsOf(cell).entrySet()) {
                    made.add(Collider.plate(panel.getKey(), Surface.CELL, turn.ring(panel.getValue())));
                }
                baskets.add(basketOf(cell, turn));
            }
        }
        List<Bore> bores = boresOf(field);
        return allOf(made)
                .then(Colliders::namedApart)
                .then(colliders -> allOf(baskets).map(cells -> new Colliders(field, colliders, tilts, cells, bores)));
    }

    public static Checked<Colliders> withoutHives(Field field) {
        List<Bore> bores = boresOf(field);
        return allOf(floorWallsAndElements(field, new LinkedHashMap<>()))
                .then(Colliders::namedApart)
                .map(colliders -> new Colliders(field, colliders, new LinkedHashMap<>(), List.of(), bores));
    }

    private static List<Checked<Collider>> floorWallsAndElements(Field field, Map<String, Integer> seen) {
        List<Checked<Collider>> made = new ArrayList<>();
        made.add(Checked.ok(Collider.floor("Floor")));
        made.addAll(walls(field.size() / 2, field.wallHeight()));
        for (Field.Element element : field.elements()) {
            Surface surface = element.group().startsWith("Flower") ? Surface.FLOWER : Surface.FRAME;
            made.add(Collider.solid(numbered(element, seen), surface, element.faces()));
        }
        return made;
    }

    private static List<Bore> boresOf(Field field) {
        List<Bore> bores = new ArrayList<>();
        for (Field.Flower flower : field.flowers()) {
            bores.add(boreOf(flower, field.elements()));
        }
        return bores;
    }

    static Checked<List<Framed>> inTheirHivesFrames(Field field) {
        Map<String, Integer> seen = new LinkedHashMap<>();
        for (Field.Element element : field.elements()) {
            numbered(element, seen);
        }
        List<Checked<Framed>> framed = new ArrayList<>();
        for (Field.Hive hive : field.hives()) {
            List<Checked<Collider>> made = new ArrayList<>();
            Map<String, Field.Cell> panels = new LinkedHashMap<>();
            for (Field.Element part : hive.parts()) {
                made.add(Collider.solid(numbered(part, seen), Surface.FRAME, part.faces()));
            }
            for (Field.Cell cell : hive.cells()) {
                for (Map.Entry<String, Ring<Vec3>> panel : panelsOf(cell).entrySet()) {
                    made.add(Collider.plate(panel.getKey(), Surface.CELL, panel.getValue()));
                    panels.put(panel.getKey(), cell);
                }
            }
            framed.add(allOf(made)
                    .map(colliders -> new Framed(
                            hive, Collections.unmodifiableList(colliders), Collections.unmodifiableMap(panels))));
        }
        return allOf(framed);
    }

    private static Checked<List<Collider>> namedApart(List<Collider> colliders) {
        Set<String> named = new HashSet<>();
        for (Collider collider : colliders) {
            if (!named.add(collider.name())) {
                return Checked.rejected(
                        "every collider has a name of its own, and '" + collider.name() + "' is not one");
            }
        }
        return Checked.ok(colliders);
    }

    private static List<Checked<Collider>> walls(double half, double height) {
        double outer = half + WALL_THICKNESS_IN;
        return List.of(
                Collider.box("Far wall", Surface.WALL, new Vec3(half, -outer, 0), new Vec3(outer, outer, height)),
                Collider.box(
                        "Audience wall", Surface.WALL, new Vec3(-outer, -outer, 0), new Vec3(-half, outer, height)),
                Collider.box("Red wall", Surface.WALL, new Vec3(-outer, half, 0), new Vec3(outer, outer, height)),
                Collider.box("Blue wall", Surface.WALL, new Vec3(-outer, -outer, 0), new Vec3(outer, -half, height)));
    }

    private static String numbered(Field.Element element, Map<String, Integer> seen) {
        String named = element.group() + " / " + element.name();
        int number = seen.getOrDefault(named, 0) + 1;
        seen.put(named, number);
        return named + " <" + number + ">";
    }

    private static Map<String, Ring<Vec3>> panelsOf(Field.Cell cell) {
        Map<String, Ring<Vec3>> panels = new LinkedHashMap<>();
        int number = 0;
        for (Ring<Vec3> wall : cell.walls()) {
            number++;
            panels.put(cell.name() + " / wall <" + number + ">", wall);
        }
        panels.put(cell.name() + " / back", cell.back());
        return panels;
    }

    private static Checked<Basket> basketOf(Field.Cell cell, Tilted turn) {
        Vec3 middle = turn.at(cell.centre());
        List<Checked<Collider.Face>> made = new ArrayList<>();
        for (Map.Entry<String, Ring<Vec3>> panel : panelsOf(cell).entrySet()) {
            made.add(Collider.Face.of(panel.getKey(), turn.ring(panel.getValue())));
        }
        return Collider.Face.of(cell.name() + "'s mouth", turn.ring(cell.mouth()))
                .then(mouth -> allOf(made).map(panels -> {
                    Collider.Face opening = mouth.outwardFrom(middle);
                    List<Collider.Face> bounds = new ArrayList<>();
                    for (Collider.Face panel : panels) {
                        bounds.add(panel.outwardFrom(middle));
                    }
                    bounds.add(opening);
                    return new Basket(
                            new Mouth(
                                    cell.name(),
                                    cell.alliance(),
                                    opening.normal().z() > 0,
                                    opening.centre(),
                                    opening.normal(),
                                    opening.corners()),
                            opening,
                            Collections.unmodifiableList(bounds));
                }));
    }

    private static Bore boreOf(Field.Flower flower, List<Field.Element> elements) {
        double top = flower.lip();
        for (Field.Element element : elements) {
            if (element.group().equals(flower.name())) {
                for (Vec3 vertex : element.vertices()) {
                    top = StrictMath.max(top, vertex.z());
                }
            }
        }
        return new Bore(flower.name(), flower.axis().x(), flower.axis().y(), flower.bore(), top);
    }

    private static <T> Checked<List<T>> allOf(List<Checked<T>> made) {
        List<T> all = new ArrayList<>();
        for (Checked<T> one : made) {
            if (one instanceof Checked.Rejected<T> rejected) {
                return Checked.rejected(rejected.rule());
            }
            one.fold(value -> all.add(value), rule -> false);
        }
        return Checked.ok(all);
    }

    public List<Collider> all() {
        return all;
    }

    public double gap(Collider collider, Vec3 centre, Length radius) {
        return collider.near(centre, radius.inches()).gap();
    }

    public Near near(Collider collider, Vec3 centre, Length radius) {
        return collider.near(centre, radius.inches());
    }

    public List<Near> near(Vec3 centre, Length radius, double within) {
        double r = radius.inches();
        double reach = r + within;
        List<Near> near = new ArrayList<>();
        for (Collider collider : all) {
            if (collider.reaches(centre, reach)) {
                Near one = collider.near(centre, r);
                if (one.gap() <= within) {
                    near.add(one);
                }
            }
        }
        return near;
    }

    public Optional<Side> over(Vec3 centre, Length radius) {
        double outerFace = size / 2 + WALL_THICKNESS_IN;
        Optional<Side> over = Optional.empty();
        double furthest = radius.inches();
        for (Side side : Side.values()) {
            double beyond = side.beyond(centre, outerFace);
            if (beyond > furthest) {
                furthest = beyond;
                over = Optional.of(side);
            }
        }
        return over;
    }

    public Optional<InCell> cellHolding(Vec3 point) {
        for (Basket basket : baskets) {
            if (basket.holds(point)) {
                Mouth mouth = basket.mouth();
                return Optional.of(new InCell(mouth.cell(), mouth.alliance(), mouth.upturned()));
            }
        }
        return Optional.empty();
    }

    public Optional<String> flowerHolding(Vec3 point) {
        for (Bore bore : bores) {
            if (bore.holds(point)) {
                return Optional.of(bore.flower());
            }
        }
        return Optional.empty();
    }

    public Optional<Crossing> crossed(Vec3 from, Vec3 to) {
        Optional<Crossing> first = Optional.empty();
        double earliest = Double.POSITIVE_INFINITY;
        for (Basket basket : baskets) {
            Collider.Face opening = basket.opening();
            double before = opening.above(from), after = opening.above(to);
            boolean into = before >= 0 && after < 0;
            boolean out = before < 0 && after >= 0;
            if (!(into || out)) {
                continue;
            }
            double fraction = before / (before - after);
            Vec3 step = to.minus(from);
            Vec3 at = from.along(step, fraction);
            if (!(opening.holds(at) && fraction < earliest)) {
                continue;
            }
            earliest = fraction;
            Mouth mouth = basket.mouth();
            Vec3 axis = into ? mouth.normal().negated() : mouth.normal();
            double cosine = step.dot(axis) / Collider.lengthOf(step);
            first = Optional.of(new Crossing(
                    mouth.cell(),
                    mouth.alliance(),
                    mouth.upturned(),
                    into,
                    fraction,
                    at,
                    StrictMath.sqrt(Collider.squaredDistance(at, mouth.centre())),
                    StrictMath.sqrt(Collider.squaredDistance(at, opening.nearestOnRim(at))),
                    StrictMath.toDegrees(StrictMath.acos(StrictMath.max(-1, StrictMath.min(1, cosine))))));
        }
        return first;
    }

    public List<Mouth> mouths() {
        List<Mouth> mouths = new ArrayList<>();
        for (Basket basket : baskets) {
            mouths.add(basket.mouth());
        }
        return Collections.unmodifiableList(mouths);
    }

    public double tilt(String alliance) {
        return tilts.getOrDefault(alliance, 0.0);
    }

    public Map<String, Double> tilts() {
        return tilts;
    }

    public double size() {
        return size;
    }

    public double wallHeight() {
        return wallHeight;
    }
}
