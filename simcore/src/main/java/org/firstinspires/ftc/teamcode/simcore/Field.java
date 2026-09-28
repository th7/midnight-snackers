package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public final class Field {
    public enum Kind {
        POLLEN("Pollen"),
        NECTAR("Nectar");

        private final String named;

        Kind(String named) {
            this.named = named;
        }

        public String named() {
            return named;
        }

        public static Checked<Kind> named(String name) {
            for (Kind kind : values()) {
                if (kind.named.equals(name)) {
                    return Checked.ok(kind);
                }
            }
            return Checked.rejected("a piece is a Pollen or a Nectar, not " + name);
        }
    }

    public static final class Obstacle {
        private final String name;
        private final ConvexPolygon footprint;
        private final double clears;
        private final double stands;

        private Obstacle(String name, ConvexPolygon footprint, double clears, double stands) {
            this.name = name;
            this.footprint = footprint;
            this.clears = clears;
            this.stands = stands;
        }

        public static Checked<Obstacle> of(String name, ConvexPolygon footprint, double clears, double stands) {
            if (!(Double.isFinite(clears) && Double.isFinite(stands) && stands > clears)) {
                return Checked.rejected(
                        name + " stands above where it clears the floor, not " + stands + " over " + clears);
            }
            return Checked.ok(new Obstacle(name, footprint, clears, stands));
        }

        public String name() {
            return name;
        }

        public ConvexPolygon footprint() {
            return footprint;
        }

        public double clears() {
            return clears;
        }

        public double stands() {
            return stands;
        }
    }

    public static final class Flower {
        private final String name;
        private final Vec2 axis;
        private final double bore;
        private final double gap;
        private final double lip;
        private final double nest;

        private Flower(String name, Vec2 axis, double bore, double gap, double lip, double nest) {
            this.name = name;
            this.axis = axis;
            this.bore = bore;
            this.gap = gap;
            this.lip = lip;
            this.nest = nest;
        }

        public static Checked<Flower> of(String name, Vec2 axis, double bore, double gap, double lip, double nest) {
            if (!(Double.isFinite(axis.x()) && Double.isFinite(axis.y()))) {
                return Checked.rejected(name + " stands at a finite axis, not " + axis);
            }
            for (double size : List.of(bore, gap, lip, nest)) {
                if (!(size > 0 && Double.isFinite(size))) {
                    return Checked.rejected(name + "'s bore, gap, lip and nest are positive and finite, not "
                            + List.of(bore, gap, lip, nest));
                }
            }
            return Checked.ok(new Flower(name, axis, bore, gap, lip, nest));
        }

        public String name() {
            return name;
        }

        public Vec2 axis() {
            return axis;
        }

        public double bore() {
            return bore;
        }

        public double gap() {
            return gap;
        }

        public double lip() {
            return lip;
        }

        public double nest() {
            return nest;
        }

        public boolean standsIn(double x, double y) {
            return Math.hypot(x - axis.x(), y - axis.y()) <= bore;
        }
    }

    public static final class Element {
        private static final int LEAST_FACES = 4;
        private static final int LEAST_CORNERS = 3;

        private final String group;
        private final String name;
        private final String colour;
        private final List<Vec3> vertices;
        private final List<Ring<Vec3>> faces;

        private Element(String group, String name, String colour, List<Vec3> vertices, List<Ring<Vec3>> faces) {
            this.group = group;
            this.name = name;
            this.colour = colour;
            this.vertices = Collections.unmodifiableList(new ArrayList<>(vertices));
            this.faces = Collections.unmodifiableList(new ArrayList<>(faces));
        }

        public static Checked<Element> of(
                String group, String name, String colour, List<Vec3> vertices, List<List<Integer>> faces) {
            Map<Integer, Vec3> numbered = new HashMap<>();
            int number = 0;
            for (Vec3 vertex : vertices) {
                if (!vertex.isFinite()) {
                    return Checked.rejected(name + "'s vertices are finite, not " + vertex);
                }
                numbered.put(number, vertex);
                number++;
            }
            if (faces.size() < LEAST_FACES) {
                return Checked.rejected(
                        name + " is a closed shape of at least " + LEAST_FACES + " faces, not " + faces.size());
            }
            List<Ring<Vec3>> rings = new ArrayList<>();
            for (List<Integer> face : faces) {
                List<Vec3> corners = new ArrayList<>();
                for (int index : face) {
                    Vec3 corner = numbered.get(index);
                    if (corner == null) {
                        return Checked.rejected(name + " has " + vertices.size() + " vertices, and no vertex " + index);
                    }
                    corners.add(corner);
                }
                if (corners.size() < LEAST_CORNERS) {
                    return Checked.rejected(name + "'s faces have at least " + LEAST_CORNERS + " corners, not " + face);
                }
                Checked<Ring<Vec3>> ring =
                        Ring.of(corners).then(r -> Vec3.unitNormalOf(r).map(normal -> r));
                if (ring instanceof Checked.Rejected<Ring<Vec3>> rejected) {
                    return Checked.rejected(name + ": " + rejected.rule());
                }
                ring.fold(r -> rings.add(r), rule -> false);
            }
            return Checked.ok(new Element(group, name, colour, vertices, rings));
        }

        public String group() {
            return group;
        }

        public String name() {
            return name;
        }

        public String colour() {
            return colour;
        }

        public List<Vec3> vertices() {
            return vertices;
        }

        public List<Ring<Vec3>> faces() {
            return faces;
        }
    }

    public record CellShape(String name, String side, List<Vec3> mouth, List<Vec3> back, List<List<Vec3>> walls) {}

    public static final class Hive {
        private final String name;
        private final String alliance;
        private final String colour;
        private final Vec3 pivot;
        private final Tilt tilt;
        private final List<Cell> cells;
        private final List<Element> parts;

        private Hive(String name, String alliance, String colour, Vec3 pivot, Tilt tilt, List<Element> parts) {
            this.name = name;
            this.alliance = alliance;
            this.colour = colour;
            this.pivot = pivot;
            this.tilt = tilt;
            this.cells = new ArrayList<>();
            this.parts = Collections.unmodifiableList(new ArrayList<>(parts));
        }

        public static Checked<Hive> of(
                String name,
                String alliance,
                String colour,
                Vec3 pivot,
                Tilt tilt,
                List<CellShape> cells,
                List<Element> parts) {
            if (!pivot.isFinite()) {
                return Checked.rejected(name + " turns about a finite pivot, not " + pivot);
            }
            if (cells.isEmpty()) {
                return Checked.rejected(name + " has a cell to score in");
            }
            Hive hive = new Hive(name, alliance, colour, pivot, tilt, parts);
            for (CellShape shape : cells) {
                Checked<Cell> cell = Cell.of(hive, shape);
                if (cell instanceof Checked.Rejected<Cell> rejected) {
                    return Checked.rejected(rejected.rule());
                }
                cell.fold(c -> hive.cells.add(c), rule -> false);
            }
            return Checked.ok(hive);
        }

        public String name() {
            return name;
        }

        public String alliance() {
            return alliance;
        }

        public String colour() {
            return colour;
        }

        public Vec3 pivot() {
            return pivot;
        }

        public Tilt tilt() {
            return tilt;
        }

        public List<Cell> cells() {
            return Collections.unmodifiableList(cells);
        }

        public List<Element> parts() {
            return parts;
        }

        public Vec3 at(double tilt, Vec3 local) {
            Vec3 turned = direction(tilt, local);
            return new Vec3(pivot.x() + turned.x(), pivot.y() + turned.y(), pivot.z() + turned.z());
        }

        public Vec3 direction(double tilt, Vec3 local) {
            double c = Math.cos(Math.toRadians(tilt)), s = Math.sin(Math.toRadians(tilt));
            return new Vec3(local.x() * c - local.z() * s, local.y(), local.x() * s + local.z() * c);
        }

        public Ring<Vec3> at(double tilt, Ring<Vec3> local) {
            return local.map(corner -> at(tilt, corner));
        }
    }

    public static final class Cell {
        private static final int LEAST_CORNERS = 3;

        private final Hive hive;
        private final String name;
        private final String side;
        private final Ring<Vec3> mouth;
        private final Ring<Vec3> back;
        private final List<Ring<Vec3>> walls;
        private final List<Ring<Vec3>> panels;
        private final Vec3 centre;
        private final Vec3 mouthCentre;
        private final Vec3 mouthNormal;

        private Cell(
                Hive hive,
                String name,
                String side,
                Ring<Vec3> mouth,
                Ring<Vec3> back,
                List<Ring<Vec3>> walls,
                Vec3 mouthNormal) {
            this.hive = hive;
            this.name = name;
            this.side = side;
            this.mouth = mouth;
            this.back = back;
            this.walls = Collections.unmodifiableList(new ArrayList<>(walls));
            List<Ring<Vec3>> panels = new ArrayList<>(walls);
            panels.add(back);
            this.panels = Collections.unmodifiableList(panels);
            this.centre = mean(List.of(mouth, back));
            this.mouthCentre = mean(List.of(mouth));
            double outward = 0;
            outward += (mouthCentre.x() - centre.x()) * mouthNormal.x();
            outward += (mouthCentre.y() - centre.y()) * mouthNormal.y();
            outward += (mouthCentre.z() - centre.z()) * mouthNormal.z();
            this.mouthNormal = outward < 0 ? mouthNormal.negated() : mouthNormal;
        }

        static Checked<Cell> of(Hive hive, CellShape shape) {
            List<List<Vec3>> rings = new ArrayList<>(shape.walls());
            rings.add(shape.mouth());
            rings.add(shape.back());
            for (List<Vec3> ring : rings) {
                if (ring.size() < LEAST_CORNERS) {
                    return Checked.rejected(shape.name() + "'s mouth, back and walls have at least " + LEAST_CORNERS
                            + " corners, not " + ring);
                }
                for (Vec3 corner : ring) {
                    if (!corner.isFinite()) {
                        return Checked.rejected(shape.name() + "'s corners are finite, not " + corner);
                    }
                }
            }
            List<Ring<Vec3>> walls = new ArrayList<>();
            for (List<Vec3> wall : shape.walls()) {
                Ring.of(wall).fold(r -> walls.add(r), rule -> false);
            }
            return Ring.of(shape.mouth())
                    .then(mouth -> Ring.of(shape.back())
                            .then(back -> Vec3.unitNormalOf(mouth)
                                    .map(normal ->
                                            new Cell(hive, shape.name(), shape.side(), mouth, back, walls, normal))));
        }

        private static Vec3 mean(List<Ring<Vec3>> rings) {
            double x = 0, y = 0, z = 0;
            int count = 0;
            for (Ring<Vec3> ring : rings) {
                for (Vec3 v : ring.all()) {
                    x += v.x();
                    y += v.y();
                    z += v.z();
                    count++;
                }
            }
            return new Vec3(x / count, y / count, z / count);
        }

        public Hive hive() {
            return hive;
        }

        public String name() {
            return name;
        }

        public String alliance() {
            return hive.alliance();
        }

        public String side() {
            return side;
        }

        public Ring<Vec3> mouth() {
            return mouth;
        }

        public Ring<Vec3> back() {
            return back;
        }

        public List<Ring<Vec3>> walls() {
            return walls;
        }

        public List<Ring<Vec3>> panels() {
            return panels;
        }

        public Vec3 centre() {
            return centre;
        }

        public Vec3 mouthCentre() {
            return mouthCentre;
        }

        public Vec3 mouthNormal() {
            return mouthNormal;
        }

        public Vec3 centreAt(double tilt) {
            return hive.at(tilt, centre);
        }

        public Vec3 mouthCentreAt(double tilt) {
            return hive.at(tilt, mouthCentre);
        }

        public Vec3 mouthNormalAt(double tilt) {
            return hive.direction(tilt, mouthNormal);
        }

        public Ring<Vec3> mouthAt(double tilt) {
            return hive.at(tilt, mouth);
        }

        public List<Ring<Vec3>> panelsAt(double tilt) {
            List<Ring<Vec3>> out = new ArrayList<>();
            for (Ring<Vec3> panel : panels) {
                out.add(hive.at(tilt, panel));
            }
            return out;
        }

        public boolean upturnedAt(double tilt) {
            return mouthNormalAt(tilt).z() > 0;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    public sealed interface Place permits Place.Loose, Place.InCell, Place.InFlower {
        record Loose() implements Place {}

        record InCell(Cell cell) implements Place {}

        record InFlower(Flower flower) implements Place {}
    }

    public static final class Piece {
        private final String name;
        private final Kind kind;
        private final Place place;
        private final Vec3 at;
        private final Length radius;

        private Piece(String name, Kind kind, Place place, Vec3 at, Length radius) {
            this.name = name;
            this.kind = kind;
            this.place = place;
            this.at = at;
            this.radius = radius;
        }

        public static Checked<Piece> of(String name, Kind kind, Place place, Vec3 at, Length radius) {
            if (!at.isFinite()) {
                return Checked.rejected(name + " is at a finite position, not " + at);
            }
            return Checked.ok(new Piece(name, kind, place, at, radius));
        }

        public String name() {
            return name;
        }

        public Kind kind() {
            return kind;
        }

        public Optional<String> alliance() {
            return name.startsWith("Blue")
                    ? Optional.of("Blue")
                    : name.startsWith("Red") ? Optional.of("Red") : Optional.empty();
        }

        public Place place() {
            return place;
        }

        public Vec3 at() {
            return at;
        }

        public double radius() {
            return radius.inches();
        }
    }

    public record Mark(String colour, List<Vec2> footprint) {}

    public static final class AllianceArea {
        public static final double EYE_IN = 60;

        private final String alliance;
        private final double minX;
        private final double maxX;
        private final double minY;
        private final double maxY;

        private AllianceArea(String alliance, double minX, double maxX, double minY, double maxY) {
            this.alliance = alliance;
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }

        static Checked<AllianceArea> markedFor(Hive hive, double size, List<Mark> tape) {
            double half = size / 2;
            double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
            double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
            for (Mark mark : tape) {
                if (!mark.colour().equalsIgnoreCase(hive.colour())) {
                    continue;
                }
                boolean outside = true;
                for (Vec2 corner : mark.footprint()) {
                    outside &= Math.max(Math.abs(corner.x()), Math.abs(corner.y())) > half;
                }
                if (!outside) {
                    continue;
                }
                for (Vec2 corner : mark.footprint()) {
                    minX = Math.min(minX, corner.x());
                    maxX = Math.max(maxX, corner.x());
                    minY = Math.min(minY, corner.y());
                    maxY = Math.max(maxY, corner.y());
                }
            }
            if (!(minX <= maxX && minY <= maxY)) {
                return Checked.rejected("the field marks no area outside the walls in " + hive.colour()
                        + ", the colour of the " + hive.alliance() + " hive, so there is nowhere for "
                        + hive.alliance() + "'s drivers to stand");
            }
            return Checked.ok(new AllianceArea(hive.alliance(), minX, maxX, minY, maxY));
        }

        public String alliance() {
            return alliance;
        }

        public double minX() {
            return minX;
        }

        public double maxX() {
            return maxX;
        }

        public double minY() {
            return minY;
        }

        public double maxY() {
            return maxY;
        }

        public Vec3 eye() {
            return new Vec3((minX + maxX) / 2, (minY + maxY) / 2, EYE_IN);
        }

        public Vec3 lookingAt() {
            return Vec3.zero();
        }

        @Override
        public String toString() {
            return alliance + "'s area, x " + minX + ".." + maxX + ", y " + minY + ".." + maxY;
        }
    }

    private final Length size;
    private final Length wallHeight;
    private final List<Element> elements;
    private final List<Obstacle> obstacles;
    private final List<Hive> hives;
    private final List<Flower> flowers;
    private final List<Piece> pieces;
    private final List<AllianceArea> allianceAreas;

    private Field(
            Length size,
            Length wallHeight,
            List<Element> elements,
            List<Obstacle> obstacles,
            List<Hive> hives,
            List<Flower> flowers,
            List<Piece> pieces,
            List<AllianceArea> allianceAreas) {
        this.size = size;
        this.wallHeight = wallHeight;
        this.elements = Collections.unmodifiableList(new ArrayList<>(elements));
        this.obstacles = Collections.unmodifiableList(new ArrayList<>(obstacles));
        this.hives = Collections.unmodifiableList(new ArrayList<>(hives));
        this.flowers = Collections.unmodifiableList(new ArrayList<>(flowers));
        this.pieces = Collections.unmodifiableList(new ArrayList<>(pieces));
        this.allianceAreas = Collections.unmodifiableList(allianceAreas);
    }

    public static Checked<Field> of(
            Length size,
            Length wallHeight,
            List<Element> elements,
            List<Obstacle> obstacles,
            List<Hive> hives,
            List<Flower> flowers,
            List<Piece> pieces,
            List<Mark> tape) {
        List<AllianceArea> areas = new ArrayList<>();
        for (Hive hive : hives) {
            Checked<AllianceArea> area = AllianceArea.markedFor(hive, size.inches(), tape);
            if (area instanceof Checked.Rejected<AllianceArea> rejected) {
                return Checked.rejected(rejected.rule());
            }
            area.fold(a -> areas.add(a), rule -> false);
        }
        return Checked.ok(new Field(size, wallHeight, elements, obstacles, hives, flowers, pieces, areas));
    }

    public double size() {
        return size.inches();
    }

    public double wallHeight() {
        return wallHeight.inches();
    }

    public List<Element> elements() {
        return elements;
    }

    public List<Obstacle> obstacles() {
        return obstacles;
    }

    public List<Hive> hives() {
        return hives;
    }

    public List<Cell> cells() {
        List<Cell> cells = new ArrayList<>();
        for (Hive hive : hives) {
            cells.addAll(hive.cells());
        }
        return Collections.unmodifiableList(cells);
    }

    public List<Flower> flowers() {
        return flowers;
    }

    public List<Piece> loosePieces() {
        List<Piece> loose = new ArrayList<>();
        for (Piece piece : pieces) {
            if (piece.place() instanceof Place.Loose) {
                loose.add(piece);
            }
        }
        return Collections.unmodifiableList(loose);
    }

    public List<Piece> cellPieces() {
        List<Piece> inCells = new ArrayList<>();
        for (Piece piece : pieces) {
            if (piece.place() instanceof Place.InCell) {
                inCells.add(piece);
            }
        }
        return Collections.unmodifiableList(inCells);
    }

    public List<Piece> flowerPieces() {
        List<Piece> inFlowers = new ArrayList<>();
        for (Piece piece : pieces) {
            if (piece.place() instanceof Place.InFlower) {
                inFlowers.add(piece);
            }
        }
        return Collections.unmodifiableList(inFlowers);
    }

    public List<Piece> movedPieces() {
        List<Piece> moved = new ArrayList<>(loosePieces());
        moved.addAll(cellPieces());
        moved.addAll(flowerPieces());
        return Collections.unmodifiableList(moved);
    }

    public List<AllianceArea> allianceAreas() {
        return allianceAreas;
    }

    public Checked<AllianceArea> allianceArea(String alliance) {
        List<String> known = new ArrayList<>();
        for (AllianceArea area : allianceAreas) {
            if (area.alliance().equals(alliance)) {
                return Checked.ok(area);
            }
            known.add(area.alliance());
        }
        return Checked.rejected("this field has nowhere for " + alliance + " to stand, only " + known);
    }

    public Optional<Obstacle> obstacle(String name) {
        for (Obstacle obstacle : obstacles) {
            if (obstacle.name().equals(name)) {
                return Optional.of(obstacle);
            }
        }
        return Optional.empty();
    }

    public Optional<Hive> hive(String name) {
        for (Hive hive : hives) {
            if (hive.name().equals(name)) {
                return Optional.of(hive);
            }
        }
        return Optional.empty();
    }

    public Optional<Flower> flower(String name) {
        for (Flower flower : flowers) {
            if (flower.name().equals(name)) {
                return Optional.of(flower);
            }
        }
        return Optional.empty();
    }

    public Optional<Cell> cell(String name) {
        for (Cell cell : cells()) {
            if (cell.name().equals(name)) {
                return Optional.of(cell);
            }
        }
        return Optional.empty();
    }
}
