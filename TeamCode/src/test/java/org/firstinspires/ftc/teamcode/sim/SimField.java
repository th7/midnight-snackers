package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * The season's field, reduced from FIRST's CAD by {@code tools/field/step_to_field.py} to
 * {@code field.json} next to this class: the size between the walls and their height, each field
 * element as a low-poly convex shape, the hives, the flowers, the game pieces where a match
 * starts, the gaffer tape on the floor, and the <b>obstacles</b>: the convex footprint of every
 * part of an element that stands lower than the robot is tall, which is what {@link SimRobot} runs
 * into. Each obstacle says how high it stands and how far it clears the floor, so a ball rolls
 * under one that overhangs. The game pieces on the floor in the open are <b>loose</b>: the
 * simulator rolls them, and the ticks say where they are; so are the pollen a flower holds, which
 * stand in its bore. The replay page draws the same model, obstacles included, so what it draws is
 * what the simulator collides.
 * <p>
 * Everything is in the field frame Road Runner uses: inches, origin at the centre of the field,
 * +x away from the audience, +y to the audience's left, +z up. A hive is the exception: it turns
 * about its axle, so what it is made of is given in its own frame and {@link Hive#at} says where
 * that is at the tilt the hive leans at.
 */
public final class SimField {
    /** A game piece's kind, which says how much of a hive's load it is. */
    public static final String NECTAR = "Nectar";

    public static final String POLLEN = "Pollen";

    /**
     * A convex polygon on the floor, wound counter-clockwise, that the robot's footprint may not
     * enter: one part of a field element, so that what is driven through between — the frame's
     * legs, a flower's pipes — blocks nothing. How high it {@link #stands} and how far it
     * {@link #clears} the floor say what meets it: a ball rolls under one that overhangs it.
     */
    public static final class Obstacle {
        public final String name;
        public final double[][] footprint;
        /** How high above the floor its underside is, in inches: how much rolls under it. */
        public final double clears;
        /** How high above the floor its top is, in inches. */
        public final double stands;

        Obstacle(String name, double[][] footprint, double clears, double stands) {
            this.name = name;
            this.footprint = footprint;
            this.clears = clears;
            this.stands = stands;
        }
    }

    /**
     * A flower: the tower at a wall whose four pipes make the <b>bore</b> a stack of pollen stands
     * in, one on another from the floor up. The bore is a circle on the floor — the {@link #axis}
     * midway between the pipes and the {@link #bore} radius the nearest of them leaves clear — and
     * the {@link #gap} between two neighbouring pipes is narrower than a pollen, so what is in the
     * bore stays in it. The bore's wall begins at the {@link #lip}, the height the pipes start at:
     * a pollen standing wholly below the lip is held by nothing above it, which is why the one at
     * the bottom of the stack is the one that comes out. What holds it is the {@link #nest}: the
     * ring around the bottom of the bore — the flower's base plate — that it sits in the middle
     * of and has to climb to leave, carrying whatever rests on it.
     */
    public static final class Flower {
        public final String name;
        /** Where the bore stands on the floor, {x, y} in the field frame. */
        public final double[] axis;
        /** How far from the axis the pipes leave clear, in inches. */
        public final double bore;
        /** The narrowest gap between two of the pipes, in inches: what keeps a pollen in the bore. */
        public final double gap;
        /** How high the pipes begin, in inches: below it the bore's wall reaches nothing. */
        public final double lip;
        /** How high the ring around the bottom of the bore stands, in inches: the climb out of the nest. */
        public final double nest;

        Flower(JsonObject json, Gson gson) {
            this.name = json.get("name").getAsString();
            this.axis = gson.fromJson(json.get("axis"), double[].class);
            this.bore = json.get("bore").getAsDouble();
            this.gap = json.get("gap").getAsDouble();
            this.lip = json.get("lip").getAsDouble();
            this.nest = json.get("nest").getAsDouble();
        }

        /** Whether a ball standing at {@code x, y} is in the bore, and so under what the bore holds. */
        public boolean standsIn(double x, double y) {
            return Math.hypot(x - axis[0], y - axis[1]) <= bore;
        }
    }

    /** A field element's shape: its vertices and its faces, each a ring of vertex indices seen from outside. */
    public static final class Element {
        public final String group;
        public final String name;
        public final String colour;
        public final double[][] vertices;
        public final int[][] faces;

        Element(String group, String name, String colour, double[][] vertices, int[][] faces) {
            this.group = group;
            this.name = name;
            this.colour = colour;
            this.vertices = vertices;
            this.faces = faces;
        }
    }

    /**
     * A hive: the see-saw that hangs from the frame's top bar with a {@link Cell} at each end and
     * tips one way or the other about its axle. Everything it is made of is given in the hive's
     * own frame — the origin on the axle, +x along the beam toward the scoring cell with the beam
     * level, +y the field's and +z up — so that the <b>tilt</b> it leans at, in degrees above
     * level, is all that says where it is. {@link #tilt} is the tilt the field is set up at; the
     * hive tips to the other side of level, {@code -tilt}.
     */
    public static final class Hive {
        public final String name;
        /** "Blue" or "Red": whose hive it is. */
        public final String alliance;

        public final String colour;
        /** The middle of the axle it turns on, in the field frame. */
        public final double[] pivot;
        /** How far above level the beam leans toward the scoring cell, in degrees, as the field is set up. */
        public final double tilt;

        public final List<Cell> cells;
        /** What the hive is made of besides its cells, in the hive's frame: drawn, and turns with it. */
        public final List<Element> parts;

        Hive(JsonObject json, Gson gson) {
            this.name = json.get("name").getAsString();
            this.alliance = json.get("alliance").getAsString();
            this.colour = json.get("colour").getAsString();
            this.pivot = gson.fromJson(json.get("pivot"), double[].class);
            this.tilt = json.get("tilt").getAsDouble();
            List<Element> parts = new ArrayList<>();
            for (JsonElement part : json.getAsJsonArray("parts")) {
                JsonObject p = part.getAsJsonObject();
                parts.add(new Element(
                        name,
                        p.get("name").getAsString(),
                        p.get("colour").getAsString(),
                        gson.fromJson(p.get("vertices"), double[][].class),
                        gson.fromJson(p.get("faces"), int[][].class)));
            }
            this.parts = Collections.unmodifiableList(parts);
            List<Cell> cells = new ArrayList<>();
            for (JsonElement cell : json.getAsJsonArray("cells")) {
                cells.add(new Cell(this, cell.getAsJsonObject(), gson));
            }
            this.cells = Collections.unmodifiableList(cells);
        }

        /** Where the hive's point {@code local} is in the field frame, with the hive leaning at {@code tilt}. */
        public double[] at(double tilt, double[] local) {
            double[] turned = direction(tilt, local);
            return new double[] {pivot[0] + turned[0], pivot[1] + turned[1], pivot[2] + turned[2]};
        }

        /** Where the hive's direction {@code local} points in the field frame at that tilt. */
        public double[] direction(double tilt, double[] local) {
            double c = Math.cos(Math.toRadians(tilt)), s = Math.sin(Math.toRadians(tilt));
            return new double[] {local[0] * c - local[2] * s, local[1], local[0] * s + local[2] * c};
        }

        /** The hive's ring {@code local} in the field frame at that tilt. */
        public double[][] at(double tilt, double[][] local) {
            double[][] out = new double[local.length][];
            for (int i = 0; i < local.length; i++) {
                out[i] = at(tilt, local[i]);
            }
            return out;
        }
    }

    /**
     * A hive cell: the basket a launched ball scores in, in its hive's frame. It is the opening
     * the CAD's goal ribs frame — the <b>mouth</b> — swept twelve inches to the <b>back</b> that
     * closes it, with a wall between every pair of the mouth's corners. A ball that crosses the
     * mouth going in is in the cell and one that meets a wall or the back bounces off, whichever
     * side it comes from.
     * <p>
     * Which way the cell is turned is the tilt's to say: a cell is <b>upturned</b> when its mouth
     * faces up, and then it holds what is in it, resting on the floor at the back; the cell at the
     * hive's other end is <b>downturned</b>, and what is in it rolls out of the mouth.
     */
    public static final class Cell {
        public final Hive hive;
        public final String name;
        /** "Blue" or "Red": whose hive the cell is in. */
        public final String alliance;
        /** "Audience" or "Scoring": which end of the hive the cell is at. */
        public final String side;
        /** The opening the ball comes in through, as its ring in the hive's frame. */
        public final double[][] mouth;
        /** The mouth's ring again at the closed end of the cell. */
        public final double[][] back;
        /** One wall per pair of the mouth's corners, each a ring in the hive's frame. */
        public final List<double[][]> walls;
        /** What stops a ball: the walls and the back. */
        public final List<double[][]> panels;
        /** The middle of the cell, in the hive's frame. */
        public final double[] centre;

        public final double[] mouthCentre;
        /** The mouth's unit normal in the hive's frame, pointing out of the cell. */
        public final double[] mouthNormal;

        Cell(Hive hive, JsonObject json, Gson gson) {
            this.hive = hive;
            this.name = json.get("name").getAsString();
            this.alliance = hive.alliance;
            this.side = json.get("side").getAsString();
            this.mouth = gson.fromJson(json.get("mouth"), double[][].class);
            this.back = gson.fromJson(json.get("back"), double[][].class);
            List<double[][]> walls = new ArrayList<>();
            for (JsonElement wall : json.getAsJsonArray("walls")) {
                walls.add(gson.fromJson(wall, double[][].class));
            }
            this.walls = Collections.unmodifiableList(walls);
            List<double[][]> panels = new ArrayList<>(walls);
            panels.add(back);
            this.panels = Collections.unmodifiableList(panels);
            this.centre = mean(List.<double[][]>of(mouth, back));
            this.mouthCentre = mean(List.<double[][]>of(mouth));
            double[] normal = normal(mouth);
            double outward = 0;
            for (int axis = 0; axis < 3; axis++) {
                outward += (mouthCentre[axis] - centre[axis]) * normal[axis];
            }
            if (outward < 0) {
                for (int axis = 0; axis < 3; axis++) {
                    normal[axis] = -normal[axis];
                }
            }
            this.mouthNormal = normal;
        }

        /** The middle of the cell in the field frame, with its hive leaning at that tilt. */
        public double[] centreAt(double tilt) {
            return hive.at(tilt, centre);
        }

        public double[] mouthCentreAt(double tilt) {
            return hive.at(tilt, mouthCentre);
        }

        /** The mouth's unit normal in the field frame at that tilt, pointing out of the cell. */
        public double[] mouthNormalAt(double tilt) {
            return hive.direction(tilt, mouthNormal);
        }

        public double[][] mouthAt(double tilt) {
            return hive.at(tilt, mouth);
        }

        /** The walls and the back in the field frame at that tilt. */
        public List<double[][]> panelsAt(double tilt) {
            List<double[][]> out = new ArrayList<>(panels.size());
            for (double[][] panel : panels) {
                out.add(hive.at(tilt, panel));
            }
            return out;
        }

        /**
         * Whether the cell holds what is in it at that tilt: its mouth faces up, so the balls rest
         * on the floor at the back. A downturned cell's roll out of the mouth.
         */
        public boolean upturnedAt(double tilt) {
            return mouthNormalAt(tilt)[2] > 0;
        }

        private static double[] mean(List<double[][]> rings) {
            double[] sum = new double[3];
            int count = 0;
            for (double[][] ring : rings) {
                for (double[] v : ring) {
                    for (int axis = 0; axis < 3; axis++) {
                        sum[axis] += v[axis];
                    }
                    count++;
                }
            }
            return new double[] {sum[0] / count, sum[1] / count, sum[2] / count};
        }
    }

    /**
     * A flat ring's unit normal by Newell's method, which is robust to near-collinear corners;
     * pointing the way the ring winds counter-clockwise around it.
     */
    static double[] normal(double[][] ring) {
        double[] n = new double[3];
        for (int i = 0; i < ring.length; i++) {
            double[] a = ring[i], b = ring[(i + 1) % ring.length];
            n[0] += (a[1] - b[1]) * (a[2] + b[2]);
            n[1] += (a[2] - b[2]) * (a[0] + b[0]);
            n[2] += (a[0] - b[0]) * (a[1] + b[1]);
        }
        double length = Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        return new double[] {n[0] / length, n[1] / length, n[2] / length};
    }

    /** A ball the field is set up with: on the floor in the open, stacked in a flower, or in a hive cell. */
    public static final class Piece {
        public final String name;
        /** {@link #NECTAR} or {@link #POLLEN}. */
        public final String kind;
        /** "Blue" or "Red" for a piece of one alliance's, else null. */
        public final String alliance;
        /** The cell the field is set up with the piece inside, or null. */
        public final String cell;
        /** The flower the field is set up with the piece stacked in the bore of, or null. */
        public final String flower;

        public final double x;
        public final double y;
        public final double z;
        public final double radius;

        Piece(String name, String kind, String cell, String flower, double x, double y, double z, double radius) {
            this.name = name;
            this.kind = kind;
            this.alliance = name.startsWith("Blue") ? "Blue" : name.startsWith("Red") ? "Red" : null;
            this.cell = cell;
            this.flower = flower;
            this.x = x;
            this.y = y;
            this.z = z;
            this.radius = radius;
        }
    }

    private static final String MODEL = "field.json";

    /** Between the walls, in inches; the field is a square of this side centred on the origin. */
    public final double size;

    public final double wallHeight;
    public final List<Element> elements;
    public final List<Obstacle> obstacles;
    /** The two hives, one per alliance. */
    public final List<Hive> hives;
    /** Every hive's cells, hive by hive: what a launched ball scores in. */
    public final List<Cell> cells;
    /** The four flowers, one at each wall: the bores the field's stacks of pollen stand in. */
    public final List<Flower> flowers;
    /** The game pieces the simulator rolls, in the order the page and the ticks name them. */
    public final List<Piece> loosePieces;
    /** The game pieces the field is set up with inside a hive cell: the nectar each hive starts with. */
    public final List<Piece> cellPieces;
    /** The game pieces the field is set up with stacked in a flower: the pollen each flower holds. */
    public final List<Piece> flowerPieces;
    /** Every piece a run moves, in the order a tick lists them. */
    public final List<Piece> movedPieces;

    private final JsonObject json;

    private SimField(JsonObject json) {
        this.json = json;
        this.size = json.get("size").getAsDouble();
        this.wallHeight = json.get("wallHeight").getAsDouble();
        Gson gson = new Gson();
        List<Element> elements = new ArrayList<>();
        for (JsonElement element : json.getAsJsonArray("elements")) {
            JsonObject e = element.getAsJsonObject();
            elements.add(new Element(
                    e.get("group").getAsString(),
                    e.get("name").getAsString(),
                    e.get("colour").getAsString(),
                    gson.fromJson(e.get("vertices"), double[][].class),
                    gson.fromJson(e.get("faces"), int[][].class)));
        }
        this.elements = Collections.unmodifiableList(elements);
        List<Hive> hives = new ArrayList<>();
        List<Cell> cells = new ArrayList<>();
        for (JsonElement hive : json.getAsJsonArray("hives")) {
            Hive built = new Hive(hive.getAsJsonObject(), gson);
            hives.add(built);
            cells.addAll(built.cells);
        }
        this.hives = Collections.unmodifiableList(hives);
        this.cells = Collections.unmodifiableList(cells);
        List<Flower> flowers = new ArrayList<>();
        for (JsonElement flower : json.getAsJsonArray("flowers")) {
            flowers.add(new Flower(flower.getAsJsonObject(), gson));
        }
        this.flowers = Collections.unmodifiableList(flowers);
        List<Piece> loose = new ArrayList<>();
        List<Piece> inCells = new ArrayList<>();
        List<Piece> inFlowers = new ArrayList<>();
        Map<Piece, JsonObject> pieceJson = new IdentityHashMap<>();
        JsonArray pieces = json.getAsJsonArray("pieces");
        for (int i = 0; i < pieces.size(); i++) {
            JsonObject p = pieces.get(i).getAsJsonObject();
            JsonElement cell = p.get("cell");
            JsonElement flower = p.get("flower");
            if (!p.get("loose").getAsBoolean() && cell == null && flower == null) {
                continue;
            }
            JsonArray centre = p.getAsJsonArray("centre");
            Piece piece = new Piece(
                    p.get("name").getAsString(),
                    p.get("kind").getAsString(),
                    cell == null ? null : cell.getAsString(),
                    flower == null ? null : flower.getAsString(),
                    centre.get(0).getAsDouble(),
                    centre.get(1).getAsDouble(),
                    centre.get(2).getAsDouble(),
                    p.get("radius").getAsDouble());
            (piece.cell != null ? inCells : piece.flower != null ? inFlowers : loose).add(piece);
            pieceJson.put(piece, p);
        }
        this.loosePieces = Collections.unmodifiableList(loose);
        this.cellPieces = Collections.unmodifiableList(inCells);
        this.flowerPieces = Collections.unmodifiableList(inFlowers);
        List<Piece> moved = new ArrayList<>(loose);
        moved.addAll(inCells);
        moved.addAll(inFlowers);
        this.movedPieces = Collections.unmodifiableList(moved);
        JsonArray movedJson = new JsonArray();
        for (Piece piece : moved) {
            movedJson.add(pieceJson.get(piece));
        }
        json.add("moved", movedJson);
        List<Obstacle> obstacles = new ArrayList<>();
        JsonArray array = json.getAsJsonArray("obstacles");
        for (int i = 0; i < array.size(); i++) {
            JsonObject o = array.get(i).getAsJsonObject();
            obstacles.add(new Obstacle(
                    o.get("name").getAsString(),
                    gson.fromJson(o.get("footprint"), double[][].class),
                    o.get("clears").getAsDouble(),
                    o.get("stands").getAsDouble()));
        }
        this.obstacles = Collections.unmodifiableList(obstacles);
    }

    /** The season's field, from the model next to this class. */
    public static SimField load() {
        try (InputStream in = SimField.class.getResourceAsStream(MODEL)) {
            if (in == null) {
                throw new IllegalStateException("missing resource " + MODEL + " next to " + SimField.class.getName());
            }
            return new SimField(
                    new Gson().fromJson(new InputStreamReader(in, StandardCharsets.UTF_8), JsonObject.class));
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /** The whole model as the page reads it. */
    public JsonObject json() {
        return json;
    }

    /** The obstacle of that name, or null. */
    public Obstacle obstacle(String name) {
        for (Obstacle obstacle : obstacles) {
            if (obstacle.name.equals(name)) {
                return obstacle;
            }
        }
        return null;
    }

    /** The hive of that name, or null. */
    public Hive hive(String name) {
        for (Hive hive : hives) {
            if (hive.name.equals(name)) {
                return hive;
            }
        }
        return null;
    }

    /** The flower of that name, or null. */
    public Flower flower(String name) {
        for (Flower flower : flowers) {
            if (flower.name.equals(name)) {
                return flower;
            }
        }
        return null;
    }

    /** The cell of that name, or null. */
    public Cell cell(String name) {
        for (Cell cell : cells) {
            if (cell.name.equals(name)) {
                return cell;
            }
        }
        return null;
    }
}
