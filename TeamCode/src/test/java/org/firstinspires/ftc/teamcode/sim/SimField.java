package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The season's field, reduced from FIRST's CAD by {@code tools/field/step_to_field.py} to
 * {@code field.json} next to this class: the size between the walls and their height, each field
 * element as a low-poly convex shape, the game pieces where a match starts, the gaffer tape on the
 * floor, and the <b>obstacles</b>: the convex footprint of every element that stands lower than
 * the robot is tall, which is what {@link SimRobot} runs into. The game pieces on the floor in the
 * open are <b>loose</b>: the simulator rolls them, and the ticks say where they are. The replay
 * page draws the same model, obstacles included, so what it draws is what the simulator collides.
 * <p>
 * Everything is in the field frame Road Runner uses: inches, origin at the centre of the field,
 * +x away from the audience, +y to the audience's left, +z up.
 */
public final class SimField {
    /** A convex polygon on the floor, wound counter-clockwise, that the robot's footprint may not enter. */
    public static final class Obstacle {
        public final String name;
        public final double[][] footprint;

        Obstacle(String name, double[][] footprint) {
            this.name = name;
            this.footprint = footprint;
        }
    }

    /**
     * A field element's shape: its vertices and its faces, each a ring of vertex indices seen
     * from outside. A <b>surface</b> is one flat polygon seen through and outlined in its colour,
     * the way a hive's panels are.
     */
    public static final class Element {
        public final String group;
        public final String name;
        public final String colour;
        public final boolean surface;
        public final double[][] vertices;
        public final int[][] faces;

        Element(String group, String name, String colour, boolean surface, double[][] vertices, int[][] faces) {
            this.group = group;
            this.name = name;
            this.colour = colour;
            this.surface = surface;
            this.vertices = vertices;
            this.faces = faces;
        }
    }

    /**
     * A hive cell: what a launched ball scores in. Its six {@link #panels}, one of which is the
     * <b>mouth</b> the ball comes in through: the rib at the cell's lower, open end, facing out of
     * the cell and down. A ball that crosses the mouth going in has scored; a ball that meets any
     * other panel bounces off.
     */
    public static final class Cell {
        public final String name;
        /** "Blue" or "Red": whose hive the cell is in. */
        public final String alliance;
        /** Each panel as its ring of {x, y, z} vertices. */
        public final List<double[][]> panels;

        public final double[][] mouth;
        public final String mouthName;
        /** The middle of the cell: the mean of its panels' vertices. */
        public final double[] centre;

        public final double[] mouthCentre;
        /** The mouth's unit normal, pointing out of the cell. */
        public final double[] mouthNormal;

        Cell(String name, String alliance, List<double[][]> panels, double[][] mouth, String mouthName) {
            this.name = name;
            this.alliance = alliance;
            this.panels = Collections.unmodifiableList(panels);
            this.mouth = mouth;
            this.mouthName = mouthName;
            this.centre = mean(panels);
            this.mouthCentre = mean(Collections.singletonList(mouth));
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

    /** A ball on the floor in the open, where the field is set up: the robot's to push. */
    public static final class Piece {
        public final String name;
        public final double x;
        public final double y;
        public final double z;
        public final double radius;

        Piece(String name, double x, double y, double z, double radius) {
            this.name = name;
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
    /** The game pieces the simulator rolls, in the order the page and the ticks name them. */
    public final List<Piece> loosePieces;
    /** The hives' cells, which a launched ball scores in. */
    public final List<Cell> cells;

    private final JsonObject json;

    private SimField(JsonObject json) {
        this.json = json;
        this.size = json.get("size").getAsDouble();
        this.wallHeight = json.get("wallHeight").getAsDouble();
        Gson gson = new Gson();
        List<Element> elements = new ArrayList<>();
        for (int i = 0; i < json.getAsJsonArray("elements").size(); i++) {
            JsonObject e = json.getAsJsonArray("elements").get(i).getAsJsonObject();
            elements.add(new Element(
                    e.get("group").getAsString(),
                    e.get("name").getAsString(),
                    e.get("colour").getAsString(),
                    e.has("surface") && e.get("surface").getAsBoolean(),
                    gson.fromJson(e.get("vertices"), double[][].class),
                    gson.fromJson(e.get("faces"), int[][].class)));
        }
        this.elements = Collections.unmodifiableList(elements);
        List<Piece> loose = new ArrayList<>();
        JsonArray pieces = json.getAsJsonArray("pieces");
        for (int i = 0; i < pieces.size(); i++) {
            JsonObject p = pieces.get(i).getAsJsonObject();
            if (p.get("loose").getAsBoolean()) {
                JsonArray centre = p.getAsJsonArray("centre");
                loose.add(new Piece(
                        p.get("name").getAsString(),
                        centre.get(0).getAsDouble(),
                        centre.get(1).getAsDouble(),
                        centre.get(2).getAsDouble(),
                        p.get("radius").getAsDouble()));
            }
        }
        this.loosePieces = Collections.unmodifiableList(loose);
        List<Obstacle> obstacles = new ArrayList<>();
        JsonArray array = json.getAsJsonArray("obstacles");
        for (int i = 0; i < array.size(); i++) {
            JsonObject o = array.get(i).getAsJsonObject();
            obstacles.add(
                    new Obstacle(o.get("name").getAsString(), gson.fromJson(o.get("footprint"), double[][].class)));
        }
        this.obstacles = Collections.unmodifiableList(obstacles);
        this.cells = Collections.unmodifiableList(cellsOf(this.elements));
    }

    private static final Pattern CELL_PANEL = Pattern.compile("^((Blue|Red) Cell \\([^)]*\\) <\\d+>) / (.+)$");

    /** The cells: each hive's see-through panels, grouped by the cell they name. */
    private static List<Cell> cellsOf(List<Element> elements) {
        Map<String, List<Element>> byCell = new LinkedHashMap<>();
        for (Element element : elements) {
            Matcher m = CELL_PANEL.matcher(element.name);
            if (element.surface && element.group.contains("Hive") && m.matches()) {
                byCell.computeIfAbsent(m.group(1), k -> new ArrayList<>()).add(element);
            }
        }
        List<Cell> cells = new ArrayList<>();
        for (Map.Entry<String, List<Element>> entry : byCell.entrySet()) {
            List<double[][]> panels = new ArrayList<>();
            Element mouth = null;
            for (Element panel : entry.getValue()) {
                panels.add(panel.vertices);
                String part = CELL_PANEL.matcher(panel.name).replaceAll("$3");
                if (part.endsWith("Goal Rib") && (mouth == null || meanZ(panel.vertices) < meanZ(mouth.vertices))) {
                    mouth = panel;
                }
            }
            if (mouth == null) {
                throw new IllegalStateException(entry.getKey() + " has no rib to be its mouth");
            }
            String alliance = CELL_PANEL.matcher(mouth.name).replaceAll("$2");
            cells.add(new Cell(
                    entry.getKey(),
                    alliance,
                    panels,
                    mouth.vertices,
                    CELL_PANEL.matcher(mouth.name).replaceAll("$3")));
        }
        return cells;
    }

    private static double meanZ(double[][] ring) {
        double sum = 0;
        for (double[] v : ring) {
            sum += v[2];
        }
        return sum / ring.length;
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
}
