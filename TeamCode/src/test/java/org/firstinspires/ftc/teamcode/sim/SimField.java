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

public final class SimField {
    public static final String NECTAR = "Nectar";

    public static final String POLLEN = "Pollen";

    public static final class Obstacle {
        public final String name;
        public final double[][] footprint;

        public final double clears;

        public final double stands;

        Obstacle(String name, double[][] footprint, double clears, double stands) {
            this.name = name;
            this.footprint = footprint;
            this.clears = clears;
            this.stands = stands;
        }
    }

    public static final class Flower {
        public final String name;

        public final double[] axis;

        public final double bore;

        public final double gap;

        public final double lip;

        public final double nest;

        Flower(JsonObject json, Gson gson) {
            this.name = json.get("name").getAsString();
            this.axis = gson.fromJson(json.get("axis"), double[].class);
            this.bore = json.get("bore").getAsDouble();
            this.gap = json.get("gap").getAsDouble();
            this.lip = json.get("lip").getAsDouble();
            this.nest = json.get("nest").getAsDouble();
        }

        public boolean standsIn(double x, double y) {
            return Math.hypot(x - axis[0], y - axis[1]) <= bore;
        }
    }

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

    public static final class Hive {
        public final String name;

        public final String alliance;

        public final String colour;

        public final double[] pivot;

        public final double tilt;

        public final List<Cell> cells;

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

        public double[] at(double tilt, double[] local) {
            double[] turned = direction(tilt, local);
            return new double[] {pivot[0] + turned[0], pivot[1] + turned[1], pivot[2] + turned[2]};
        }

        public double[] direction(double tilt, double[] local) {
            double c = Math.cos(Math.toRadians(tilt)), s = Math.sin(Math.toRadians(tilt));
            return new double[] {local[0] * c - local[2] * s, local[1], local[0] * s + local[2] * c};
        }

        public double[][] at(double tilt, double[][] local) {
            double[][] out = new double[local.length][];
            for (int i = 0; i < local.length; i++) {
                out[i] = at(tilt, local[i]);
            }
            return out;
        }
    }

    public static final class Cell {
        public final Hive hive;
        public final String name;

        public final String alliance;

        public final String side;

        public final double[][] mouth;

        public final double[][] back;

        public final List<double[][]> walls;

        public final List<double[][]> panels;

        public final double[] centre;

        public final double[] mouthCentre;

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

        public double[] centreAt(double tilt) {
            return hive.at(tilt, centre);
        }

        public double[] mouthCentreAt(double tilt) {
            return hive.at(tilt, mouthCentre);
        }

        public double[] mouthNormalAt(double tilt) {
            return hive.direction(tilt, mouthNormal);
        }

        public double[][] mouthAt(double tilt) {
            return hive.at(tilt, mouth);
        }

        public List<double[][]> panelsAt(double tilt) {
            List<double[][]> out = new ArrayList<>(panels.size());
            for (double[][] panel : panels) {
                out.add(hive.at(tilt, panel));
            }
            return out;
        }

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

    public static final class Piece {
        public final String name;

        public final String kind;

        public final String alliance;

        public final String cell;

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

    public final double size;

    public final double wallHeight;
    public final List<Element> elements;
    public final List<Obstacle> obstacles;

    public final List<Hive> hives;

    public final List<Cell> cells;

    public final List<Flower> flowers;

    public final List<Piece> loosePieces;

    public final List<Piece> cellPieces;

    public final List<Piece> flowerPieces;

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

    public JsonObject json() {
        return json;
    }

    public Obstacle obstacle(String name) {
        for (Obstacle obstacle : obstacles) {
            if (obstacle.name.equals(name)) {
                return obstacle;
            }
        }
        return null;
    }

    public Hive hive(String name) {
        for (Hive hive : hives) {
            if (hive.name.equals(name)) {
                return hive;
            }
        }
        return null;
    }

    public Flower flower(String name) {
        for (Flower flower : flowers) {
            if (flower.name.equals(name)) {
                return flower;
            }
        }
        return null;
    }

    public Cell cell(String name) {
        for (Cell cell : cells) {
            if (cell.name.equals(name)) {
                return cell;
            }
        }
        return null;
    }
}
