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
import java.util.List;

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
    private final JsonObject json;

    private SimField(JsonObject json) {
        this.json = json;
        this.size = json.get("size").getAsDouble();
        this.wallHeight = json.get("wallHeight").getAsDouble();
        Gson gson = new Gson();
        List<Element> elements = new ArrayList<>();
        for (int i = 0; i < json.getAsJsonArray("elements").size(); i++) {
            JsonObject e = json.getAsJsonArray("elements").get(i).getAsJsonObject();
            elements.add(new Element(e.get("group").getAsString(), e.get("name").getAsString(), e.get("colour").getAsString(),
                    e.has("surface") && e.get("surface").getAsBoolean(),
                    gson.fromJson(e.get("vertices"), double[][].class), gson.fromJson(e.get("faces"), int[][].class)));
        }
        this.elements = Collections.unmodifiableList(elements);
        List<Piece> loose = new ArrayList<>();
        JsonArray pieces = json.getAsJsonArray("pieces");
        for (int i = 0; i < pieces.size(); i++) {
            JsonObject p = pieces.get(i).getAsJsonObject();
            if (p.get("loose").getAsBoolean()) {
                JsonArray centre = p.getAsJsonArray("centre");
                loose.add(new Piece(p.get("name").getAsString(), centre.get(0).getAsDouble(), centre.get(1).getAsDouble(),
                        centre.get(2).getAsDouble(), p.get("radius").getAsDouble()));
            }
        }
        this.loosePieces = Collections.unmodifiableList(loose);
        List<Obstacle> obstacles = new ArrayList<>();
        JsonArray array = json.getAsJsonArray("obstacles");
        for (int i = 0; i < array.size(); i++) {
            JsonObject o = array.get(i).getAsJsonObject();
            obstacles.add(new Obstacle(o.get("name").getAsString(), gson.fromJson(o.get("footprint"), double[][].class)));
        }
        this.obstacles = Collections.unmodifiableList(obstacles);
    }

    /** The season's field, from the model next to this class. */
    public static SimField load() {
        try (InputStream in = SimField.class.getResourceAsStream(MODEL)) {
            if (in == null) {
                throw new IllegalStateException("missing resource " + MODEL + " next to " + SimField.class.getName());
            }
            return new SimField(new Gson().fromJson(new InputStreamReader(in, StandardCharsets.UTF_8), JsonObject.class));
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
