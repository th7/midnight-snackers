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
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import org.firstinspires.ftc.teamcode.simcore.ConvexPolygon;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Tilt;
import org.firstinspires.ftc.teamcode.simcore.Vec2;
import org.firstinspires.ftc.teamcode.simcore.Vec3;

public final class SimField {
    private static final String MODEL = "field.json";

    public record Loaded(Field field, JsonObject page) {}

    private SimField() {}

    public static Loaded load() {
        try (InputStream in = SimField.class.getResourceAsStream(MODEL)) {
            if (in == null) {
                throw new IllegalStateException("missing resource " + MODEL + " next to " + SimField.class.getName());
            }
            return parse(new Gson().fromJson(new InputStreamReader(in, StandardCharsets.UTF_8), JsonObject.class));
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    static Loaded parse(JsonObject json) {
        Gson gson = new Gson();
        List<Field.Element> elements = new ArrayList<>();
        for (JsonElement element : json.getAsJsonArray("elements")) {
            JsonObject e = element.getAsJsonObject();
            elements.add(element(e, e.get("group").getAsString(), gson));
        }
        List<Field.Obstacle> obstacles = new ArrayList<>();
        for (JsonElement element : json.getAsJsonArray("obstacles")) {
            JsonObject o = element.getAsJsonObject();
            obstacles.add(Valid.value(Field.Obstacle.of(
                    o.get("name").getAsString(),
                    Valid.value(ConvexPolygon.of(vec2s(gson.fromJson(o.get("footprint"), double[][].class)))),
                    o.get("clears").getAsDouble(),
                    o.get("stands").getAsDouble())));
        }
        List<Field.Hive> hives = new ArrayList<>();
        for (JsonElement element : json.getAsJsonArray("hives")) {
            hives.add(hive(element.getAsJsonObject(), gson));
        }
        List<Field.Flower> flowers = new ArrayList<>();
        for (JsonElement element : json.getAsJsonArray("flowers")) {
            JsonObject f = element.getAsJsonObject();
            double[] axis = gson.fromJson(f.get("axis"), double[].class);
            flowers.add(Valid.value(Field.Flower.of(
                    f.get("name").getAsString(),
                    new Vec2(axis[0], axis[1]),
                    f.get("bore").getAsDouble(),
                    f.get("gap").getAsDouble(),
                    f.get("lip").getAsDouble(),
                    f.get("nest").getAsDouble())));
        }
        List<Field.Piece> pieces = new ArrayList<>();
        Map<Field.Piece, JsonObject> pieceJson = new IdentityHashMap<>();
        for (JsonElement element : json.getAsJsonArray("pieces")) {
            JsonObject p = element.getAsJsonObject();
            JsonElement cell = p.get("cell");
            JsonElement flower = p.get("flower");
            if (!p.get("loose").getAsBoolean() && cell == null && flower == null) {
                continue;
            }
            Field.Place place = cell != null
                    ? new Field.Place.InCell(named(cellsOf(hives), cell.getAsString(), Field.Cell::name))
                    : flower != null
                            ? new Field.Place.InFlower(named(flowers, flower.getAsString(), Field.Flower::name))
                            : new Field.Place.Loose();
            double[] centre = gson.fromJson(p.get("centre"), double[].class);
            Field.Piece piece = Valid.value(Field.Piece.of(
                    p.get("name").getAsString(),
                    Valid.value(Field.Kind.named(p.get("kind").getAsString())),
                    place,
                    new Vec3(centre[0], centre[1], centre[2]),
                    Valid.value(Length.of(p.get("radius").getAsDouble()))));
            pieces.add(piece);
            pieceJson.put(piece, p);
        }
        List<Field.Mark> tape = new ArrayList<>();
        for (JsonElement element : json.getAsJsonArray("tape")) {
            JsonObject mark = element.getAsJsonObject();
            tape.add(new Field.Mark(
                    mark.get("colour").getAsString(), vec2s(gson.fromJson(mark.get("footprint"), double[][].class))));
        }
        Field field = Valid.value(Field.of(
                Valid.value(Length.of(json.get("size").getAsDouble())),
                Valid.value(Length.of(json.get("wallHeight").getAsDouble())),
                elements,
                obstacles,
                hives,
                flowers,
                pieces,
                tape));
        JsonArray moved = new JsonArray();
        for (Field.Piece piece : field.movedPieces()) {
            moved.add(pieceJson.get(piece));
        }
        json.add("moved", moved);
        return new Loaded(field, json);
    }

    private static Field.Element element(JsonObject e, String group, Gson gson) {
        List<List<Integer>> faces = new ArrayList<>();
        for (int[] face : gson.fromJson(e.get("faces"), int[][].class)) {
            List<Integer> ring = new ArrayList<>();
            for (int index : face) {
                ring.add(index);
            }
            faces.add(ring);
        }
        return Valid.value(Field.Element.of(
                group,
                e.get("name").getAsString(),
                e.get("colour").getAsString(),
                vec3s(gson.fromJson(e.get("vertices"), double[][].class)),
                faces));
    }

    private static Field.Hive hive(JsonObject h, Gson gson) {
        String name = h.get("name").getAsString();
        List<Field.Element> parts = new ArrayList<>();
        for (JsonElement part : h.getAsJsonArray("parts")) {
            parts.add(element(part.getAsJsonObject(), name, gson));
        }
        List<Field.CellShape> cells = new ArrayList<>();
        for (JsonElement cell : h.getAsJsonArray("cells")) {
            JsonObject c = cell.getAsJsonObject();
            List<List<Vec3>> walls = new ArrayList<>();
            for (JsonElement wall : c.getAsJsonArray("walls")) {
                walls.add(vec3s(gson.fromJson(wall, double[][].class)));
            }
            cells.add(new Field.CellShape(
                    c.get("name").getAsString(),
                    c.get("side").getAsString(),
                    vec3s(gson.fromJson(c.get("mouth"), double[][].class)),
                    vec3s(gson.fromJson(c.get("back"), double[][].class)),
                    walls));
        }
        double[] pivot = gson.fromJson(h.get("pivot"), double[].class);
        return Valid.value(Field.Hive.of(
                name,
                h.get("alliance").getAsString(),
                h.get("colour").getAsString(),
                new Vec3(pivot[0], pivot[1], pivot[2]),
                Valid.value(Tilt.of(h.get("tilt").getAsDouble())),
                cells,
                parts));
    }

    private static List<Field.Cell> cellsOf(List<Field.Hive> hives) {
        List<Field.Cell> cells = new ArrayList<>();
        for (Field.Hive hive : hives) {
            cells.addAll(hive.cells());
        }
        return cells;
    }

    private static <T> T named(List<T> all, String name, Function<T, String> nameOf) {
        for (T one : all) {
            if (nameOf.apply(one).equals(name)) {
                return one;
            }
        }
        throw new IllegalArgumentException("the field has no " + name);
    }

    private static List<Vec2> vec2s(double[][] points) {
        List<Vec2> out = new ArrayList<>();
        for (double[] p : points) {
            out.add(new Vec2(p[0], p[1]));
        }
        return out;
    }

    private static List<Vec3> vec3s(double[][] points) {
        List<Vec3> out = new ArrayList<>();
        for (double[] p : points) {
            out.add(new Vec3(p[0], p[1], p[2]));
        }
        return out;
    }
}
