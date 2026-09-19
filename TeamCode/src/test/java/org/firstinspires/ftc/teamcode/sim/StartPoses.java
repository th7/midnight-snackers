package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.LinkedHashMap;
import java.util.Map;

final class StartPoses {
    static final Pose2d ORIGIN = new Pose2d(0, 0, 0);

    static final long DEFAULT_SEED = 1;

    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private static final class Start {
        Pose2d pose = ORIGIN;
        Long seed = DEFAULT_SEED;
    }

    private final Path file;
    private final Map<String, Start> byOpMode = new LinkedHashMap<>();

    StartPoses(Path file) {
        this.file = file;
        if (!Files.exists(file)) {
            return;
        }
        try {
            JsonObject stored =
                    GSON.fromJson(new String(Files.readAllBytes(file), StandardCharsets.UTF_8), JsonObject.class);
            if (stored == null) {
                throw new IllegalStateException("empty");
            }
            for (Map.Entry<String, JsonElement> entry : stored.entrySet()) {
                JsonObject record = entry.getValue().getAsJsonObject();
                Start start = new Start();
                start.pose = fromJson(record);

                if (record.has("seed")) {
                    start.seed = seedFromJson(record.get("seed"));
                }
                byOpMode.put(entry.getKey(), start);
            }
        } catch (IOException | RuntimeException e) {
            throw new IllegalStateException("could not read the start poses in " + file + ": " + e, e);
        }
    }

    synchronized Pose2d get(String opMode) {
        return startOf(opMode).pose;
    }

    synchronized Long seed(String opMode) {
        return startOf(opMode).seed;
    }

    private Start startOf(String opMode) {
        return byOpMode.getOrDefault(opMode, new Start());
    }

    synchronized Pose2d put(String opMode, Pose2d pose) {
        Pose2d placed = SimPlacement.onTheField(pose);
        byOpMode.computeIfAbsent(opMode, name -> new Start()).pose = placed;
        save();
        return placed;
    }

    synchronized Long putSeed(String opMode, Long seed) {
        byOpMode.computeIfAbsent(opMode, name -> new Start()).seed = seed;
        save();
        return seed;
    }

    private void save() {
        JsonObject body = new JsonObject();
        for (Map.Entry<String, Start> entry : byOpMode.entrySet()) {
            JsonObject record = toJson(entry.getValue().pose);
            record.add("seed", seedToJson(entry.getValue().seed));
            body.add(entry.getKey(), record);
        }
        try {
            Files.createDirectories(file.toAbsolutePath().getParent());
            Path temp = Files.createTempFile(file.toAbsolutePath().getParent(), "." + file.getFileName(), ".saving");
            try {
                Files.write(temp, GSON.toJson(body).getBytes(StandardCharsets.UTF_8));
                Files.move(temp, file, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
            } finally {
                Files.deleteIfExists(temp);
            }
        } catch (IOException e) {
            throw new UncheckedIOException("could not save the start poses to " + file + ": " + e.getMessage(), e);
        }
    }

    static JsonObject toJson(Pose2d pose) {
        JsonObject json = new JsonObject();
        json.addProperty("x", pose.position.x);
        json.addProperty("y", pose.position.y);
        json.addProperty("heading", pose.heading.toDouble());
        return json;
    }

    static Pose2d fromJson(JsonObject json) {
        return new Pose2d(number(json, "x"), number(json, "y"), number(json, "heading"));
    }

    static JsonElement seedToJson(Long seed) {
        return seed == null ? JsonNull.INSTANCE : new JsonPrimitive(seed);
    }

    static Long seedFromJson(JsonElement value) {
        if (value == null || value.isJsonNull()) {
            return null;
        }
        try {
            double asDouble = value.getAsDouble();
            long asLong = value.getAsLong();
            if (asDouble != asLong
                    || !value.isJsonPrimitive()
                    || !value.getAsJsonPrimitive().isNumber()) {
                throw new IllegalArgumentException("not a whole number");
            }
            return asLong;
        } catch (RuntimeException e) {
            throw new IllegalArgumentException("a seed is a whole number, or null for the exact robot: " + value);
        }
    }

    private static double number(JsonObject json, String name) {
        JsonElement value = json.get(name);
        if (value == null) {
            throw new IllegalArgumentException("a start pose needs a " + name + ": " + json);
        }
        double number;
        try {
            number = value.getAsDouble();
        } catch (RuntimeException e) {
            throw new IllegalArgumentException(name + " is not a number: " + value);
        }
        if (!Double.isFinite(number)) {
            throw new IllegalArgumentException(name + " is not a number: " + value);
        }
        return number;
    }
}
