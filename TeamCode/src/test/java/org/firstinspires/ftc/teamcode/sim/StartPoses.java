package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Where the simulated robot is placed before a run, per op mode: its true pose as the op mode's
 * time begins, always where the field allows: inside the walls and clear of the obstacles. The origin until someone places it. Remembered in one
 * JSON file, {@code {"<op mode name>": {"x": .., "y": .., "heading": ..}, ...}}, written whole
 * and moved into place; a file that exists but cannot be read is a failure to start, not a fresh
 * start. The pose's JSON form here is the one the routes answer, the file holds, and the
 * {@link SimDriverStation#startLine start line} carries.
 */
final class StartPoses {
    static final Pose2d ORIGIN = new Pose2d(0, 0, 0);
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private final Path file;
    private final Map<String, Pose2d> byOpMode = new LinkedHashMap<>();

    /**
     * @throws IllegalStateException when {@code file} exists but is not a store of start poses
     */
    StartPoses(Path file) {
        this.file = file;
        if (!Files.exists(file)) {
            return;
        }
        try {
            JsonObject stored = GSON.fromJson(new String(Files.readAllBytes(file), StandardCharsets.UTF_8), JsonObject.class);
            if (stored == null) {
                throw new IllegalStateException("empty");
            }
            for (Map.Entry<String, JsonElement> entry : stored.entrySet()) {
                byOpMode.put(entry.getKey(), fromJson(entry.getValue().getAsJsonObject()));
            }
        } catch (IOException | RuntimeException e) {
            throw new IllegalStateException("could not read the start poses in " + file + ": " + e, e);
        }
    }

    /** Where {@code opMode}'s runs start: where it was last placed, else the origin. */
    synchronized Pose2d get(String opMode) {
        return byOpMode.getOrDefault(opMode, ORIGIN);
    }

    /**
     * Places the robot for {@code opMode}'s runs, against a wall or an obstacle when {@code pose}
     * is beyond one.
     *
     * @return the pose as placed
     */
    synchronized Pose2d put(String opMode, Pose2d pose) {
        Pose2d placed = SimRobot.onTheField(pose);
        byOpMode.put(opMode, placed);
        save();
        return placed;
    }

    private void save() {
        JsonObject body = new JsonObject();
        for (Map.Entry<String, Pose2d> entry : byOpMode.entrySet()) {
            body.add(entry.getKey(), toJson(entry.getValue()));
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

    /** {@code {"x": .., "y": .., "heading": ..}}, in inches and radians. */
    static JsonObject toJson(Pose2d pose) {
        JsonObject json = new JsonObject();
        json.addProperty("x", pose.position.x);
        json.addProperty("y", pose.position.y);
        json.addProperty("heading", pose.heading.toDouble());
        return json;
    }

    /**
     * @throws IllegalArgumentException when a coordinate is missing or not a finite number, named
     */
    static Pose2d fromJson(JsonObject json) {
        return new Pose2d(number(json, "x"), number(json, "y"), number(json, "heading"));
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
