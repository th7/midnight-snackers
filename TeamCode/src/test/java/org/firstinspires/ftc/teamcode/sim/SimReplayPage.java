package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

public final class SimReplayPage {
    public interface Source {
        String name();

        String kind();

        JsonArray ticksJson(int from);

        String outcome();
    }

    private static final String TEMPLATE = "replay.html";

    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private SimReplayPage() {}

    public static void write(Source run, Path page) {
        String html = page(run, false);
        try {
            Files.createDirectories(page.toAbsolutePath().getParent());
            Files.write(page, html.getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new UncheckedIOException("could not write replay page " + page, e);
        }
    }

    public static String page(Source run, boolean live) {
        JsonObject root = new JsonObject();
        root.addProperty("name", run.name());
        root.addProperty("kind", run.kind());
        root.addProperty("live", live);
        root.addProperty("outcome", live ? null : run.outcome());
        root.add("ticks", live ? new JsonArray() : run.ticksJson(0));
        return fill(run.name(), root);
    }

    public static String placement(String opMode, String kind, Pose2d start) {
        JsonObject root = new JsonObject();
        root.addProperty("name", opMode);
        root.addProperty("kind", kind);
        root.addProperty("live", false);
        root.addProperty("outcome", (String) null);
        root.add("ticks", new JsonArray());
        root.add("placing", StartPoses.toJson(start));
        return fill(opMode, root);
    }

    private static String fill(String title, JsonObject root) {
        return template()
                .replace("__TITLE__", title)
                .replace("__FIELD_IN__", String.valueOf(SimPlacement.FIELD_SIZE_IN))
                .replace("__ROBOT_IN__", String.valueOf(SimPlacement.ROBOT_SIZE_IN))
                .replace("__WALL_IN__", String.valueOf(SimPlacement.WALL_HEIGHT_IN))
                .replace("__FIELD__", GSON.toJson(SimPlacement.FIELD.json()))
                .replace("__DATA__", GSON.toJson(root));
    }

    public static String update(Source run, int from) {
        JsonObject root = new JsonObject();
        root.addProperty("outcome", run.outcome());
        root.add("ticks", run.ticksJson(from));
        return GSON.toJson(root);
    }

    private static String template() {
        try (InputStream in = SimReplayPage.class.getResourceAsStream(TEMPLATE)) {
            if (in == null) {
                throw new IllegalStateException(
                        "missing resource " + TEMPLATE + " next to " + SimReplayPage.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
