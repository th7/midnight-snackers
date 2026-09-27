package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.google.gson.stream.JsonWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.StringWriter;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;

public final class SimReplayPage {
    public interface Source {
        String name();

        String kind();

        /**
         * The ticks from the given one on, as the text of a JSON array of the run stream's tick
         * lines: what a live view's poll is sent as it is, and what a written page reads.
         */
        String ticksJson(int from);

        String outcome();

        /**
         * The match the run is, for its page: a game's period and where its drivers stand, or JSON's
         * null for a run that is not one.
         */
        JsonElement match();
    }

    private static final String TEMPLATE = "replay.html";

    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private SimReplayPage() {}

    public static void write(Source run, Path page) {
        String html = written(run);
        try {
            Files.createDirectories(page.toAbsolutePath().getParent());
            Files.write(page, html.getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new UncheckedIOException("could not write replay page " + page, e);
        }
    }

    public static String written(Source run) {
        JsonObject root = root(run.name(), run.kind(), false);
        root.addProperty("outcome", run.outcome());
        root.add("match", run.match());
        // Read back and written out again, rather than set in as it came: the page carries it inside
        // a script element, and it is the writer that keeps a "</script>" in a step's name from
        // closing it.
        root.add("ticks", GSON.fromJson(run.ticksJson(0), JsonArray.class));
        return fill(run.name(), root, Optional.empty());
    }

    public static String live(Source run, String assetsUnder) {
        JsonObject root = root(run.name(), run.kind(), true);
        root.addProperty("outcome", (String) null);
        root.add("match", run.match());
        root.add("ticks", new JsonArray());
        return fill(run.name(), root, Optional.of(assetsUnder));
    }

    public static String placement(String opMode, String kind, Pose2d start) {
        JsonObject root = root(opMode, kind, false);
        root.addProperty("outcome", (String) null);
        root.add("match", JsonNull.INSTANCE);
        root.add("ticks", new JsonArray());
        root.add("placing", StartPoses.toJson(start));
        return fill(opMode, root, Optional.empty());
    }

    private static JsonObject root(String name, String kind, boolean live) {
        JsonObject root = new JsonObject();
        root.addProperty("name", name);
        root.addProperty("kind", kind);
        root.addProperty("live", live);
        return root;
    }

    private static String fill(String title, JsonObject root, Optional<String> assetsUnder) {
        return template()
                .replace(
                        "__IMPORTMAP__",
                        assetsUnder.map(SimReplayPage::importMap).orElse(""))
                .replace("__ASSETS__", assetsUnder.map(GSON::toJson).orElse("null"))
                .replace("__TITLE__", title)
                .replace("__FIELD_IN__", String.valueOf(SimPlacement.FIELD_SIZE_IN))
                .replace("__ROBOT_IN__", String.valueOf(SimPlacement.ROBOT_SIZE_IN))
                .replace("__WALL_IN__", String.valueOf(SimPlacement.WALL_HEIGHT_IN))
                .replace("__FIELD__", GSON.toJson(SimPlacement.FIELD.json()))
                .replace("__DATA__", GSON.toJson(root));
    }

    private static String importMap(String assetsUnder) {
        return "<script type=\"importmap\">\n{\"imports\": {\"three\": \"" + assetsUnder
                + "vendor/three.module.min.js\", \"three/addons/\": \"" + assetsUnder + "vendor/jsm/\"}}\n</script>";
    }

    /**
     * What a live view's poll is sent: the outcome, if there is one yet, and the ticks it has not
     * had, set in as the lines they came as rather than read into a tree and written out again.
     */
    public static String update(Source run, int from) {
        StringWriter text = new StringWriter();
        try (JsonWriter json = GSON.newJsonWriter(text)) {
            json.beginObject();
            json.name("outcome").value(run.outcome());
            json.name("ticks").jsonValue(run.ticksJson(from));
            json.endObject();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return text.toString();
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
