package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSerializer;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/**
 * Writes a {@link SimRecording} as a single HTML file that replays the run: the field, the true
 * pose, and the dashboard drawing the robot code produced, with play/pause/scrub controls.
 * The page loads nothing from the network, so it can be opened from anywhere the file is.
 */
public final class SimReplayPage {
    private static final String TEMPLATE = "replay.html";

    private SimReplayPage() {
    }

    public static void write(SimRecording recording, Path page) {
        String html = page(recording, false);
        try {
            Files.createDirectories(page.toAbsolutePath().getParent());
            Files.write(page, html.getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new UncheckedIOException("could not write replay page " + page, e);
        }
    }

    /**
     * The page as HTML. A live page starts empty and polls {@code /ticks} for the run as it happens;
     * a file page embeds the whole recording.
     */
    public static String page(SimRecording recording, boolean live) {
        JsonObject root = new JsonObject();
        root.addProperty("name", recording.name());
        root.addProperty("live", live);
        root.addProperty("outcome", live ? null : recording.outcome());
        root.add("ticks", ticksJson(live ? List.of() : recording.ticks()));
        return template()
                .replace("__TITLE__", recording.name())
                .replace("__DATA__", GSON.toJson(root));
    }

    /**
     * What a live page fetches: the ticks from {@code from} onward and the outcome once there is one.
     */
    public static String update(SimRecording recording, int from) {
        JsonObject root = new JsonObject();
        root.addProperty("outcome", recording.outcome());
        root.add("ticks", ticksJson(recording.ticksFrom(from)));
        return GSON.toJson(root);
    }

    private static final Gson GSON = gson();

    private static Gson gson() {
        JsonSerializer<Double> threeDecimals = (value, type, context) ->
                new JsonPrimitive(Math.round(value * 1000) / 1000d);
        return new GsonBuilder()
                .serializeNulls() // "outcome": null says "still running" explicitly
                .registerTypeAdapter(double.class, threeDecimals)
                .registerTypeAdapter(Double.class, threeDecimals)
                .create();
    }

    private static JsonArray ticksJson(List<SimRecording.Tick> source) {
        Gson gson = GSON;
        JsonArray ticks = new JsonArray();
        for (SimRecording.Tick tick : source) {
            JsonObject t = new JsonObject();
            t.add("t", gson.toJsonTree(tick.seconds));
            t.add("x", gson.toJsonTree(tick.truePose.position.x));
            t.add("y", gson.toJsonTree(tick.truePose.position.y));
            t.add("heading", gson.toJsonTree(tick.truePose.heading.toDouble()));
            t.addProperty("step", tick.step);
            t.add("powers", gson.toJsonTree(tick.wheelPowers));
            JsonArray packets = new JsonArray();
            for (TelemetryPacket packet : tick.packets) {
                JsonObject p = new JsonObject();
                p.add("data", gson.toJsonTree(packet).getAsJsonObject().get("data"));
                p.add("ops", gson.toJsonTree(packet.fieldOverlay().getOperations()));
                packets.add(p);
            }
            t.add("packets", packets);
            ticks.add(t);
        }
        // Gson escapes '<' and '>' so the JSON is safe inside a <script> element.
        return ticks;
    }

    private static String template() {
        try (InputStream in = SimReplayPage.class.getResourceAsStream(TEMPLATE)) {
            if (in == null) {
                throw new IllegalStateException("missing resource " + TEMPLATE + " next to " + SimReplayPage.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
