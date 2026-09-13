package org.firstinspires.ftc.teamcode.sim;

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

/**
 * Writes a run as a single HTML file that replays it: the field in three dimensions, the true pose,
 * and the dashboard drawing the robot code produced, with play/pause/scrub controls. The field,
 * robot and wall sizes come from {@link SimRobot}, so the page draws what the simulator collides.
 * The page loads nothing from
 * the network, so it can be opened from anywhere the file is. The run comes from a
 * {@link Source}: a {@link SimRecording} in this JVM, or a run the bench knows only by the lines
 * its child streamed ({@link SimRunStream}); the page is the same either way.
 */
public final class SimReplayPage {
    /** A run as the page reads it: its ticks in their line form, and its outcome once it has one. */
    public interface Source {
        String name();

        /** {@link SimCatalog#AUTO} or {@link SimCatalog#TELEOP}; a TeleOp page shows the controller. */
        String kind();

        /** The ticks from index {@code from} onward, each as {@link SimRunStream#tickJson}. */
        JsonArray ticksJson(int from);

        /** How the run ended, or null while it is still running. */
        String outcome();
    }

    private static final String TEMPLATE = "replay.html";
    /** "outcome": null says "still running" explicitly. */
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private SimReplayPage() {
    }

    public static void write(Source run, Path page) {
        String html = page(run, false);
        try {
            Files.createDirectories(page.toAbsolutePath().getParent());
            Files.write(page, html.getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new UncheckedIOException("could not write replay page " + page, e);
        }
    }

    /**
     * The page as HTML. A live page starts empty and polls {@code /ticks} for the run as it happens,
     * and while the run is a TeleOp drives it from the controller; a file page embeds the whole run.
     */
    public static String page(Source run, boolean live) {
        JsonObject root = new JsonObject();
        root.addProperty("name", run.name());
        root.addProperty("kind", run.kind());
        root.addProperty("live", live);
        root.addProperty("outcome", live ? null : run.outcome());
        root.add("ticks", live ? new JsonArray() : run.ticksJson(0));
        // Gson escapes '<' and '>' so the JSON is safe inside a <script> element.
        return template()
                .replace("__TITLE__", run.name())
                .replace("__FIELD_IN__", String.valueOf(SimRobot.FIELD_SIZE_IN))
                .replace("__ROBOT_IN__", String.valueOf(SimRobot.ROBOT_SIZE_IN))
                .replace("__WALL_IN__", String.valueOf(SimRobot.WALL_HEIGHT_IN))
                .replace("__DATA__", GSON.toJson(root));
    }

    /**
     * What a live page fetches: the ticks from {@code from} onward and the outcome once there is one.
     */
    public static String update(Source run, int from) {
        JsonObject root = new JsonObject();
        root.addProperty("outcome", run.outcome());
        root.add("ticks", run.ticksJson(from));
        return GSON.toJson(root);
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
