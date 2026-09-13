package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSerializer;

import java.util.List;

/**
 * The lines a run's child prints, one JSON object per line, in order: {@code {"started": true}}
 * as the op mode's time begins, each tick as it happens, and finally {@code {"outcome": "..."}}.
 * They are written here and read here, so the child and the bench agree by construction, and
 * every way a run can end is named here in {@link Outcome}. A tick's line is also the form the
 * replay page reads, so a run the bench knows only by its lines is the page the child wrote.
 * (The driver station's lines, the other way, are {@link SimDriverStation#accept}.)
 */
public final class SimRunStream {
    /** What the lines say, in the order they were printed. */
    public interface Listener {
        void started();

        void tick(JsonObject tick);

        void finished(String outcome);
    }

    /** How a run ended, as the pages show it. */
    public static final class Outcome {
        private Outcome() {
        }

        /** An auto's plan finished, or a TeleOp's period was over. */
        public static String done() {
            return "done";
        }

        /** The driver pressed Stop, or the bench was stopped. */
        public static String stopped() {
            return "stopped";
        }

        /** An auto's plan was not done within its timeout. */
        public static String timedOut(double seconds) {
            return String.format("timed out after %.1fs", seconds);
        }

        /** The op mode threw. */
        public static String failed(Throwable e) {
            return "failed: " + e;
        }

        /** The sources did not compile; the run's message says what the compiler said. */
        public static String buildFailed() {
            return "build failed";
        }

        public static String couldNotStartChild() {
            return "could not start the child JVM";
        }

        /** The bench killed the child after {@code seconds}, for {@code why}. */
        public static String killed(double seconds, String why) {
            return String.format("killed after %.1fs: %s", seconds, why);
        }

        /** The child ignored Stop for {@code seconds} and was killed. */
        public static String killedAfterStop(double seconds) {
            return String.format("killed %.1fs after Stop: the op mode did not return", seconds);
        }

        /** The child ended without saying how; the run's message is its log. */
        public static String childExited(int code) {
            return "child exited with code " + code;
        }

        public static String noOpModeNamed(String name) {
            return "no op mode named " + name;
        }

        public static String couldNotBuild(String name, Throwable e) {
            return "could not build " + name + ": " + e;
        }

        /** A line on the child's standard input was not a driver station line. */
        public static String badDriverStation(String problem) {
            return "could not read the driver station: " + problem;
        }
    }

    private static final Gson GSON = gson();

    private SimRunStream() {
    }

    public static String started() {
        JsonObject line = new JsonObject();
        line.addProperty("started", true);
        return GSON.toJson(line);
    }

    public static String tick(SimRecording.Tick tick) {
        return GSON.toJson(tickJson(tick));
    }

    public static String finished(String outcome) {
        JsonObject line = new JsonObject();
        line.addProperty("outcome", outcome);
        return GSON.toJson(line);
    }

    /**
     * Reads one line to the listener.
     *
     * @throws IllegalArgumentException for a line that is none of the three
     */
    public static void accept(String line, Listener listener) {
        JsonObject json;
        try {
            json = GSON.fromJson(line, JsonObject.class);
        } catch (RuntimeException e) {
            throw new IllegalArgumentException("not a line of the run stream: " + line, e);
        }
        if (json == null) {
            throw new IllegalArgumentException("not a line of the run stream: " + line);
        }
        if (json.has("started")) {
            listener.started();
        } else if (json.has("outcome")) {
            listener.finished(json.get("outcome").getAsString());
        } else if (json.has("t") && json.has("x")) {
            listener.tick(json);
        } else {
            throw new IllegalArgumentException("not a line of the run stream: " + line);
        }
    }

    /** The run's time at a tick, in seconds. */
    public static double seconds(JsonObject tick) {
        return tick.get("t").getAsDouble();
    }

    /** One tick in the form the page reads and the child streams: numbers rounded to three decimals. */
    public static JsonObject tickJson(SimRecording.Tick tick) {
        JsonObject t = new JsonObject();
        t.add("t", GSON.toJsonTree(tick.seconds));
        t.add("x", GSON.toJsonTree(tick.truePose.position.x));
        t.add("y", GSON.toJsonTree(tick.truePose.position.y));
        t.add("heading", GSON.toJsonTree(tick.truePose.heading.toDouble()));
        t.addProperty("step", tick.step);
        t.add("powers", GSON.toJsonTree(tick.wheelPowers));
        JsonArray packets = new JsonArray();
        for (TelemetryPacket packet : tick.packets) {
            JsonObject p = new JsonObject();
            p.add("data", GSON.toJsonTree(packet).getAsJsonObject().get("data"));
            p.add("ops", GSON.toJsonTree(packet.fieldOverlay().getOperations()));
            packets.add(p);
        }
        t.add("packets", packets);
        JsonObject gamepads = new JsonObject();
        if (tick.gamepad1 != null && !tick.gamepad1.neutral()) {
            gamepads.add("1", tick.gamepad1.toJson());
        }
        if (tick.gamepad2 != null && !tick.gamepad2.neutral()) {
            gamepads.add("2", tick.gamepad2.toJson());
        }
        if (gamepads.size() > 0) {
            t.add("gamepads", gamepads); // absent means neutral, so a replay stays small
        }
        return t;
    }

    public static JsonArray ticksJson(List<SimRecording.Tick> ticks) {
        JsonArray array = new JsonArray();
        for (SimRecording.Tick tick : ticks) {
            array.add(tickJson(tick));
        }
        return array;
    }

    private static Gson gson() {
        JsonSerializer<Double> threeDecimals = (value, type, context) ->
                new JsonPrimitive(Math.round(value * 1000) / 1000d);
        return new GsonBuilder()
                .registerTypeAdapter(double.class, threeDecimals)
                .registerTypeAdapter(Double.class, threeDecimals)
                .create();
    }
}
