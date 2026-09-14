package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSerializer;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * The lines a run's child prints, one JSON object per line, in order: {@code {"protocol": N}}
 * first, {@code {"started": true}} as the op mode's time begins, each tick as it happens, and
 * finally {@code {"outcome": "..."}}. They are written here and read here; but the child runs the
 * simulator of the sources it was built from, which may be another version of this code, so the
 * lines carry a {@link #PROTOCOL version}: the child says its own first, and a bench refuses one
 * it cannot read ({@link #afterHello}) rather than misreading it. Every way a run can end is
 * named here in {@link Outcome}. A tick's line is also the form the replay page reads, so a run
 * the bench knows only by its lines is the page the child wrote. (The driver station's lines,
 * the other way, are {@link SimDriverStation#accept}.)
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
        private Outcome() {}

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

        /** The child's simulator speaks a protocol this bench cannot read; the run's message says whose the fix is. */
        public static String wrongProtocol(int childProtocol) {
            return "wrong protocol: the simulator speaks " + childProtocol + ", this server " + PROTOCOL;
        }

        /**
         * The child's simulator is from before the bench placed the robot, and the robot is placed
         * somewhere other than the origin, where such a child starts on its own; the run's message
         * says whose the fix is.
         */
        public static String cannotPlace(int childProtocol) {
            return "wrong protocol: the simulator speaks " + childProtocol
                    + " and cannot place the robot; placing needs " + PLACED_PROTOCOL;
        }

        /**
         * The child's simulator is from before the seed, and the op mode runs on one; such a child
         * runs the exact robot whatever it is told. The run's message says whose the fix is.
         */
        public static String cannotSeed(int childProtocol) {
            return "wrong protocol: the simulator speaks " + childProtocol
                    + " and cannot run a seeded robot; a seed needs " + SEEDED_PROTOCOL;
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

    /**
     * The version of these lines, and of what the child reads. Bump it when a change would leave
     * a bench of the old version misreading a child of the new, or sending it a line it cannot
     * read; then {@link #OLDEST_PROTOCOL_READ} says how old a child a bench still reads, and the
     * test of the oldest one pins what such a child printed.
     */
    public static final int PROTOCOL = 4;
    /** The oldest child a bench still reads. A child of version 1 prints no hello: its first line is content. */
    public static final int OLDEST_PROTOCOL_READ = 1;
    /**
     * From this version on, a child waits to be placed ({@link SimDriverStation#startLine}) before
     * its run starts, and the bench places it first thing. An older child places itself at the
     * origin, so the bench lets it run from there and refuses to run it from anywhere else.
     */
    public static final int PLACED_PROTOCOL = 3;
    /**
     * From this version on, the start line may name a {@link SimNoise} seed and the child runs the
     * robot drawn from it. An older child runs the exact robot whatever it is told, so the bench
     * lets it run when that is the op mode's robot and refuses to run it on a seed.
     */
    public static final int SEEDED_PROTOCOL = 4;

    /** The child speaks a protocol this bench cannot read; the message names both and whose the fix is. */
    public static final class WrongProtocol extends RuntimeException {
        public final int childProtocol;

        WrongProtocol(int childProtocol, String message) {
            super(message);
            this.childProtocol = childProtocol;
        }
    }

    private static final Gson GSON = gson();

    private SimRunStream() {}

    /** The child's first line, before the catalog or the run: which protocol it speaks. */
    public static String hello() {
        JsonObject line = new JsonObject();
        line.addProperty("protocol", PROTOCOL);
        return GSON.toJson(line);
    }

    /**
     * Reads the child's first line. A hello is consumed, and null comes back: the next line is
     * content. A version-1 child prints no hello, so its first line is content and comes back as it was.
     *
     * @throws WrongProtocol when the child's protocol is newer than this bench's, or older than the oldest it reads
     */
    public static String afterHello(String firstLine) {
        return protocolOf(firstLine) == 1 ? firstLine : null;
    }

    /**
     * The protocol the child's first line says it speaks: 1 when the line is content, since a
     * version-1 child prints no hello.
     *
     * @throws WrongProtocol when the child's protocol is newer than this bench's, or older than the oldest it reads
     */
    public static int protocolOf(String firstLine) {
        JsonObject json;
        try {
            json = GSON.fromJson(firstLine, JsonObject.class);
        } catch (RuntimeException e) {
            return 1; // a catalog line is an array: content
        }
        if (json == null || !json.has("protocol")) {
            return 1;
        }
        int child = json.get("protocol").getAsInt();
        if (child > PROTOCOL) {
            throw new WrongProtocol(
                    child,
                    "the simulator in these sources speaks protocol " + child + " and this server speaks"
                            + " protocol " + PROTOCOL
                            + ": the server is older than the sources. Restart the server from a checkout"
                            + " with the newer code.");
        }
        if (child < OLDEST_PROTOCOL_READ) {
            throw new WrongProtocol(
                    child,
                    "the simulator in these sources speaks protocol " + child + " and this server reads"
                            + " protocol " + OLDEST_PROTOCOL_READ + " to " + PROTOCOL
                            + ": the sources are older than the server."
                            + " Pull develop.");
        }
        return child;
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
        if (tick.pieces != null && tick.pieces.length > 0) {
            t.add("pieces", GSON.toJsonTree(tick.pieces)); // absent means where the field was set up
        }
        if (tick.held > 0) {
            t.addProperty("held", tick.held); // absent means none
        }
        JsonObject scored = new JsonObject();
        for (Map.Entry<String, Integer> entry : new TreeMap<>(tick.scored).entrySet()) {
            if (entry.getValue() > 0) {
                scored.addProperty(entry.getKey(), entry.getValue());
            }
        }
        if (scored.size() > 0) {
            t.add("scored", scored); // absent means nothing scored
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
        JsonSerializer<Double> threeDecimals =
                (value, type, context) -> new JsonPrimitive(Math.round(value * 1000) / 1000d);
        return new GsonBuilder()
                .registerTypeAdapter(double.class, threeDecimals)
                .registerTypeAdapter(Double.class, threeDecimals)
                .create();
    }
}
