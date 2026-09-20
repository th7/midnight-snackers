package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSerializer;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public final class SimRunStream {
    public interface Listener {
        void started();

        void tick(JsonObject tick);

        void finished(String outcome);
    }

    public static final class Outcome {
        private Outcome() {}

        public static String done() {
            return "done";
        }

        public static String stopped() {
            return "stopped";
        }

        public static String timedOut(double seconds) {
            return String.format("timed out after %.1fs", seconds);
        }

        public static String failed(Throwable e) {
            return "failed: " + e;
        }

        public static String buildFailed() {
            return "build failed";
        }

        public static String wrongProtocol(int childProtocol) {
            return "wrong protocol: the simulator speaks " + childProtocol + ", this server " + PROTOCOL;
        }

        public static String cannotPlace(int childProtocol) {
            return "wrong protocol: the simulator speaks " + childProtocol
                    + " and cannot place the robot; placing needs " + PLACED_PROTOCOL;
        }

        public static String cannotSeed(int childProtocol) {
            return "wrong protocol: the simulator speaks " + childProtocol
                    + " and cannot run a seeded robot; a seed needs " + SEEDED_PROTOCOL;
        }

        public static String couldNotStartChild() {
            return "could not start the child JVM";
        }

        public static String killed(double seconds, String why) {
            return String.format("killed after %.1fs: %s", seconds, why);
        }

        public static String killedAfterStop(double seconds) {
            return String.format("killed %.1fs after Stop: the op mode did not return", seconds);
        }

        public static String childExited(int code) {
            return "child exited with code " + code;
        }

        public static String noOpModeNamed(String name) {
            return "no op mode named " + name;
        }

        public static String couldNotBuild(String name, Throwable e) {
            return "could not build " + name + ": " + e;
        }

        public static String badDriverStation(String problem) {
            return "could not read the driver station: " + problem;
        }
    }

    public static final int PROTOCOL = 4;

    public static final int OLDEST_PROTOCOL_READ = 1;

    public static final int PLACED_PROTOCOL = 3;

    public static final int SEEDED_PROTOCOL = 4;

    public static final class WrongProtocol extends RuntimeException {
        public final int childProtocol;

        WrongProtocol(int childProtocol, String message) {
            super(message);
            this.childProtocol = childProtocol;
        }
    }

    private static final Gson GSON = gson();

    private SimRunStream() {}

    public static String hello() {
        return helloOf(PROTOCOL);
    }

    /**
     * What a child speaking that protocol says first. A test of what a bench does with an older
     * child writes its hello with this, so the line it is handed is the line one would print.
     */
    public static String helloOf(int protocol) {
        JsonObject line = new JsonObject();
        line.addProperty("protocol", protocol);
        return GSON.toJson(line);
    }

    public static String afterHello(String firstLine) {
        return protocolOf(firstLine) == 1 ? firstLine : null;
    }

    public static int protocolOf(String firstLine) {
        JsonObject json;
        try {
            json = GSON.fromJson(firstLine, JsonObject.class);
        } catch (RuntimeException e) {
            return 1;
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

    public static final class Handshake {
        public final int protocol;

        public final String firstContentLine;

        public final JsonObject startLine;

        public final String outcome;

        public final String message;

        private Handshake(int protocol, String firstContentLine, JsonObject startLine, String outcome, String message) {
            this.protocol = protocol;
            this.firstContentLine = firstContentLine;
            this.startLine = startLine;
            this.outcome = outcome;
            this.message = message;
        }

        public boolean refused() {
            return outcome != null;
        }
    }

    public static Handshake handshake(String firstLine, Pose2d start, Long seed) {
        int protocol = protocolOf(firstLine);
        String content = protocol == 1 ? firstLine : null;
        if (seed != null && protocol < SEEDED_PROTOCOL) {
            return new Handshake(
                    protocol,
                    content,
                    null,
                    Outcome.cannotSeed(protocol),
                    "the simulator in these sources speaks protocol "
                            + protocol
                            + " and runs the exact robot on its own; a seed needs protocol "
                            + SEEDED_PROTOCOL
                            + ": the sources are older than the server. Pull develop, or clear the seed.");
        }
        if (protocol >= PLACED_PROTOCOL) {
            return new Handshake(protocol, content, SimDriverStation.startLine(start, seed), null, null);
        }
        Twist2d fromOrigin = start.minus(StartPoses.ORIGIN);
        if (Math.hypot(fromOrigin.line.x, fromOrigin.line.y) < 1e-9 && Math.abs(fromOrigin.angle) < 1e-9) {
            return new Handshake(protocol, content, null, null, null);
        }
        return new Handshake(
                protocol,
                content,
                null,
                Outcome.cannotPlace(protocol),
                "the simulator in these sources speaks protocol "
                        + protocol
                        + " and starts the robot at the origin on its own; placing it elsewhere needs protocol "
                        + PLACED_PROTOCOL
                        + ": the sources are older than the server. Pull develop, or place the robot"
                        + " back at the origin.");
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

    public static double seconds(JsonObject tick) {
        return tick.get("t").getAsDouble();
    }

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
            t.add("gamepads", gamepads);
        }
        if (tick.pieces != null && tick.pieces.length > 0) {
            t.add("pieces", GSON.toJsonTree(tick.pieces));
        }
        if (tick.held > 0) {
            t.addProperty("held", tick.held);
        }
        JsonObject scored = new JsonObject();
        for (Map.Entry<String, Integer> entry : new TreeMap<>(tick.scored).entrySet()) {
            if (entry.getValue() > 0) {
                scored.addProperty(entry.getKey(), entry.getValue());
            }
        }
        if (scored.size() > 0) {
            t.add("scored", scored);
        }
        JsonObject tilt = new JsonObject();
        for (SimField.Hive hive : SimPlacement.FIELD.hives) {
            Double leaning = tick.tilt.get(hive.alliance);
            if (leaning != null && leaning != hive.tilt) {
                tilt.add(hive.alliance, GSON.toJsonTree(leaning));
            }
        }
        if (tilt.size() > 0) {
            t.add("tilt", tilt);
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
