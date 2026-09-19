package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.CountDownLatch;

public final class SimDriverStation {
    public static final class State {
        public static final List<String> BUTTONS = List.of(
                "dpad_up",
                "dpad_down",
                "dpad_left",
                "dpad_right",
                "cross",
                "circle",
                "square",
                "triangle",
                "left_bumper",
                "right_bumper",
                "left_stick_button",
                "right_stick_button",
                "share",
                "options",
                "touchpad",
                "ps");

        public static final List<String> TRIGGERS = List.of("left_trigger", "right_trigger");

        public static final List<String> AXES =
                List.of("left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y");

        public static final State NEUTRAL = new State(new float[6], Set.of());

        public final float leftStickX;
        public final float leftStickY;
        public final float rightStickX;
        public final float rightStickY;
        public final float leftTrigger;
        public final float rightTrigger;

        public final Set<String> pressed;

        private State(float[] axes, Set<String> pressed) {
            this.leftStickX = axes[0];
            this.leftStickY = axes[1];
            this.rightStickX = axes[2];
            this.rightStickY = axes[3];
            this.leftTrigger = axes[4];
            this.rightTrigger = axes[5];
            this.pressed = pressed;
        }

        public static State fromJson(JsonObject json) {
            float[] axes = new float[6];
            Set<String> pressed = new TreeSet<>();
            for (Map.Entry<String, JsonElement> field : json.entrySet()) {
                String name = field.getKey();
                if (BUTTONS.contains(name)) {
                    if (field.getValue().getAsBoolean()) {
                        pressed.add(name);
                    }
                } else if (AXES.contains(name)) {
                    axes[AXES.indexOf(name)] = clamp(field.getValue().getAsFloat(), -1, 1);
                } else if (TRIGGERS.contains(name)) {
                    axes[4 + TRIGGERS.indexOf(name)] = clamp(field.getValue().getAsFloat(), 0, 1);
                } else {
                    throw new IllegalArgumentException("no gamepad input is named '" + name + "'");
                }
            }
            return new State(axes, Collections.unmodifiableSet(pressed));
        }

        public JsonObject toJson() {
            JsonObject json = new JsonObject();
            for (String button : BUTTONS) {
                if (pressed.contains(button)) {
                    json.addProperty(button, true);
                }
            }
            float[] axes = {leftStickX, leftStickY, rightStickX, rightStickY};
            for (int i = 0; i < axes.length; i++) {
                if (axes[i] != 0) {
                    json.addProperty(AXES.get(i), axes[i]);
                }
            }
            float[] triggers = {leftTrigger, rightTrigger};
            for (int i = 0; i < triggers.length; i++) {
                if (triggers[i] != 0) {
                    json.addProperty(TRIGGERS.get(i), triggers[i]);
                }
            }
            return json;
        }

        public boolean neutral() {
            return pressed.isEmpty()
                    && leftStickX == 0
                    && leftStickY == 0
                    && rightStickX == 0
                    && rightStickY == 0
                    && leftTrigger == 0
                    && rightTrigger == 0;
        }

        public void applyTo(Gamepad gamepad) {
            Gamepad source = new Gamepad();
            source.type = Gamepad.Type.SONY_PS4;
            source.left_stick_x = leftStickX;
            source.left_stick_y = leftStickY;
            source.right_stick_x = rightStickX;
            source.right_stick_y = rightStickY;
            source.left_trigger = leftTrigger;
            source.right_trigger = rightTrigger;
            source.dpad_up = pressed.contains("dpad_up");
            source.dpad_down = pressed.contains("dpad_down");
            source.dpad_left = pressed.contains("dpad_left");
            source.dpad_right = pressed.contains("dpad_right");

            source.a = pressed.contains("cross");
            source.b = pressed.contains("circle");
            source.x = pressed.contains("square");
            source.y = pressed.contains("triangle");
            source.left_bumper = pressed.contains("left_bumper");
            source.right_bumper = pressed.contains("right_bumper");
            source.left_stick_button = pressed.contains("left_stick_button");
            source.right_stick_button = pressed.contains("right_stick_button");
            source.back = pressed.contains("share");
            source.start = pressed.contains("options");
            source.touchpad = pressed.contains("touchpad");
            source.guide = pressed.contains("ps");
            gamepad.copy(source);
        }

        private static float clamp(float value, float low, float high) {
            return Math.max(low, Math.min(high, value));
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof State)) {
                return false;
            }
            State that = (State) other;
            return leftStickX == that.leftStickX
                    && leftStickY == that.leftStickY
                    && rightStickX == that.rightStickX
                    && rightStickY == that.rightStickY
                    && leftTrigger == that.leftTrigger
                    && rightTrigger == that.rightTrigger
                    && pressed.equals(that.pressed);
        }

        @Override
        public int hashCode() {
            return Objects.hash(leftStickX, leftStickY, rightStickX, rightStickY, leftTrigger, rightTrigger, pressed);
        }

        @Override
        public String toString() {
            return toJson().toString();
        }
    }

    private final State[] states = {State.NEUTRAL, State.NEUTRAL};
    private volatile boolean stopRequested = false;
    private final CountDownLatch placed = new CountDownLatch(1);
    private volatile Pose2d start;
    private volatile Long seed;

    public static JsonObject startLine(Pose2d start) {
        return startLine(start, null);
    }

    public static JsonObject startLine(Pose2d start, Long seed) {
        JsonObject line = new JsonObject();
        line.add("start", StartPoses.toJson(start));
        if (seed != null) {
            line.addProperty("seed", seed);
        }
        return line;
    }

    public void place(Pose2d pose) {
        place(pose, null);
    }

    public void place(Pose2d pose, Long seed) {
        start = Objects.requireNonNull(pose, "start");
        this.seed = seed;
        placed.countDown();
    }

    public Long seed() {
        return seed;
    }

    public Optional<Pose2d> awaitPlacement() {
        try {
            placed.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return Optional.empty();
        }
        return Optional.ofNullable(start);
    }

    public synchronized void set(int gamepad, State state) {
        states[index(gamepad)] = Objects.requireNonNull(state, "state");
    }

    public synchronized State state(int gamepad) {
        return states[index(gamepad)];
    }

    public synchronized Applied applyTo(Gamepad gamepad1, Gamepad gamepad2) {
        State one = states[0];
        State two = states[1];
        one.applyTo(gamepad1);
        two.applyTo(gamepad2);
        return new Applied(one, two);
    }

    public static final class Applied {
        public final State gamepad1;
        public final State gamepad2;

        Applied(State gamepad1, State gamepad2) {
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
        }
    }

    public void stop() {
        stopRequested = true;
        placed.countDown();
    }

    public boolean stopRequested() {
        return stopRequested;
    }

    public void accept(JsonObject line) {
        if (line.has("start")) {
            if (!line.get("start").isJsonObject()) {
                throw new IllegalArgumentException("not a start pose: " + line.get("start"));
            }
            place(StartPoses.fromJson(line.getAsJsonObject("start")), StartPoses.seedFromJson(line.get("seed")));
            return;
        }
        if (line.has("gamepad") && line.has("state")) {
            set(line.get("gamepad").getAsInt(), State.fromJson(line.getAsJsonObject("state")));
            return;
        }
        if (line.has("stop")) {
            if (line.get("stop").getAsBoolean()) {
                stop();
            }
            return;
        }
        throw new IllegalArgumentException("not a driver station line: " + line);
    }

    private static int index(int gamepad) {
        if (gamepad != 1 && gamepad != 2) {
            throw new IllegalArgumentException("there is no gamepad " + gamepad + ", only 1 and 2");
        }
        return gamepad - 1;
    }
}
