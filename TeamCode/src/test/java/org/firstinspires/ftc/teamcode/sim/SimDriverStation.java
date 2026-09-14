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

/**
 * What the driver does during a run: the state of the two gamepads and the Stop button; and before
 * it, where the robot is placed ({@link #startLine}), which the child waits for before its run
 * starts. The controller page sends the gamepads, the bench relays them, and the runner copies the
 * gamepad states into the op mode's gamepads before every loop, the way the robot controller
 * copies in each packet from the real driver station, so the SDK's own edge detection
 * ({@code crossWasPressed()}) sees presses and releases exactly as it does on the robot. A state
 * holds until the next one replaces it, like a real gamepad. Safe to set from another thread while
 * the run reads it.
 */
public final class SimDriverStation {
    /**
     * One gamepad's inputs as a value, named as the SDK names the {@link Gamepad} fields: the
     * PlayStation names for the buttons, {@code left_stick_x} and the like for the axes.
     */
    public static final class State {
        /** The on/off inputs, by the SDK's PlayStation field names. */
        public static final List<String> BUTTONS = List.of(
                "dpad_up", "dpad_down", "dpad_left", "dpad_right",
                "cross", "circle", "square", "triangle",
                "left_bumper", "right_bumper", "left_stick_button", "right_stick_button",
                "share", "options", "touchpad", "ps");
        /** The 0 to 1 inputs. */
        public static final List<String> TRIGGERS = List.of("left_trigger", "right_trigger");
        /** The -1 to 1 inputs; a stick pushed forward reads negative y, as on the robot. */
        public static final List<String> AXES = List.of("left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y");
        public static final State NEUTRAL = new State(new float[6], Set.of());

        public final float leftStickX;
        public final float leftStickY;
        public final float rightStickX;
        public final float rightStickY;
        public final float leftTrigger;
        public final float rightTrigger;
        /** The buttons held, from {@link #BUTTONS}. */
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

        /**
         * @param json the inputs that are not neutral, e.g. {@code {"cross": true, "left_stick_y": -1}}
         * @throws IllegalArgumentException for a name that is not a gamepad input
         */
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

        /** The inputs that are not neutral; an empty object for {@link #NEUTRAL}. */
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
            return pressed.isEmpty() && leftStickX == 0 && leftStickY == 0 && rightStickX == 0 && rightStickY == 0
                    && leftTrigger == 0 && rightTrigger == 0;
        }

        /**
         * Make {@code gamepad} read these inputs, through the SDK's own packet copy so that its
         * edge detection notices what changed since the last state it was given.
         */
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
            // The SDK serializes the Xbox names; the PlayStation names are aliases it derives.
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
            return leftStickX == that.leftStickX && leftStickY == that.leftStickY
                    && rightStickX == that.rightStickX && rightStickY == that.rightStickY
                    && leftTrigger == that.leftTrigger && rightTrigger == that.rightTrigger
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

    /** The line that places the robot before the run: {@code {"start": {"x": .., "y": .., "heading": ..}}}. */
    public static JsonObject startLine(Pose2d start) {
        JsonObject line = new JsonObject();
        line.add("start", StartPoses.toJson(start));
        return line;
    }

    /** The robot is placed: the run may start. */
    public void place(Pose2d pose) {
        start = Objects.requireNonNull(pose, "start");
        placed.countDown();
    }

    /**
     * Waits until the robot is placed, or Stop is pressed first.
     *
     * @return where it was placed, or empty when the run was stopped (or this thread interrupted) before it was placed
     */
    public Optional<Pose2d> awaitPlacement() {
        try {
            placed.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return Optional.empty();
        }
        return Optional.ofNullable(start);
    }

    /**
     * @param gamepad 1 or 2
     */
    public synchronized void set(int gamepad, State state) {
        states[index(gamepad)] = Objects.requireNonNull(state, "state");
    }

    public synchronized State state(int gamepad) {
        return states[index(gamepad)];
    }

    /** Copy both states into the op mode's gamepads, as the robot controller does each packet. */
    public synchronized void applyTo(Gamepad gamepad1, Gamepad gamepad2) {
        states[0].applyTo(gamepad1);
        states[1].applyTo(gamepad2);
    }

    /** The driver pressed Stop: the run ends after the loop in progress, or never starts if not yet placed. */
    public void stop() {
        stopRequested = true;
        placed.countDown();
    }

    public boolean stopRequested() {
        return stopRequested;
    }

    /**
     * One line of what the bench sends the child: the {@link #startLine start line} that places
     * the robot, {@code {"gamepad": 1, "state": {...}}}, or {@code {"stop": true}}.
     *
     * @throws IllegalArgumentException for anything else, a start that is not a pose included
     */
    public void accept(JsonObject line) {
        if (line.has("start")) {
            if (!line.get("start").isJsonObject()) {
                throw new IllegalArgumentException("not a start pose: " + line.get("start"));
            }
            place(StartPoses.fromJson(line.getAsJsonObject("start")));
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
