package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.sim.SimDriverStation.State;
import org.junit.Test;

public class SimDriverStationTest {
    private static JsonObject json(String text) {
        return new Gson().fromJson(text, JsonObject.class);
    }

    @Test
    public void aStateAppliedToAGamepadSetsItsFieldsAndTheEdgeDetectionSeesOnePress() {
        Gamepad gamepad = new Gamepad();
        State state = State.fromJson(json("{\"cross\": true, \"left_stick_y\": -1, \"right_trigger\": 0.7, \"dpad_left\": true}"));

        state.applyTo(gamepad);

        assertTrue(gamepad.cross);
        assertTrue("cross is the PlayStation name for a", gamepad.a);
        assertTrue(gamepad.dpad_left);
        assertFalse(gamepad.circle);
        assertEquals(-1, gamepad.left_stick_y, 0);
        assertEquals(0, gamepad.left_stick_x, 0);
        assertEquals(0.7, gamepad.right_trigger, 1e-6);
        assertTrue("the loop after the press sees it", gamepad.crossWasPressed());
        assertFalse("and only that loop", gamepad.crossWasPressed());

        state.applyTo(gamepad);
        assertTrue("still held", gamepad.cross);
        assertFalse("holding is not pressing again", gamepad.crossWasPressed());

        State.NEUTRAL.applyTo(gamepad);
        assertFalse(gamepad.cross);
        assertTrue(gamepad.crossWasReleased());
        assertEquals(0, gamepad.left_stick_y, 0);
    }

    @Test
    public void everyPlayStationButtonReachesTheGamepadUnderItsOwnNameAndItsXboxAlias() throws Exception {
        for (String button : State.BUTTONS) {
            Gamepad gamepad = new Gamepad();
            State.fromJson(json("{\"" + button + "\": true}")).applyTo(gamepad);
            assertTrue(button, Gamepad.class.getField(button).getBoolean(gamepad));
            assertTrue(button, Gamepad.class.getField(aliasOf(button)).getBoolean(gamepad));
            for (String other : State.BUTTONS) {
                if (!other.equals(button)) {
                    assertFalse(button + " also pressed " + other, Gamepad.class.getField(other).getBoolean(gamepad));
                }
            }
        }
    }

    /** The field the SDK serializes each button as; the PlayStation names are aliases of the Xbox ones. */
    private static String aliasOf(String button) {
        switch (button) {
            case "cross": return "a";
            case "circle": return "b";
            case "square": return "x";
            case "triangle": return "y";
            case "share": return "back";
            case "options": return "start";
            case "ps": return "guide";
            default: return button;
        }
    }

    @Test
    public void jsonRoundTripsAndCarriesOnlyWhatIsNotNeutral() {
        State state = State.fromJson(json("{\"square\": true, \"left_stick_x\": 0.25, \"left_trigger\": 1}"));

        JsonObject out = state.toJson();

        assertEquals(3, out.size());
        assertTrue(out.get("square").getAsBoolean());
        assertEquals(0.25, out.get("left_stick_x").getAsDouble(), 1e-6);
        assertEquals(1, out.get("left_trigger").getAsDouble(), 1e-6);
        assertEquals(state, State.fromJson(out));
        assertEquals(0, State.NEUTRAL.toJson().size());
        assertTrue(State.NEUTRAL.neutral());
        assertFalse(state.neutral());
        assertTrue("neutral inputs are not carried", State.fromJson(json("{\"cross\": false, \"left_stick_x\": 0}")).neutral());
    }

    @Test
    public void axesAreClampedAndUnknownInputsAreRefused() {
        State state = State.fromJson(json("{\"left_stick_x\": -7, \"right_stick_y\": 3, \"right_trigger\": 2, \"left_trigger\": -1}"));

        assertEquals(-1, state.leftStickX, 0);
        assertEquals(1, state.rightStickY, 0);
        assertEquals(1, state.rightTrigger, 0);
        assertEquals(0, state.leftTrigger, 0);
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class,
                () -> State.fromJson(json("{\"corss\": true}")));
        assertTrue(e.getMessage(), e.getMessage().contains("corss"));
    }

    @Test
    public void theStationHoldsBothGamepadsAndTheStopButton() {
        SimDriverStation station = new SimDriverStation();
        Gamepad one = new Gamepad();
        Gamepad two = new Gamepad();

        station.set(1, State.fromJson(json("{\"triangle\": true}")));
        station.set(2, State.fromJson(json("{\"dpad_up\": true}")));
        station.applyTo(one, two);

        assertTrue(one.triangle);
        assertFalse(one.dpad_up);
        assertTrue(two.dpad_up);
        assertFalse(two.triangle);
        assertEquals(State.fromJson(json("{\"triangle\": true}")), station.state(1));
        assertThrows(IllegalArgumentException.class, () -> station.set(3, State.NEUTRAL));
        assertFalse(station.stopRequested());
        station.stop();
        assertTrue(station.stopRequested());
    }

    @Test
    public void acceptsTheLinesTheChildReadsFromItsParent() {
        SimDriverStation station = new SimDriverStation();

        station.accept(json("{\"gamepad\": 2, \"state\": {\"circle\": true, \"right_stick_x\": 0.5}}"));
        assertTrue(station.state(2).pressed.contains("circle"));
        assertEquals(0.5, station.state(2).rightStickX, 1e-6);
        assertTrue(station.state(1).neutral());
        assertFalse(station.stopRequested());

        station.accept(json("{\"stop\": true}"));
        assertTrue(station.stopRequested());

        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> station.accept(json("{\"nonsense\": 1}")));
        assertTrue(e.getMessage(), e.getMessage().contains("nonsense"));
    }
}
