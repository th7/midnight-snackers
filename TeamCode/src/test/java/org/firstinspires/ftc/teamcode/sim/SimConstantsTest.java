package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.junit.Test;

public class SimConstantsTest {
    private static JsonElement json(String text) {
        return new Gson().fromJson(text, JsonElement.class);
    }

    private static Constants constants(Object... pairs) {
        Map<Constant, Double> set = new EnumMap<>(Constant.class);
        for (int i = 0; i < pairs.length; i += 2) {
            set.put((Constant) pairs[i], ((Number) pairs[i + 1]).doubleValue());
        }
        return Valid.value(Constants.of(set));
    }

    private static void assertRefused(String saying, Checked<Constants> read) {
        String said = read.fold(value -> "accepted " + value, rule -> rule);
        assertTrue(said, read instanceof Checked.Rejected && said.contains(saying));
    }

    @Test
    public void theListingSaysEveryConstantInOrderWithWhatItIsWhereItMayGoAndWhereItIs() {
        JsonObject listing = SimConstants.listing(constants(Constant.FLYWHEEL_TICKS_PER_TURN, 30.5));

        JsonArray listed = listing.getAsJsonArray("constants");
        List<String> names = new ArrayList<>();
        for (JsonElement element : listed) {
            names.add(element.getAsJsonObject().get("name").getAsString());
        }
        List<String> every = new ArrayList<>();
        for (Constant constant : Constant.values()) {
            every.add(constant.asked());
        }
        assertEquals(every, names);

        Constant ticksPerTurn = Constant.FLYWHEEL_TICKS_PER_TURN;
        JsonObject ticks = listed.get(ticksPerTurn.ordinal()).getAsJsonObject();
        assertEquals("mechanisms", ticks.get("group").getAsString());
        assertEquals(ticksPerTurn.label(), ticks.get("label").getAsString());
        assertEquals(ticksPerTurn.unit(), ticks.get("unit").getAsString());
        assertEquals(ticksPerTurn.says(), ticks.get("says").getAsString());
        assertEquals(ticksPerTurn.least(), ticks.get("least").getAsDouble(), 0);
        assertEquals(ticksPerTurn.most(), ticks.get("most").getAsDouble(), 0);
        assertEquals(ticksPerTurn.byDefault(), ticks.get("byDefault").getAsDouble(), 0);
        assertEquals(30.5, ticks.get("value").getAsDouble(), 0);
        assertFalse("nothing was dropped from constants set as they are", listing.has("dropped"));
        JsonObject spread = listed.get(Constant.MOTOR_SPREAD.ordinal()).getAsJsonObject();
        assertEquals(Constant.MOTOR_SPREAD.byDefault(), spread.get("value").getAsDouble(), 0);

        JsonArray groups = listing.getAsJsonArray("groups");
        assertEquals(Constant.Group.values().length, groups.size());
        JsonObject noise = groups.get(0).getAsJsonObject();
        assertEquals("noise", noise.get("name").getAsString());
        assertEquals(Constant.Group.NOISE.label(), noise.get("label").getAsString());
        assertEquals(Constant.Group.NOISE.says(), noise.get("says").getAsString());
    }

    @Test
    public void whatChangedIsWrittenByNameAndReadBackAsTheSameConstants() {
        Constants changed = constants(Constant.SAG_VOLTS_PER_POWER, 0.3, Constant.FLYWHEEL_TICKS_PER_TURN, 30.5);

        JsonObject written = SimConstants.changed(changed);

        assertEquals("{\"sag_volts_per_power\":0.3,\"flywheel_ticks_per_turn\":30.5}", written.toString());
        assertEquals(changed, Valid.value(SimConstants.fromJson(written)));
        assertEquals("{}", SimConstants.changed(Constants.defaults()).toString());
    }

    @Test
    public void nothingAtAllIsTheConstantsAsBuilt() {
        assertEquals(Constants.defaults(), Valid.value(SimConstants.fromJson(null)));
        assertEquals(Constants.defaults(), Valid.value(SimConstants.fromJson(JsonNull.INSTANCE)));
        assertEquals(Constants.defaults(), Valid.value(SimConstants.fromJson(json("{}"))));
    }

    @Test
    public void constantsThatAreNotAnObjectOfNumbersByNameAreRefusedSayingWhy() {
        assertRefused("an object", SimConstants.fromJson(json("[0.2]")));
        assertRefused("an object", SimConstants.fromJson(json("0.2")));
        assertRefused("'gravity'", SimConstants.fromJson(json("{\"gravity\": 1}")));
        assertRefused("flywheel_ticks_per_turn", SimConstants.fromJson(json("{\"gravity\": 1}")));
        String ticks = "Flywheel ticks per turn";
        assertRefused(ticks + " is a number", SimConstants.fromJson(json("{\"flywheel_ticks_per_turn\": \"fast\"}")));
        assertRefused("\"fast\"", SimConstants.fromJson(json("{\"flywheel_ticks_per_turn\": \"fast\"}")));
        assertRefused(ticks + " is a number", SimConstants.fromJson(json("{\"flywheel_ticks_per_turn\": null}")));
        assertRefused(ticks + " is a number", SimConstants.fromJson(json("{\"flywheel_ticks_per_turn\": [1]}")));
        assertRefused(ticks + " is from", SimConstants.fromJson(json("{\"flywheel_ticks_per_turn\": 2}")));
        assertRefused("is no longer a simulation constant", SimConstants.fromJson(json("{\"launch_throw\": 0.25}")));
        assertRefused(
                "Flattest battery may not be more than freshest battery",
                SimConstants.fromJson(json("{\"flattest_volts\": 14}")));
    }

    @Test
    public void savedConstantsPastTheirRangeAreReadAtTheNearerEndAndListedWithWhatWasSaved() {
        Constants.Restored restored =
                Valid.value(SimConstants.restoredFromJson(json("{\"turntable_speed\": 20000, \"hiccup_chance\": 0}")));

        assertEquals(Constant.TURNTABLE_SPEED.most(), restored.constants().value(Constant.TURNTABLE_SPEED), 0);
        assertEquals(0, restored.constants().value(Constant.HICCUP_CHANCE), 0);
        JsonArray listed = SimConstants.listing(restored).getAsJsonArray("constants");
        JsonObject turntable = listed.get(Constant.TURNTABLE_SPEED.ordinal()).getAsJsonObject();
        assertEquals(Constant.TURNTABLE_SPEED.most(), turntable.get("value").getAsDouble(), 0);
        assertEquals(20000, turntable.get("saved").getAsDouble(), 0);
        for (JsonElement other : listed) {
            if (other != turntable) {
                assertFalse(other.toString(), other.getAsJsonObject().has("saved"));
            }
        }
        assertEquals(
                Constants.defaults(),
                Valid.value(SimConstants.restoredFromJson(null)).constants());
        assertTrue(
                "saved as a number is still the rule",
                SimConstants.restoredFromJson(json("{\"turntable_speed\": \"fast\"}"))
                        .fold(read -> "accepted " + read, rule -> rule)
                        .contains("Turntable speed is a number"));
        assertFalse(
                "a listing of constants set on the page says nothing was saved past its range",
                SimConstants.listing(restored.constants()).toString().contains("saved"));
    }

    @Test
    public void constantsRestoredPastTheirRangeAreWrittenAsTheyWereSavedAndReadBackAsTheSame() {
        Constants.Restored restored =
                Valid.value(SimConstants.restoredFromJson(json("{\"turntable_speed\": 20000, \"hiccup_chance\": 0}")));

        JsonObject written = SimConstants.saved(restored);

        assertEquals("{\"hiccup_chance\":0.0,\"turntable_speed\":20000.0}", written.toString());
        Constants.Restored again = Valid.value(SimConstants.restoredFromJson(written));
        assertEquals(restored.constants(), again.constants());
        assertEquals(restored.pastTheirRange(), again.pastTheirRange());
    }

    @Test
    public void aSavedRetiredConstantIsSetAsideWhateverItsValueListedAsDroppedAndWrittenBackAsItWasSaved() {
        for (double saved : new double[] {0.25, 5, -1}) {
            Constants.Restored restored = Valid.value(
                    SimConstants.restoredFromJson(json("{\"launch_throw\": " + saved + ", \"hiccup_chance\": 0}")));

            assertEquals(constants(Constant.HICCUP_CHANCE, 0), restored.constants());
            JsonArray dropped = SimConstants.listing(restored).getAsJsonArray("dropped");
            assertEquals(1, dropped.size());
            JsonObject thrown = dropped.get(0).getAsJsonObject();
            assertEquals("launch_throw", thrown.get("name").getAsString());
            assertEquals("Launcher throw", thrown.get("label").getAsString());
            assertEquals(saved, thrown.get("saved").getAsDouble(), 0);
            assertEquals(Constant.Retired.LAUNCH_THROW.why(), thrown.get("why").getAsString());
            JsonObject written = SimConstants.saved(restored);
            assertEquals(saved, written.get("launch_throw").getAsDouble(), 0);
            assertEquals(
                    restored.setAside(),
                    Valid.value(SimConstants.restoredFromJson(written)).setAside());
        }
        assertTrue(
                "saved as a number is still the rule",
                SimConstants.restoredFromJson(json("{\"launch_throw\": \"fast\"}"))
                        .fold(read -> "accepted " + read, rule -> rule)
                        .contains("Launcher throw is a number"));
        assertTrue(
                "a name that was never a constant still stops it",
                SimConstants.restoredFromJson(json("{\"gravity\": 1}"))
                        .fold(read -> "accepted " + read, rule -> rule)
                        .contains("'gravity'"));
    }

    @Test
    public void aRunsLogSaysWhichConstantsWereChangedAndFromWhat() {
        assertEquals("as built", SimConstants.described(Constants.defaults()));

        String said =
                SimConstants.described(constants(Constant.FLYWHEEL_TICKS_PER_TURN, 30.5, Constant.HICCUP_CHANCE, 0));

        assertEquals(
                "Hiccup chance 0.0 per loop (built 0.02); Flywheel ticks per turn 30.5 ticks per turn of the flywheel"
                        + " (built 28.0)",
                said);
    }
}
