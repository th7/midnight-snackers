package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import java.util.EnumMap;
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
    public void whatChangedIsWrittenByNameAndReadBackAsTheSameConstants() {
        Constants changed = constants(Constant.SAG_VOLTS_PER_POWER, 0.3, Constant.LAUNCH_THROW, 0.25);

        JsonObject written = SimConstants.changed(changed);

        assertEquals("{\"sag_volts_per_power\":0.3,\"launch_throw\":0.25}", written.toString());
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
        assertRefused("launch_throw", SimConstants.fromJson(json("{\"gravity\": 1}")));
        assertRefused("Launcher throw is a number", SimConstants.fromJson(json("{\"launch_throw\": \"fast\"}")));
        assertRefused("\"fast\"", SimConstants.fromJson(json("{\"launch_throw\": \"fast\"}")));
        assertRefused("Launcher throw is a number", SimConstants.fromJson(json("{\"launch_throw\": null}")));
        assertRefused("Launcher throw is a number", SimConstants.fromJson(json("{\"launch_throw\": [1]}")));
        assertRefused("Launcher throw is from", SimConstants.fromJson(json("{\"launch_throw\": 2}")));
        assertRefused(
                "Flattest battery may not be more than freshest battery",
                SimConstants.fromJson(json("{\"flattest_volts\": 14}")));
    }

    @Test
    public void aRunsLogSaysWhichConstantsWereChangedAndFromWhat() {
        assertEquals("as built", SimConstants.described(Constants.defaults()));

        String said = SimConstants.described(constants(Constant.LAUNCH_THROW, 0.25, Constant.HICCUP_CHANCE, 0));

        assertEquals(
                "Hiccup chance 0.0 per loop (built 0.02); Launcher throw 0.25 in/s per tick/s (built 0.1899)", said);
    }
}
