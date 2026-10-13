package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.Test;

public class ShotSettingsTest {
    private static void assertRejected(String rule, Checked<?> checked) {
        String said = said(checked);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    private static String said(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, reason -> reason);
    }

    private static Map<ShotSetting, Double> set(Object... pairs) {
        Map<ShotSetting, Double> set = new HashMap<>();
        for (int i = 0; i < pairs.length; i += 2) {
            set.put((ShotSetting) pairs[i], ((Number) pairs[i + 1]).doubleValue());
        }
        return set;
    }

    private static Checked<ShotSettings> of(Object... pairs) {
        return ShotSettings.of(set(pairs));
    }

    @Test
    public void theDefaultsHoldEverySettingAtItsDefaultAndNothingChanged() {
        ShotSettings defaults = ShotSettings.defaults();

        for (ShotSetting setting : ShotSetting.values()) {
            assertEquals(setting.name(), setting.byDefault(), defaults.value(setting), 0);
        }
        assertTrue(defaults.changed().isEmpty());
        assertEquals(55, defaults.value(ShotSetting.X), 0);
        assertEquals(-12.76, defaults.value(ShotSetting.Y), 0);
        assertEquals(0.634, defaults.value(ShotSetting.INERTIA), 0);
    }

    @Test
    public void settingNothingIsTheDefaults() {
        ShotSettings nothing = Valid.value(ShotSettings.of(new EnumMap<>(ShotSetting.class)));

        assertEquals(ShotSettings.defaults(), nothing);
        assertEquals(ShotSettings.defaults().hashCode(), nothing.hashCode());
        assertEquals(ShotSettings.defaults(), Valid.value(of(ShotSetting.PITCH, 70, ShotSetting.PIECES, 1)));
    }

    @Test
    public void aSetKeepsTheValuesItIsGivenAndDefaultsTheRest() {
        ShotSettings set = Valid.value(of(ShotSetting.PITCH, 45, ShotSetting.WIND_Y, -30, ShotSetting.CELL_GRIP, 0.4));

        assertEquals(45, set.value(ShotSetting.PITCH), 0);
        assertEquals(-30, set.value(ShotSetting.WIND_Y), 0);
        assertEquals(0.4, set.value(ShotSetting.CELL_GRIP), 0);
        for (ShotSetting setting : ShotSetting.values()) {
            if (setting != ShotSetting.PITCH && setting != ShotSetting.WIND_Y && setting != ShotSetting.CELL_GRIP) {
                assertEquals(setting.name(), setting.byDefault(), set.value(setting), 0);
            }
        }
    }

    @Test
    public void aSettingGivenNoValueAtAllTakesItsDefault() {
        Map<ShotSetting, Double> given = new HashMap<>();
        given.put(ShotSetting.WIND_X, null);
        given.put(ShotSetting.YAW, 90.0);

        ShotSettings set = Valid.value(ShotSettings.of(given));

        assertEquals(0, set.value(ShotSetting.WIND_X), 0);
        assertEquals(90, set.value(ShotSetting.YAW), 0);
    }

    @Test
    public void aSetWithAValueOutsideItsSettingsRangeIsRefusedAsTheSettingRefusesIt() {
        for (ShotSetting setting : ShotSetting.values()) {
            for (double wrong : List.of(
                    Math.nextDown(setting.least()),
                    Math.nextUp(setting.most()),
                    Double.NaN,
                    Double.POSITIVE_INFINITY,
                    Double.NEGATIVE_INFINITY)) {
                assertEquals(setting.name() + " at " + wrong, said(setting.admits(wrong)), said(of(setting, wrong)));
            }
        }
        assertRejected("Wheels slow down", of(ShotSetting.WHEELS_SLOW, 0.5));
        assertRejected("0.5", of(ShotSetting.WHEELS_SLOW, 0.5));
    }

    @Test
    public void ofSeveralBadValuesTheFirstInTheSettingsOrderIsTheOneRefused() {
        Checked<ShotSettings> refused = of(ShotSetting.LONGEST, 99, ShotSetting.DRAG, 2, ShotSetting.PITCH, 95);

        assertRejected("Pitch", refused);
        assertFalse(said(refused).contains("Drag"));
        assertFalse(said(refused).contains("Longest"));
    }

    @Test
    public void whatChangedIsWhatDiffersFromTheDefaultsInTheOrderTheSettingsAreListed() {
        ShotSettings changed = Valid.value(of(
                ShotSetting.LONGEST,
                12,
                ShotSetting.YAW,
                ShotSetting.YAW.byDefault(),
                ShotSetting.FLOOR_FRICTION,
                0.7,
                ShotSetting.WHEELS_SLOW,
                1,
                ShotSetting.X,
                40));

        assertEquals(
                List.of(ShotSetting.X, ShotSetting.WHEELS_SLOW, ShotSetting.FLOOR_FRICTION, ShotSetting.LONGEST),
                List.copyOf(changed.changed().keySet()));
        assertEquals(40, changed.changed().get(ShotSetting.X), 0);
        assertEquals(1, changed.changed().get(ShotSetting.WHEELS_SLOW), 0);
        assertEquals(0.7, changed.changed().get(ShotSetting.FLOOR_FRICTION), 0);
        assertEquals(12, changed.changed().get(ShotSetting.LONGEST), 0);
        assertEquals(ShotSetting.YAW.byDefault(), changed.value(ShotSetting.YAW), 0);
    }

    @Test
    public void aNegativeZeroIsNoChange() {
        ShotSettings zero = Valid.value(of(ShotSetting.ROLL, -0.0, ShotSetting.WHEELS_SLOW, -0.0));

        assertTrue(zero.changed().toString(), zero.changed().isEmpty());
        assertEquals(ShotSettings.defaults(), zero);
    }

    @Test
    public void aSwitchIsOnWhenItIsOne() {
        ShotSettings defaults = ShotSettings.defaults();
        assertFalse(defaults.on(ShotSetting.WHEELS_SLOW));
        assertTrue(defaults.on(ShotSetting.PIECES));
        assertFalse(defaults.on(ShotSetting.BLUE_HIVE_TIPPED));
        assertFalse(defaults.on(ShotSetting.RED_HIVE_TIPPED));

        ShotSettings flipped = Valid.value(of(
                ShotSetting.WHEELS_SLOW,
                1,
                ShotSetting.PIECES,
                0,
                ShotSetting.BLUE_HIVE_TIPPED,
                1,
                ShotSetting.RED_HIVE_TIPPED,
                1));
        assertTrue(flipped.on(ShotSetting.WHEELS_SLOW));
        assertFalse(flipped.on(ShotSetting.PIECES));
        assertTrue(flipped.on(ShotSetting.BLUE_HIVE_TIPPED));
        assertTrue(flipped.on(ShotSetting.RED_HIVE_TIPPED));
    }

    @Test
    public void twoSetsOfTheSameValuesAreTheSame() {
        assertEquals(Valid.value(of(ShotSetting.DRAG, 0.3)), Valid.value(of(ShotSetting.DRAG, 0.3)));
        assertEquals(
                Valid.value(of(ShotSetting.DRAG, 0.3)).hashCode(),
                Valid.value(of(ShotSetting.DRAG, 0.3)).hashCode());
        assertNotEquals(ShotSettings.defaults(), Valid.value(of(ShotSetting.DRAG, 0.3)));
        assertNotEquals(Valid.value(of(ShotSetting.DRAG, 0.3)), Valid.value(of(ShotSetting.DRAG, 0.31)));
        assertNotEquals(ShotSettings.defaults(), "the defaults");
    }

    @Test
    public void aSetSaysWhatChanged() {
        assertEquals("the defaults", ShotSettings.defaults().toString());
        String said =
                Valid.value(of(ShotSetting.DRAG, 0.3, ShotSetting.PITCH, 45)).toString();
        assertTrue(said, said.contains("PITCH=45.0"));
        assertTrue(said, said.indexOf("PITCH") < said.indexOf("DRAG=0.3"));
    }

    @Test
    public void whatASetIsHandedIsNotWhatItHolds() {
        Map<ShotSetting, Double> handed = set(ShotSetting.DRAG, 0.3);
        ShotSettings settings = Valid.value(ShotSettings.of(handed));
        handed.put(ShotSetting.DRAG, 0.5);
        settings.changed().put(ShotSetting.DRAG, 0.7);

        assertEquals(0.3, settings.value(ShotSetting.DRAG), 0);
    }
}
