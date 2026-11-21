package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.EnumMap;
import java.util.Map;
import org.junit.Test;

public class FeedSettingsTest {
    private static Map<FeedSetting, Double> set(FeedSetting setting, double value) {
        Map<FeedSetting, Double> set = new EnumMap<>(FeedSetting.class);
        set.put(setting, value);
        return set;
    }

    private static String refusal(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    @Test
    public void theDefaultSettingsPlanTheDefaultFeed() {
        Feed.Fed fromSettings =
                Valid.value(Valid.value(FeedSettings.defaults().plan()).run());

        assertEquals(Valid.value(Valid.value(Feed.Plan.defaults()).run()), fromSettings);
    }

    @Test
    public void aSettingBeyondItsRangeIsRefusedNamingItsLabelAndRange() {
        assertEquals(
                "Spinner speed is from 0.2 to 3.0 rev/s, not 3.5",
                refusal(FeedSettings.of(set(FeedSetting.SPINNER_SPEED, 3.5))));
        assertEquals(
                "Back sill is from 16.0 to 95.0 mm, not 10.0",
                refusal(FeedSettings.of(set(FeedSetting.BACK_SILL, 10))));
        assertEquals("Pollen is 0, 1 or 2, not 1.5", refusal(FeedSettings.of(set(FeedSetting.POLLEN, 1.5))));
    }

    @Test
    public void aRampLoweredBeforeItIsRaisedOrUpNotAboveDownOrMovedAfterTheRunIsRefused() {
        Map<FeedSetting, Double> early = set(FeedSetting.RAMP_LOWERED_AT, 0.2);
        Map<FeedSetting, Double> flat = set(FeedSetting.RAMP_DOWN, 40);
        Map<FeedSetting, Double> late = set(FeedSetting.RAMP_LOWERED_AT, 4);

        assertEquals(
                "a ramp is raised before it is lowered, not raised at 0.3 s and lowered at 0.2 s",
                refusal(FeedSettings.of(early)));
        assertEquals(
                "a ramp is down below where it is up, not down at 40.0 degrees and up at 35.0 degrees",
                refusal(FeedSettings.of(flat)));
        assertEquals("a ramp is lowered within the run of 3.0 s, not at 4.0 s", refusal(FeedSettings.of(late)));
    }

    @Test
    public void anUnknownSettingIsRefusedNamingEveryOne() {
        String refused = refusal(FeedSetting.named("speed"));

        assertTrue(refused, refused.startsWith("a feed setting is pollen, first_pollen, second_pollen,"));
        assertTrue(refused, refused.endsWith(", not 'speed'"));
    }

    @Test
    public void oneSetPollenPlansAFeedOfOnlyTheFirstAndChangedNamesOnlyWhatDiffersFromTheDefaults() {
        FeedSettings one = Valid.value(FeedSettings.of(set(FeedSetting.POLLEN, 1)));

        Feed.Fed fed = Valid.value(Valid.value(one.plan()).run());
        assertEquals(1, fed.frames().get(0).pollen().size());
        assertEquals(Map.of(FeedSetting.POLLEN, 1.0), one.changed());
        assertEquals(Map.of(), FeedSettings.defaults().changed());
    }
}
