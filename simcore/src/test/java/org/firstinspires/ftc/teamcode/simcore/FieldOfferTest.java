package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import org.junit.Test;

public class FieldOfferTest {
    private static final String HIGH_DIGEST = "47b0fe386d7ef525acfca5899e41e3e160d08880c45bc8240fc4b76deec009d4";
    private static final String LOW_DIGEST = "431eab46959ac990d34c3a80f475db662121e75830e02f201209e2db9307cc0f";

    private static FieldOffer.Said said(String resolution, String file, long bytes, String sha256) {
        return new FieldOffer.Said(resolution, file, bytes, sha256);
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    private static FieldOffer offerOfLowAndHigh() {
        return Valid.value(FieldOffer.read(
                1, List.of(said("low", "field.glb", 17, LOW_DIGEST), said("high", "field-high.glb", 22, HIGH_DIGEST))));
    }

    @Test
    public void anOfferNamesEachModelBuiltWithItsSizeAndDigest() {
        List<FieldOffer.Offered> models = offerOfLowAndHigh().models();

        assertEquals(2, models.size());
        assertEquals(FieldModel.LOW, models.get(0).model());
        assertEquals(17, models.get(0).bytes());
        assertEquals(LOW_DIGEST, models.get(0).sha256());
        assertEquals(FieldModel.HIGH, models.get(1).model());
        assertEquals("field-high.glb", models.get(1).model().file());
    }

    @Test
    public void anOfferInAVersionNewerThanThisSimulatorReadsIsRefusedSayingToPull() {
        String said = rejection(FieldOffer.read(2, List.of(said("high", "field-high.glb", 22, HIGH_DIGEST))));

        assertTrue(said, said.contains("version 2"));
        assertTrue(said, said.contains("reads version 1"));
        assertTrue(said, said.contains("pull"));
    }

    @Test
    public void anOfferInAVersionNoBuildEverWroteIsRefused() {
        String said = rejection(FieldOffer.read(0, List.of()));

        assertTrue(said, said.contains("version 0"));
    }

    @Test
    public void aModelOfNoResolutionThisSimulatorKnowsIsRefusedNamingTheOnesItDoes() {
        String said = rejection(FieldOffer.read(1, List.of(said("ultra", "field-ultra.glb", 22, HIGH_DIGEST))));

        assertTrue(said, said.contains("ultra"));
        assertTrue(said, said.contains("low, medium or high"));
    }

    @Test
    public void aModelWhoseFileIsNotItsResolutionsIsRefusedSoNoOfferNamesAPathToWrite() {
        String said = rejection(FieldOffer.read(1, List.of(said("high", "../sessions.json", 22, HIGH_DIGEST))));

        assertTrue(said, said.contains("../sessions.json"));
        assertTrue(said, said.contains("field-high.glb"));
    }

    @Test
    public void aDigestIsSixtyFourLowercaseHexDigits() {
        assertTrue(rejection(FieldOffer.read(1, List.of(said("high", "field-high.glb", 22, "abc"))))
                .contains("SHA-256"));
        assertTrue(rejection(FieldOffer.read(1, List.of(said("high", "field-high.glb", 22, HIGH_DIGEST.toUpperCase()))))
                .contains("SHA-256"));
    }

    @Test
    public void aSizeIsNotNegative() {
        String said = rejection(FieldOffer.read(1, List.of(said("high", "field-high.glb", -1, HIGH_DIGEST))));

        assertTrue(said, said.contains("-1"));
    }

    @Test
    public void anOfferNamesEachResolutionOnce() {
        String said = rejection(FieldOffer.read(
                1,
                List.of(
                        said("high", "field-high.glb", 22, HIGH_DIGEST),
                        said("high", "field-high.glb", 22, HIGH_DIGEST))));

        assertTrue(said, said.contains("twice"));
    }

    @Test
    public void whatIsFetchedIsEveryModelOfferedThatIsNotKeptAsTheHostBuiltIt() {
        Map<FieldModel, String> kept = new EnumMap<>(FieldModel.class);
        kept.put(FieldModel.LOW, LOW_DIGEST);
        kept.put(FieldModel.MEDIUM, HIGH_DIGEST);

        List<FieldOffer.Offered> fetched = offerOfLowAndHigh().toFetch(kept);

        assertEquals(1, fetched.size());
        assertEquals(FieldModel.HIGH, fetched.get(0).model());
    }

    @Test
    public void aModelTheHostRebuiltIsFetchedAgain() {
        Map<FieldModel, String> kept = new EnumMap<>(FieldModel.class);
        kept.put(FieldModel.LOW, LOW_DIGEST);
        kept.put(FieldModel.HIGH, LOW_DIGEST);

        List<FieldOffer.Offered> fetched = offerOfLowAndHigh().toFetch(kept);

        assertEquals(1, fetched.size());
        assertEquals(FieldModel.HIGH, fetched.get(0).model());
    }
}
