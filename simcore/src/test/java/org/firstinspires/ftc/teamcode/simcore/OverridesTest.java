package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.Test;

public class OverridesTest {
    private static final RobotDescription REGINALD = RobotDescription.reginalds();

    private static Overrides.Kept known(Overrides.Edit edit) {
        return new Overrides.Kept.Known(edit);
    }

    private static Overrides.Kept constant(Constant constant, double value) {
        return known(new Overrides.Edit.OfConstant(constant, value));
    }

    private static Overrides asked(RobotDescription defaults, Overrides.Kept... kept) {
        return Valid.value(Overrides.asked(defaults, List.of(kept)));
    }

    private static String refusal(Checked<?> checked) {
        return checked.fold(value -> "taken, not refused", rule -> rule);
    }

    @Test
    public void noOverridesIsTheDefaultsWithNothingEdited() {
        Overrides none = Overrides.none(REGINALD);

        assertEquals(REGINALD, none.description());
        assertEquals(List.of(), none.edits());
    }

    @Test
    public void anEditedConstantIsTheDescriptionsValueAndIsListedAsAnEdit() {
        Overrides moved = asked(REGINALD, constant(Constant.PARALLEL_DEAD_WHEEL_Y_IN, -1.5));

        assertEquals(-1.5, moved.description().constants().value(Constant.PARALLEL_DEAD_WHEEL_Y_IN), 0);
        assertEquals(REGINALD.hub(), moved.description().hub());
        assertEquals(List.of(new Overrides.Edit.OfConstant(Constant.PARALLEL_DEAD_WHEEL_Y_IN, -1.5)), moved.edits());
    }

    @Test
    public void anEditToTheDefaultValueIsNoEdit() {
        Overrides same = asked(REGINALD, constant(Constant.MASS_KG, 15));

        assertEquals(List.of(), same.edits());
        assertEquals(REGINALD, same.description());
    }

    @Test
    public void theDefaultsAreTheOnesGivenNotReginaldAsBuilt() {
        RobotDescription heavier = REGINALD.with(Valid.value(Constants.of(Map.of(Constant.MASS_KG, 18.0))));

        Overrides backToBuilt = asked(heavier, constant(Constant.MASS_KG, 15));
        Overrides atTheDefault = asked(heavier, constant(Constant.MASS_KG, 18));

        assertEquals(List.of(new Overrides.Edit.OfConstant(Constant.MASS_KG, 15)), backToBuilt.edits());
        assertEquals(List.of(), atTheDefault.edits());
        assertEquals(18, atTheDefault.description().constants().value(Constant.MASS_KG), 0);
    }

    @Test
    public void aFlattestBatteryAboveTheFreshestIsRefusedNamingBoth() {
        String rule = refusal(Overrides.asked(REGINALD, List.of(constant(Constant.FLATTEST_VOLTS, 14))));

        assertEquals("Flattest battery may not be more than freshest battery, not 14.0 V against 13.8 V", rule);
    }

    @Test
    public void aValuePastItsRangeIsRefusedNamingTheRange() {
        String rule = refusal(Overrides.asked(REGINALD, List.of(constant(Constant.MASS_KG, 40))));

        assertEquals("Mass is from 1.0 to 30.0 kg, not 40.0", rule);
    }

    @Test
    public void aNameTheDescriptionDoesNotHaveIsRefusedNamingIt() {
        String rule = refusal(Overrides.asked(REGINALD, List.of(new Overrides.Kept.Gone("launch_throw"))));

        assertEquals("the robot description has no value named 'launch_throw'", rule);
    }

    @Test
    public void anUnreadableValueIsRefusedSayingWhy() {
        String rule = refusal(Overrides.asked(
                REGINALD, List.of(new Overrides.Kept.Unreadable("mass_kg", "Mass is a number, not \"heavy\""))));

        assertEquals("Mass is a number, not \"heavy\"", rule);
    }

    @Test
    public void theHubsFacesAreEditedAndCheckedTogether() {
        Overrides turned = asked(REGINALD, known(new Overrides.Edit.OfHub(Overrides.HubFace.LOGO, Facing.LEFT)));
        String rule = refusal(Overrides.asked(
                REGINALD, List.of(known(new Overrides.Edit.OfHub(Overrides.HubFace.LOGO, Facing.DOWN)))));

        assertEquals(
                Valid.value(HubMounting.of(Facing.LEFT, Facing.UP)),
                turned.description().hub());
        assertEquals(List.of(new Overrides.Edit.OfHub(Overrides.HubFace.LOGO, Facing.LEFT)), turned.edits());
        assertTrue(rule, rule.startsWith("the hub's logo and its USB ports face along different axes"));
    }

    @Test
    public void aConfiguredDeviceIsRenamedAndAMotorRewiredByItsDefaultName() {
        Overrides edited = asked(
                REGINALD,
                known(new Overrides.Edit.Renamed("ramp", "lift")),
                known(new Overrides.Edit.Wired("rightBack", EncoderReads.NOTHING)));

        List<Configuration.Device> devices =
                edited.description().configuration().devices();
        assertTrue(devices.contains(new Configuration.Servo("lift")));
        assertTrue(devices.contains(
                new Configuration.Motor("rightBack", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.NOTHING)));
        assertEquals(
                List.of(
                        new Overrides.Edit.Wired("rightBack", EncoderReads.NOTHING),
                        new Overrides.Edit.Renamed("ramp", "lift")),
                edited.edits());
    }

    @Test
    public void aMotorsTypeIsEditedByItsDefaultName() {
        Overrides typed = asked(REGINALD, known(new Overrides.Edit.Typed("intake", MotorType.ULTRAPLANETARY_HD_HEX)));

        assertEquals(List.of(), typed.edits());
    }

    @Test
    public void anEditOfADeviceThatIsNotConfiguredIsRefusedNamingIt() {
        String rule =
                refusal(Overrides.asked(REGINALD, List.of(known(new Overrides.Edit.Renamed("periscope", "eye")))));

        assertEquals("no device is configured as 'periscope'", rule);
    }

    @Test
    public void aMotorsTypeOrEncoderOnADeviceThatIsNoMotorIsRefused() {
        String rule = refusal(
                Overrides.asked(REGINALD, List.of(known(new Overrides.Edit.Wired("imu", EncoderReads.OWN_MOTOR)))));

        assertEquals("'imu' is configured as imu, not as a motor, so it has no encoder port", rule);
    }

    @Test
    public void aConfigurationTheCoreRefusesIsRefusedWhole() {
        String rule = refusal(Overrides.asked(
                REGINALD, List.of(known(new Overrides.Edit.Wired("intake", EncoderReads.PARALLEL_DEAD_WHEEL)))));

        assertEquals("the parallel dead wheel is read on one port at most, not on both 'rightBack' and 'intake'", rule);
    }

    @Test
    public void aRecalledValuePastItsRangeGoesBackToItsDefaultAndSaysSo() {
        Overrides.Recalled recalled = Overrides.recalled(
                REGINALD, List.of(constant(Constant.MASS_KG, 40), constant(Constant.TRACK_WIDTH_IN, 14)));

        assertEquals(
                List.of(new Overrides.Edit.OfConstant(Constant.TRACK_WIDTH_IN, 14)),
                recalled.overrides().edits());
        assertEquals(15, recalled.overrides().description().constants().value(Constant.MASS_KG), 0);
        assertEquals(
                List.of(new Overrides.Notice(
                        "Mass", Overrides.Fate.BACK_TO_DEFAULT, "Mass is from 1.0 to 30.0 kg, not 40.0")),
                recalled.notices());
    }

    @Test
    public void aRecalledNameTheDescriptionNoLongerHasIsDroppedOnTheNextSaveAndSaysSo() {
        Overrides.Recalled recalled = Overrides.recalled(
                REGINALD, List.of(new Overrides.Kept.Gone("launch_throw"), constant(Constant.MASS_KG, 16)));

        assertEquals(
                List.of(new Overrides.Edit.OfConstant(Constant.MASS_KG, 16)),
                recalled.overrides().edits());
        assertEquals(
                List.of(new Overrides.Notice(
                        "launch_throw",
                        Overrides.Fate.DROPPED_ON_NEXT_SAVE,
                        "the robot description has no such value now")),
                recalled.notices());
    }

    @Test
    public void aRecalledEditOfADeviceNoLongerConfiguredIsDroppedOnTheNextSave() {
        Overrides.Recalled recalled =
                Overrides.recalled(REGINALD, List.of(known(new Overrides.Edit.Renamed("turntable", "spinner2"))));

        assertEquals(List.of(), recalled.overrides().edits());
        assertEquals(
                List.of(new Overrides.Notice(
                        "turntable's name",
                        Overrides.Fate.DROPPED_ON_NEXT_SAVE,
                        "no device is configured as 'turntable'")),
                recalled.notices());
    }

    @Test
    public void aRecalledUnreadableValueGoesBackToItsDefault() {
        Overrides.Recalled recalled = Overrides.recalled(
                REGINALD, List.of(new Overrides.Kept.Unreadable("hub.logo", "a face is UP, DOWN, not 'SIDEWAYS'")));

        assertEquals(
                List.of(new Overrides.Notice(
                        "hub.logo", Overrides.Fate.BACK_TO_DEFAULT, "a face is UP, DOWN, not 'SIDEWAYS'")),
                recalled.notices());
    }

    @Test
    public void aRecalledSetTheCoreRefusesWholeGoesBackToTheDefaultsAndSaysWhy() {
        Overrides.Recalled recalled = Overrides.recalled(REGINALD, List.of(constant(Constant.FLATTEST_VOLTS, 14)));

        assertEquals(REGINALD, recalled.overrides().description());
        assertEquals(
                List.of(new Overrides.Notice(
                        "every edit",
                        Overrides.Fate.BACK_TO_DEFAULT,
                        "Flattest battery may not be more than freshest battery, not 14.0 V against 13.8 V")),
                recalled.notices());
    }

    @Test
    public void aNoticeSaysWhatBecameOfTheValueAndWhy() {
        assertEquals(
                "Mass went back to its default: Mass is from 1.0 to 30.0 kg, not 40.0.",
                new Overrides.Notice("Mass", Overrides.Fate.BACK_TO_DEFAULT, "Mass is from 1.0 to 30.0 kg, not 40.0")
                        .said());
        assertEquals(
                "launch_throw is dropped on the next save: the robot description has no such value now.",
                new Overrides.Notice(
                                "launch_throw",
                                Overrides.Fate.DROPPED_ON_NEXT_SAVE,
                                "the robot description has no such value now")
                        .said());
    }
}
