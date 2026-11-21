package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.EnumSet;
import java.util.List;
import org.junit.Test;

public class DrawnFieldTest {
    @Test
    public void withNothingKeptTheCommittedLowFieldIsDrawnAndSaidToBe() {
        DrawnField drawn = DrawnField.of(EnumSet.noneOf(FieldModel.class));

        assertEquals(FieldModel.LOW, drawn.model());
        assertTrue(drawn.said(), drawn.said().contains("committed with the project"));
    }

    @Test
    public void theMostDetailedKeptModelIsDrawnAndSaidToBeTheHosts() {
        DrawnField drawn = DrawnField.of(EnumSet.of(FieldModel.LOW, FieldModel.MEDIUM));

        assertEquals(FieldModel.MEDIUM, drawn.model());
        assertTrue(drawn.said(), drawn.said().contains("medium"));
        assertTrue(drawn.said(), drawn.said().contains("the host built"));
    }

    @Test
    public void highIsDrawnOverEveryOtherKept() {
        assertEquals(
                FieldModel.HIGH,
                DrawnField.of(EnumSet.of(FieldModel.HIGH, FieldModel.LOW)).model());
    }

    @Test
    public void whatCanBeDrawnIsEveryKeptModelAndTheCommittedLowOneWhereNoLowIsKept() {
        List<DrawnField> drawable = DrawnField.drawable(EnumSet.of(FieldModel.HIGH));

        assertEquals(List.of(new DrawnField.Committed(), new DrawnField.Kept(FieldModel.HIGH)), drawable);
    }

    @Test
    public void aKeptLowModelIsDrawnInPlaceOfTheCommittedOne() {
        assertEquals(List.of(new DrawnField.Kept(FieldModel.LOW)), DrawnField.drawable(EnumSet.of(FieldModel.LOW)));
    }
}
