package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.EnumSet;
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
}
