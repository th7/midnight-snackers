package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class EncoderReadsTest {
    private static final Checked<Count> OWN = Checked.ok(new Count(10, 20));
    private static final Checked<Count> UNKNOWN = Checked.rejected("nobody knows how far this motor turns");
    private static final DeadWheels.Counts DEAD = new DeadWheels.Counts(new Count(30, 40), new Count(50, 60));

    @Test
    public void aPortCountsWhatIsWiredToIt() {
        assertEquals(new Count(10, 20), Valid.value(EncoderReads.OWN_MOTOR.count(OWN, DEAD)));
        assertEquals(new Count(30, 40), Valid.value(EncoderReads.PARALLEL_DEAD_WHEEL.count(OWN, DEAD)));
        assertEquals(new Count(50, 60), Valid.value(EncoderReads.PERPENDICULAR_DEAD_WHEEL.count(OWN, DEAD)));
        assertEquals(new Count(0, 0), Valid.value(EncoderReads.NOTHING.count(OWN, DEAD)));
    }

    @Test
    public void aMotorWhoseTurningIsNotKnownIsRefusedOnlyWhereItsOwnPortReadsIt() {
        Checked<Count> refused = EncoderReads.OWN_MOTOR.count(UNKNOWN, DEAD);

        assertTrue(
                refused.fold(value -> "accepted " + value, reason -> reason),
                refused.fold(value -> false, reason -> reason.contains("nobody knows")));
        assertEquals(new Count(30, 40), Valid.value(EncoderReads.PARALLEL_DEAD_WHEEL.count(UNKNOWN, DEAD)));
        assertEquals(new Count(0, 0), Valid.value(EncoderReads.NOTHING.count(UNKNOWN, DEAD)));
    }
}
