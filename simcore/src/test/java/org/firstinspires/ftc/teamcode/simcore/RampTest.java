package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class RampTest {
    private static final double SLEW = 0.168;
    private static final double UP = 35;

    private static ServoSignal at(double position) {
        return Valid.value(ServoSignal.of(true, position, 600, 2400));
    }

    private static Ramp after(Ramp ramp, ServoSignal signal, double seconds) {
        Ramp now = ramp;
        for (int i = 0; i < seconds / 0.001; i++) {
            now = now.next(signal, SLEW, Valid.value(Seconds.of(0.001)));
        }
        return now;
    }

    @Test
    public void positionOneThroughTheSdksRangeStandsTwoHundredFortyThreeDegreesAboveStowed() {
        assertEquals(243, after(Ramp.stowed(), at(1), 1).degrees(), 1e-9);
    }

    @Test
    public void itSlewsSixtyDegreesInPointOneSixEightSeconds() {
        assertEquals(60, after(Ramp.stowed(), at(1), 0.168).degrees(), 0.5);
    }

    @Test
    public void positionPointOneFiveIsUpAfterAboutATenthOfASecond() {
        Ramp early = after(Ramp.stowed(), at(0.15), 0.09);
        Ramp later = after(Ramp.stowed(), at(0.15), 0.11);

        assertFalse("still rising at " + early.degrees(), early.up(UP));
        assertTrue("up at " + later.degrees(), later.up(UP));
        assertEquals(36.45, later.degrees(), 1e-9);
    }

    @Test
    public void positionPointOneIsDown() {
        Ramp settled = after(Ramp.stowed(), at(0.1), 1);

        assertEquals(24.3, settled.degrees(), 1e-9);
        assertFalse(settled.up(UP));
    }

    @Test
    public void withNoPulseItStaysWhereItIs() {
        Ramp raised = after(Ramp.stowed(), at(0.2), 1);

        assertEquals(raised, after(raised, ServoSignal.none(), 1));
    }

    @Test
    public void itSlewsBackDownTowardALowerPosition() {
        Ramp raised = after(Ramp.stowed(), at(1), 1);

        assertEquals(243 - 60, after(raised, at(0), 0.168).degrees(), 0.5);
    }
}
