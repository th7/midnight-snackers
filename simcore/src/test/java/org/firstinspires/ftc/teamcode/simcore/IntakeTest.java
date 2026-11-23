package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class IntakeTest {
    private static final MotorType HD_HEX = MotorType.ULTRAPLANETARY_HD_HEX;

    @Test
    public void aMotorSpinningForwardTurnsAnIntakeAsBuiltInward() {
        assertEquals(Intake.Way.IN, Intake.way(2800, HD_HEX, false));
    }

    @Test
    public void aMotorSpinningBackwardTurnsAnIntakeAsBuiltOutward() {
        assertEquals(Intake.Way.OUT, Intake.way(-2800, HD_HEX, false));
    }

    @Test
    public void aReversedIntakeTurnsTheOtherWay() {
        assertEquals(Intake.Way.OUT, Intake.way(2800, HD_HEX, true));
        assertEquals(Intake.Way.IN, Intake.way(-2800, HD_HEX, true));
    }

    @Test
    public void anIntakeTurningUnderAHundredAndFortyTicksASecondIsStill() {
        assertEquals(Intake.Way.STILL, Intake.way(139, HD_HEX, false));
        assertEquals(Intake.Way.STILL, Intake.way(-139, HD_HEX, false));
        assertEquals(Intake.Way.STILL, Intake.way(0, HD_HEX, false));
        assertEquals(Intake.Way.IN, Intake.way(141, HD_HEX, false));
    }

    @Test
    public void turningInwardItTakesEveryPollenTouchingTheFrontUpToTwo() {
        Intake.Taken taken = Intake.take(Intake.Way.IN, Intake.Held.none(), 3);

        assertEquals(2, taken.taken());
        assertEquals(2, taken.held().count());
    }

    @Test
    public void turningInwardWithOneHeldItTakesOneMore() {
        Intake.Taken taken = Intake.take(Intake.Way.IN, Intake.Held.none().plus(1), 2);

        assertEquals(1, taken.taken());
        assertEquals(2, taken.held().count());
    }

    @Test
    public void turningInwardWithTwoHeldItTakesNone() {
        Intake.Held full = Intake.Held.none().plus(2);

        assertEquals(0, Intake.take(Intake.Way.IN, full, 1).taken());
    }

    @Test
    public void turningOutwardOrStillItTakesNone() {
        assertEquals(0, Intake.take(Intake.Way.OUT, Intake.Held.none(), 1).taken());
        assertEquals(0, Intake.take(Intake.Way.STILL, Intake.Held.none(), 1).taken());
    }

    @Test
    public void heldIsNeverMoreThanTwoNorLessThanNone() {
        assertEquals(2, Intake.Held.none().plus(5).count());
        assertEquals(0, Intake.Held.none().plus(2).less(3).count());
        assertEquals(1, Intake.Held.none().plus(2).less(1).count());
    }
}
