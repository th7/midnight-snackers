package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class HandoverTest {
    private static final double SIXTY_RPM = 60;
    private static final Spinner.Turning CLOCKWISE_FULL = new Spinner.Turning(Spinner.Way.CLOCKWISE, 1);
    private static final Spinner.Turning COUNTERCLOCKWISE_FULL = new Spinner.Turning(Spinner.Way.COUNTERCLOCKWISE, 1);

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    @Test
    public void atSixtyRpmTheSpinnerTurnsHalfATurnInHalfASecond() {
        Handover.Stepped stepped = Handover.step(0, CLOCKWISE_FULL, SIXTY_RPM, true, 0, seconds(0.25));

        assertEquals(0.25, stepped.turns(), 1e-9);
    }

    @Test
    public void clockwiseWithTheRampUpItHandsOnePollenEachHalfTurn() {
        Handover.Stepped short_ = Handover.step(0.4, CLOCKWISE_FULL, SIXTY_RPM, true, 2, seconds(0.05));
        Handover.Stepped past = Handover.step(0.4, CLOCKWISE_FULL, SIXTY_RPM, true, 2, seconds(0.15));
        Handover.Stepped pastTwo = Handover.step(0.4, CLOCKWISE_FULL, SIXTY_RPM, true, 2, seconds(0.65));

        assertEquals(0, short_.handed());
        assertEquals(1, past.handed());
        assertEquals(2, pastTwo.handed());
        assertEquals(Handover.Why.HANDING, past.why());
    }

    @Test
    public void itHandsNoMoreThanItHolds() {
        Handover.Stepped stepped = Handover.step(0.4, CLOCKWISE_FULL, SIXTY_RPM, true, 1, seconds(0.65));

        assertEquals(1, stepped.handed());
    }

    @Test
    public void holdingNothingItHandsNothingAndSaysSo() {
        Handover.Stepped stepped = Handover.step(0.4, CLOCKWISE_FULL, SIXTY_RPM, true, 0, seconds(0.15));

        assertEquals(0, stepped.handed());
        assertEquals(Handover.Why.NOTHING_HELD, stepped.why());
    }

    @Test
    public void turningCounterclockwiseOrStillItHandsNothingAndSaysSo() {
        Handover.Stepped back = Handover.step(0.6, COUNTERCLOCKWISE_FULL, SIXTY_RPM, true, 2, seconds(0.15));
        Handover.Stepped still = Handover.step(0.4, Spinner.Turning.still(), SIXTY_RPM, true, 2, seconds(0.15));

        assertEquals(0.45, back.turns(), 1e-9);
        assertEquals(0, back.handed());
        assertEquals(Handover.Why.SPINNER_NOT_CLOCKWISE, back.why());
        assertEquals(0, still.handed());
        assertEquals(Handover.Why.SPINNER_NOT_CLOCKWISE, still.why());
    }

    @Test
    public void withTheRampDownItHandsNothingAndSaysSo() {
        Handover.Stepped stepped = Handover.step(0.4, CLOCKWISE_FULL, SIXTY_RPM, false, 2, seconds(0.15));

        assertEquals(0, stepped.handed());
        assertEquals(Handover.Why.RAMP_DOWN, stepped.why());
    }

    @Test
    public void anArmTurnedBackPastTheRampHandsAgainWhenItComesForward() {
        Handover.Stepped back = Handover.step(0.55, COUNTERCLOCKWISE_FULL, SIXTY_RPM, true, 2, seconds(0.1));
        Handover.Stepped forward = Handover.step(back.turns(), CLOCKWISE_FULL, SIXTY_RPM, true, 2, seconds(0.1));

        assertEquals(1, forward.handed());
    }
}
