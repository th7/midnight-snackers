package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class VelocityLoopTest {
    private static final Seconds STEP = Valid.value(Seconds.of(0.005));
    private static final int ONE_SECOND = 200;

    @Test
    public void aMotorThatTurnsItsOwnEncoderSettlesAtTheTarget() {
        VelocityLoop loop = VelocityLoop.idle();
        Spin spin = Spin.still();
        for (int i = 0; i < 2 * ONE_SECOND; i++) {
            VelocityLoop.Stepped stepped = loop.toward(1500, spin.count(), STEP);
            loop = stepped.loop();
            spin = spin.next(MotorType.ULTRAPLANETARY_HD_HEX, stepped.power(), 12, STEP);
        }

        assertEquals(1500, spin.ticksPerSecond(), 20);
    }

    @Test
    public void aMotorWhoseEncoderReadsNothingRunsAwayToFullPower() {
        VelocityLoop loop = VelocityLoop.idle();
        Power power = Power.none();
        for (int i = 0; i < ONE_SECOND; i++) {
            VelocityLoop.Stepped stepped = loop.toward(300, Count.none(), STEP);
            loop = stepped.loop();
            power = stepped.power();
        }

        assertEquals(1, power.value(), 0);
    }

    @Test
    public void aMotorWhoseEncoderCountsAgainstItRunsAwayTheOtherWayToFullPower() {
        VelocityLoop loop = VelocityLoop.idle();
        Power power = Power.none();
        double counted = 0;
        for (int i = 0; i < ONE_SECOND; i++) {
            VelocityLoop.Stepped stepped = loop.toward(-300, new Count(0, counted), STEP);
            loop = stepped.loop();
            power = stepped.power();
            counted = -1000 * power.value();
        }

        assertEquals(-1, power.value(), 0);
    }

    @Test
    public void runningAwayWindsUpNoMoreThanFullPower() {
        VelocityLoop loop = VelocityLoop.idle();
        for (int i = 0; i < 10 * ONE_SECOND; i++) {
            loop = loop.toward(300, Count.none(), STEP).loop();
        }

        Power offTarget = loop.toward(0, new Count(0, 3000), STEP).power();

        assertTrue("it comes back as soon as it overshoots, not after ten seconds of unwinding", offTarget.value() < 0);
    }
}
