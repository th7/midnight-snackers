package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.junit.Test;

public class TurntableTest {
    private static final double DELTA = 0.0001;

    private final FakeDcMotorEx motor = new FakeDcMotorEx();
    private final Turntable turntable = new Turntable(motor, new ElapsedTime(), new FakeTelemetry());

    @Test
    public void drivesTowardTheTargetPosition() {
        turntable.setTurnTablePosition(Math.PI / 2);

        turntable.loop();

        assertTrue(motor.power > 0);
    }

    @Test
    public void stopsInsideTheDeadband() {
        turntable.setTurnTablePosition(0);
        motor.currentPosition = 3;

        turntable.loop();

        assertEquals(0, motor.power, DELTA);
    }

    @Test
    public void weakCommandsAreRaisedToTheMinimumPower() {
        turntable.setTurnTablePosition(0);
        motor.currentPosition = -8;

        turntable.loop();

        assertEquals(0.1, motor.power, DELTA);
    }

    @Test
    public void wrapsTargetsPastHalfARevolutionToTurnTheShortWay() {
        turntable.setTurnTablePosition(3 * Math.PI / 2);

        turntable.loop();

        assertTrue(motor.power < 0);
    }

    @Test
    public void reportsTheOffsetInRadians() {
        motor.currentPosition = 1700;

        assertEquals(2 * Math.PI, turntable.getTurnTableOffsetRadians(), DELTA);
    }
}
