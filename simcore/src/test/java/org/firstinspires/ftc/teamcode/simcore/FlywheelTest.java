package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class FlywheelTest {
    @Test
    public void theFlywheelsMotorTurnsAtMostItsFreeSpeedInEncoderTicks() {
        assertEquals(Flywheel.MOTOR, 6000, Flywheel.MOTOR_RPM, 0);
        assertEquals(Flywheel.MOTOR, 28, Flywheel.MOTOR_TICKS_PER_TURN, 0);
        assertEquals(2800, Flywheel.TOP_TICKS_PER_SECOND, 0);
    }

    @Test
    public void aFlywheelTurnsAsCommandedUpToItsTopSpeedEitherWayAndNotAtAllForACommandThatIsNotANumber() {
        assertEquals(1050, Flywheel.turnsAt(1050), 0);
        assertEquals(-1050, Flywheel.turnsAt(-1050), 0);
        assertEquals(Flywheel.TOP_TICKS_PER_SECOND, Flywheel.turnsAt(10000), 0);
        assertEquals(-Flywheel.TOP_TICKS_PER_SECOND, Flywheel.turnsAt(Double.NEGATIVE_INFINITY), 0);
        assertEquals(0, Flywheel.turnsAt(Double.NaN), 0);
    }
}
