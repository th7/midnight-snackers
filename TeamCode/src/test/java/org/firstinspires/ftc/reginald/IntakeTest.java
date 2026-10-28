package org.firstinspires.ftc.reginald;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

public class IntakeTest {
    private final SimDevices devices = new SimDevices();
    private final FakeDcMotorEx motor = devices.intake;
    private final Intake intake = new Intake(motor);

    @Test
    public void startsOnWithoutWaitingForTheFirstLoop() {
        assertTrue(intake.isOn());
        assertEquals(Intake.RUNNING_POWER, motor.power, 0.0001);
    }

    @Test
    public void offStopsTheMotor() {
        intake.off();
        intake.loop();

        assertFalse(intake.isOn());
        assertEquals(0, motor.power, 0.0001);
    }

    @Test
    public void onStartsTheMotorAgain() {
        intake.off();
        intake.loop();

        intake.on();
        intake.loop();

        assertTrue(intake.isOn());
        assertEquals(Intake.RUNNING_POWER, motor.power, 0.0001);
    }

    @Test
    public void staysAsItWasLeftFromLoopToLoop() {
        intake.off();

        intake.loop();
        intake.loop();

        assertFalse(intake.isOn());
        assertEquals(0, motor.power, 0.0001);
    }
}
