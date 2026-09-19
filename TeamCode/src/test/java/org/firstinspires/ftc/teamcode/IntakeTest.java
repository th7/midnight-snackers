package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

public class IntakeTest {
    private final SimDevices devices = new SimDevices();
    private final FakeDcMotorEx motor = devices.intake;
    private final Robot robot = new Robot(devices.hardware(), Alliance.RELATIVE, new FakeTelemetry());

    @Test
    public void startsOnWithoutWaitingForTheFirstLoop() {
        assertTrue(robot.intake.isOn());
        assertEquals(Intake.RUNNING_POWER, motor.power, 0.0001);
    }

    @Test
    public void offStopsTheMotor() {
        robot.intake.off();
        robot.intake.loop();

        assertFalse(robot.intake.isOn());
        assertEquals(0, motor.power, 0.0001);
    }

    @Test
    public void onStartsTheMotorAgain() {
        robot.intake.off();
        robot.intake.loop();

        robot.intake.on();
        robot.intake.loop();

        assertTrue(robot.intake.isOn());
        assertEquals(Intake.RUNNING_POWER, motor.power, 0.0001);
    }

    @Test
    public void staysAsItWasLeftFromLoopToLoop() {
        robot.intake.off();

        robot.intake.loop();
        robot.intake.loop();

        assertFalse(robot.intake.isOn());
        assertEquals(0, motor.power, 0.0001);
    }
}
