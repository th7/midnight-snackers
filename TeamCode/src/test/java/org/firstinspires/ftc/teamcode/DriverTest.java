package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/** The driver's gamepads drive the robot through the Driver super system, not through an op mode. */
public class DriverTest {
    private static final double DELTA = 0.0001;

    private final SimRobot sim = new SimRobot();
    private final Gamepad gamepad1 = new Gamepad();
    private final Gamepad gamepad2 = new Gamepad();
    private final Robot robot = new Robot(sim.hardware(), Alliance.BLUE, new FakeTelemetry(), gamepad1, gamepad2);

    public DriverTest() {
        robot.add(new Driver());
    }

    @Test
    public void theLeftStickDrivesStraight() {
        gamepad1.left_stick_y = -1;

        robot.loop();

        assertEquals(1, sim.leftFront.power, DELTA);
        assertEquals(1, sim.rightFront.power, DELTA);
        assertEquals(1, sim.leftBack.power, DELTA);
        assertEquals(1, sim.rightBack.power, DELTA);
    }

    @Test
    public void squareStartsALaunch() {
        gamepad1.square = true;

        robot.loop();

        assertFalse(robot.launcher.launchDone());
    }
}
