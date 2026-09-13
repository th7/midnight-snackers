package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Action;
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

    private void assertPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        assertEquals("leftFront", leftFront, sim.leftFront.power, DELTA);
        assertEquals("rightFront", rightFront, sim.rightFront.power, DELTA);
        assertEquals("leftBack", leftBack, sim.leftBack.power, DELTA);
        assertEquals("rightBack", rightBack, sim.rightBack.power, DELTA);
    }

    /** Puts the robot where its launch pose is, so aiming has nothing left to do. */
    private void parkAtTheLaunchPose() {
        robot.nav.setPose(robot.nav.launchPose());
        robot.loop();
    }

    @Test
    public void theLeftStickDrivesStraight() {
        gamepad1.left_stick_y = -1;

        robot.loop();

        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void theRightStickTurns() {
        gamepad1.right_stick_x = 1;

        robot.loop();

        assertPowers(1, -1, 1, -1);
    }

    @Test
    public void squareStartsALaunch() {
        gamepad1.square = true;

        robot.loop();

        assertFalse(robot.launcher.launchDone());
    }

    @Test
    public void theLeftBumperAimsAtTheLaunchPoseAndTheSticksNudgeStrafeAndTurn() {
        parkAtTheLaunchPose();
        gamepad1.left_bumper = true;
        robot.loop();
        assertPowers(0, 0, 0, 0);

        gamepad1.left_stick_x = 1;
        robot.loop();
        assertPowers(1, -1, -1, 1);

        gamepad1.left_stick_x = 0;
        gamepad1.left_stick_y = -1; // straight is the drive's under the left bumper, not the driver's
        robot.loop();
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void theRightBumperLetsTheDriverTranslateWhileTheDriveKeepsTheHeading() {
        parkAtTheLaunchPose();
        gamepad1.right_bumper = true;
        gamepad1.left_stick_y = -1;
        robot.loop();
        assertPowers(1, 1, 1, 1);

        gamepad1.left_stick_y = 0;
        gamepad1.right_stick_x = 1;
        robot.loop();
        assertPowers(1, -1, 1, -1);
    }

    @Test
    public void aDeflectedStickTakesOverFromAnActionTheSameLoop() {
        robot.drive.follow(packet -> true);
        gamepad1.left_stick_y = -1;

        robot.loop();

        assertTrue(robot.drive.done());
        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void aRestingStickLeavesAnActionAlone() {
        Action forever = packet -> true;
        robot.drive.follow(forever);

        robot.loop();

        assertFalse(robot.drive.done());
    }
}
