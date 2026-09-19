package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

public class DriverTest {
    private static final double DELTA = 0.0001;

    private final SimDevices devices = new SimDevices();
    private final Gamepad gamepad1 = new Gamepad();
    private final Gamepad gamepad2 = new Gamepad();
    private final Robot robot = new Robot(devices.hardware(), Alliance.BLUE, new FakeTelemetry(), gamepad1, gamepad2);

    private final Driver driver =
            new Driver(robot.drive, robot.launcher, robot.brain, robot.nav, robot.turntable, gamepad1, gamepad2);

    private void tick() {
        robot.loop();
        driver.loop();
    }

    private void assertPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        assertEquals("leftFront", leftFront, devices.leftFront.power, DELTA);
        assertEquals("rightFront", rightFront, devices.rightFront.power, DELTA);
        assertEquals("leftBack", leftBack, devices.leftBack.power, DELTA);
        assertEquals("rightBack", rightBack, devices.rightBack.power, DELTA);
    }

    private void parkAtTheLaunchPose() {
        robot.nav.placeAt(robot.nav.launchPose().get());
        tick();
    }

    @Test
    public void theLeftStickDrivesStraight() {
        gamepad1.left_stick_y = -1;

        tick();

        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void theRightStickTurns() {
        gamepad1.right_stick_x = 1;

        tick();

        assertPowers(1, -1, 1, -1);
    }

    @Test
    public void squareStartsALaunch() {
        gamepad1.square = true;

        tick();

        assertFalse(robot.launcher.launchDone());
    }

    @Test
    public void theLeftBumperAimsAtTheLaunchPoseAndTheSticksNudgeStrafeAndTurn() {
        parkAtTheLaunchPose();
        gamepad1.left_bumper = true;
        tick();
        assertPowers(0, 0, 0, 0);

        gamepad1.left_stick_x = 1;
        tick();
        assertPowers(1, -1, -1, 1);

        gamepad1.left_stick_x = 0;
        gamepad1.left_stick_y = -1;
        tick();
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void theRightBumperLetsTheDriverTranslateWhileTheDriveKeepsTheHeading() {
        parkAtTheLaunchPose();
        gamepad1.right_bumper = true;
        gamepad1.left_stick_y = -1;
        tick();
        assertPowers(1, 1, 1, 1);

        gamepad1.left_stick_y = 0;
        gamepad1.right_stick_x = 1;
        tick();
        assertPowers(1, -1, 1, -1);
    }

    @Test
    public void playingForNoAllianceThereIsNoGoalToAimAtSoTheBumpersJustDrive() {
        Robot relative = new Robot(devices.hardware(), Alliance.RELATIVE, new FakeTelemetry(), gamepad1, gamepad2);
        Driver relativeDriver = new Driver(
                relative.drive,
                relative.launcher,
                relative.brain,
                relative.nav,
                relative.turntable,
                gamepad1,
                gamepad2);
        gamepad1.left_bumper = true;
        gamepad1.left_stick_y = -1;

        relative.loop();
        relativeDriver.loop();

        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void aDeflectedStickTakesOverFromAnActionTheSameLoop() {
        robot.drive.follow(packet -> true);
        gamepad1.left_stick_y = -1;

        tick();

        assertTrue(robot.drive.done());
        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void aRestingStickLeavesAnActionAlone() {
        Action forever = packet -> true;
        robot.drive.follow(forever);

        tick();

        assertFalse(robot.drive.done());
    }
}
