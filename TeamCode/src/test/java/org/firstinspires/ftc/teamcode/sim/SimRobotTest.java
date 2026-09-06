package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.junit.Test;

public class SimRobotTest {
    private static final double DELTA = 0.001;

    private final SimRobot sim = new SimRobot();

    @Test
    public void withTheRobotsMotorDirectionsForwardPowerDrivesStraightAheadNearMaxSpeed() {
        robotDrive();
        setPowers(1, 1, 1, 1);

        sim.step(0.5);

        Pose2d pose = sim.pose();
        assertTrue("x=" + pose.position.x, pose.position.x > 20 && pose.position.x < 25);
        assertEquals(0, pose.position.y, DELTA);
        assertEquals(0, pose.heading.toDouble(), DELTA);
    }

    @Test
    public void withoutTheRobotsMotorDirectionsTheMirroredBackWheelsFightTheFrontOnes() {
        setPowers(1, 1, 1, 1);

        sim.step(0.5);

        Pose2d pose = sim.pose();
        assertEquals(0, pose.position.x, DELTA);
        assertEquals(0, pose.position.y, DELTA);
        assertEquals(0, pose.heading.toDouble(), DELTA);
    }

    @Test
    public void powerBelowStaticFrictionDoesNotMove() {
        robotDrive();
        setPowers(0.05, 0.05, 0.05, 0.05);

        sim.step(0.5);

        assertEquals(0, sim.pose().position.x, DELTA);
    }

    @Test
    public void oppositeSidesSpinTheRobotInPlace() {
        robotDrive();
        setPowers(-1, 1, -1, 1);

        sim.step(0.2);

        Pose2d pose = sim.pose();
        assertEquals(0, pose.position.x, DELTA);
        assertEquals(0, pose.position.y, DELTA);
        assertTrue("heading=" + pose.heading.toDouble(), pose.heading.toDouble() > 0.5);
    }

    @Test
    public void theRobotsOwnLocalizerTracksTheTruePose() {
        MecanumDrive drive = robotDrive();
        drive.localizer.update();

        // A gentle arc: forward with a little more on the right side.
        setPowers(0.6, 0.9, 0.6, 0.9);
        for (int i = 0; i < 40; i++) {
            sim.step(0.025);
            drive.localizer.update();
        }

        Pose2d truePose = sim.pose();
        Pose2d estimated = drive.localizer.getPose();
        assertTrue("the arc should have turned the robot; heading=" + truePose.heading.toDouble(),
                Math.abs(truePose.heading.toDouble()) > 0.2);
        assertEquals(truePose.position.x, estimated.position.x, 0.5);
        assertEquals(truePose.position.y, estimated.position.y, 0.5);
        assertEquals(truePose.heading.toDouble(), estimated.heading.toDouble(), 0.02);
    }

    /**
     * The robot's own drive on the simulated motors, which applies the motor directions the robot uses.
     */
    private MecanumDrive robotDrive() {
        return new MecanumDrive(
                sim.leftFront, sim.leftBack, sim.rightBack, sim.rightFront,
                () -> sim.imu, sim.voltageSensor, new Pose2d(0, 0, 0));
    }

    private void setPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        sim.leftFront.setPower(leftFront);
        sim.rightFront.setPower(rightFront);
        sim.leftBack.setPower(leftBack);
        sim.rightBack.setPower(rightBack);
    }
}
