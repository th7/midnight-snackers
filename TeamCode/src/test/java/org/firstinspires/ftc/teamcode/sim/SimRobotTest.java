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

    @Test
    public void theWallStopsTheRobotWhereItsFrontEdgeMeetsIt() {
        robotDrive();
        double wall = SimRobot.FIELD_SIZE_IN / 2;
        double halfRobot = SimRobot.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(wall - halfRobot - 10, 0, 0));
        setPowers(1, 1, 1, 1);

        sim.step(1.0);

        Pose2d pose = sim.pose();
        assertEquals(wall - halfRobot, pose.position.x, DELTA);
        assertEquals(0, pose.position.y, DELTA);
        assertEquals(0, pose.heading.toDouble(), DELTA);
    }

    @Test
    public void everyWallStopsTheRobot() {
        robotDrive();
        double edge = SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2;

        sim.setPose(new Pose2d(-edge + 5, 0, 0));
        setPowers(-1, -1, -1, -1);
        sim.step(1.0);
        assertEquals("back wall", -edge, sim.pose().position.x, DELTA);

        sim.setPose(new Pose2d(0, edge - 5, 0));
        setPowers(-1, 1, 1, -1); // strafe left
        sim.step(1.0);
        assertEquals("left wall", edge, sim.pose().position.y, DELTA);

        sim.setPose(new Pose2d(0, -edge + 5, 0));
        setPowers(1, -1, -1, 1); // strafe right
        sim.step(1.0);
        assertEquals("right wall", -edge, sim.pose().position.y, DELTA);
    }

    @Test
    public void aTurnedRobotStopsWhereItsCornerMeetsTheWall() {
        robotDrive();
        double wall = SimRobot.FIELD_SIZE_IN / 2;
        double cornerReach = SimRobot.ROBOT_SIZE_IN / 2 * Math.sqrt(2);
        sim.setPose(new Pose2d(wall - cornerReach - 10, 0, Math.PI / 4));
        setPowers(1, 1, 1, 1);

        sim.step(1.0);

        assertEquals(wall - cornerReach, sim.pose().position.x, DELTA);
        assertEquals(Math.PI / 4, sim.pose().heading.toDouble(), DELTA);
    }

    @Test
    public void drivingDiagonallyIntoTheWallSlidesAlongIt() {
        robotDrive();
        double edge = SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(edge, 0, 0));
        setPowers(0, 1, 1, 0); // forward and left

        sim.step(0.5);

        Pose2d pose = sim.pose();
        assertEquals(edge, pose.position.x, DELTA);
        assertTrue("y=" + pose.position.y, pose.position.y > 5);
        assertEquals(0, pose.heading.toDouble(), DELTA);
    }

    @Test
    public void againstTheWallTheDeadWheelsReadTheRobotStandingStillNotTheWheelsSpinning() {
        MecanumDrive drive = robotDrive();
        double edge = SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(edge - 10, 0, 0));
        drive.localizer.setPose(sim.pose());
        drive.localizer.update();
        setPowers(1, 1, 1, 1);

        for (int i = 0; i < 40; i++) {
            sim.step(0.025);
            drive.localizer.update();
        }

        Pose2d estimated = drive.localizer.getPose();
        assertEquals(edge, sim.pose().position.x, DELTA);
        assertEquals(edge, estimated.position.x, 0.5);
        assertEquals(0, estimated.position.y, 0.5);
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
