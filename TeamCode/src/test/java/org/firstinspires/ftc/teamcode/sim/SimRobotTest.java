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

    /** A robot placed beyond a wall is placed against it instead: the walls hold whatever pose it is given. */
    @Test
    public void aRobotPlacedOutsideTheWallsIsPlacedAgainstThem() {
        double edge = SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2;

        sim.setPose(new Pose2d(1000, -1000, 0));

        assertEquals(edge, sim.pose().position.x, DELTA);
        assertEquals(-edge, sim.pose().position.y, DELTA);
        assertEquals(0, sim.pose().heading.toDouble(), DELTA);

        sim.setPose(new Pose2d(1000, 0, Math.PI / 4));
        assertEquals("a turned robot reaches the wall with its corner", SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2 * Math.sqrt(2),
                sim.pose().position.x, DELTA);
        assertEquals(Math.PI / 4, sim.pose().heading.toDouble(), DELTA);

        Pose2d inside = new Pose2d(12, -7, 1);
        sim.setPose(inside);
        assertEquals("a pose inside the walls is placed as it is", inside, sim.pose());
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
     * The flowers stand against the walls. Driving straight at one stops the robot where its front
     * edge meets the flower's inward face, as the field model places it.
     */
    @Test
    public void aFlowerStopsTheRobotWhereItsFrontEdgeMeetsIt() {
        robotDrive();
        SimField.Obstacle flower = SimRobot.FIELD.obstacle("Flower Assembly <4>");
        double face = maxY(flower.footprint);
        double x = (minX(flower.footprint) + maxX(flower.footprint)) / 2;
        double halfRobot = SimRobot.ROBOT_SIZE_IN / 2;
        // Facing the right wall (-y), ten inches short of the flower.
        sim.setPose(new Pose2d(x, face + halfRobot + 10, -Math.PI / 2));
        setPowers(1, 1, 1, 1);

        sim.step(1.0);

        Pose2d pose = sim.pose();
        assertEquals(face + halfRobot, pose.position.y, DELTA);
        assertEquals(x, pose.position.x, DELTA);
        assertEquals(-Math.PI / 2, pose.heading.toDouble(), DELTA);
    }

    @Test
    public void drivingDiagonallyIntoAFlowerSlidesAlongIt() {
        robotDrive();
        SimField.Obstacle flower = SimRobot.FIELD.obstacle("Flower Assembly <4>");
        double face = maxY(flower.footprint);
        double x = (minX(flower.footprint) + maxX(flower.footprint)) / 2;
        double halfRobot = SimRobot.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(x, face + halfRobot, -Math.PI / 2));
        setPowers(0, 1, 1, 0); // forward and left, which facing -y is toward +x

        sim.step(0.1);

        Pose2d pose = sim.pose();
        assertEquals(face + halfRobot, pose.position.y, DELTA);
        assertTrue("x=" + pose.position.x, pose.position.x > x + 2);
        assertEquals(-Math.PI / 2, pose.heading.toDouble(), DELTA);
    }

    /** The frame in the middle of the field is driven through, between its legs. */
    @Test
    public void theRobotDrivesUnderTheHivesBetweenTheFramesLegs() {
        robotDrive();
        sim.setPose(new Pose2d(-14, -8, 0));
        setPowers(1, 1, 1, 1);

        sim.step(0.3);

        Pose2d pose = sim.pose();
        assertTrue("x=" + pose.position.x, pose.position.x > -14 + 10);
        assertEquals(-8, pose.position.y, DELTA);
    }

    /** The frame's feet and legs stand in the way; the nearest part in the robot's path stops it. */
    @Test
    public void theFrameStopsTheRobotAtItsNearestPart() {
        robotDrive();
        double halfRobot = SimRobot.ROBOT_SIZE_IN / 2;
        double y = -28;
        double nearestFace = Double.POSITIVE_INFINITY;
        for (SimField.Obstacle part : SimRobot.FIELD.obstacles) {
            boolean inThePath = part.name.startsWith("Frame") && minX(part.footprint) < 0
                    && maxY(part.footprint) > y - halfRobot && minY(part.footprint) < y + halfRobot;
            if (inThePath) {
                nearestFace = Math.min(nearestFace, minX(part.footprint));
            }
        }
        assertTrue("some part of the frame is in the way", nearestFace < 0);
        sim.setPose(new Pose2d(-50, y, 0));
        setPowers(1, 1, 1, 1);

        sim.step(1.0);

        assertEquals(nearestFace - halfRobot, sim.pose().position.x, DELTA);
        assertEquals(y, sim.pose().position.y, DELTA);
    }

    @Test
    public void againstAFlowerTheDeadWheelsReadTheRobotStandingStill() {
        MecanumDrive drive = robotDrive();
        SimField.Obstacle flower = SimRobot.FIELD.obstacle("Flower Assembly <4>");
        double face = maxY(flower.footprint);
        double x = (minX(flower.footprint) + maxX(flower.footprint)) / 2;
        double halfRobot = SimRobot.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(x, face + halfRobot + 10, -Math.PI / 2));
        drive.localizer.setPose(sim.pose());
        drive.localizer.update();
        setPowers(1, 1, 1, 1);

        for (int i = 0; i < 40; i++) {
            sim.step(0.025);
            drive.localizer.update();
        }

        Pose2d estimated = drive.localizer.getPose();
        assertEquals(face + halfRobot, sim.pose().position.y, DELTA);
        assertEquals(face + halfRobot, estimated.position.y, 0.5);
        assertEquals(x, estimated.position.x, 0.5);
    }

    private static double minX(double[][] ring) {
        double v = Double.POSITIVE_INFINITY;
        for (double[] p : ring) v = Math.min(v, p[0]);
        return v;
    }

    private static double maxX(double[][] ring) {
        double v = Double.NEGATIVE_INFINITY;
        for (double[] p : ring) v = Math.max(v, p[0]);
        return v;
    }

    private static double minY(double[][] ring) {
        double v = Double.POSITIVE_INFINITY;
        for (double[] p : ring) v = Math.min(v, p[1]);
        return v;
    }

    private static double maxY(double[][] ring) {
        double v = Double.NEGATIVE_INFINITY;
        for (double[] p : ring) v = Math.max(v, p[1]);
        return v;
    }

    // --- the loose pollen: balls the robot pushes and that roll on and stop ---

    private static final double BALL = SimRobot.FIELD.loosePieces.get(0).radius;

    @Test
    public void theRobotPushesALooseBallAheadOfIt() {
        robotDrive();
        sim.placePiece(0, 10, -40);
        sim.setPose(new Pose2d(-8, -40, 0));
        setPowers(1, 1, 1, 1);

        sim.step(0.3);

        double[] ball = sim.pieces()[0];
        double front = sim.pose().position.x + SimRobot.ROBOT_SIZE_IN / 2;
        assertTrue("the robot drove; x=" + sim.pose().position.x, sim.pose().position.x > 0);
        assertTrue("the ball is ahead of the robot's front edge: " + ball[0] + " vs " + front, ball[0] - BALL >= front - 0.01);
        assertEquals(-40, ball[1], 0.01);
    }

    @Test
    public void aPushedBallRollsOnThenComesToRestInsideTheWalls() {
        robotDrive();
        sim.placePiece(0, 10, -40);
        sim.setPose(new Pose2d(-8, -40, 0));
        setPowers(1, 1, 1, 1);
        sim.step(0.3);
        double pushedTo = sim.pieces()[0][0];
        setPowers(0, 0, 0, 0);

        sim.step(0.05);
        double rollingTo = sim.pieces()[0][0];
        sim.step(3.0);
        double restingAt = sim.pieces()[0][0];
        sim.step(1.0);

        assertTrue("rolled on after the push: " + rollingTo + " vs " + pushedTo, rollingTo > pushedTo + 0.5);
        assertTrue("came to rest: " + restingAt + " vs " + rollingTo, restingAt > rollingTo);
        assertEquals("stays at rest", restingAt, sim.pieces()[0][0], DELTA);
        assertTrue("inside the walls", restingAt < SimRobot.FIELD_SIZE_IN / 2 - BALL);
    }

    @Test
    public void aBallStopsAtTheWall() {
        robotDrive();
        sim.placePiece(0, 60, -40);
        sim.setPose(new Pose2d(42, -40, 0));
        setPowers(1, 1, 1, 1);

        sim.step(1.0);

        assertEquals(SimRobot.FIELD_SIZE_IN / 2 - BALL, sim.pieces()[0][0], DELTA);
        assertEquals(-40, sim.pieces()[0][1], DELTA);
    }

    @Test
    public void aBallPushesTheBallInFrontOfIt() {
        robotDrive();
        sim.placePiece(0, 10, -40);
        sim.placePiece(1, 10 + 2 * BALL + 0.5, -40);
        sim.setPose(new Pose2d(-8, -40, 0));
        setPowers(1, 1, 1, 1);

        sim.step(0.3);

        double[] first = sim.pieces()[0], second = sim.pieces()[1];
        double front = sim.pose().position.x + SimRobot.ROBOT_SIZE_IN / 2;
        assertTrue("both ahead of the robot", first[0] - BALL >= front - 0.01 && second[0] - BALL >= front - 0.01);
        assertTrue("not through each other: " + first[0] + " and " + second[0], second[0] - first[0] >= 2 * BALL - 0.01);
    }

    @Test
    public void aRollingBallStopsAtAnObstacle() {
        robotDrive();
        double y = -23.7; // the frame's foot bar on that side runs along y from -24.7 to -22.8, x from -19.5 to 19.5
        sim.placePiece(0, -40, y);
        sim.setPose(new Pose2d(-58, y, 0));
        setPowers(1, 1, 1, 1);
        sim.step(0.2);
        setPowers(0, 0, 0, 0);

        sim.step(3.0);

        double[] ball = sim.pieces()[0];
        assertTrue("it rolled: x=" + ball[0], ball[0] > -35);
        assertTrue("and not into the foot bar: x=" + ball[0], ball[0] <= -19.5 - BALL + 0.01);
    }

    @Test
    public void theBallsStartWhereTheFieldIsSetUp() {
        double[][] pieces = sim.pieces();
        assertEquals(SimRobot.FIELD.loosePieces.size(), pieces.length);
        for (int i = 0; i < pieces.length; i++) {
            assertEquals(SimRobot.FIELD.loosePieces.get(i).x, pieces[i][0], 0.1);
            assertEquals(SimRobot.FIELD.loosePieces.get(i).y, pieces[i][1], 0.1);
        }
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
