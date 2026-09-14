package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Turntable;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.junit.Test;

public class SimRobotTest {
    private static final double DELTA = 0.001;
    /** How close a body rests to what it is pressed against: the engine's contact tolerance, with room. */
    private static final double CONTACT = 0.1;
    /** The free speed of the tuned drive at full power: (battery - kS) / kV ticks per second, in inches. */
    private static final double MAX_SPEED_IN_PER_S =
            (SimRobot.BATTERY_VOLTS - MecanumDrive.PARAMS.kS) / MecanumDrive.PARAMS.kV * MecanumDrive.PARAMS.inPerTick;
    /** What the launcher spins at for a close shot, as {@code Launcher} sets it: encoder ticks per second. */
    private static final double CLOSE_LAUNCH_VELOCITY = 1050;
    /** How far from the goal the robot code launches from ({@code Nav}), in inches. */
    private static final double LAUNCH_DISTANCE = 40;
    /** The gate positions {@code Launcher} drives the servos to. */
    private static final double TOP_GATE_OPEN = 1, TOP_GATE_CLOSED = 0.6;

    private static final double BOTTOM_GATE_OPEN = 0.5, BOTTOM_GATE_CLOSED = 0.4;

    private final SimRobot sim = new SimRobot();

    // --- the clock: the simulation owns time, and the robot code reads it through its hardware ---

    @Test
    public void theClockStartsAtZeroAndAdvancesExactlyWithTheWorld() {
        assertEquals(0, sim.nanoTime());

        sim.step(0.5);
        sim.step(0.02);

        assertEquals(520_000_000L, sim.nanoTime());
        assertEquals(
                "the hardware's clock is the world's",
                520_000_000L,
                sim.hardware().clock.getAsLong());
    }

    // --- the drive: the tuned feedforward model, inertia included ---

    @Test
    public void withTheRobotsMotorDirectionsForwardPowerDrivesStraightAheadAndReachesNearMaxSpeed() {
        robotDrive();
        sim.setPose(new Pose2d(-60, 0, 0));
        setPowers(1, 1, 1, 1);

        sim.step(2.0);
        double before = sim.pose().position.x;
        sim.step(0.5);

        Pose2d pose = sim.pose();
        double speed = (pose.position.x - before) / 0.5;
        assertEquals("at full speed after two seconds", MAX_SPEED_IN_PER_S, speed, MAX_SPEED_IN_PER_S * 0.05);
        assertEquals(0, pose.position.y, 0.01);
        assertEquals(0, pose.heading.toDouble(), 0.01);
    }

    /** The drive has inertia: kA in the tuned model. It takes time to reach speed and time to stop. */
    @Test
    public void theRobotTakesTimeToGetUpToSpeed() {
        robotDrive();
        setPowers(1, 1, 1, 1);

        sim.step(0.1);
        double early = sim.pose().position.x;
        sim.step(0.4);
        double later = sim.pose().position.x;

        assertTrue("moving within a tenth of a second: " + early, early > 0);
        assertTrue("but well short of full speed: " + early, early < MAX_SPEED_IN_PER_S * 0.1 * 0.5);
        assertTrue("and still accelerating: " + later, later < MAX_SPEED_IN_PER_S * 0.5 && later > early);
    }

    @Test
    public void withThePowerCutTheRobotCoastsToAStop() {
        robotDrive();
        sim.setPose(new Pose2d(-60, 0, 0));
        setPowers(1, 1, 1, 1);
        sim.step(2.0);
        setPowers(0, 0, 0, 0);
        double cutAt = sim.pose().position.x;

        sim.step(0.05);
        double coasting = sim.pose().position.x;
        sim.step(2.0);
        double stopped = sim.pose().position.x;
        sim.step(1.0);

        assertTrue("still moving just after the cut", coasting > cutAt + 0.5);
        assertTrue("came to a stop", stopped > coasting);
        assertEquals("and stays stopped", stopped, sim.pose().position.x, DELTA);
        assertTrue("within a few feet: " + (stopped - cutAt), stopped - cutAt < 36);
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

        sim.step(0.5);

        Pose2d pose = sim.pose();
        assertEquals(0, pose.position.x, 0.01);
        assertEquals(0, pose.position.y, 0.01);
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
        assertTrue(
                "the arc should have turned the robot; heading=" + truePose.heading.toDouble(),
                Math.abs(truePose.heading.toDouble()) > 0.2);
        assertEquals(truePose.position.x, estimated.position.x, 0.5);
        assertEquals(truePose.position.y, estimated.position.y, 0.5);
        assertEquals(truePose.heading.toDouble(), estimated.heading.toDouble(), 0.02);
    }

    // --- the walls and the obstacles ---

    /** A robot placed beyond a wall is placed against it instead: the walls hold whatever pose it is given. */
    @Test
    public void aRobotPlacedOutsideTheWallsIsPlacedAgainstThem() {
        double edge = SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2;

        sim.setPose(new Pose2d(1000, -1000, 0));

        assertEquals(edge, sim.pose().position.x, DELTA);
        assertEquals(-edge, sim.pose().position.y, DELTA);
        assertEquals(0, sim.pose().heading.toDouble(), DELTA);

        sim.setPose(new Pose2d(1000, 0, Math.PI / 4));
        assertEquals(
                "a turned robot reaches the wall with its corner",
                SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2 * Math.sqrt(2),
                sim.pose().position.x,
                DELTA);
        assertEquals(Math.PI / 4, sim.pose().heading.toDouble(), DELTA);

        Pose2d inside = new Pose2d(12, -7, 1);
        sim.setPose(inside);
        assertEquals("a pose inside the walls is placed as it is", inside.position.x, sim.pose().position.x, DELTA);
        assertEquals(inside.position.y, sim.pose().position.y, DELTA);
        assertEquals(inside.heading.toDouble(), sim.pose().heading.toDouble(), DELTA);
    }

    /** Placing the robot puts it down still: whatever it was doing before does not carry over. */
    @Test
    public void aPlacedRobotStartsFromRest() {
        robotDrive();
        setPowers(1, 1, 1, 1);
        sim.step(1.0);
        setPowers(0, 0, 0, 0);

        sim.setPose(new Pose2d(0, 0, 0));
        sim.step(0.5);

        assertEquals(0, sim.pose().position.x, DELTA);
        assertEquals(0, sim.pose().position.y, DELTA);
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
        assertEquals(wall - halfRobot, pose.position.x, CONTACT);
        assertEquals(0, pose.position.y, 0.05);
        assertEquals(0, pose.heading.toDouble(), 0.01);
    }

    @Test
    public void everyWallStopsTheRobot() {
        robotDrive();
        double edge = SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2;

        sim.setPose(new Pose2d(-edge + 5, 0, 0));
        setPowers(-1, -1, -1, -1);
        sim.step(1.0);
        assertEquals("back wall", -edge, sim.pose().position.x, CONTACT);

        sim.setPose(new Pose2d(0, edge - 5, 0));
        setPowers(-1, 1, 1, -1); // strafe left
        sim.step(1.0);
        assertEquals("left wall", edge, sim.pose().position.y, CONTACT);

        sim.setPose(new Pose2d(0, -edge + 5, 0));
        setPowers(1, -1, -1, 1); // strafe right
        sim.step(1.0);
        assertEquals("right wall", -edge, sim.pose().position.y, CONTACT);
    }

    @Test
    public void aTurnedRobotStopsWhereItsCornerMeetsTheWall() {
        robotDrive();
        double wall = SimRobot.FIELD_SIZE_IN / 2;
        double cornerReach = SimRobot.ROBOT_SIZE_IN / 2 * Math.sqrt(2);
        sim.setPose(new Pose2d(wall - cornerReach - 10, 0, Math.PI / 4));
        setPowers(1, 1, 1, 1);

        sim.step(0.7);

        assertEquals(wall - cornerReach, sim.pose().position.x, CONTACT);
        assertEquals(Math.PI / 4, sim.pose().heading.toDouble(), 0.02);
        // Pressing a corner into the wall while sliding along it, the robot pivots; it still stays inside.
        sim.step(0.5);
        assertTrue("no corner past the wall: " + sim.pose(), maxX(corners(sim.pose())) <= wall + CONTACT);
    }

    @Test
    public void drivingDiagonallyIntoTheWallSlidesAlongIt() {
        robotDrive();
        double edge = SimRobot.FIELD_SIZE_IN / 2 - SimRobot.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(edge, 0, 0));
        setPowers(0, 1, 1, 0); // forward and left

        sim.step(1.0);

        Pose2d pose = sim.pose();
        double wall = SimRobot.FIELD_SIZE_IN / 2;
        assertEquals("still against the wall", wall, maxX(corners(pose)), CONTACT);
        assertTrue("y=" + pose.position.y, pose.position.y > 5);
        assertEquals(0, pose.heading.toDouble(), 0.1);
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
        assertEquals(edge, sim.pose().position.x, CONTACT);
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
        assertEquals(face + halfRobot, pose.position.y, CONTACT);
        assertEquals(x, pose.position.x, 0.05);
        assertEquals(-Math.PI / 2, pose.heading.toDouble(), 0.02);
    }

    /**
     * A flower's inward face is only a few inches wide, so a robot pushed into it off-centre
     * pivots on it as it slides by; it never gets into the flower.
     */
    @Test
    public void drivingDiagonallyIntoAFlowerSlidesAlongItWithoutEnteringIt() {
        robotDrive();
        SimField.Obstacle flower = SimRobot.FIELD.obstacle("Flower Assembly <4>");
        double face = maxY(flower.footprint);
        double x = (minX(flower.footprint) + maxX(flower.footprint)) / 2;
        double halfRobot = SimRobot.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(x, face + halfRobot, -Math.PI / 2));
        setPowers(0, 1, 1, 0); // forward and left, which facing -y is toward +x

        for (int i = 0; i < 10; i++) {
            sim.step(0.1);
            assertTrue(
                    "never into the flower: " + sim.pose(),
                    deepestInside(flower.footprint, corners(sim.pose())) <= CONTACT);
        }

        Pose2d pose = sim.pose();
        assertTrue("x=" + pose.position.x, pose.position.x > x + 2);
    }

    /** The frame in the middle of the field is driven through, between its legs. */
    @Test
    public void theRobotDrivesUnderTheHivesBetweenTheFramesLegs() {
        robotDrive();
        sim.setPose(new Pose2d(-14, -8, 0));
        setPowers(1, 1, 1, 1);

        sim.step(0.6);

        Pose2d pose = sim.pose();
        assertTrue("x=" + pose.position.x, pose.position.x > -14 + 10);
        assertEquals(-8, pose.position.y, 0.01);
    }

    /** The frame's feet and legs stand in the way; the nearest part in the robot's path stops it. */
    @Test
    public void theFrameStopsTheRobotAtItsNearestPart() {
        robotDrive();
        double halfRobot = SimRobot.ROBOT_SIZE_IN / 2;
        double y = -28;
        double nearestFace = Double.POSITIVE_INFINITY;
        for (SimField.Obstacle part : SimRobot.FIELD.obstacles) {
            boolean inThePath = part.name.startsWith("Frame")
                    && minX(part.footprint) < 0
                    && maxY(part.footprint) > y - halfRobot
                    && minY(part.footprint) < y + halfRobot;
            if (inThePath) {
                nearestFace = Math.min(nearestFace, minX(part.footprint));
            }
        }
        assertTrue("some part of the frame is in the way", nearestFace < 0);
        sim.setPose(new Pose2d(-50, y, 0));
        setPowers(1, 1, 1, 1);

        sim.step(0.8);
        assertEquals("stopped at the nearest part", nearestFace - halfRobot, sim.pose().position.x, 0.5);
        assertEquals(y, sim.pose().position.y, 0.5);
        // Pushed into the end of a foot bar off-centre, the robot pivots on it; it never gets into the frame.
        for (int i = 0; i < 12; i++) {
            sim.step(0.1);
            for (SimField.Obstacle part : SimRobot.FIELD.obstacles) {
                assertTrue(
                        "never into " + part.name + ": " + sim.pose(),
                        deepestInside(part.footprint, corners(sim.pose())) <= CONTACT);
            }
        }
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
        assertEquals(face + halfRobot, sim.pose().position.y, CONTACT);
        assertEquals(face + halfRobot, estimated.position.y, 0.5);
        assertEquals(x, estimated.position.x, 0.5);
    }

    /** The robot's square's corners at a pose. */
    private static double[][] corners(Pose2d pose) {
        double h = SimRobot.ROBOT_SIZE_IN / 2;
        double[][] local = {{h, h}, {-h, h}, {-h, -h}, {h, -h}};
        double[][] corners = new double[4][];
        for (int i = 0; i < 4; i++) {
            corners[i] = new double[] {
                pose.position.x + local[i][0] * pose.heading.real - local[i][1] * pose.heading.imag,
                pose.position.y + local[i][0] * pose.heading.imag + local[i][1] * pose.heading.real
            };
        }
        return corners;
    }

    /**
     * How far the deepest of the points is inside a convex counter-clockwise polygon: its least
     * distance to an edge, or negative for a point outside.
     */
    private static double deepestInside(double[][] polygon, double[][] points) {
        double deepest = Double.NEGATIVE_INFINITY;
        for (double[] p : points) {
            double depth = Double.POSITIVE_INFINITY;
            for (int i = 0; i < polygon.length; i++) {
                double[] a = polygon[i], b = polygon[(i + 1) % polygon.length];
                double length = Math.hypot(b[0] - a[0], b[1] - a[1]);
                double inward = ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) / length;
                depth = Math.min(depth, inward);
            }
            deepest = Math.max(deepest, depth);
        }
        return deepest;
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

    // --- the loose pollen: balls the robot pushes, that roll on and stop, and that stop the robot ---

    private static final double BALL = SimRobot.FIELD.loosePieces.get(0).radius;
    private static final int LOOSE = SimRobot.FIELD.loosePieces.size();

    @Test
    public void theRobotPushesALooseBallAheadOfIt() {
        robotDrive();
        sim.placePiece(0, 10, -40);
        sim.setPose(new Pose2d(-8, -40, 0));
        setPowers(1, 1, 1, 1);

        sim.step(0.6);

        double[] ball = sim.pieces()[0];
        double front = sim.pose().position.x + SimRobot.ROBOT_SIZE_IN / 2;
        assertTrue("the robot drove; x=" + sim.pose().position.x, sim.pose().position.x > 0);
        assertTrue(
                "the ball is ahead of the robot's front edge: " + ball[0] + " vs " + front,
                ball[0] - BALL >= front - CONTACT);
        assertEquals(-40, ball[1], 0.05);
        assertEquals("on the floor", BALL, ball[2], DELTA);
    }

    @Test
    public void aPushedBallRollsOnThenComesToRestInsideTheWalls() {
        robotDrive();
        sim.placePiece(0, 10, -40);
        sim.setPose(new Pose2d(-8, -40, 0));
        setPowers(1, 1, 1, 1);
        sim.step(0.6);
        double pushedTo = sim.pieces()[0][0];
        setPowers(0, 0, 0, 0);

        sim.step(0.05);
        double rollingTo = sim.pieces()[0][0];
        sim.step(4.0);
        double restingAt = sim.pieces()[0][0];
        sim.step(1.0);

        assertTrue("rolled on after the push: " + rollingTo + " vs " + pushedTo, rollingTo > pushedTo + 0.5);
        assertTrue("came to rest: " + restingAt + " vs " + rollingTo, restingAt > rollingTo);
        assertEquals("stays at rest", restingAt, sim.pieces()[0][0], DELTA);
        assertTrue("inside the walls", restingAt < SimRobot.FIELD_SIZE_IN / 2 - BALL);
    }

    /** The wall holds the ball, the ball holds the robot: nothing goes through anything. */
    @Test
    public void aBallPinnedAgainstTheWallStopsTheRobotShortOfIt() {
        robotDrive();
        double wall = SimRobot.FIELD_SIZE_IN / 2;
        sim.placePiece(0, 60, -40);
        sim.setPose(new Pose2d(42, -40, 0));
        setPowers(1, 1, 1, 1);

        sim.step(1.5);
        double stoppedAt = sim.pose().position.x;
        sim.step(0.5);

        double[] ball = sim.pieces()[0];
        double front = sim.pose().position.x + SimRobot.ROBOT_SIZE_IN / 2;
        assertEquals("the ball is at the wall", wall - BALL, ball[0], CONTACT);
        assertEquals(-40, ball[1], 0.1);
        assertEquals("the robot is stopped by the ball", wall - 2 * BALL, front, CONTACT);
        assertEquals("and stays stopped with its wheels spinning", stoppedAt, sim.pose().position.x, 0.02);
    }

    @Test
    public void aBallPinnedAgainstAnObstacleStopsTheRobotShortOfIt() {
        robotDrive();
        // The frame's foot bar on that side runs along y from -24.7 to -22.8, x from -19.5 to 19.5.
        SimField.Obstacle bar = SimRobot.FIELD.obstacle("Frame <1> / Sheet Metal Foot Bar <1>");
        assertNotNull(bar);
        double face = minY(bar.footprint);
        double x = 0;
        sim.placePiece(0, x, face - BALL - 4);
        sim.setPose(new Pose2d(x, face - BALL - 4 - BALL - SimRobot.ROBOT_SIZE_IN / 2 - 6, Math.PI / 2));
        setPowers(1, 1, 1, 1);

        sim.step(1.5);

        double[] ball = sim.pieces()[0];
        double front = sim.pose().position.y + SimRobot.ROBOT_SIZE_IN / 2;
        assertEquals("the ball is at the bar", face - BALL, ball[1], CONTACT);
        assertEquals("the robot is stopped by the ball", face - 2 * BALL, front, CONTACT);
    }

    @Test
    public void aBallPushesTheBallInFrontOfIt() {
        robotDrive();
        sim.placePiece(0, 10, -40);
        sim.placePiece(1, 10 + 2 * BALL + 0.5, -40);
        sim.setPose(new Pose2d(-8, -40, 0));
        setPowers(1, 1, 1, 1);

        sim.step(0.6);

        double[] first = sim.pieces()[0], second = sim.pieces()[1];
        double front = sim.pose().position.x + SimRobot.ROBOT_SIZE_IN / 2;
        assertTrue(
                "both ahead of the robot", first[0] - BALL >= front - CONTACT && second[0] - BALL >= front - CONTACT);
        assertTrue(
                "not through each other: " + first[0] + " and " + second[0],
                second[0] - first[0] >= 2 * BALL - CONTACT);
    }

    @Test
    public void aRollingBallStopsAtAnObstacle() {
        robotDrive();
        SimField.Obstacle bar = SimRobot.FIELD.obstacle("Frame <1> / Sheet Metal Foot Bar <1>");
        double face = minX(bar.footprint);
        double y = (minY(bar.footprint) + maxY(bar.footprint)) / 2;
        sim.placePiece(0, -44, y);
        sim.setPose(new Pose2d(-58, y, 0));
        setPowers(1, 1, 1, 1);
        sim.step(0.7);
        setPowers(0, 0, 0, 0);

        sim.step(4.0);

        double[] ball = sim.pieces()[0];
        assertTrue("it rolled: x=" + ball[0], ball[0] > -35);
        assertTrue("and not into the foot bar: x=" + ball[0], ball[0] <= face - BALL + CONTACT);
    }

    @Test
    public void theBallsStartWhereTheFieldIsSetUpAndTheRobotsPreloadInTheRobot() {
        double[][] pieces = sim.pieces();
        assertEquals(LOOSE + SimRobot.PRELOAD, pieces.length);
        for (int i = 0; i < LOOSE; i++) {
            assertEquals(SimRobot.FIELD.loosePieces.get(i).x, pieces[i][0], 0.1);
            assertEquals(SimRobot.FIELD.loosePieces.get(i).y, pieces[i][1], 0.1);
            assertEquals(BALL, pieces[i][2], 0.1);
        }
        for (int i = LOOSE; i < pieces.length; i++) {
            assertNull("held in the robot, so nowhere on the field", pieces[i]);
        }
        assertEquals(SimRobot.PRELOAD, sim.held());
        assertEquals(0, sim.scored("Blue"));
        assertEquals(0, sim.scored("Red"));
    }

    // --- the launcher: a held ball drops through the gates into the flywheel and flies ---

    /**
     * The flywheel spinning and a ball in the chamber, both gates closed, as the robot code has it
     * just before it opens the bottom gate: the top gate was open for the ball to drop in.
     */
    private void readyToLaunch(double flywheelVelocity) {
        sim.topGate.position = TOP_GATE_OPEN;
        sim.bottomGate.position = BOTTOM_GATE_CLOSED;
        sim.launcher.commandedVelocity = flywheelVelocity;
        sim.step(0.1);
        sim.topGate.position = TOP_GATE_CLOSED;
        sim.step(0.05);
    }

    @Test
    public void openingTheBottomGateWithTheFlywheelSpinningLaunchesTheChamberedBall() {
        sim.setPose(new Pose2d(-60, 0, 0));
        readyToLaunch(CLOSE_LAUNCH_VELOCITY);
        assertEquals(SimRobot.PRELOAD, sim.held());

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(0.2);

        assertEquals(SimRobot.PRELOAD - 1, sim.held());
        double[] ball = sim.pieces()[LOOSE];
        assertNotNull("the first preloaded ball is on its way", ball);
        assertTrue("ahead of the robot: x=" + ball[0], ball[0] > -60 + SimRobot.ROBOT_SIZE_IN / 2);
        assertTrue("in the air: z=" + ball[2], ball[2] > SimRobot.LAUNCH_HEIGHT_IN);
        assertEquals("straight ahead", 0, ball[1], 0.01);
    }

    @Test
    public void aLaunchedBallFliesOnAnArcLandsAndRollsToRestInsideTheWalls() {
        sim.setPose(new Pose2d(-60, 0, 0));
        readyToLaunch(CLOSE_LAUNCH_VELOCITY);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(0.3);
        double[] flying = sim.pieces()[LOOSE];
        sim.step(1.7);
        double[] landed = sim.pieces()[LOOSE];
        sim.step(6.0);
        double[] resting = sim.pieces()[LOOSE];
        sim.step(1.0);
        double[] still = sim.pieces()[LOOSE];

        assertTrue("flying: z=" + flying[2], flying[2] > SimRobot.LAUNCH_HEIGHT_IN + 5);
        assertEquals("on the floor", BALL, landed[2], DELTA);
        assertTrue("well down the field: x=" + landed[0], landed[0] > 20);
        assertEquals("at rest", resting[0], still[0], DELTA);
        assertEquals(resting[1], still[1], DELTA);
        double half = SimRobot.FIELD_SIZE_IN / 2 - BALL;
        assertTrue(
                "inside the walls", Math.abs(resting[0]) <= half + CONTACT && Math.abs(resting[1]) <= half + CONTACT);
    }

    @Test
    public void withTheFlywheelStoppedTheBallJustDropsOutOfTheRobot() {
        sim.setPose(new Pose2d(-60, 0, 0));
        readyToLaunch(0);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(2.0);

        double[] ball = sim.pieces()[LOOSE];
        assertEquals(SimRobot.PRELOAD - 1, sim.held());
        assertEquals("on the floor", BALL, ball[2], DELTA);
        assertTrue("just ahead of the robot: x=" + ball[0], ball[0] > -60 && ball[0] < -60 + SimRobot.ROBOT_SIZE_IN);
    }

    /** The launcher is on the turntable, which turns it from straight ahead by its encoder's angle. */
    @Test
    public void theTurntableTurnsTheLauncherCounterClockwiseWithItsEncoder() {
        sim.setPose(new Pose2d(0, -40, 0));
        sim.turnTable.currentPosition = Turntable.TICKS_PER_REVOLUTION / 4;
        readyToLaunch(CLOSE_LAUNCH_VELOCITY);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(0.3);

        double[] ball = sim.pieces()[LOOSE];
        assertTrue("launched to the robot's left (+y): y=" + ball[1], ball[1] > -40 + 10);
        assertEquals(0, ball[0], 0.5);
    }

    /** The gates cycle one ball at a time: the next drops into the chamber only when the top gate opens over an empty one. */
    @Test
    public void theNextBallDropsIntoTheChamberWhenTheTopGateOpensOverAnEmptyOne() {
        sim.setPose(new Pose2d(-60, 0, 0));
        readyToLaunch(CLOSE_LAUNCH_VELOCITY);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(0.2);
        assertEquals("one launched", SimRobot.PRELOAD - 1, sim.held());
        sim.step(0.5);
        assertEquals(
                "the chamber is empty and the top gate closed, so nothing else drops through",
                SimRobot.PRELOAD - 1,
                sim.held());

        sim.bottomGate.position = BOTTOM_GATE_CLOSED;
        sim.topGate.position = TOP_GATE_OPEN;
        sim.step(0.1);
        sim.topGate.position = TOP_GATE_CLOSED;
        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(0.2);
        assertEquals("the ball that dropped in is launched", SimRobot.PRELOAD - 2, sim.held());
    }

    // --- the hives: a ball through a cell's mouth scores ---

    private static SimField.Cell lowestCellOf(String alliance) {
        SimField.Cell lowest = null;
        for (SimField.Cell cell : SimRobot.FIELD.cells) {
            if (cell.alliance.equals(alliance) && (lowest == null || cell.mouthCentre[2] < lowest.mouthCentre[2])) {
                lowest = cell;
            }
        }
        return lowest;
    }

    /** The pose {@code distance} inches from the cell's mouth, straight out from it, facing it. */
    private static Pose2d facing(SimField.Cell cell, double distance) {
        double nx = cell.mouthNormal[0], ny = cell.mouthNormal[1];
        double length = Math.hypot(nx, ny);
        nx /= length;
        ny /= length;
        return new Pose2d(
                cell.mouthCentre[0] + nx * distance, cell.mouthCentre[1] + ny * distance, Math.atan2(-ny, -nx));
    }

    private static boolean within(SimField.Cell cell, double[] point) {
        double[] min = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
        double[] max = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (double[][] panel : cell.panels) {
            for (double[] v : panel) {
                for (int axis = 0; axis < 3; axis++) {
                    min[axis] = Math.min(min[axis], v[axis]);
                    max[axis] = Math.max(max[axis], v[axis]);
                }
            }
        }
        for (int axis = 0; axis < 3; axis++) {
            if (point[axis] < min[axis] - CONTACT || point[axis] > max[axis] + CONTACT) {
                return false;
            }
        }
        return true;
    }

    /**
     * The launcher is calibrated to the robot code: a close shot from the code's launch distance
     * reaches the mouth of the nearer cell.
     */
    @Test
    public void aCloseShotFromTheLaunchDistanceScoresInTheAlliancesLowerCell() {
        for (String alliance : new String[] {"Blue", "Red"}) {
            SimRobot sim = new SimRobot();
            SimField.Cell cell = lowestCellOf(alliance);
            sim.setPose(facing(cell, LAUNCH_DISTANCE));
            sim.topGate.position = TOP_GATE_OPEN;
            sim.bottomGate.position = BOTTOM_GATE_CLOSED;
            sim.launcher.commandedVelocity = CLOSE_LAUNCH_VELOCITY;
            sim.step(0.1);
            sim.topGate.position = TOP_GATE_CLOSED;
            sim.step(0.05);

            sim.bottomGate.position = BOTTOM_GATE_OPEN;
            sim.step(2.0);

            assertEquals(alliance + " scored", 1, sim.scored(alliance));
            assertEquals(SimRobot.PRELOAD - 1, sim.held());
            double[] ball = sim.pieces()[LOOSE];
            assertTrue(
                    alliance + "'s ball rests in the cell: " + ball[0] + ", " + ball[1] + ", " + ball[2],
                    within(cell, ball));
            sim.step(1.0);
            assertEquals("and stays there", ball[0], sim.pieces()[LOOSE][0], DELTA);
        }
    }

    @Test
    public void aBallThatHitsTheHiveAnywhereButTheMouthDoesNotScore() {
        SimField.Cell cell = lowestCellOf("Blue");
        // From behind the cell, facing its back: the ball meets the back panel, not the mouth.
        Pose2d behind = facing(cell, -LAUNCH_DISTANCE);
        sim.setPose(new Pose2d(behind.position, behind.heading.plus(Math.PI)));
        readyToLaunch(CLOSE_LAUNCH_VELOCITY);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(4.0);

        assertEquals(0, sim.scored("Blue"));
        assertEquals(0, sim.scored("Red"));
        double[] ball = sim.pieces()[LOOSE];
        assertEquals("the ball fell to the floor", BALL, ball[2], DELTA);
    }

    /**
     * The robot's own drive on the simulated motors, which applies the motor directions the robot uses.
     */
    private MecanumDrive robotDrive() {
        return new MecanumDrive(
                sim.leftFront,
                sim.leftBack,
                sim.rightBack,
                sim.rightFront,
                () -> sim.imu,
                sim.voltageSensor,
                new Pose2d(0, 0, 0),
                sim::nanoTime);
    }

    private void setPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        sim.leftFront.setPower(leftFront);
        sim.rightFront.setPower(rightFront);
        sim.leftBack.setPower(leftBack);
        sim.rightBack.setPower(rightBack);
    }
}
