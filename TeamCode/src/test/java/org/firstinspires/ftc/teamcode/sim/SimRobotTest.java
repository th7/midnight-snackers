package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.Turntable;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.hardware.Wheels;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.junit.Test;

public class SimRobotTest {
    private static final double DELTA = 0.001;

    private static final double CONTACT = 0.1;

    private static final double MAX_SPEED_IN_PER_S = (SimRobot.BATTERY_VOLTS - MecanumDrive.PARAMS.kSVolts)
            / MecanumDrive.PARAMS.kVVoltSecondsPerTick
            * MecanumDrive.PARAMS.inPerTick;

    private static final double CLOSE_LAUNCH_VELOCITY = 1050;

    private static final double LAUNCH_DISTANCE_INCHES = 40;

    private static final double TOP_GATE_OPEN = 1, TOP_GATE_CLOSED = 0.6;
    private static final double BOTTOM_GATE_OPEN = 0.5, BOTTOM_GATE_CLOSED = 0.4;

    private final SimRobot sim = new SimRobot();

    @Test
    public void theClockStartsAtZeroAndAdvancesExactlyWithTheWorld() {
        assertEquals(0, sim.nanoTime());

        sim.step(0.5);
        sim.step(0.02);

        assertEquals(520_000_000L, sim.nanoTime());
        assertEquals(
                "the hardware's nanoClock is the world's",
                520_000_000L,
                sim.hardware().nanoClock.getAsLong());
    }

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
    public void theRobotsOwnDriveSetsEveryWheelToBrake() {
        robotDrive();

        for (FakeDcMotorEx wheel : wheelsOf(sim)) {
            assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, wheel.getZeroPowerBehavior());
        }
    }

    @Test
    public void withTheWheelsBrakingCuttingThePowerStopsTheRobotShort() {
        double braked = rollOutAfterTheCut(sim, DcMotor.ZeroPowerBehavior.BRAKE);

        assertTrue("still moving just after the cut: " + braked, braked > 0.5);
        assertTrue("but down within a foot or two: " + braked, braked < 24);
        double stopped = sim.pose().position.x;
        sim.step(1.0);
        assertEquals("and stays stopped", stopped, sim.pose().position.x, DELTA);
    }

    @Test
    public void withTheWheelsFloatingCuttingThePowerLetsTheRobotRollOnMuchFurther() {
        double braked = rollOutAfterTheCut(new SimRobot(), DcMotor.ZeroPowerBehavior.BRAKE);

        double floated = rollOutAfterTheCut(sim, DcMotor.ZeroPowerBehavior.FLOAT);

        assertTrue(
                "floating rolls on several times as far as braking: " + floated + " against " + braked,
                floated > 3 * braked);
        double stopped = sim.pose().position.x;
        sim.step(2.0);
        assertEquals("and friction still brings it to a stop", stopped, sim.pose().position.x, DELTA);
    }

    @Test
    public void aWheelRollingAtZeroPowerWithNoBehaviorSetIsRefusedRatherThanGuessedAt() {
        robotDrive();
        sim.setPose(new Pose2d(-60, 0, 0));
        setPowers(1, 1, 1, 1);
        sim.step(0.5);
        sim.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.UNKNOWN);
        setPowers(0, 0, 0, 0);

        IllegalStateException thrown = assertThrows(IllegalStateException.class, () -> sim.step(0.02));

        assertTrue(thrown.getMessage(), thrown.getMessage().contains("zero power behavior"));
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
        Localizer localizer = robotLocalizer(sim);
        MecanumDrive drive = robotDrive(sim, localizer);
        localizer.update();

        setPowers(0.6, 0.9, 0.6, 0.9);
        for (int i = 0; i < 40; i++) {
            sim.step(0.025);
            localizer.update();
        }

        Pose2d truePose = sim.pose();
        Pose2d estimated = localizer.pose();
        assertTrue(
                "the arc should have turned the robot; heading=" + truePose.heading.toDouble(),
                Math.abs(truePose.heading.toDouble()) > 0.2);
        assertEquals(truePose.position.x, estimated.position.x, 0.5);
        assertEquals(truePose.position.y, estimated.position.y, 0.5);
        assertEquals(truePose.heading.toDouble(), estimated.heading.toDouble(), 0.02);
    }

    @Test
    public void theRobotsOwnLocalizerReadsACreepingRobotAsCreeping() {
        SimRobot creeper = new SimRobot(SimNoise.NONE.withMotors(new SimNoise.Motor(0.9, 1, 1)));
        Localizer localizer = robotLocalizer(creeper);
        MecanumDrive drive = new MecanumDrive(
                new Wheels(creeper.leftFront, creeper.leftBack, creeper.rightBack, creeper.rightFront),
                () -> creeper.imu,
                creeper.voltageSensor,
                localizer,
                creeper::nanoTime);
        localizer.update();

        double power = MecanumDrive.PARAMS.kSVolts / SimRobot.BATTERY_VOLTS;
        creeper.leftFront.setPower(power);
        creeper.rightFront.setPower(power);
        creeper.leftBack.setPower(power);
        creeper.rightBack.setPower(power);
        creeper.step(3.0);

        double creep = 0.1
                * MecanumDrive.PARAMS.kSVolts
                / MecanumDrive.PARAMS.kVVoltSecondsPerTick
                * MecanumDrive.PARAMS.inPerTick;
        localizer.update();
        assertEquals("inches per second", creep, localizer.velocity().linearVel.norm(), 0.02);
        assertEquals("a multiple of the hub's step", 0, Math.round(creeper.rightBack.getVelocity()) % 20);
    }

    @Test
    public void aRobotPlacedOutsideTheWallsIsPlacedAgainstThem() {
        double edge = SimPlacement.FIELD_SIZE_IN / 2 - SimPlacement.ROBOT_SIZE_IN / 2;

        sim.setPose(new Pose2d(1000, -1000, 0));

        assertEquals(edge, sim.pose().position.x, DELTA);
        assertEquals(-edge, sim.pose().position.y, DELTA);
        assertEquals(0, sim.pose().heading.toDouble(), DELTA);

        sim.setPose(new Pose2d(1000, 0, Math.PI / 4));
        assertEquals(
                "a turned robot reaches the wall with its corner",
                SimPlacement.FIELD_SIZE_IN / 2 - SimPlacement.ROBOT_SIZE_IN / 2 * Math.sqrt(2),
                sim.pose().position.x,
                DELTA);
        assertEquals(Math.PI / 4, sim.pose().heading.toDouble(), DELTA);

        Pose2d inside = new Pose2d(12, -7, 1);
        sim.setPose(inside);
        assertEquals("a pose inside the walls is placed as it is", inside.position.x, sim.pose().position.x, DELTA);
        assertEquals(inside.position.y, sim.pose().position.y, DELTA);
        assertEquals(inside.heading.toDouble(), sim.pose().heading.toDouble(), DELTA);
    }

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
        double wall = SimPlacement.FIELD_SIZE_IN / 2;
        double halfRobot = SimPlacement.ROBOT_SIZE_IN / 2;
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
        double edge = SimPlacement.FIELD_SIZE_IN / 2 - SimPlacement.ROBOT_SIZE_IN / 2;

        sim.setPose(new Pose2d(-edge + 5, 0, 0));
        setPowers(-1, -1, -1, -1);
        sim.step(1.0);
        assertEquals("back wall", -edge, sim.pose().position.x, CONTACT);

        sim.setPose(new Pose2d(0, edge - 5, 0));
        setPowers(-1, 1, 1, -1);
        sim.step(1.0);
        assertEquals("left wall", edge, sim.pose().position.y, CONTACT);

        sim.setPose(new Pose2d(0, -edge + 5, 0));
        setPowers(1, -1, -1, 1);
        sim.step(1.0);
        assertEquals("right wall", -edge, sim.pose().position.y, CONTACT);
    }

    @Test
    public void aTurnedRobotStopsWhereItsCornerMeetsTheWall() {
        robotDrive();
        double wall = SimPlacement.FIELD_SIZE_IN / 2;
        double cornerReach = SimPlacement.ROBOT_SIZE_IN / 2 * Math.sqrt(2);
        sim.setPose(new Pose2d(wall - cornerReach - 10, 0, Math.PI / 4));
        setPowers(1, 1, 1, 1);

        sim.step(0.7);

        assertEquals(wall - cornerReach, sim.pose().position.x, CONTACT);
        assertEquals(Math.PI / 4, sim.pose().heading.toDouble(), 0.02);

        sim.step(0.5);
        assertTrue("no corner past the wall: " + sim.pose(), maxX(corners(sim.pose())) <= wall + CONTACT);
    }

    @Test
    public void drivingDiagonallyIntoTheWallSlidesAlongIt() {
        robotDrive();
        double edge = SimPlacement.FIELD_SIZE_IN / 2 - SimPlacement.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(edge, 0, 0));
        setPowers(0, 1, 1, 0);

        sim.step(1.0);

        Pose2d pose = sim.pose();
        double wall = SimPlacement.FIELD_SIZE_IN / 2;
        assertEquals("still against the wall", wall, maxX(corners(pose)), CONTACT);
        assertTrue("y=" + pose.position.y, pose.position.y > 5);
        assertEquals(0, pose.heading.toDouble(), 0.1);
    }

    @Test
    public void againstTheWallTheDeadWheelsReadTheRobotStandingStillNotTheWheelsSpinning() {
        Localizer localizer = robotLocalizer(sim);
        MecanumDrive drive = robotDrive(sim, localizer);
        double edge = SimPlacement.FIELD_SIZE_IN / 2 - SimPlacement.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(edge - 10, 0, 0));
        localizer.setPose(sim.pose());
        localizer.update();
        setPowers(1, 1, 1, 1);

        for (int i = 0; i < 40; i++) {
            sim.step(0.025);
            localizer.update();
        }

        Pose2d estimated = localizer.pose();
        assertEquals(edge, sim.pose().position.x, CONTACT);
        assertEquals(edge, estimated.position.x, 0.5);
        assertEquals(0, estimated.position.y, 0.5);
    }

    @Test
    public void aFlowerStopsTheRobotWhereItsFrontEdgeMeetsIt() {
        robotDrive();
        double[][] flower = cornersOf("Flower Assembly <4>");
        double face = maxY(flower);
        double x = (minX(flower) + maxX(flower)) / 2;
        double halfRobot = SimPlacement.ROBOT_SIZE_IN / 2;

        sim.setPose(new Pose2d(x, face + halfRobot + 10, -Math.PI / 2));
        setPowers(1, 1, 1, 1);

        sim.step(1.0);

        Pose2d pose = sim.pose();
        assertEquals(face + halfRobot, pose.position.y, CONTACT);
        assertEquals(x, pose.position.x, 0.05);
        assertEquals(-Math.PI / 2, pose.heading.toDouble(), 0.02);
    }

    @Test
    public void drivingDiagonallyIntoAFlowerSlidesAlongItWithoutEnteringIt() {
        robotDrive();
        double[][] flower = cornersOf("Flower Assembly <4>");
        double face = maxY(flower);
        double x = (minX(flower) + maxX(flower)) / 2;
        double halfRobot = SimPlacement.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(x, face + halfRobot, -Math.PI / 2));
        setPowers(0, 1, 1, 0);

        for (int i = 0; i < 10; i++) {
            sim.step(0.1);
            for (SimField.Obstacle pipe : partsOf("Flower Assembly <4>")) {
                assertTrue(
                        "never into " + pipe.name + ": " + sim.pose(),
                        deepestInside(pipe.footprint, corners(sim.pose())) <= CONTACT);
            }
        }

        Pose2d pose = sim.pose();
        assertTrue("x=" + pose.position.x, pose.position.x > x + 2);
    }

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

    @Test
    public void theFrameStopsTheRobotAtItsNearestPart() {
        robotDrive();
        double halfRobot = SimPlacement.ROBOT_SIZE_IN / 2;
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
        Localizer localizer = robotLocalizer(sim);
        MecanumDrive drive = robotDrive(sim, localizer);
        double[][] flower = cornersOf("Flower Assembly <4>");
        double face = maxY(flower);
        double x = (minX(flower) + maxX(flower)) / 2;
        double halfRobot = SimPlacement.ROBOT_SIZE_IN / 2;
        sim.setPose(new Pose2d(x, face + halfRobot + 10, -Math.PI / 2));
        localizer.setPose(sim.pose());
        localizer.update();
        setPowers(1, 1, 1, 1);

        for (int i = 0; i < 40; i++) {
            sim.step(0.025);
            localizer.update();
        }

        Pose2d estimated = localizer.pose();
        assertEquals(face + halfRobot, sim.pose().position.y, CONTACT);
        assertEquals(face + halfRobot, estimated.position.y, 0.5);
        assertEquals(x, estimated.position.x, 0.5);
    }

    private static double[][] corners(Pose2d pose) {
        double h = SimPlacement.ROBOT_SIZE_IN / 2;
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

    private static List<SimField.Obstacle> partsOf(String element) {
        List<SimField.Obstacle> parts = new ArrayList<>();
        for (SimField.Obstacle obstacle : SimRobot.FIELD.obstacles) {
            if (obstacle.name.startsWith(element + " / ")) {
                parts.add(obstacle);
            }
        }
        assertTrue(element + " is in the robot's way", !parts.isEmpty());
        return parts;
    }

    private static double[][] cornersOf(String element) {
        List<double[]> corners = new ArrayList<>();
        for (SimField.Obstacle part : partsOf(element)) {
            corners.addAll(List.of(part.footprint));
        }
        return corners.toArray(new double[0][]);
    }

    private static double maxY(double[][] ring) {
        double v = Double.NEGATIVE_INFINITY;
        for (double[] p : ring) v = Math.max(v, p[1]);
        return v;
    }

    private static final double BALL = SimRobot.FIELD.loosePieces.get(0).radius;
    private static final double BALL_NECTAR = SimRobot.FIELD.cellPieces.get(0).radius;
    private static final int LOOSE = SimRobot.FIELD.loosePieces.size();

    private static final int IN_FLOWERS = LOOSE + SimRobot.FIELD.cellPieces.size();

    private static final int PRELOADED = IN_FLOWERS + SimRobot.FIELD.flowerPieces.size();

    @Test
    public void theRobotPushesALooseBallAheadOfIt() {
        robotDrive();
        sim.placePiece(0, 10, -40);
        sim.setPose(new Pose2d(-8, -40, 0));
        setPowers(1, 1, 1, 1);

        sim.step(0.6);

        double[] ball = sim.pieces()[0];
        double front = sim.pose().position.x + SimPlacement.ROBOT_SIZE_IN / 2;
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
        assertTrue("inside the walls", restingAt < SimPlacement.FIELD_SIZE_IN / 2 - BALL);
    }

    @Test
    public void aBallPinnedAgainstTheWallStopsTheRobotShortOfIt() {
        robotDrive();
        double wall = SimPlacement.FIELD_SIZE_IN / 2;
        sim.placePiece(0, 60, -40);
        sim.setPose(new Pose2d(42, -40, 0));
        setPowers(1, 1, 1, 1);

        sim.step(1.5);
        double stoppedAt = sim.pose().position.x;
        sim.step(0.5);

        double[] ball = sim.pieces()[0];
        double front = sim.pose().position.x + SimPlacement.ROBOT_SIZE_IN / 2;
        assertEquals("the ball is at the wall", wall - BALL, ball[0], CONTACT);
        assertEquals(-40, ball[1], 0.1);
        assertEquals("the robot is stopped by the ball", wall - 2 * BALL, front, CONTACT);
        assertEquals("and stays stopped with its wheels spinning", stoppedAt, sim.pose().position.x, 0.02);
    }

    @Test
    public void aBallPinnedAgainstAnObstacleStopsTheRobotShortOfIt() {
        robotDrive();

        SimField.Obstacle bar = SimRobot.FIELD.obstacle("Frame <1> / Sheet Metal Foot Bar <1>");
        assertNotNull(bar);
        double face = minY(bar.footprint);
        double x = 0;
        sim.placePiece(0, x, face - BALL - 4);
        sim.setPose(new Pose2d(x, face - BALL - 4 - BALL - SimPlacement.ROBOT_SIZE_IN / 2 - 6, Math.PI / 2));
        setPowers(1, 1, 1, 1);

        sim.step(1.5);

        double[] ball = sim.pieces()[0];
        double front = sim.pose().position.y + SimPlacement.ROBOT_SIZE_IN / 2;
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
        double front = sim.pose().position.x + SimPlacement.ROBOT_SIZE_IN / 2;
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
        assertEquals(PRELOADED + SimRobot.PRELOAD, pieces.length);
        for (int i = 0; i < LOOSE; i++) {
            assertEquals(SimRobot.FIELD.loosePieces.get(i).x, pieces[i][0], 0.1);
            assertEquals(SimRobot.FIELD.loosePieces.get(i).y, pieces[i][1], 0.1);
            assertEquals(BALL, pieces[i][2], 0.1);
        }
        for (int i = LOOSE; i < IN_FLOWERS; i++) {
            SimField.Piece nectar = SimRobot.FIELD.cellPieces.get(i - LOOSE);
            assertTrue(
                    "the nectar rests in the cell the field is set up with it in",
                    within(sim, SimRobot.FIELD.cell(nectar.cell), pieces[i]));
        }
        for (int i = IN_FLOWERS; i < PRELOADED; i++) {
            SimField.Flower flower = SimRobot.FIELD.flower(SimRobot.FIELD.flowerPieces.get(i - IN_FLOWERS).flower);
            assertTrue(
                    "the pollen stands in the bore of the flower the field is set up with it in",
                    flower.standsIn(pieces[i][0], pieces[i][1]));
        }
        for (int i = PRELOADED; i < pieces.length; i++) {
            assertNull("held in the robot, so nowhere on the field", pieces[i]);
        }
        assertEquals(SimRobot.PRELOAD, sim.held());
        assertEquals("the nectar each hive is set up with", 3, sim.scored("Blue"));
        assertEquals(3, sim.scored("Red"));
        assertEquals("three fifths full", 0.6, sim.load("Blue"), 0.001);
    }

    @Test
    public void eachFlowerStandsItsPollenInAStackThatStaysPut() {
        for (SimField.Flower flower : SimRobot.FIELD.flowers) {
            assertStandingIn(flower, 4);
        }
        double[][] setUp = sim.pieces();

        sim.step(2.0);

        double[][] now = sim.pieces();
        for (int piece = IN_FLOWERS; piece < PRELOADED; piece++) {
            assertArrayEquals("the flowers' pollen have not moved", setUp[piece], now[piece], 0);
        }
    }

    @Test
    public void onlyTheBottomPollenOfAFlowerStandsBelowTheLip() {
        SimField.Flower flower = SimRobot.FIELD.flowers.get(0);
        double[][] pieces = sim.pieces();
        List<Integer> stack = pollenOf(flower);
        stack.sort((a, b) -> Double.compare(pieces[a][2], pieces[b][2]));

        assertEquals("the bottom one rests on the floor", BALL, pieces[stack.get(0)][2], DELTA);
        assertTrue(
                "and stands wholly below the lip: " + pieces[stack.get(0)][2],
                pieces[stack.get(0)][2] + BALL <= flower.lip);
        for (int above = 1; above < stack.size(); above++) {
            assertTrue(
                    "the rest reach the lip, so the bore holds them: " + pieces[stack.get(above)][2],
                    pieces[stack.get(above)][2] + BALL > flower.lip);
        }
    }

    @Test
    public void takingTheBottomPollenOutOfAFlowerDropsTheStackOnePlaceAndItStandsAgain() {
        SimField.Flower flower = SimRobot.FIELD.flowers.get(0);
        int bottom = bottomOf(flower);

        sim.placePiece(bottom, 0, 0);
        sim.step(1.0);

        assertStandingIn(flower, 3);
        assertEquals("the one knocked out is where it was put", 0, sim.pieces()[bottom][0], CONTACT);
        double[][] settled = sim.pieces();
        sim.step(2.0);
        double[][] now = sim.pieces();
        for (int piece : pollenOf(flower)) {
            assertArrayEquals("the stack that is left stands still", settled[piece], now[piece], 0);
        }
    }

    @Test
    public void aPollenTheRobotHasLetGoOfCannotKnockTheBottomPollenOut() {
        SimField.Flower flower = SimRobot.FIELD.flower("Flower Assembly <1>");
        int bottom = bottomOf(flower);
        double lane = flower.axis[1] + 0.8;
        robotDrive();
        sim.placePiece(0, flower.axis[0] - 50, lane);
        sim.setPose(new Pose2d(flower.axis[0] - 50 - BALL - SimPlacement.ROBOT_SIZE_IN / 2 - 1, lane, 0));
        setPowers(1, 1, 1, 1);
        sim.step(1.0);
        double[][] before = sim.pieces();
        setPowers(0, 0, 0, 0);

        double closest = Double.MAX_VALUE;
        double knockedTo = 0;
        for (int loop = 0; loop < 250; loop++) {
            sim.step(0.02);
            double[] nested = sim.pieces()[bottom], loose = sim.pieces()[0];
            closest = Math.min(closest, Math.hypot(loose[0] - nested[0], loose[1] - nested[1]));
            knockedTo = Math.max(knockedTo, fromTheAxis(flower, nested));
        }

        assertTrue(
                "the loose pollen rolled into the bore and reached the nested one: " + closest,
                closest <= 2 * BALL + CONTACT);
        assertTrue("but never rolled it out of the bore: " + knockedTo, knockedTo <= flower.bore);
        assertEquals("it is still in its nest", 0, fromTheAxis(flower, sim.pieces()[bottom]), CONTACT);
        for (int piece : pollenOf(flower)) {
            if (piece != bottom) {
                assertArrayEquals("the stack above it never moved", before[piece], sim.pieces()[piece], 0);
            }
        }
    }

    @Test
    public void aPollenTheRobotIsStillPushingKnocksTheBottomPollenOutAndTheStackComesDown() {
        SimField.Flower flower = SimRobot.FIELD.flower("Flower Assembly <1>");
        int bottom = bottomOf(flower);

        double lane = flower.axis[1] + 0.8;
        robotDrive();
        sim.placePiece(0, flower.axis[0] - 10, lane);
        sim.setPose(new Pose2d(flower.axis[0] - 10 - BALL - SimPlacement.ROBOT_SIZE_IN / 2 - 2, lane, 0));
        setPowers(1, 1, 1, 1);

        sim.step(2.0);

        double[] knocked = sim.pieces()[bottom];
        assertTrue(
                "the flower's bottom pollen is out of the bore: " + knocked[0] + ", " + knocked[1],
                !flower.standsIn(knocked[0], knocked[1]));
        assertEquals("and loose on the floor", BALL, knocked[2], DELTA);
        assertStandingIn(flower, 3);
        double[][] settled = sim.pieces();
        sim.step(1.0);
        assertStandingIn(flower, 3);
        for (int piece : pollenOf(flower)) {
            if (flower.standsIn(settled[piece][0], settled[piece][1])) {
                assertArrayEquals("the stack that is left stands still", settled[piece], sim.pieces()[piece], 0);
            }
        }
    }

    private static List<Integer> pollenOf(SimField.Flower flower) {
        List<Integer> pollen = new ArrayList<>();
        for (int piece = 0; piece < SimRobot.FIELD.flowerPieces.size(); piece++) {
            if (flower.name.equals(SimRobot.FIELD.flowerPieces.get(piece).flower)) {
                pollen.add(IN_FLOWERS + piece);
            }
        }
        return pollen;
    }

    private static double fromTheAxis(SimField.Flower flower, double[] at) {
        return Math.hypot(at[0] - flower.axis[0], at[1] - flower.axis[1]);
    }

    private int bottomOf(SimField.Flower flower) {
        int bottom = -1;
        for (int piece : pollenOf(flower)) {
            double[] at = sim.pieces()[piece];
            if (at != null && (bottom < 0 || at[2] < sim.pieces()[bottom][2])) {
                bottom = piece;
            }
        }
        assertTrue(flower.name + " holds nothing", bottom >= 0);
        return bottom;
    }

    private void assertStandingIn(SimField.Flower flower, int standing) {
        List<double[]> stack = new ArrayList<>();
        for (double[] at : sim.pieces()) {
            if (at != null && flower.standsIn(at[0], at[1])) {
                stack.add(at);
            }
        }
        stack.sort((a, b) -> Double.compare(a[2], b[2]));
        assertEquals(flower.name + " holds " + standing + " balls", standing, stack.size());
        double resting = BALL;
        for (double[] at : stack) {
            assertEquals(flower.name + ": a ball rests on what is under it", resting, at[2], DELTA);
            resting = at[2] + 2 * BALL;
        }
    }

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
        double[] ball = sim.pieces()[PRELOADED];
        assertNotNull("the first preloaded ball is on its way", ball);
        assertTrue("ahead of the robot: x=" + ball[0], ball[0] > -60 + SimPlacement.ROBOT_SIZE_IN / 2);
        assertTrue("in the air: z=" + ball[2], ball[2] > SimRobot.LAUNCH_HEIGHT_IN);
        assertEquals("straight ahead", 0, ball[1], 0.01);
    }

    @Test
    public void aLaunchedBallFliesOnAnArcLandsAndRollsToRestInsideTheWalls() {
        sim.setPose(new Pose2d(-60, 0, 0));
        readyToLaunch(CLOSE_LAUNCH_VELOCITY);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(0.3);
        double[] flying = sim.pieces()[PRELOADED];
        sim.step(1.7);
        double[] landed = sim.pieces()[PRELOADED];
        sim.step(6.0);
        double[] resting = sim.pieces()[PRELOADED];
        sim.step(1.0);
        double[] still = sim.pieces()[PRELOADED];

        assertTrue("flying: z=" + flying[2], flying[2] > SimRobot.LAUNCH_HEIGHT_IN + 5);
        assertEquals("on the floor", BALL, landed[2], DELTA);
        assertTrue("well down the field: x=" + landed[0], landed[0] > 20);
        assertEquals("at rest", resting[0], still[0], DELTA);
        assertEquals(resting[1], still[1], DELTA);
        double half = SimPlacement.FIELD_SIZE_IN / 2 - BALL;
        assertTrue(
                "inside the walls", Math.abs(resting[0]) <= half + CONTACT && Math.abs(resting[1]) <= half + CONTACT);
    }

    @Test
    public void withTheFlywheelStoppedTheBallJustDropsOutOfTheRobot() {
        sim.setPose(new Pose2d(-60, 0, 0));
        readyToLaunch(0);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(2.0);

        double[] ball = sim.pieces()[PRELOADED];
        assertEquals(SimRobot.PRELOAD - 1, sim.held());
        assertEquals("on the floor", BALL, ball[2], DELTA);
        assertTrue(
                "just ahead of the robot: x=" + ball[0], ball[0] > -60 && ball[0] < -60 + SimPlacement.ROBOT_SIZE_IN);
    }

    @Test
    public void theTurntableTurnsTheLauncherCounterClockwiseWithItsEncoder() {
        sim.setPose(new Pose2d(0, -40, 0));
        sim.turnTable.currentPosition = Turntable.TICKS_PER_REVOLUTION / 4;
        readyToLaunch(CLOSE_LAUNCH_VELOCITY);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(0.3);

        double[] ball = sim.pieces()[PRELOADED];
        assertTrue("launched to the robot's left (+y): y=" + ball[1], ball[1] > -40 + 10);
        assertEquals(0, ball[0], 0.5);
    }

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

    private static Pose2d facing(SimRobot sim, SimField.Cell cell, double distance) {
        double[] centre = cell.mouthCentreAt(sim.tilt(cell.alliance));
        double[] normal = cell.mouthNormalAt(sim.tilt(cell.alliance));
        double length = Math.hypot(normal[0], normal[1]);
        double nx = normal[0] / length, ny = normal[1] / length;
        return new Pose2d(centre[0] + nx * distance, centre[1] + ny * distance, Math.atan2(-ny, -nx));
    }

    private static boolean within(SimRobot sim, SimField.Cell cell, double[] point) {
        double[] min = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
        double[] max = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
        double tilt = sim.tilt(cell.alliance);
        for (double[][] panel : cell.panelsAt(tilt)) {
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

    private static void emptyTheHive(SimRobot sim, String alliance) {
        for (int i = 0; i < SimRobot.FIELD.cellPieces.size(); i++) {
            if (SimRobot.FIELD.cellPieces.get(i).alliance.equals(alliance)) {
                sim.placePiece(LOOSE + i, -60, -60 + 4 * i);
            }
        }
    }

    private static int[] ballsOf(String kind) {
        List<Integer> out = new ArrayList<>();
        for (int i = 0; i < LOOSE; i++) {
            if (SimRobot.FIELD.loosePieces.get(i).kind.equals(kind)) {
                out.add(i);
            }
        }
        for (int i = 0; i < SimRobot.FIELD.cellPieces.size(); i++) {
            if (SimRobot.FIELD.cellPieces.get(i).kind.equals(kind)) {
                out.add(LOOSE + i);
            }
        }
        return out.stream().mapToInt(Integer::intValue).toArray();
    }

    @Test
    public void aCloseShotFromTheLaunchDistanceScoresInTheAlliancesUpturnedCell() {
        for (String alliance : new String[] {"Blue", "Red"}) {
            SimRobot sim = new SimRobot();
            SimField.Cell cell = sim.upturnedCell(alliance);
            int already = sim.scored(alliance);
            sim.setPose(facing(sim, cell, LAUNCH_DISTANCE_INCHES));
            sim.topGate.position = TOP_GATE_OPEN;
            sim.bottomGate.position = BOTTOM_GATE_CLOSED;
            sim.launcher.commandedVelocity = CLOSE_LAUNCH_VELOCITY;
            sim.step(0.1);
            sim.topGate.position = TOP_GATE_CLOSED;
            sim.step(0.05);

            sim.bottomGate.position = BOTTOM_GATE_OPEN;
            sim.step(2.0);

            assertEquals(alliance + " scored", already + 1, sim.scored(alliance));
            assertEquals(SimRobot.PRELOAD - 1, sim.held());
            double[] ball = sim.pieces()[PRELOADED];
            assertTrue(
                    alliance + "'s ball rests in the cell: " + ball[0] + ", " + ball[1] + ", " + ball[2],
                    within(sim, cell, ball));
            sim.step(1.0);
            assertEquals("and stays there", ball[0], sim.pieces()[PRELOADED][0], DELTA);
        }
    }

    @Test
    public void aBallThatHitsTheHiveAnywhereButTheMouthDoesNotScore() {
        SimField.Cell cell = sim.upturnedCell("Blue");
        int already = sim.scored("Blue");

        Pose2d behind = facing(sim, cell, -LAUNCH_DISTANCE_INCHES);
        sim.setPose(new Pose2d(behind.position, behind.heading.plus(Math.PI)));
        readyToLaunch(CLOSE_LAUNCH_VELOCITY);

        sim.bottomGate.position = BOTTOM_GATE_OPEN;
        sim.step(4.0);

        assertEquals(already, sim.scored("Blue"));
        assertEquals(3, sim.scored("Red"));
        double[] ball = sim.pieces()[PRELOADED];
        assertEquals("the ball fell to the floor", BALL, ball[2], DELTA);
    }

    @Test
    public void aBallInADownturnedCellRollsOutAndFallsToTheFloor() {
        SimField.Cell down = downturnedCell("Blue");
        sim.placePiece(PRELOADED, down);
        assertEquals("in the cell for now", 4, sim.scored("Blue"));

        sim.step(2.0);

        assertEquals("but not for long", 3, sim.scored("Blue"));
        double[] ball = sim.pieces()[PRELOADED];
        assertEquals("it rolled out and fell to the floor", BALL, ball[2], DELTA);
        assertTrue("under the hive it fell out of: " + ball[0], ball[0] < down.mouthCentreAt(sim.tilt("Blue"))[0]);
    }

    private static SimField.Cell downturnedCell(String alliance) {
        for (SimField.Cell cell : SimRobot.FIELD.cells) {
            if (cell.alliance.equals(alliance) && !cell.upturnedAt(cell.hive.tilt)) {
                return cell;
            }
        }
        throw new IllegalStateException("no downturned cell for " + alliance);
    }

    @Test
    public void fiveNectarTipAHive() {
        emptyTheHive(sim, "Blue");
        double leaning = sim.tilt("Blue");
        int[] nectar = ballsOf(SimField.NECTAR);
        assertTrue("there is nectar enough to fill a hive", nectar.length >= 5);

        for (int i = 0; i < 4; i++) {
            sim.placePiece(nectar[i], sim.upturnedCell("Blue"));
        }
        sim.step(0.5);
        assertEquals("four fifths full is not full", leaning, sim.tilt("Blue"), DELTA);
        assertEquals(0.8, sim.load("Blue"), 0.001);

        sim.placePiece(nectar[4], sim.upturnedCell("Blue"));
        sim.step(0.5);

        assertEquals("the fifth fills it and it tips", -leaning, sim.tilt("Blue"), DELTA);
    }

    @Test
    public void eightPollenTipAHive() {
        emptyTheHive(sim, "Blue");
        double leaning = sim.tilt("Blue");
        int[] pollen = ballsOf(SimField.POLLEN);

        for (int i = 0; i < 7; i++) {
            sim.placePiece(pollen[i], sim.upturnedCell("Blue"));
        }
        sim.step(0.5);
        assertEquals("seven eighths full is not full", leaning, sim.tilt("Blue"), DELTA);
        assertEquals(0.875, sim.load("Blue"), 0.001);

        sim.placePiece(pollen[7], sim.upturnedCell("Blue"));
        sim.step(0.5);

        assertEquals("the eighth fills it and it tips", -leaning, sim.tilt("Blue"), DELTA);
    }

    @Test
    public void nectarAndPollenTogetherFillAHive() {
        double leaning = sim.tilt("Blue");
        int[] pollen = ballsOf(SimField.POLLEN);

        for (int i = 0; i < 3; i++) {
            sim.placePiece(pollen[i], sim.upturnedCell("Blue"));
        }
        sim.step(0.5);
        assertEquals("three nectar and three pollen leave it just short", leaning, sim.tilt("Blue"), DELTA);
        assertEquals(0.975, sim.load("Blue"), 0.001);

        sim.placePiece(pollen[3], sim.upturnedCell("Blue"));
        sim.step(0.5);

        assertEquals("the fourth pollen fills it", -leaning, sim.tilt("Blue"), DELTA);
    }

    @Test
    public void aHiveTipsBackWhenTheCellThatCameUpIsFilled() {
        emptyTheHive(sim, "Blue");
        double leaning = sim.tilt("Blue");
        int[] pollen = ballsOf(SimField.POLLEN);
        assertTrue("there is pollen enough to fill a hive", pollen.length >= 8);

        SimField.Cell first = sim.upturnedCell("Blue");
        for (int i = 0; i < 8; i++) {
            sim.placePiece(pollen[i], first);
        }
        sim.step(3.0);
        assertEquals("the first eight tip it", -leaning, sim.tilt("Blue"), DELTA);

        SimField.Cell second = sim.upturnedCell("Blue");
        assertTrue("the other cell is up now", second != first);
        for (int i = 0; i < 8; i++) {
            sim.placePiece(pollen[i], second);
        }
        sim.step(3.0);

        assertEquals("the next eight tip it back", leaning, sim.tilt("Blue"), DELTA);
        assertTrue("the cell that was up first is up again", first == sim.upturnedCell("Blue"));
        assertEquals("nothing is left in the hive", 0, sim.scored("Blue"));
        double[][] pieces = sim.pieces();
        for (int i = 0; i < 8; i++) {
            assertEquals("the pollen it held fell to the floor", BALL, pieces[pollen[i]][2], DELTA);
        }
    }

    @Test
    public void aHiveThatTipsDropsWhatWasInItAndHoldsUpItsOtherCell() {
        SimField.Cell was = sim.upturnedCell("Blue");
        int[] pollen = ballsOf(SimField.POLLEN);
        for (int i = 0; i < 4; i++) {
            sim.placePiece(pollen[i], was);
        }

        sim.step(3.0);

        assertEquals("nothing is left in the hive", 0, sim.scored("Blue"));
        assertEquals(0, sim.load("Blue"), DELTA);
        SimField.Cell now = sim.upturnedCell("Blue");
        assertTrue("the other cell is up now", now != was);
        assertEquals("the hive's cells swapped ends", was.hive, now.hive);
        double[][] pieces = sim.pieces();
        for (int i = 0; i < 4; i++) {
            assertEquals("a pollen that was in it is on the floor", BALL, pieces[pollen[i]][2], DELTA);
        }
        for (int i = 0; i < SimRobot.FIELD.cellPieces.size(); i++) {
            if (SimRobot.FIELD.cellPieces.get(i).alliance.equals("Blue")) {
                assertEquals("so is the nectar it held", BALL_NECTAR, pieces[LOOSE + i][2], DELTA);
            }
        }

        SimRobot after = sim;
        after.setPose(facing(after, now, LAUNCH_DISTANCE_INCHES));
        after.topGate.position = TOP_GATE_OPEN;
        after.bottomGate.position = BOTTOM_GATE_CLOSED;
        after.launcher.commandedVelocity = CLOSE_LAUNCH_VELOCITY;
        after.step(0.1);
        after.topGate.position = TOP_GATE_CLOSED;
        after.step(0.05);
        after.bottomGate.position = BOTTOM_GATE_OPEN;
        after.step(2.0);

        assertEquals("and the cell that came up takes a ball", 1, after.scored("Blue"));
        assertTrue(within(after, now, after.pieces()[PRELOADED]));
    }

    private static double rollOutAfterTheCut(SimRobot sim, DcMotor.ZeroPowerBehavior behavior) {
        robotDrive(sim, robotLocalizer(sim));
        for (FakeDcMotorEx wheel : wheelsOf(sim)) {
            wheel.setZeroPowerBehavior(behavior);
        }
        sim.setPose(new Pose2d(-60, 0, 0));
        setPowers(sim, 1, 1, 1, 1);
        sim.step(0.5);
        setPowers(sim, 0, 0, 0, 0);

        double cutAt = sim.pose().position.x;
        sim.step(4.0);
        return sim.pose().position.x - cutAt;
    }

    private MecanumDrive robotDrive() {
        return robotDrive(sim, robotLocalizer(sim));
    }

    private static FakeDcMotorEx[] wheelsOf(SimRobot sim) {
        return new FakeDcMotorEx[] {sim.leftFront, sim.rightFront, sim.leftBack, sim.rightBack};
    }

    private static Localizer robotLocalizer(SimRobot sim) {
        return new Localizer(
                sim.rightBack, sim.leftFront, () -> sim.imu, new Pose2d(0, 0, 0), sim::nanoTime, Prints.NOWHERE);
    }

    private static MecanumDrive robotDrive(SimRobot sim, Localizer localizer) {
        return new MecanumDrive(
                new Wheels(sim.leftFront, sim.leftBack, sim.rightBack, sim.rightFront),
                () -> sim.imu,
                sim.voltageSensor,
                localizer,
                sim::nanoTime);
    }

    private void setPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        setPowers(sim, leftFront, rightFront, leftBack, rightBack);
    }

    private static void setPowers(
            SimRobot sim, double leftFront, double rightFront, double leftBack, double rightBack) {
        sim.leftFront.setPower(leftFront);
        sim.rightFront.setPower(rightFront);
        sim.leftBack.setPower(leftBack);
        sim.rightBack.setPower(rightBack);
    }
}
