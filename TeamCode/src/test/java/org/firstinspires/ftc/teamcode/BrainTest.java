package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.base.Hardware;
import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.junit.Test;

/** The brain places the robot on the field from what the camera sees, less the turntable's turn. */
public class BrainTest {
    private static final double DELTA = 0.001;
    private static final int BLUE_GOAL_TAG = 20;

    private final SimRobot sim = new SimRobot();
    private final List<AprilTagDetection> detections = new ArrayList<>();

    private Robot robotFor(Alliance alliance) {
        Hardware hardware = sim.hardware();
        hardware.aprilTags = () -> detections;
        return new Robot(hardware, alliance, new FakeTelemetry());
    }

    private void see(Robot robot, double x, double y, double yawRadians) {
        for (int i = 0; i < 3; i++) {
            detections.clear();
            long now = System.nanoTime();
            Pose3D robotPose = new Pose3D(
                    new Position(DistanceUnit.INCH, x, y, 0, now),
                    new YawPitchRollAngles(AngleUnit.RADIANS, yawRadians, 0, 0, now));
            detections.add(new AprilTagDetection(BLUE_GOAL_TAG, 0, 0, null, null, null, null, null, robotPose, now));
            robot.loop();
        }
    }

    @Test
    public void aSightingPlacesTheRobotWhereTheCameraSaysLessTheTurntablesTurn() {
        Robot robot = robotFor(Alliance.BLUE);
        sim.turnTable.currentPosition = Turntable.TICKS_PER_REVOLUTION / 4;

        see(robot, 10, 20, Math.PI / 2);

        assertEquals(-10, robot.nav.currentPose().x(), DELTA);
        assertEquals(-20, robot.nav.currentPose().y(), DELTA);
        assertEquals(
                "the camera's heading is the turntable's, so the robot's is that less the turn",
                -Math.PI / 2,
                robot.nav.currentPose().heading(),
                DELTA);
    }

    @Test
    public void playingForNoAllianceTheCameraNeverPlacesTheRobot() {
        Robot robot = robotFor(Alliance.RELATIVE);

        see(robot, 10, 20, Math.PI / 2);

        assertEquals(0, robot.nav.currentPose().x(), DELTA);
        assertEquals(0, robot.nav.currentPose().y(), DELTA);
    }
}
