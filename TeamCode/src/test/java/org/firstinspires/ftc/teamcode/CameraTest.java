package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class CameraTest {
    private static final double DELTA = 0.0001;
    private static final int BLUE_GOAL_TAG = 20;
    private static final int OTHER_TAG = 21;

    private final List<AprilTagDetection> detections = new ArrayList<>();
    private final Camera camera = new Camera(() -> detections, new ElapsedTime(), new FakeTelemetry());

    @Test
    public void noDetectionsMeansNoPose() {
        camera.init();

        camera.loop();

        assertNull(camera.calculateRoadrunnerPose());
    }

    @Test
    public void threeConsistentGoalDetectionsProduceARoadrunnerPose() {
        camera.init();
        for (int i = 0; i < 3; i++) {
            detections.clear();
            detections.add(goalDetection(BLUE_GOAL_TAG, 10, 20, Math.PI / 2));
            camera.loop();
        }

        Pose2d pose = camera.calculateRoadrunnerPose();

        assertNotNull(pose);
        assertEquals(-10, pose.position.x, DELTA);
        assertEquals(-20, pose.position.y, DELTA);
        assertEquals(0, pose.heading.toDouble(), DELTA);
    }

    @Test
    public void detectionsOfOtherTagsAreIgnored() {
        camera.init();
        for (int i = 0; i < 3; i++) {
            detections.clear();
            detections.add(goalDetection(OTHER_TAG, 10, 20, Math.PI / 2));
            camera.loop();
        }

        assertNull(camera.calculateRoadrunnerPose());
    }

    private AprilTagDetection goalDetection(int id, double x, double y, double yawRadians) {
        long now = System.nanoTime();
        Pose3D robotPose = new Pose3D(
                new Position(DistanceUnit.INCH, x, y, 0, now),
                new YawPitchRollAngles(AngleUnit.RADIANS, yawRadians, 0, 0, now));
        return new AprilTagDetection(id, 0, 0, null, null, null, null, null, robotPose, now);
    }
}
