package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.junit.Test;

public class CameraTest {
    private static final double DELTA = 0.0001;
    private static final int BLUE_GOAL_TAG = 20;
    private static final int OTHER_TAG = 21;

    private final List<AprilTagDetection> detections = new ArrayList<>();
    private final SimDevices devices = new SimDevices();
    private final Camera camera = cameraFed(devices, detections);

    /** A robot whose camera sees {@code detections} instead of the simulator's empty view. */
    private static Camera cameraFed(SimDevices devices, List<AprilTagDetection> detections) {
        return new Robot(devices.hardware(() -> detections), Alliance.RELATIVE, new FakeTelemetry()).camera;
    }

    /** A detection's age is judged on the robot's clock, the one its frames are stamped with. */
    @Test
    public void aSightingGoesStaleATenthOfASecondAfterItsFrameOnTheRobotsClock() {
        for (int i = 0; i < 3; i++) {
            detections.clear();
            detections.add(goalDetection(BLUE_GOAL_TAG, 10, 20, Math.PI / 2));
            camera.loop();
        }
        assertTrue(camera.sighting().isPresent());
        detections.clear();

        devices.advance(0.09);
        camera.loop();
        assertTrue("still fresh", camera.sighting().isPresent());

        devices.advance(0.02);
        camera.loop();
        assertTrue("stale", camera.sighting().isEmpty());
    }

    @Test
    public void noDetectionsMeansNoPose() {

        camera.loop();

        assertTrue(camera.sighting().isEmpty());
    }

    @Test
    public void threeConsistentGoalDetectionsPlaceTheRobotOnTheField() {
        for (int i = 0; i < 3; i++) {
            detections.clear();
            detections.add(goalDetection(BLUE_GOAL_TAG, 10, 20, Math.PI / 2));
            camera.loop();
        }

        Nav.Pose pose = camera.sighting().get();

        assertEquals(-10, pose.x(), DELTA);
        assertEquals(-20, pose.y(), DELTA);
        assertEquals(0, pose.heading(), DELTA);
    }

    @Test
    public void detectionsOfOtherTagsAreIgnored() {
        for (int i = 0; i < 3; i++) {
            detections.clear();
            detections.add(goalDetection(OTHER_TAG, 10, 20, Math.PI / 2));
            camera.loop();
        }

        assertTrue(camera.sighting().isEmpty());
    }

    private AprilTagDetection goalDetection(int id, double x, double y, double yawRadians) {
        long now = devices.nanoTime();
        Pose3D robotPose = new Pose3D(
                new Position(DistanceUnit.INCH, x, y, 0, now),
                new YawPitchRollAngles(AngleUnit.RADIANS, yawRadians, 0, 0, now));
        return new AprilTagDetection(id, 0, 0, null, null, null, null, null, robotPose, now);
    }
}
