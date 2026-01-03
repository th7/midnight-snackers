package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.LinkedList;

/**
 * Track 3 detections. Only return "clean" detection if all x and y readings are within maxPositionDifference (default 1) and most recent detection is less than maxDetectionAgeSeconds old (default 0.1).
 */
public class DetectionFilter {
    private final LinkedList<AprilTagDetection> storedDetections = new LinkedList<>();
    private final int detectionCount = 3;
    private final int maxPositionDifference;
    private final double maxDetectionAgeSeconds;

    public DetectionFilter() {
        this.maxPositionDifference = 1;
        this.maxDetectionAgeSeconds = 0.1;
    }
    public DetectionFilter(int maxPositionDifference, double maxDetectionAgeSeconds) {
        this.maxPositionDifference = maxPositionDifference;
        this.maxDetectionAgeSeconds = maxDetectionAgeSeconds;
    }

    public void addDetection(AprilTagDetection detection) {
        storedDetections.add(detection);
        while (storedDetections.size() > detectionCount) {
            storedDetections.removeFirst();
        }
    }

    public AprilTagDetection getCleanDetection() {
        if (!dataSeemsClean()) {
            return null;
        }
        return storedDetections.getLast();
    }

    private boolean dataSeemsClean() {
        if (storedDetections.size() < detectionCount) {
            return false;
        }
        return dataIsConsistent() && lastDetectionIsRecent();
    }

    private boolean dataIsConsistent() {
        double minX = 1000000;
        double maxX = -1000000;
        double minY = 1000000;
        double maxY = -1000000;
        for (AprilTagDetection detection : storedDetections) {
            Position position = detection.robotPose.getPosition();
            if (position.x < minX) { minX = position.x; }
            if (position.x > maxX) { maxX = position.x; }
            if (position.y < minY) { minY = position.y; }
            if (position.y > maxY) { maxY = position.y; }
        }

        return Math.abs(maxX - minX) < maxPositionDifference && Math.abs(maxY - minY) < maxPositionDifference;
    }

    private boolean lastDetectionIsRecent() {
        AprilTagDetection lastDetection = storedDetections.getLast();
        return lastDetection == null || !(System.nanoTime() - lastDetection.frameAcquisitionNanoTime > maxDetectionAgeSeconds * 1_000_000_000);
    }
}
