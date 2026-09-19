package org.firstinspires.ftc.teamcode.control;

import java.util.LinkedList;
import java.util.Optional;
import java.util.OptionalLong;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class DetectionFilter {
    public static final long MAX_AGE_NANO = 100_000_000L;

    private static final int DETECTION_COUNT = 3;
    private static final int MAX_POSITION_DIFFERENCE_INCHES = 1;

    private final LinkedList<AprilTagDetection> storedDetections = new LinkedList<>();
    private final LongSupplier nanoClock;

    public record Agreed(AprilTagDetection detection, long ageNano) {}

    public DetectionFilter(LongSupplier nanoClock) {
        this.nanoClock = nanoClock;
    }

    public void addDetection(AprilTagDetection detection) {
        storedDetections.add(detection);
        while (storedDetections.size() > DETECTION_COUNT) {
            storedDetections.removeFirst();
        }
    }

    public Optional<Agreed> agreed() {
        if (storedDetections.size() < DETECTION_COUNT || !dataIsConsistent()) {
            return Optional.empty();
        }
        long ageNano = ageOf(storedDetections.getLast());
        if (ageNano >= MAX_AGE_NANO) {
            return Optional.empty();
        }
        return Optional.of(new Agreed(storedDetections.getLast(), ageNano));
    }

    public OptionalLong lastAgeNano() {
        if (storedDetections.isEmpty()) {
            return OptionalLong.empty();
        }
        return OptionalLong.of(ageOf(storedDetections.getLast()));
    }

    private long ageOf(AprilTagDetection detection) {
        return nanoClock.getAsLong() - detection.frameAcquisitionNanoTime;
    }

    private boolean dataIsConsistent() {
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        for (AprilTagDetection detection : storedDetections) {
            if (detection.robotPose == null) {
                return false;
            }
            Position position = detection.robotPose.getPosition();
            if (position == null) {
                return false;
            }

            minX = Math.min(minX, position.x);
            maxX = Math.max(maxX, position.x);
            minY = Math.min(minY, position.y);
            maxY = Math.max(maxY, position.y);
        }

        return Math.abs(maxX - minX) < MAX_POSITION_DIFFERENCE_INCHES
                && Math.abs(maxY - minY) < MAX_POSITION_DIFFERENCE_INCHES;
    }
}
