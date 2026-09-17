package org.firstinspires.ftc.teamcode.control;

import java.util.LinkedList;
import java.util.Optional;
import java.util.OptionalLong;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Answers one question about the last three detections: have they <b>agreed</b>? They have when all
 * three place the robot within {@value #MAX_POSITION_DIFFERENCE_INCHES} inch of one another and the
 * newest is less than {@value #MAX_AGE_NANO} nanoseconds old on the robot's clock.
 *
 * <p>Every way of asking is total. A filter that has seen nothing answers that it has seen nothing,
 * rather than throwing: the driver who turns the camera's telemetry on before the robot has looked
 * at anything is the ordinary case, not a misuse, and the answer it wants is exactly the one an
 * empty filter has.
 */
public class DetectionFilter {
    /** How old the newest of three agreeing detections may be: a tenth of a second. */
    public static final long MAX_AGE_NANO = 100_000_000L;

    private static final int DETECTION_COUNT = 3;
    private static final int MAX_POSITION_DIFFERENCE_INCHES = 1;

    private final LinkedList<AprilTagDetection> storedDetections = new LinkedList<>();
    private final LongSupplier clock;

    /** Three detections that agree: the newest of them, and how old it is on the robot's clock. */
    public record Agreed(AprilTagDetection detection, long ageNano) {}

    /**
     * @param clock the clock the detections' {@code frameAcquisitionNanoTime} is on: the robot's
     */
    public DetectionFilter(LongSupplier clock) {
        this.clock = clock;
    }

    public void addDetection(AprilTagDetection detection) {
        storedDetections.add(detection);
        while (storedDetections.size() > DETECTION_COUNT) {
            storedDetections.removeFirst();
        }
    }

    /**
     * The newest detection, once three consistent ones agree and the newest is still fresh; empty
     * until then, and empty again once it goes stale.
     */
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

    /**
     * How old the newest detection is, agreement or not, and empty only when nothing has been seen
     * at all: it is what a driver watching telemetry needs exactly when nothing is agreeing.
     */
    public OptionalLong lastAgeNano() {
        if (storedDetections.isEmpty()) {
            return OptionalLong.empty();
        }
        return OptionalLong.of(ageOf(storedDetections.getLast()));
    }

    private long ageOf(AprilTagDetection detection) {
        return clock.getAsLong() - detection.frameAcquisitionNanoTime;
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
