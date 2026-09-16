package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.List;
import java.util.Optional;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.control.DetectionFilter;
import org.firstinspires.ftc.teamcode.hardware.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Camera extends SubSystem {
    private final DetectionFilter detectionFilter;
    private final Supplier<List<AprilTagDetection>> detectionSource;
    private AprilTagDetection goalDetection;

    /**
     * @param detectionSource the latest AprilTag detections; {@link AprilTagWebcam#detections} on the robot.
     * @param clock the robot's clock, which the detections' frames are stamped on, to judge their age
     */
    public Camera(Supplier<List<AprilTagDetection>> detectionSource, LongSupplier clock) {
        this.detectionSource = detectionSource;
        this.detectionFilter = new DetectionFilter(clock);
    }

    @Override
    protected void onInit() {}

    @Override
    protected void onLoop() {
        List<AprilTagDetection> detections = detectionSource.get();

        if (detections == null) {
            return;
        }

        for (AprilTagDetection detection : detections) {
            if (detection.id == 20 || detection.id == 24) {
                detectionFilter.addDetection(detection);
            }
        }

        goalDetection = detectionFilter.getCleanDetection();
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("detectionFilter.maxDetectionAgeNano", detectionFilter.maxDetectionAgeNano);
        telemetry.addData("detectionFilter.lastDetectionAgeNano", detectionFilter.lastDetectionAgeNano());
        telemetry.addData("detectionFilter.lastDetectionIsRecent", detectionFilter.lastDetectionIsRecent());
        if (goalDetection != null) {
            telemetry.addData("goalDetection.id", goalDetection.id);
            if (goalDetection.ftcPose != null) {
                telemetry.addData("goalDetection.ftcPose.bearing", goalDetection.ftcPose.bearing);
                telemetry.addData("goalDetection.ftcPose.elevation", goalDetection.ftcPose.elevation);
                telemetry.addData("goalDetection.ftcPose.range", goalDetection.ftcPose.range);
                telemetry.addData("goalDetection.ftcPose.yaw", goalDetection.ftcPose.yaw);
                telemetry.addData("goalDetection.ftcPose.pitch", goalDetection.ftcPose.pitch);
                telemetry.addData("goalDetection.ftcPose.roll", goalDetection.ftcPose.roll);
            }

            if (goalDetection.robotPose != null) {
                Position position = goalDetection.robotPose.getPosition();
                YawPitchRollAngles orientation = goalDetection.robotPose.getOrientation();
                if (position != null && orientation != null) {
                    telemetry.addData(
                            "ftc x, y, h(rads)",
                            "%.02f, %.02f, %.02f",
                            position.x,
                            position.y,
                            orientation.getYaw(AngleUnit.RADIANS));
                }

                Optional<Nav.Pose> sighting = sighting();
                if (sighting.isPresent()) {
                    telemetry.addData(
                            "roadrunner x, y, h(rads)",
                            "%.02f, %.02f, %.02f",
                            sighting.get().x(),
                            sighting.get().y(),
                            -sighting.get().heading());
                }
            }
        }
    }

    /**
     * Where the goal's tag says the robot is on the field, as the camera faces: the turntable's
     * heading, not the robot's. Empty until three consistent recent detections agree.
     */
    public Optional<Nav.Pose> sighting() {
        if (goalDetection == null || goalDetection.robotPose == null) {
            return Optional.empty();
        }
        Position position = goalDetection.robotPose.getPosition();
        YawPitchRollAngles orientation = goalDetection.robotPose.getOrientation();
        if (position == null || orientation == null) {
            return Optional.empty();
        }
        return Optional.of(new Nav.Pose(
                new Pose2d(-position.x, -position.y, orientation.getYaw(AngleUnit.RADIANS) - Math.PI / 2)));
    }
}
