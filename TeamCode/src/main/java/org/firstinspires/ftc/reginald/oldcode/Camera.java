package org.firstinspires.ftc.reginald;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.firstinspires.ftc.reginald.control.DetectionFilter;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Camera extends SubSystem {
    public static final String CHANNEL = "Camera";

    private final Prints telemetry;
    private final DetectionFilter detectionFilter;
    private final Supplier<List<AprilTagDetection>> detectionSource;
    private AprilTagDetection goalDetection;

    public Camera(Hardware hardware, Prints telemetry) {
        this.detectionSource = hardware.aprilTags();
        this.detectionFilter = new DetectionFilter(hardware.nanoClock());
        this.telemetry = telemetry;
    }

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

    @Override
    public void loop() {
        List<AprilTagDetection> detections = detectionSource.get();

        if (detections == null) {
            return;
        }

        for (AprilTagDetection detection : detections) {
            if (detection.id == 20 || detection.id == 24) {
                detectionFilter.addDetection(detection);
            }
        }

        goalDetection =
                detectionFilter.agreed().map(DetectionFilter.Agreed::detection).orElse(null);

        telemetry.addData("detectionFilter.maxAgeNano", DetectionFilter.MAX_AGE_NANO);
        telemetry.addData(
                "detectionFilter.lastAgeNano",
                detectionFilter.lastAgeNano().isPresent()
                        ? String.valueOf(detectionFilter.lastAgeNano().getAsLong())
                        : "nothing seen yet");
        telemetry.addData("detectionFilter.agreed", detectionFilter.agreed().isPresent());
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
                            sighting.get().xInches(),
                            sighting.get().yInches(),
                            -sighting.get().headingRadians());
                }
            }
        }
    }
}
