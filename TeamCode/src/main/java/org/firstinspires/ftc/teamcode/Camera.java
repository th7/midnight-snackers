package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.base.DetectionFilter;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.function.Supplier;

public class Camera extends SubSystem {
    private final DetectionFilter detectionFilter = new DetectionFilter();
    private final Supplier<List<AprilTagDetection>> detectionSource;
    private boolean telemetryOn = false;
    private AprilTagDetection goalDetection;

    /**
     * @param detectionSource the latest AprilTag detections; {@link AprilTagWebcam#detections} on the robot.
     */
    public Camera(Supplier<List<AprilTagDetection>> detectionSource, ElapsedTime runtime, Telemetry telemetry) {
        super(runtime, telemetry);
        this.detectionSource = detectionSource;
    }

    @Override
    public void init() {
        telemetry.addData("Camera.init()", true);
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

        goalDetection = detectionFilter.getCleanDetection();

        if (telemetryOn) {
            setTelemetry();
        }
    }

    private void setTelemetry() {
        telemetry.addData("Camera", "telemetry on");
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
                    telemetry.addData("ftc x, y, h(rads)", "%.02f, %.02f, %.02f", position.x, position.y, orientation.getYaw(AngleUnit.RADIANS));
                }

                Pose2d currentPose = calculateRoadrunnerPose();
                if (currentPose != null) {
                    double headingRadians = Rotation2d.exp(0).minus(currentPose.heading);
                    telemetry.addData("roadrunner x, y, h(rads)", "%.02f, %.02f, %.02f", currentPose.position.x, currentPose.position.y, headingRadians);
                }
            }
        }
    }

    public Pose2d calculateRoadrunnerPose() {
        if (goalDetection == null) {
            return null;
        }

        Pose3D robotPose = goalDetection.robotPose;

        if (robotPose == null) {
            return null;
        }

        Position position = robotPose.getPosition();

        if (position == null) {
            return null;
        }

        YawPitchRollAngles orientation = robotPose.getOrientation();

        if (orientation == null) {
            return null;
        }

        return new Pose2d(-position.x, -position.y, orientation.getYaw(AngleUnit.RADIANS) - Math.PI / 2);
    }

    public void toggleTelemetry() {
        telemetryOn = !telemetryOn;
    }
}
