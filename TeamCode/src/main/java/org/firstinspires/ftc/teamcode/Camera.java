package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Camera extends SubSystem {
    private AprilTagProcessor aprilTagProcessor;
    private boolean telemetryOn = false;
    private AprilTagDetection goalDetection;

    public Camera(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    @Override
    public void init() {
        AprilTagProcessor.Builder atpb = new AprilTagProcessor.Builder();

//        Robot axes: (this is typical, but you can define this however you want)
//
//        Origin location: Center of the robot at field height
//
//        Axes orientation: +x right, +y forward, +z upward
//
//        Position:
//
//        If all values are zero (no translation), that implies the camera is at the center of the robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12 inches above the ground - you would need to set the position to (-5, 7, 12).
        Position cameraPosition = new Position(DistanceUnit.INCH,
                -3.5, 0, 16.5, 0);

//        Orientation:
//
//        If all values are zero (no rotation), that implies the camera is pointing straight up. In most cases, you’ll need to set the pitch to -90 degrees (rotation about the x-axis), meaning the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if it’s pointing straight left, -90 degrees for straight right, etc. You can also set the roll to +/-90 degrees if it’s vertical, or 180 degrees if it’s upside-down.
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                -10, -62, 2, 0);
        atpb.setCameraPose(cameraPosition, cameraOrientation);
        aprilTagProcessor = atpb.build();

        VisionPortal.Builder vpb = new VisionPortal.Builder();
        vpb.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        vpb.setCameraResolution(new Size(640, 480));
        vpb.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        vpb.addProcessor(aprilTagProcessor);
        vpb.build();

        telemetry.addData("Camera.init()", true);
    }

    @Override
    public void loop() {
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections == null) {
            return;
        }

        for (AprilTagDetection detection : detections) {
            if (detection.id == 20 || detection.id == 24) {
                goalDetection = detection;
            }
        }

        if (goalDetection == null) {
            return;
        }

        if (System.nanoTime() - goalDetection.frameAcquisitionNanoTime > 0.1 * 1_000_000_000) {
            goalDetection = null;
        }

        if (telemetryOn) {
            setTelemetry();
        }

//        for (Candle candle : candlesOnTable()) {
//            candle.turnOn();
//        }

    }

    private void setTelemetry() {
        telemetry.addData("Camera", "telemetry on");
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
