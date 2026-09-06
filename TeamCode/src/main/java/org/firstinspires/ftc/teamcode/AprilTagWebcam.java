package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.function.Supplier;

/**
 * The physical webcam and its AprilTag pipeline. Only valid on the robot controller; everything
 * downstream consumes the detection supplier so it can be fed from a test or a simulation instead.
 */
public final class AprilTagWebcam {
    private AprilTagWebcam() {
    }

    public static Supplier<List<AprilTagDetection>> detections(HardwareMap hardwareMap) {
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
                0.75, 4, 16, 0);

//        Orientation:
//
//        If all values are zero (no rotation), that implies the camera is pointing straight up. In most cases, you’ll need to set the pitch to -90 degrees (rotation about the x-axis), meaning the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if it’s pointing straight left, -90 degrees for straight right, etc. You can also set the roll to +/-90 degrees if it’s vertical, or 180 degrees if it’s upside-down.
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -68, 0, 0);
        atpb.setCameraPose(cameraPosition, cameraOrientation);
        AprilTagProcessor aprilTagProcessor = atpb.build();

        VisionPortal.Builder vpb = new VisionPortal.Builder();
        vpb.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        vpb.setCameraResolution(new Size(640, 480));
        vpb.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        vpb.addProcessor(aprilTagProcessor);
        vpb.build();

        return aprilTagProcessor::getDetections;
    }
}
