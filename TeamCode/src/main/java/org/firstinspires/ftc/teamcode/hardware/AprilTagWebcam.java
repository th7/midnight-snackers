package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public final class AprilTagWebcam {
    private AprilTagWebcam() {}

    public static Supplier<List<AprilTagDetection>> detections(HardwareMap hardwareMap) {
        AprilTagProcessor.Builder atpb = new AprilTagProcessor.Builder();

        Position cameraPosition = new Position(DistanceUnit.INCH, 0.75, 4, 16, 0);

        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -68, 0, 0);
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
