package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.base.DriveRunner;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Drive extends SubSystem {
    private final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private final MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, zeroPose);
    private final DriveRunner driveRunner = new DriveRunner();
    private final AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
    private boolean telemetryOn = false;
    private Pose2d savedPose1;
    private Pose2d savedPose2;

    public Drive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        VisionPortal.Builder vpb = new VisionPortal.Builder();
        vpb.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        vpb.setCameraResolution(new Size(640, 480));
        vpb.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        vpb.addProcessor(aprilTagProcessor);
        vpb.build();
//        Size size = vpb.getActiveCamera().getCameraCharacteristics().getDefaultSize(0);
        telemetry.addData("Drive.init()", true);
//        telemetry.addData("CameraHeight", size.getHeight());
//        telemetry.addData("CameraWidth", size.getWidth());


    }

    public void loop() {
        driveRunner.loop();
        if (telemetryOn) {
            setTelemetry();
        }
    }

    private void setTelemetry() {
        Pose2d currentPose = getPose();
        telemetry.addData("Coordinates x: ", currentPose.position.x);
        telemetry.addData("Coordinates y: ", currentPose.position.y);
        aprilTagTelemetry();
    }

    @Override
    public boolean done() {
        return driveRunner.done();
    }

    public void drivePath(Pose2d... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(getPose());
        for (Pose2d pose : poseList) {
            builder = builder.splineToSplineHeading(pose, 0);
        }
        driveRunner.drive(builder.build());
    }

    public void splineSquiggleSquare() {
        drivePath(
                new Pose2d(72, 0, 0),
                new Pose2d(96, 0, Math.PI / 4),
                new Pose2d(96, 24, Math.PI * 3 / 4),
                new Pose2d(72, 24, Math.PI * 1),
                new Pose2d(48, 24, Math.PI * 1),
                new Pose2d(24, 24, Math.PI * 3 / 4),
                new Pose2d(24, 48, Math.PI * 3 / 4),
                new Pose2d(0, 48, Math.PI * -3 / 4),
                new Pose2d(0, 24, Math.PI / -2),
                new Pose2d(0, 0, 0)
        );
    }

    public void toZeroPosition() {
        drivePath(
                new Pose2d(0, 0, 0)
        );
    }

    public void toggleTelemetry() {
        telemetryOn = !telemetryOn;
    }
    
    public void savePose1() {
        savedPose1 = getPose();
    }
    public void goToPose1() {
        drivePath(savedPose1);
    }

    public void savePose2() {
        savedPose2 = getPose();
    }
    public void goToPose2() {
        drivePath(savedPose2);
    }
    
    private Pose2d getPose() {
        return mecanumDrive.localizer.getPose();
    }

    public void setPose(Pose2d pose) {
        mecanumDrive.localizer.setPose(pose);
    }

    private void aprilTagTelemetry() {
        // detections may be stale, use getFreshDetections() for latest detections since last processed
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections == null) { return; }

        for (AprilTagDetection detection : currentDetections) {
            String tag = "detection." + detection.id;

            if (detection.ftcPose == null) {
                telemetry.addData(tag + ".ftcPose", null);
            } else {
                telemetry.addData(tag + ".ftcPose.range", detection.ftcPose.range);
                telemetry.addData(tag + ".ftcPose.yaw", detection.ftcPose.yaw);
                telemetry.addData(tag + ".ftcPose.bearing", detection.ftcPose.bearing);
                telemetry.addData(tag + ".ftcPose.x", detection.ftcPose.x);
                telemetry.addData(tag + ".ftcPose.y", detection.ftcPose.y);
            }

            if (detection.robotPose == null) {
                telemetry.addData(tag + ".robotPose", null);
            } else {
                telemetry.addData(tag + ".robotPose.getOrientation().getYaw()", detection.robotPose.getOrientation().getYaw());
//                telemetry.addData(tag + ".robotPose.getPosition().x", detection.robotPose.getPosition().x);
//                telemetry.addData(tag + ".robotPose.getPosition().y", detection.robotPose.getPosition().y);
            }

            if (detection.metadata == null) {
                telemetry.addData(tag + ".metadata", "null");
                continue;
            }

            if (detection.metadata.fieldPosition == null) {
                telemetry.addData(tag + ".metadata.fieldPosition", "null");
            } else {
                float[] fieldPositionData = detection.metadata.fieldPosition.getData();
                if (fieldPositionData == null) {
                    telemetry.addData(tag + ".metadata.fieldPosition.getData()", "null");
                } else {
                    telemetry.addData(tag + ".metadata.fieldPosition.data[0]", fieldPositionData[0]);
                    telemetry.addData(tag + ".metadata.fieldPosition.data[1]", fieldPositionData[1]);
//                    telemetry.addData(tag + ".metadata.fieldPosition.data[2]", fieldPositionData[2]);

                    if (detection.ftcPose != null) {
//                        double sinBearing = Math.sin(Math.toRadians(detection.ftcPose.bearing));
//                        double cosBearing = Math.cos(Math.toRadians(detection.ftcPose.bearing));
                        double relativeX = detection.ftcPose.y;
                        double relativeY = detection.ftcPose.x;
                        telemetry.addData(tag + ".x", fieldPositionData[0] * -1 - relativeX);
                        telemetry.addData(tag + ".y", fieldPositionData[1] * -1 - relativeY);
//                        telemetry.addData(tag + ".sinBearing", sinBearing);
//                        telemetry.addData(tag + ".cosBearing", cosBearing);
                    }
                }
            }
        }
    }

    public Plans.Motif motif() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections == null) { return null; }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 21) {
                return Plans.Motif.GPP;
            }
            if (detection.id == 22) {
                return Plans.Motif.PGP;
            }
            if (detection.id == 23) {
                return Plans.Motif.PPG;
            }
        }
        return null;
    }
}
