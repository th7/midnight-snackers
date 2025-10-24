package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.DriveRunner;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Drive extends SubSystem {
    private final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private final MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, zeroPose);
    private final DriveRunner driveRunner = new DriveRunner();

//    private final AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

    public Drive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
//        new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTagProcessor)
//                .build();
        telemetry.addData("Drive.init()", true);
    }

    public void loop() {
        driveRunner.loop();
        telemetry.addData("driveRunner.done()", driveRunner.done());
        telemetry.addData("drive.loop()", true);
        Pose2d currentPose = getPose();
        telemetry.addData("Coordinates x: ", currentPose.position.x);
        telemetry.addData("Coordinates y: ", currentPose.position.y);
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

    private Pose2d getPose() {
        return mecanumDrive.localizer.getPose();
    }

    public void setPose(Pose2d pose) {
        mecanumDrive.localizer.setPose(pose);
    }

//    private void aprilTagTelemetry() {
//        // detections may be stale, use getFreshDetections() for latest detections since last processed
//        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//            String tag = "detection." + detection.id;
//            telemetry.addData(tag, true);
//
//            if (detection.ftcPose == null) {
//                telemetry.addData(tag + ".ftcPose", null);
//            } else {
//                telemetry.addData(tag + ".ftcPose.range", detection.ftcPose.range);
//                telemetry.addData(tag + ".ftcPose.yaw", detection.ftcPose.yaw);
//                telemetry.addData(tag + ".ftcPose.bearing", detection.ftcPose.bearing);
//            }
//
//            if (detection.metadata == null) {
//                telemetry.addData(tag + ".metadata", "null");
//                continue;
//            }
//
//            if (detection.metadata.fieldPosition == null) {
//                telemetry.addData(tag + ".metadata.fieldPosition", "null");
//            } else {
//                float[] fieldPositionData = detection.metadata.fieldPosition.getData();
//                if (fieldPositionData == null) {
//                    telemetry.addData(tag + ".metadata.fieldPosition.getData()", "null");
//                } else {
//                    telemetry.addData(tag + ".metadata.fieldPosition.data[0]", fieldPositionData[0]);
//                    telemetry.addData(tag + ".metadata.fieldPosition.data[1]", fieldPositionData[1]);
//                    telemetry.addData(tag + ".metadata.fieldPosition.data[2]", fieldPositionData[2]);
//                }
//            }
//        }
//    }
}
