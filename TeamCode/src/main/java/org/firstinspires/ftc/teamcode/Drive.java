package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
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
    public enum Alliance {
        blue(1),
        red(-1);

        public final int side;

        Alliance(int side) {
            this.side = side;
        }
    }
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
    }

    @Override
    public boolean done() {
        return false;
    }

    public void drivePath(double[]... coordsList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(getPose());
        for (double[] coords : coordsList) {
            builder = builder.splineToSplineHeading(new Pose2d(coords[0], coords[1], coords[2]), coords[2]);
        }
        Action action = builder.build();
        driveRunner.drive(action);
    }

    public void drivePath(Pose2d... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(getPose());
        for (Pose2d pose : poseList) {
            builder = builder.splineToSplineHeading(pose, 0);
        }
        driveRunner.drive(builder.build());
    }

    public void splineUsingCoords() {
        drivePath(
                new double[]{24, 24, Math.PI / -2},
                new double[]{0, 48, Math.PI * -1}
        );
    }

    public void splineOneStep() {
        drivePath(
                new Pose2d(24, 24, Math.PI / 2)
        );
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

    public void splineUsingPoses() {
        drivePath(
                new Pose2d(24, 24, Math.PI / 2),
                new Pose2d(0, 48, Math.PI * 1)
        );
    }

    public void blueFarScoreAThing() {
        farScoreAThing(Alliance.blue);
    }

    public void redFarScoreAThing() {
        farScoreAThing(Alliance.red);
    }

    public void farScoreAThing(Alliance alliance) {
        drivePath(
                new Pose2d(72, 0, Math.PI / 4 * alliance.side),
                new Pose2d(73, -1, Math.PI / 4 * alliance.side)
        );
    }

    public void splineBasic() {

        Action action = mecanumDrive.actionBuilder(getPose())
                .splineToSplineHeading(new Pose2d(24, 24, Math.PI / -2), Math.PI / -2)
                .splineToSplineHeading(new Pose2d(0, 48, Math.PI * -1), Math.PI * -1)
                .build();
        driveRunner.drive(action);
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
