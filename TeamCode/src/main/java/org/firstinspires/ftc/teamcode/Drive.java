package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.base.DriveRunner;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Drive extends SubSystem {
    private final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private final MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, zeroPose);
    private final DriveRunner driveRunner = new DriveRunner();
    private AprilTagProcessor aprilTagProcessor;
    private boolean telemetryOn = false;
    private Pose2d savedPose1;
    private Pose2d savedPose2;

    public Drive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
//        AprilTagProcessor.Builder atpb = new AprilTagProcessor.Builder();
//
////        Robot axes: (this is typical, but you can define this however you want)
////
////        Origin location: Center of the robot at field height
////
////        Axes orientation: +x right, +y forward, +z upward
////
////        Position:
////
////        If all values are zero (no translation), that implies the camera is at the center of the robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12 inches above the ground - you would need to set the position to (-5, 7, 12).
//        Position cameraPosition = new Position(DistanceUnit.INCH,
//                5, 5, 5, 0);
//
////        Orientation:
////
////        If all values are zero (no rotation), that implies the camera is pointing straight up. In most cases, you’ll need to set the pitch to -90 degrees (rotation about the x-axis), meaning the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if it’s pointing straight left, -90 degrees for straight right, etc. You can also set the roll to +/-90 degrees if it’s vertical, or 180 degrees if it’s upside-down.
//        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//                0, -75, 0, 0);
//        atpb.setCameraPose(cameraPosition, cameraOrientation);
//        aprilTagProcessor = atpb.build();
//
//        VisionPortal.Builder vpb = new VisionPortal.Builder();
//        vpb.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        vpb.setCameraResolution(new Size(640, 480));
//        vpb.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//        vpb.addProcessor(aprilTagProcessor);
//        vpb.build();
//        Size size = vpb.getActiveCamera().getCameraCharacteristics().getDefaultSize(0);
        telemetry.addData("Drive.init()", true);
//        telemetry.addData("CameraHeight", size.getHeight());
//        telemetry.addData("CameraWidth", size.getWidth());


    }

    public void loop() {
        mecanumDrive.localizer.update();
        driveRunner.loop();
        if (telemetryOn) {
            setTelemetry();
        }
    }

    private void setTelemetry() {
        Pose2d currentPose = getPose();
        telemetry.addData("current x,y,h", "%.04f,%.04f,%.04f", currentPose.position.x, currentPose.position.y, currentPose.heading.real);
        if (savedPose1 != null) {
            telemetry.addData("Saved 1 x,y,h", "%.04f,%.04f,%.04f", savedPose1.position.x, savedPose1.position.y, savedPose1.heading.real);
        } else {
            telemetry.addData("Saved 1", null);
        }
        if (savedPose2 != null) {
            telemetry.addData("Saved 2 x,y,h", "%.04f,%.04f,%.04f", savedPose2.position.x, savedPose2.position.y, currentPose.heading.real);
        } else {
            telemetry.addData("Saved 2", null);
        }
        aprilTagTelemetry();
    }
    
    public boolean done() {
        return driveRunner.done();
    }

    public void drivePathForward(Pose2d... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(getPose());
        for (Pose2d pose : poseList) {
            builder = builder.splineToSplineHeading(pose, 0);
        }
        driveRunner.drive(builder.build());
    }

    public void drivePathBackward(Pose2d... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(getPose());
        for (Pose2d pose : poseList) {
            builder = builder.setReversed(true).splineToSplineHeading(pose, Math.PI);
        }
        driveRunner.drive(builder.build());
    }

    public void strafePath(Pose2d... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(getPose());
        for (Pose2d pose : poseList) {
            builder = builder.strafeToSplineHeading(pose.position, pose.heading);
        }
        driveRunner.driveOverride(builder.build());
    }

    public void splineSquiggleSquare() {
        drivePathForward(
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
        drivePathForward(
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
        if (savedPose1 != null) {
            strafePath(savedPose1);
        }
    }

    public void savePose2() {
        savedPose2 = getPose();
    }

    public void goToPose2() {
        if (savedPose2 != null) {
            strafePath(savedPose2);
        }
    }

    private Pose2d getPose() {
        return mecanumDrive.localizer.getPose();
    }

    public void setPose(Pose2d pose) {
        mecanumDrive.localizer.setPose(pose);
    }

    private void aprilTagTelemetry() {
        if (aprilTagProcessor == null) {
            return;
        }
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections == null) {
            return;
        }

        for (AprilTagDetection detection : currentDetections) {
            String tag = "detection." + detection.id;

            if (detection.robotPose == null) {
                telemetry.addData(tag + ".robotPose", null);
            } else {
                YawPitchRollAngles orientation = detection.robotPose.getOrientation();
                telemetry.addData(tag + ".robotPose.getOrientation().getYaw()", detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
                double roadrunnerYaw = orientation.getYaw(AngleUnit.RADIANS) - Math.PI / 2;
                telemetry.addData(tag + " roadrunnerYaw", roadrunnerYaw);
                Position position = detection.robotPose.getPosition();
                telemetry.addData(tag + ".robotPose.getPosition() x,y,z", "%.04f,%.04f,%.04f", position.x, position.y, position.z);
                double roadRunnerX = position.x * -1;
                double roadRunnerY = position.y * -1;
                telemetry.addData(tag + " roodrunner x,y", "%.04f,%.04f", roadRunnerX, roadRunnerY);
            }
        }
    }

    public Plans.Motif motif() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections == null) {
            return null;
        }

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

    public void cancel() {
        driveRunner.cancel();
    }
}
