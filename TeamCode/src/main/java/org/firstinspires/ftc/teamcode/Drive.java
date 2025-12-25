package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.DriveRunner;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Drive extends SubSystem {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private final MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, zeroPose);
    private final DriveRunner driveRunner = new DriveRunner();
    private AprilTagProcessor aprilTagProcessor;
    private boolean telemetryOn = false;
    private Pose2d savedPose1;
    private Pose2d savedPose2;
    private float straightPower;
    private float strafePower;
    private float turnPower;
    private MoveData moveData;
    private boolean fieldPositionKnown = false;

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
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        setPose(new Pose2d(0, 0, 0));

        telemetry.addData("Drive.init()", true);
//        telemetry.addData("CameraHeight", size.getHeight());
//        telemetry.addData("CameraWidth", size.getWidth());
    }

    public void useDirectPower() {
        MoveData straight = MoveData.straight(straightPower, 0f, 1f);
        MoveData strafe = MoveData.strafe(strafePower, 0f, 1f);
        MoveData turn = MoveData.turn(turnPower, 0f, 1f);
        moveData = straight.add(strafe, turn);

        if (done()) {
            leftFront.setPower(moveData.frontLeftPower);
            rightFront.setPower(moveData.frontRightPower);
            leftBack.setPower(moveData.rearLeftPower);
            rightBack.setPower(moveData.rearRightPower);
        }
    }

    public void setStrafePower(float newStrafePower) {
        strafePower = newStrafePower;
    }

    public void setTurnPower(float newTurnPower) {
        turnPower = newTurnPower;
    }

    public void setStraightPower(float newStraightPower) {
        straightPower = newStraightPower;
    }

    public void loop() {
        mecanumDrive.localizer.update();
        driveRunner.loop();
        if (telemetryOn) {
            setTelemetry();
        }
    }

    private void setTelemetry() {
        telemetry.addData("Drive", "telemetry on");
        Pose2d currentPose = getPose();

        telemetry.addData("fieldPositionKnown", fieldPositionKnown);

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

    }

    public boolean done() {
        return driveRunner.done();
    }

    public void setFieldPosition(Pose2d pose) {
       fieldPositionKnown = true;
       setPose(pose);
    }

    public boolean turnToBlue() {
        if (!fieldPositionKnown) {
            return false;
        }

        Pose2d botPose = mecanumDrive.localizer.getPose();

        double blueAprilTagX = 58.3;
        double blueAprilTagY = 55.6;
        double blueX = blueAprilTagX + 10;
        double blueY = blueAprilTagY + 10;

        double fieldBearingToTarget = Math.atan2(blueY - botPose.position.y, blueX - botPose.position.x);

        double headingErrorRadians = Rotation2d.exp(fieldBearingToTarget).minus(botPose.heading);

        telemetry.addData("headingErrorRadians", headingErrorRadians);

        if (Math.abs(headingErrorRadians) < Math.PI / 80) {
            turnPower = 0;
            return true;
        } else {
            turnPower = clampMinPower(headingErrorRadians / (Math.PI / 6), 0.03);
            return false;
        }
    }

    public boolean driveToBlue() {
        if (!fieldPositionKnown) {
            return false;
        }

        Pose2d botPose = mecanumDrive.localizer.getPose();

        double blueAprilTagX = 58.3;
        double blueAprilTagY = 55.6;
        double blueX = blueAprilTagX + 10;
        double blueY = blueAprilTagY + 10;

        double fieldDistanceToTarget = Math.sqrt(Math.pow(blueY - botPose.position.y, 2) + Math.pow(blueX - botPose.position.x, 2));

        double targetShootingDistance = 40;

        double distanceError = fieldDistanceToTarget - targetShootingDistance;

        telemetry.addData("distanceError", distanceError);

        if (Math.abs(distanceError) < 1) {
            straightPower = 0;
            return true;
        } else {
            straightPower = clampMinPower(distanceError / 20, 0.1);
            return false;
        }
    }

    private float clampMinPower(double power, double min) {
        if (power > 0 && power < min) {
            return (float) min;
        } else if (power < 0 && power > -min) {
            return (float) -min;
        } else {
            return (float) power;
        }
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
