package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
    // COORDINATES!!! ARGH
    // Imagine facing the field from the audience. Blue goal is forward left, red goal is forward right. Any further mention of left/right or forward/backward is relative to this perspective.

    // FTC Coordinates
    // +x backward, +y right, straight forward heading is PI/2

    // Roadrunner Coordinates (this is what we use)
    // +x forward, +y left, straight forward heading is 0

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private final MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, zeroPose);
    private final DriveRunner driveRunner = new DriveRunner();
    private AprilTagProcessor aprilTagProcessor;
    private boolean telemetryOn = false;
    private Pose2d currentPose;
    private Pose2d lastPose;
    private float straightPower;
    private float strafePower;
    private float turnPower;
    private MoveData moveData;
    private boolean fieldPositionKnown = false;
    private int fieldPositionUpdated;
    private double xChange;
    private double yChange;
    private double headingChange;

    private final double nearlyStoppedMaxInchesPerSecond = 3;
    private final double nearlyStoppedMaxRadiansPerSecond = Math.toRadians(10);
    private final double assumedTicksPerSecond = 50;

    private final double nearlyStoppedInchesPerTick = nearlyStoppedMaxInchesPerSecond / assumedTicksPerSecond;
    private final double nearlyStoppedDegreesPerTick = nearlyStoppedMaxRadiansPerSecond / assumedTicksPerSecond;

    private final double blueAprilTagX = 58.3;
    private final double blueAprilTagY = 55.6;
    private final double blueLaunchTargetX = blueAprilTagX + 8;
    private final double blueLaunchTargetY = blueAprilTagY + 8;
    private final double targetLaunchDistance = 40;

    public Drive(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        super(hardwareMap, runtime, telemetry);
    }

    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        setPose(zeroPose);

        currentPose = zeroPose;
        lastPose = zeroPose;

        telemetry.addData("Drive.init()", true);
    }

    public void loop() {
        mecanumDrive.localizer.update();
        currentPose = mecanumDrive.localizer.getPose();
        updateChanges();
        driveRunner.loop();
        if (telemetryOn) {
            setTelemetry();
        }

        lastPose = currentPose;
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

    private void updateChanges() {
        xChange = currentPose.position.x - lastPose.position.x;
        yChange = currentPose.position.y - lastPose.position.y;
        headingChange = currentPose.heading.minus(lastPose.heading);
    }
    private void setTelemetry() {
        telemetry.addData("Drive", "telemetry on");

        telemetry.addData("fieldPositionKnown", fieldPositionKnown);
        telemetry.addData("fieldPositionUpdated", fieldPositionUpdated);

        double headingRadians = Rotation2d.exp(0).minus(currentPose.heading);
        telemetry.addData("current x, y, h(rads)", "%.02f, %.02f, %.02f", currentPose.position.x, currentPose.position.y, headingRadians);
        telemetry.addData("changes x, y, h(rads)", "%.02f, %.02f, %.02f", xChange, yChange, headingChange);
//        if (savedPose1 != null) {
//            telemetry.addData("Saved 1 x,y,h", "%.04f,%.04f,%.04f", savedPose1.position.x, savedPose1.position.y, savedPose1.heading.real);
//        } else {
//            telemetry.addData("Saved 1", null);
//        }
//        if (savedPose2 != null) {
//            telemetry.addData("Saved 2 x,y,h", "%.04f,%.04f,%.04f", savedPose2.position.x, savedPose2.position.y, currentPose.heading.real);
//        } else {
//            telemetry.addData("Saved 2", null);
//        }

    }

    public boolean done() {
        return driveRunner.done();
    }

    public void setFieldPosition(Pose2d pose) {
        if (!fieldPositionKnown) {
            fieldPositionKnown = true;
            setPose(pose);
            return;
        }

        double xError = pose.position.x - currentPose.position.x;
        double yError = pose.position.y - currentPose.position.y;

        if (xError > 1) {
            xError = 1;
        } else if (xError < -1) {
            xError = -1;
        }

        if (yError > 1) {
            yError = 1;
        } else if (yError < -1) {
            yError = -1;
        }

        Vector2d adjustedPosition = new Vector2d(currentPose.position.x + xError, currentPose.position.y + yError);
        setPose(new Pose2d(adjustedPosition, currentPose.heading));
        fieldPositionUpdated = fieldPositionUpdated + 1;
    }

    public boolean nearlyStopped() {
        return !(Math.abs(xChange) > nearlyStoppedInchesPerTick) && !(Math.abs(yChange) > nearlyStoppedInchesPerTick) && !(Math.abs(headingChange) > nearlyStoppedDegreesPerTick);
    }

    public boolean turnToBlue() {
        if (!fieldPositionKnown) {
            return false;
        }

        double fieldBearingToTarget = Math.atan2(blueLaunchTargetY - currentPose.position.y, blueLaunchTargetX - currentPose.position.x);
        double headingErrorRadians = Rotation2d.exp(fieldBearingToTarget).minus(currentPose.heading);

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

        double fieldDistanceToTarget = Math.sqrt(Math.pow(blueLaunchTargetY - currentPose.position.y, 2) + Math.pow(blueLaunchTargetX - currentPose.position.x, 2));
        double distanceError = fieldDistanceToTarget - targetLaunchDistance;

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
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(currentPose);
        for (Pose2d pose : poseList) {
            builder = builder.splineToSplineHeading(pose, 0);
        }
        driveRunner.drive(builder.build());
    }

    public void drivePathBackward(Pose2d... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(currentPose);
        for (Pose2d pose : poseList) {
            builder = builder.setReversed(true).splineToSplineHeading(pose, Math.PI);
        }
        driveRunner.drive(builder.build());
    }

    public void strafePath(Pose2d... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(currentPose);
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

    public void forwardLeftBackwardRight() {
        strafePath(
                new Pose2d(24, 0, 0),
                new Pose2d(24, 24, 0),
                new Pose2d(0, 24, 0),
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
