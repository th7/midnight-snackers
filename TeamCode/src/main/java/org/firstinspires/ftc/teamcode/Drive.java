package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Dashboard;
import org.firstinspires.ftc.teamcode.base.DriveRunner;
import org.firstinspires.ftc.teamcode.base.FastDrive;
import org.firstinspires.ftc.teamcode.base.MoveData;
import org.firstinspires.ftc.teamcode.base.SubSystem;

import java.util.function.Supplier;

public class Drive extends SubSystem {
    private final DriveRunner driveRunner;
    private final FastDrive fastDrive = new FastDrive();
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;
    private boolean telemetryOn = false;
    private float straightPower;
    private float strafePower;
    private float turnPower;

    public Drive(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, Dashboard dashboard, ElapsedTime runtime, Telemetry telemetry) {
        super(runtime, telemetry);
        this.driveRunner = new DriveRunner(dashboard);
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }

    public void init() {
        telemetry.addData("Drive.init()", true);
    }

    /**
     * Supplies the robot's current pose so the dashboard field view shows the robot
     * whenever the op mode is running, not only during RoadRunner actions.
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        driveRunner.setPoseSupplier(poseSupplier);
    }

    public void loop() {
        driveRunner.loop();
        if (telemetryOn) {
            setTelemetry();
        }
    }

    public boolean fastDriveTo(Nav.Pose to, Nav.Pose current) {
        fastDrive.setDestination(to.pose2d);
        fastDrive.update(current.pose2d);

        straightPower = fastDrive.straightPower();
        strafePower = fastDrive.strafePower();
        turnPower = fastDrive.turnPower();

        return fastDrive.doneMoving();
    }

    public void useDirectPower() {
        MoveData straight = MoveData.straight(straightPower, 0f, 1f);
        MoveData strafe = MoveData.strafe(strafePower, 0f, 1f);
        MoveData turn = MoveData.turn(turnPower, 0f, 1f);
        MoveData moveData = straight.add(strafe, turn);

        if (done()) {
            leftFront.setPower(moveData.frontLeftPower());
            rightFront.setPower(moveData.frontRightPower());
            leftBack.setPower(moveData.rearLeftPower());
            rightBack.setPower(moveData.rearRightPower());
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

    private void setTelemetry() {
        telemetry.addData("Drive", "telemetry on");

        Pose2d error = fastDrive.error();
        if (error != null) {
            telemetry.addData("fastDriveError.x", error.position.x);
            telemetry.addData("fastDriveError.y", error.position.y);
            telemetry.addData("fastDriveError.h", Rotation2d.exp(0).minus(error.heading));
        }

        telemetry.addData("fastDriveStraightPower", fastDrive.straightPower());
        telemetry.addData("fastDriveStrafePower", fastDrive.strafePower());
        telemetry.addData("fastDriveTurnPower", fastDrive.turnPower());
        telemetry.addData("fastDrive.atDestination();", fastDrive.doneMoving());
        telemetry.addData("fastDrive.nearXDestination();", fastDrive.nearXDestination());
        telemetry.addData("fastDrive.nearYDestination();", fastDrive.nearYDestination());
        telemetry.addData("fastDrive.nearHDestination();", fastDrive.nearHDestination());
        telemetry.addData("fastDrive.notMoving();", fastDrive.notMoving());
    }

    public boolean done() {
        return driveRunner.done();
    }

    public void to(Action action) {
        driveRunner.drive(action);
    }

    public void toggleTelemetry() {
        telemetryOn = !telemetryOn;
    }

    public void cancel() {
        driveRunner.cancel();
    }
}
