package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private boolean telemetryOn = false;

    @Override
    public void init() {
        super.init();
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        drive.setPose(new Pose2d(0, 0, 0));

        telemetry.addData("TeleOp.init()", true);
    }

    @Override
    public void loop() {
        super.loop();

        if (gamepad1.squareWasPressed()) {
            launcher.setCloseLaunchPower();
        }
        if (gamepad1.circleWasPressed()) {
            launcher.loadyLoad();
        }
        if (gamepad1.circleWasReleased()) {
            launcher.finishLoading();
        }
        if (gamepad1.crossWasPressed()) {
            launcher.noPower();
        }
        if (gamepad1.triangleWasPressed() && launcher.launchDone()) {
//            launcher.launch();
            launcher.launchyLaunch();
        }
        if(Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2) {
            drive.cancel();
        }

        //adjust settings using second controller
        if (gamepad2.rightBumperWasPressed()) {
            launcher.increaseGateWaitTime();
        }
        if (gamepad2.leftBumperWasPressed()) {
            launcher.decreaseGateWaitTime();
        }
        if (gamepad2.dpadUpWasPressed()) {
            launcher.increasePower();
        }
        if (gamepad2.dpadDownWasPressed()) {
            launcher.decreasePower();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            launcher.decreaseAdjustable();
        }
        if (gamepad2.dpadRightWasPressed()) {
            launcher.increaseAdjustable();
        }

        MoveData straight = MoveData.straight(-gamepad1.left_stick_y, 0f, 1f);
        MoveData strafe = MoveData.strafe(-gamepad1.left_stick_x, 0f, 1f);
        MoveData turn = MoveData.turn(-gamepad1.right_stick_x, 0f, 1f);
        MoveData moveData = straight.add(strafe, turn);

        if(drive.done()) {
            leftFront.setPower(moveData.frontLeftPower);
            rightFront.setPower(moveData.frontRightPower);
            leftBack.setPower(moveData.rearLeftPower);
            rightBack.setPower(moveData.rearRightPower);
        }

        if (gamepad1.leftBumperWasPressed()) {
            drive.savePose1();
        }
        if (gamepad1.left_trigger > 0.5) {
            drive.goToPose1();
        }
        if (gamepad1.rightBumperWasPressed()) {
            drive.savePose2();
        }
        if (gamepad1.right_trigger > 0.5) {
            drive.goToPose2();
        }

        if (gamepad2.triangleWasPressed()) { telemetryOn = !telemetryOn; }
        if (telemetryOn) {
            telemetry.addData("frontLeftPower", moveData.frontLeftPower);
            telemetry.addData("frontRightPower", moveData.frontRightPower);
            telemetry.addData("rearLeftPower", moveData.rearLeftPower);
            telemetry.addData("rearRightPower", moveData.rearRightPower);
            telemetry.update();
        }
    }
}
