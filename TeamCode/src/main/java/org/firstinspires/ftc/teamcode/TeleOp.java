package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.OpMode;

abstract class TeleOp extends OpMode {
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

        if (gamepad1.square) {
            launcher.launchyLaunch();
        }
//        if (gamepad1.circleWasPressed()) {
//            launcher.loadyLoad();
//        }
//        if (gamepad1.circleWasReleased()) {
//            launcher.finishLoading();
//        }
        if (gamepad1.crossWasPressed()) {
            launcher.noPower();
        }
        if (gamepad1.triangle) {
            launcher.slowLaunchyLaunch();
        }
        if (gamepad1.right_trigger > 0.2) {
            brain.autoShootSlow(launchTarget());
        } else if (gamepad1.left_trigger > 0.2) {
            brain.autoShootFast(launchTarget());
        } else if (gamepad1.left_bumper) {
            brain.cancelPlan();
            drive.spazToLaunchPose(launchTarget());
            if (Math.abs(gamepad1.left_stick_x) > 0.05) {
                drive.setStrafePower(-gamepad1.left_stick_x);
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                drive.setTurnPower(-gamepad1.right_stick_x);
            }
        } else if (gamepad1.right_bumper) {
            brain.cancelPlan();
            drive.spazToLaunchPose(launchTarget());
            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                drive.setTurnPower(-gamepad1.right_stick_x);
            }
            drive.setStraightPower(-gamepad1.left_stick_y);
            drive.setStrafePower(-gamepad1.left_stick_x);
        } else {
            brain.cancelPlan();
            drive.setTurnPower(-gamepad1.right_stick_x);
            drive.setStraightPower(-gamepad1.left_stick_y);
            drive.setStrafePower(-gamepad1.left_stick_x);
        }

        if (Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_y) > 0.2) {
            drive.cancel();
        }

        //adjust settings using second controller
        if (gamepad2.rightBumperWasPressed()) {
            launcher.increaseBottomGateWaitTime();
        }
        if (gamepad2.leftBumperWasPressed()) {
            launcher.decreaseBottomGateWaitTime();
        }
//        if (gamepad2.rightBumperWasPressed()) {
//            launcher.increaseTopGatePosition();
//        }
//        if (gamepad2.leftBumperWasPressed()) {
//            launcher.decreaseTopGatePosition();
//        }
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
        if (gamepad2.right_trigger > 0.2) {
            brain.toggleCameraLocalization();
        }

        drive.useDirectPower();

//        if (gamepad1.leftBumperWasPressed()) {
//            drive.savePose1();
//        }
//        if (gamepad1.left_trigger > 0.5) {
//            drive.goToPose1();
//        }
//        if (gamepad1.rightBumperWasPressed()) {
//            drive.savePose2();
//        }
//        if (gamepad1.right_trigger > 0.5) {
//            drive.goToPose2();
//        }

        if (gamepad2.triangleWasPressed()) {
            telemetryOn = !telemetryOn;
        }
    }

    abstract Vector2d launchTarget();

}
