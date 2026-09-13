package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.base.OpMode;

public abstract class TeleOp extends OpMode {
    @Override
    public void init() {
        super.init();
        telemetry.addData("TeleOp.init()", true);
    }

    @Override
    public void loop() {
        super.loop();

        if (gamepad1.square) {
            launcher.launchyLaunch();
        }

        if (gamepad1.triangle) {
            launcher.slowLaunchyLaunch();
        }
        if (gamepad1.crossWasPressed()) {
            brain.turnTableToZeroModeOn();
        }
        if (gamepad1.circleWasPressed()) {
            brain.turnTableToTargetModeOn();
        }
        if (gamepad1.right_trigger > 0.2) {
            brain.autoShootSlow();
        } else if (gamepad1.left_trigger > 0.2) {
            brain.autoShootFast();
        } else if (gamepad1.left_bumper) {
            brain.cancelPlan();
            drive.fastDriveTo(nav.launchPose(), nav.currentPose());
            if (Math.abs(gamepad1.left_stick_x) > 0.05) {
                drive.setStrafePower(-gamepad1.left_stick_x);
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                drive.setTurnPower(-gamepad1.right_stick_x);
            }
        } else if (gamepad1.right_bumper) {
            brain.cancelPlan();
            drive.fastDriveTo(nav.launchPose(), nav.currentPose());
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
        if (gamepad2.dpadUpWasPressed()) {
            launcher.increasePower();
        }
        if (gamepad2.dpadDownWasPressed()) {
            launcher.decreasePower();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            brain.setTurnTableDebugOverrideModeOn();
            turntable.turnTableToLeft();
        }
        if (gamepad2.dpadRightWasPressed()) {
            brain.setTurnTableDebugOverrideModeOn();
            turntable.turnTableToRight();
        }
        if (gamepad2.right_trigger > 0.2) {
            brain.toggleCameraLocalization();
        }

        drive.useDirectPower();

    }
}
