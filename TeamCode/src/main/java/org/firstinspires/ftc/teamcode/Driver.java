package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.base.SuperSystem;

/** Drives the robot from the gamepads. A TeleOp adds one; an auto has no driver. */
public class Driver extends SuperSystem {
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    @Override
    public void init() {
        gamepad1 = robot.gamepad1;
        gamepad2 = robot.gamepad2;
    }

    @Override
    protected void onLoop() {
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
