package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Drive.Held;
import org.firstinspires.ftc.teamcode.base.SuperSystem;

/** Drives the robot from the gamepads. A TeleOp adds one; an auto has no driver. */
public class Driver extends SuperSystem {
    /** How far a stick must move before it counts as held. */
    private static final float HELD = 0.05f;
    /** How far a stick must move to take the wheels back from an action. */
    private static final float TAKEOVER = 0.2f;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    @Override
    public void init() {
        gamepad1 = robot.gamepad1;
        gamepad2 = robot.gamepad2;
    }

    @Override
    protected void onLoop() {
        if (Math.abs(gamepad1.left_stick_x) > TAKEOVER || Math.abs(gamepad1.left_stick_y) > TAKEOVER
                || Math.abs(gamepad1.right_stick_x) > TAKEOVER || Math.abs(gamepad1.right_stick_y) > TAKEOVER) {
            drive.cancel();
        }

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
            // aim at the goal; the driver may nudge sideways and around
            brain.cancelPlan();
            Held held = Held.NONE;
            if (Math.abs(gamepad1.left_stick_x) > HELD) {
                held = held.strafe(-gamepad1.left_stick_x);
            }
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(-gamepad1.right_stick_x);
            }
            drive.toward(nav.launchPose(), held);
        } else if (gamepad1.right_bumper) {
            // the driver moves the robot; the drive keeps it facing the goal unless the driver turns
            brain.cancelPlan();
            Held held = Held.NONE.straight(-gamepad1.left_stick_y).strafe(-gamepad1.left_stick_x);
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(-gamepad1.right_stick_x);
            }
            drive.toward(nav.launchPose(), held);
        } else {
            brain.cancelPlan();
            drive.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
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
    }
}
