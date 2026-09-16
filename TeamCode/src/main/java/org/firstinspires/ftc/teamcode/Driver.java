package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.Drive.Held;
import org.firstinspires.ftc.teamcode.base.SubSystem;

/** Drives the robot from the gamepads. A TeleOp adds one; an auto has no driver. */
public class Driver extends SubSystem {
    /** How far a stick must move before it counts as held. */
    private static final float HELD = 0.05f;
    /** How far a stick must move to take the wheels back from an action. */
    private static final float TAKEOVER = 0.2f;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    @Override
    protected void onInit() {
        gamepad1 = robot.gamepad1;
        gamepad2 = robot.gamepad2;
    }

    @Override
    protected void onTelemetry() {}

    /** Steers toward the launch pose on the axes the driver is not holding; with no goal, just drives. */
    private void aim(Held held) {
        Optional<Nav.Pose> launchPose = robot.nav.launchPose();
        if (launchPose.isPresent()) {
            robot.drive.toward(launchPose.get(), held);
        } else {
            robot.drive.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
    }

    @Override
    protected void onLoop() {
        if (Math.abs(gamepad1.left_stick_x) > TAKEOVER
                || Math.abs(gamepad1.left_stick_y) > TAKEOVER
                || Math.abs(gamepad1.right_stick_x) > TAKEOVER
                || Math.abs(gamepad1.right_stick_y) > TAKEOVER) {
            robot.drive.cancel();
        }

        if (gamepad1.square) {
            robot.launcher.launchyLaunch();
        }

        if (gamepad1.triangle) {
            robot.launcher.slowLaunchyLaunch();
        }
        if (gamepad1.crossWasPressed()) {
            robot.brain.turnTableToZeroModeOn();
        }
        if (gamepad1.circleWasPressed()) {
            robot.brain.turnTableToTargetModeOn();
        }
        if (gamepad1.right_trigger > 0.2) {
            robot.brain.autoShootSlow();
        } else if (gamepad1.left_trigger > 0.2) {
            robot.brain.autoShootFast();
        } else if (gamepad1.left_bumper) {
            // aim at the goal; the driver may nudge sideways and around
            robot.brain.cancelPlan();
            Held held = Held.NONE;
            if (Math.abs(gamepad1.left_stick_x) > HELD) {
                held = held.strafe(-gamepad1.left_stick_x);
            }
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(-gamepad1.right_stick_x);
            }
            aim(held);
        } else if (gamepad1.right_bumper) {
            // the driver moves the robot; the drive keeps it facing the goal unless the driver turns
            robot.brain.cancelPlan();
            Held held = Held.NONE.straight(-gamepad1.left_stick_y).strafe(-gamepad1.left_stick_x);
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(-gamepad1.right_stick_x);
            }
            aim(held);
        } else {
            robot.brain.cancelPlan();
            robot.drive.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        // adjust settings using second controller
        if (gamepad2.rightBumperWasPressed()) {
            robot.launcher.increaseBottomGateWaitTime();
        }
        if (gamepad2.leftBumperWasPressed()) {
            robot.launcher.decreaseBottomGateWaitTime();
        }
        if (gamepad2.dpadUpWasPressed()) {
            robot.launcher.increasePower();
        }
        if (gamepad2.dpadDownWasPressed()) {
            robot.launcher.decreasePower();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            robot.brain.setTurnTableDebugOverrideModeOn();
            robot.turntable.turnTableToLeft();
        }
        if (gamepad2.dpadRightWasPressed()) {
            robot.brain.setTurnTableDebugOverrideModeOn();
            robot.turntable.turnTableToRight();
        }
        if (gamepad2.right_trigger > 0.2) {
            robot.brain.toggleCameraLocalization();
        }
    }
}
