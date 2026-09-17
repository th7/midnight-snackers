package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.Drive.Held;
import org.firstinspires.ftc.teamcode.base.SubSystem;

/** Drives the robot from the gamepads. A TeleOp adds one; an auto has no driver. */
public class Driver extends SubSystem {
    /** What a driver drives: named here, so forgetting to wire one up does not compile. */
    private final Drive drive;

    private final Launcher launcher;
    private final Brain brain;
    private final Nav nav;
    private final Turntable turntable;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public Driver(
            Drive drive,
            Launcher launcher,
            Brain brain,
            Nav nav,
            Turntable turntable,
            Gamepad gamepad1,
            Gamepad gamepad2) {
        this.drive = drive;
        this.launcher = launcher;
        this.brain = brain;
        this.nav = nav;
        this.turntable = turntable;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /** How far a stick must move before it counts as held. */
    private static final float HELD = 0.05f;
    /** How far a stick must move to take the wheels back from an action. */
    private static final float TAKEOVER = 0.2f;

    @Override
    protected void onInit() {}

    @Override
    protected void onTelemetry() {}

    /** Steers toward the launch pose on the axes the driver is not holding; with no goal, just drives. */
    private void aim(Held held) {
        Optional<Nav.Pose> launchPose = nav.launchPose();
        if (launchPose.isPresent()) {
            drive.toward(launchPose.get(), held);
        } else {
            drive.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
    }

    @Override
    protected void onLoop() {
        if (Math.abs(gamepad1.left_stick_x) > TAKEOVER
                || Math.abs(gamepad1.left_stick_y) > TAKEOVER
                || Math.abs(gamepad1.right_stick_x) > TAKEOVER
                || Math.abs(gamepad1.right_stick_y) > TAKEOVER) {
            drive.cancel();
        }

        if (gamepad1.square) {
            launcher.launchyLaunch();
        }

        if (gamepad1.triangle) {
            launcher.slowLaunchyLaunch();
        }
        if (gamepad1.crossWasPressed()) {
            turntable.parkStraightAhead();
        }
        if (gamepad1.circleWasPressed()) {
            turntable.followTheGoal();
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
            aim(held);
        } else if (gamepad1.right_bumper) {
            // the driver moves the robot; the drive keeps it facing the goal unless the driver turns
            brain.cancelPlan();
            Held held = Held.NONE.straight(-gamepad1.left_stick_y).strafe(-gamepad1.left_stick_x);
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(-gamepad1.right_stick_x);
            }
            aim(held);
        } else {
            brain.cancelPlan();
            drive.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        // adjust settings using second controller
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
            turntable.nudgeLeft();
        }
        if (gamepad2.dpadRightWasPressed()) {
            turntable.nudgeRight();
        }
        if (gamepad2.right_trigger > 0.2) {
            brain.toggleCameraLocalization();
        }
    }
}
