package org.firstinspires.ftc.reginald.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Optional;
import org.firstinspires.ftc.framework.drive.Chassis;
import org.firstinspires.ftc.framework.drive.Nav;
import org.firstinspires.ftc.framework.plans.Step;
import org.firstinspires.ftc.framework.robot.Gamepads;
import org.firstinspires.ftc.framework.robot.SubSystem;

public class Drive extends SubSystem {
    private static final float HELD = 0.05f;

    private static final float TAKEOVER = 0.2f;

    private final Chassis chassis;
    private final Nav nav;

    public Drive(Chassis chassis, Nav nav) {
        this.chassis = chassis;
        this.nav = nav;
    }

    public Step to(double x, double y, double heading) {
        return chassis.to(x, y, heading);
    }

    public Step near(double x, double y, double heading) {
        return chassis.near(x, y, heading);
    }

    @Override
    public void teleOp(Gamepads gamepads) {
        Gamepad gamepad1 = gamepads.gamepad1;
        if (Math.abs(gamepad1.left_stick_x) > TAKEOVER
                || Math.abs(gamepad1.left_stick_y) > TAKEOVER
                || Math.abs(gamepad1.right_stick_x) > TAKEOVER
                || Math.abs(gamepad1.right_stick_y) > TAKEOVER) {
            chassis.cancel();
        }

        if (gamepad1.left_bumper) {
            Chassis.Held held = Chassis.Held.NONE;
            if (Math.abs(gamepad1.left_stick_x) > HELD) {
                held = held.strafe(gamepad1.left_stick_x);
            }
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(gamepad1.right_stick_x);
            }
            aim(gamepad1, held);
        } else if (gamepad1.right_bumper) {
            Chassis.Held held = Chassis.Held.NONE.straight(gamepad1.left_stick_y).strafe(gamepad1.left_stick_x);
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(gamepad1.right_stick_x);
            }
            aim(gamepad1, held);
        } else {
            chassis.manual(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    private void aim(Gamepad gamepad1, Chassis.Held held) {
        Optional<Nav.Pose> launchPose = nav.launchPose();
        if (launchPose.isPresent()) {
            chassis.toward(launchPose.get(), held);
        } else {
            chassis.manual(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}
