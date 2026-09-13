package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.base.OpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * Tiny TeleOps for exercising the simulator itself. Nested classes, so the catalog never lists
 * them on the real bench.
 */
public final class TestTeleOps {
    private TestTeleOps() {
    }

    /**
     * Drives straight from gamepad 1's left stick and counts the presses of its cross button.
     */
    @TeleOp(name = "Stick", group = "Test")
    public static class StickTeleOp extends OpMode {
        public int presses = 0;

        @Override
        protected Nav getNav(MecanumDrive mecanumDrive) {
            return Nav.red(mecanumDrive, runtime, telemetry);
        }

        @Override
        protected void onLoop() {
            if (gamepad1.crossWasPressed()) {
                presses++;
            }
            drive.setStraightPower(-gamepad1.left_stick_y);
            drive.useDirectPower();
        }
    }
}
