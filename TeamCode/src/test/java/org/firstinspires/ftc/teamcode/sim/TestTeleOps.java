package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.base.OpMode;

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

        public StickTeleOp() {
            super(Alliance.RED);
        }

        @Override
        protected void onLoop() {
            if (gamepad1.crossWasPressed()) {
                presses++;
            }
            robot.drive.setStraightPower(-gamepad1.left_stick_y);
            robot.drive.useDirectPower();
        }
    }
}
