package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.nugget.NuggetHardware;
import org.firstinspires.ftc.nugget.NuggetOpMode;

public final class TestNuggetOpModes {
    private TestNuggetOpModes() {}

    /** Nugget driven tank-wise, its left motor reversed since its two sides are mounted mirror image. */
    @TeleOp(name = "Tank drive", group = "Test")
    public static class TankTeleOp extends NuggetOpMode {
        public NuggetHardware devices() {
            return hardware();
        }

        @Override
        public void init() {
            hardware().left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware().right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware().left.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public void loop() {
            double straight = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double left = straight - turn;
            double right = straight + turn;
            double most = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
            hardware().left.setPower(left / most);
            hardware().right.setPower(right / most);
        }
    }
}
