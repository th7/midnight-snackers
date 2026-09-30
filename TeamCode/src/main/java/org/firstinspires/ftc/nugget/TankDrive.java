package org.firstinspires.ftc.nugget;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class TankDrive {
    private final DcMotorEx left;
    private final DcMotorEx right;

    public TankDrive(NuggetHardware hardware) {
        this.left = hardware.left;
        this.right = hardware.right;

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive(double straight, double turn) {
        sides(straight - turn, straight + turn);
    }

    public void sides(double leftPower, double rightPower) {
        double most = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        left.setPower(leftPower / most);
        right.setPower(rightPower / most);
    }

    public void stop() {
        drive(0, 0);
    }
}
