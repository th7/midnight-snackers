package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;

public final class Wheels {
    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final DcMotorEx rightFront;

    public Wheels(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightBack, DcMotorEx rightFront) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheels =
                new MecanumKinematics(1).inverse(PoseVelocity2dDual.constant(powers, 1));

        double most = 1;
        for (DualNum<Time> power : wheels.all()) {
            most = Math.max(most, Math.abs(power.value()));
        }

        set(
                wheels.leftFront.get(0) / most,
                wheels.leftBack.get(0) / most,
                wheels.rightBack.get(0) / most,
                wheels.rightFront.get(0) / most);
    }

    public void set(double leftFront, double leftBack, double rightBack, double rightFront) {
        this.leftFront.setPower(leftFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
        this.rightFront.setPower(rightFront);
    }

    public void stop() {
        set(0, 0, 0, 0);
    }

    public List<DcMotorEx> left() {
        return List.of(leftFront, leftBack);
    }

    public List<DcMotorEx> right() {
        return List.of(rightFront, rightBack);
    }
}
