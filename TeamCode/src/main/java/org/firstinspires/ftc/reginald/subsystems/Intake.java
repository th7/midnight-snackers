package org.firstinspires.ftc.reginald.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.reginald.framework.SubSystem;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.teamcode.base.Gamepads;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Intake extends SubSystem {
    public static final double RUNNING_POWER = 0.7;

    private final DcMotorEx intake;
    private boolean on = true;

    public Intake(Hardware hardware) {
        this.intake = hardware.motor("intake");
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public boolean isOn() {
        return on;
    }

    public Step spin() {
        return new Step("spin the intake", this::on, () -> false);
    }

    private void driveTheMotor() {
        intake.setPower(on ? RUNNING_POWER : 0);
    }

    @Override
    public void teleOp(Gamepads gamepads) {
        if (gamepads.gamepad1.square) {
            on();
        } else {
            off();
        }
    }

    @Override
    public void loop() {
        driveTheMotor();
    }
}
