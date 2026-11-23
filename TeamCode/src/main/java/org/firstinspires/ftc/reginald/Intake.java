package org.firstinspires.ftc.reginald;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.teamcode.base.SubSystem;

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

    private void driveTheMotor() {
        intake.setPower(on ? RUNNING_POWER : 0);
    }

    @Override
    public void loop() {
        driveTheMotor();
    }
}
