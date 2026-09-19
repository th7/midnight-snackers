package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.base.Loopable;

public class Intake implements Loopable {
    public static final double RUNNING_POWER = 1;

    private final DcMotorEx intake;
    private boolean on = true;

    public Intake(DcMotorEx intake) {
        this.intake = intake;

        driveTheMotor();
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
