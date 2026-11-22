package org.firstinspires.ftc.reginald;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.base.LoopOrder;
import org.firstinspires.ftc.teamcode.base.Subsystem;

public class Intake extends Subsystem {
    public static final double RUNNING_POWER = 0.7;

    private final DcMotorEx intake;
    private boolean on = true;

    public Intake(LoopOrder loopOrder, DcMotorEx intake) {
        super(loopOrder);
        this.intake = intake;
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
