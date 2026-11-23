package org.firstinspires.ftc.reginald.opmode;

import org.firstinspires.ftc.reginald.Driver;
import org.firstinspires.ftc.teamcode.base.Alliance;

public abstract class TeleOp extends OpMode {
    protected TeleOp(Alliance alliance) {
        super(alliance, Driver.class);
    }
}
