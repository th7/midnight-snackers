package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public abstract class BlueAutoOp extends AutoOp {
    protected Nav getNav(MecanumDrive mecanumDrive) {
        return Nav.blue(mecanumDrive, runtime, telemetry);
    }
}
