package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public abstract class RedAutoOp extends AutoOp {
    protected Nav getNav(MecanumDrive mecanumDrive) {
        return Nav.red(mecanumDrive, runtime, telemetry);
    }
}
