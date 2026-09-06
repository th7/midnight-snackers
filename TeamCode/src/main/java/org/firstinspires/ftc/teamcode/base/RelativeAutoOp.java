package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public abstract class RelativeAutoOp extends AutoOp {
    @Override
    public void init() {
        super.init();
        brain.disableCameraLocalization();
    }

    protected Nav getNav(MecanumDrive mecanumDrive) {
        return Nav.relative(mecanumDrive, runtime, telemetry);
    }
}
