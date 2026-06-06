package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Nav;

public abstract class RelativeAutoOp extends AutoOp {
    @Override
    public void init() {
        super.init();
        brain.disableCameraLocalization();
    }

    protected Nav getNav() {
        return Nav.relative(hardwareMap, runtime, telemetry);
    }
}
