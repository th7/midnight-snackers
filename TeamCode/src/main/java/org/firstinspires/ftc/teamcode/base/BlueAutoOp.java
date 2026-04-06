package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Nav;

public abstract class BlueAutoOp extends AutoOp {
    protected Nav getNav() {
        return Nav.blue(hardwareMap, runtime, telemetry);
    }
}
