package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Nav;

public abstract class RedAutoOp extends AutoOp {
    protected Nav getNav() {
        return Nav.red(hardwareMap, runtime, telemetry);
    }
}
