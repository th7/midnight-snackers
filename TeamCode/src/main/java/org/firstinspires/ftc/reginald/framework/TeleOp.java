package org.firstinspires.ftc.reginald.framework;

import org.firstinspires.ftc.teamcode.base.Alliance;

public abstract class TeleOp extends OpMode {
    protected TeleOp(Alliance alliance) {
        super(alliance);
    }

    @Override
    void beforeTheRobotTicks() {
        robot.teleOp();
    }
}
