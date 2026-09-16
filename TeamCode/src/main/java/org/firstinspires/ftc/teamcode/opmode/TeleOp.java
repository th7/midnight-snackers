package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.base.Alliance;

/** An op mode a driver drives, for one alliance. */
public abstract class TeleOp extends OpMode {
    protected TeleOp(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init() {
        super.init();
        add(new Driver(
                robot.drive, robot.launcher, robot.brain, robot.nav, robot.turntable, robot.gamepad1, robot.gamepad2));
    }
}
