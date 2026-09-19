package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.base.Alliance;

public abstract class TeleOp extends OpMode {
    private Driver driver;

    protected TeleOp(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init() {
        super.init();
        driver = new Driver(
                robot.drive, robot.launcher, robot.brain, robot.nav, robot.turntable, robot.gamepad1, robot.gamepad2);
    }

    @Override
    protected void onLoop() {
        driver.loop();
    }
}
