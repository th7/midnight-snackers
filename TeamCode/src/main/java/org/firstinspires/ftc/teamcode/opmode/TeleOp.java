package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.base.Alliance;

/** An op mode a driver drives, for one alliance. */
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

    /**
     * The driver is the TeleOp's own, so the TeleOp ticks it, after the robot has: what the
     * driver asks for this loop is acted on by the subsystems on the next one, as before.
     */
    @Override
    protected void onLoop() {
        driver.loop();
    }
}
