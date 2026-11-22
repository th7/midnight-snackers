package org.firstinspires.ftc.reginald.opmode;

import org.firstinspires.ftc.reginald.Driver;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.LoopOrder;

public abstract class TeleOp extends OpMode {
    private final LoopOrder loopOrder = new LoopOrder();

    protected TeleOp(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init() {
        super.init();
        new Driver(loopOrder, robot.drive, robot.brain, robot.nav, robot.intake, robot.gamepad1, robot.gamepad2);
    }

    @Override
    protected void onLoop() {
        loopOrder.loop();
    }
}
