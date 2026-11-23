package org.firstinspires.ftc.framework.robot;


final class TeleOp extends OpMode {
    static final String WHERE = "the TeleOp every robot has";

    TeleOp(Alliance alliance) {
        super(alliance);
    }

    @Override
    void beforeTheRobotTicks() {
        robot.teleOp();
    }

    @Override
    public String where() {
        return WHERE;
    }
}
