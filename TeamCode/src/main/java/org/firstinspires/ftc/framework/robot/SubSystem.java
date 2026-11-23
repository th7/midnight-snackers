package org.firstinspires.ftc.framework.robot;


public abstract class SubSystem implements Loopable {
    public void init() {}

    public void teleOp(Gamepads gamepads) {}

    @Override
    public void loop() {}
}
