package org.firstinspires.ftc.reginald.framework;

import org.firstinspires.ftc.teamcode.base.Gamepads;
import org.firstinspires.ftc.teamcode.base.Loopable;

public abstract class SubSystem implements Loopable {
    public void init() {}

    public void teleOp(Gamepads gamepads) {}

    @Override
    public void loop() {}
}
