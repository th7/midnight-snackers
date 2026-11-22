package org.firstinspires.ftc.teamcode.base;

public abstract class Subsystem implements Loopable {
    protected Subsystem(LoopOrder loopOrder) {
        loopOrder.add(this);
    }
}
