package org.firstinspires.ftc.framework.plans;

import java.util.function.LongSupplier;
import org.firstinspires.ftc.framework.hardware.Hardware;

public class Pause {
    private final LongSupplier nanoClock;

    public Pause(Hardware hardware) {
        this.nanoClock = hardware.nanoClock();
    }

    public Step forSeconds(double seconds) {
        return Step.waitFor("pause", seconds, nanoClock);
    }
}
