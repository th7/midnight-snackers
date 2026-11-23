package org.firstinspires.ftc.reginald.subsystems;

import java.util.function.LongSupplier;
import org.firstinspires.ftc.reginald.framework.SubSystem;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Pause extends SubSystem {
    private final LongSupplier nanoClock;

    public Pause(Hardware hardware) {
        this.nanoClock = hardware.nanoClock();
    }

    public Step forSeconds(double seconds) {
        return Step.waitFor("pause", seconds, nanoClock);
    }
}
