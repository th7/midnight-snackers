package org.firstinspires.ftc.nugget;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class NuggetOpMode extends OpMode {
    private NuggetHardware injectedHardware = null;

    public void useHardware(NuggetHardware hardware) {
        this.injectedHardware = hardware;
    }

    protected NuggetHardware hardware() {
        if (injectedHardware == null) {
            injectedHardware = NuggetHardware.fromHardwareMap(hardwareMap);
        }
        return injectedHardware;
    }
}
