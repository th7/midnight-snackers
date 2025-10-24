package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubSystem {
    public final HardwareMap hardwareMap;
    public final ElapsedTime runtime;
    public final Telemetry telemetry;

    public SubSystem(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    public abstract void init();

    public void initAuto() {
    }

    ;

    public void initTeleOp() {
    }

    ;

    public abstract void loop();

    public abstract boolean done();
}
