package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubSystem {
    public final ElapsedTime runtime;
    public final Telemetry telemetry;

    public SubSystem(ElapsedTime runtime, Telemetry telemetry) {
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    public abstract void init();

    public abstract void loop();
}
