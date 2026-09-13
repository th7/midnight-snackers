package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubSystem implements Loopable {
    public final ElapsedTime runtime;
    public final Telemetry telemetry;
    private final LoopGroup helpers = new LoopGroup();

    public SubSystem(ElapsedTime runtime, Telemetry telemetry) {
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    /** Registers a helper (a PlanRunner, a DriveRunner) to be ticked before {@link #onLoop()}. */
    protected <T extends Loopable> T add(T helper) {
        return helpers.add(helper);
    }

    public abstract void init();

    /** Ticks registered helpers, then this subsystem's own work. Subclasses override {@link #onLoop()}. */
    @Override
    public final void loop() {
        helpers.loop();
        onLoop();
    }

    protected abstract void onLoop();
}
