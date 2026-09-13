package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubSystem implements Loopable {
    /** The robot this subsystem was added to, and so the other subsystems; set before {@link #init()}. */
    protected Robot robot;
    protected Telemetry telemetry;
    private final LoopGroup helpers = new LoopGroup();

    void attach(Robot robot) {
        this.robot = robot;
        this.telemetry = robot.telemetry;
    }

    /** Registers a helper (a PlanRunner, a DriveRunner) to be ticked before {@link #onLoop()}. */
    protected <T extends Loopable> T add(T helper) {
        return helpers.add(helper);
    }

    /** Sets up hardware once the robot is known. Nothing, unless a subsystem has something to set up. */
    public void init() {
    }

    /** Ticks registered helpers, then this subsystem's own work. Subclasses override {@link #onLoop()}. */
    @Override
    public final void loop() {
        helpers.loop();
        onLoop();
    }

    protected abstract void onLoop();
}
