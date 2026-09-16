package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * One part of the robot. A subsystem answers three questions, each of which the compiler asks of
 * every subclass: what it sets up ({@link #onInit()}), what it does each tick ({@link #onLoop()}),
 * and what it prints when a driver asks to see it ({@link #onTelemetry()}). An empty body is a
 * fine answer; leaving one out is not an option, so nothing is forgotten by silence.
 *
 * <p>A subsystem is handed what it needs when it is built and reaches for nothing else; the only
 * thing it is given afterwards is somewhere to print.
 *
 * <p>When each of those runs is this class's to decide, so {@link #init()}, {@link #loop()} and
 * {@link #toggleTelemetry()} are final: a subsystem cannot skip its helpers or its telemetry by
 * overriding them.
 */
public abstract class SubSystem implements Loopable {
    protected Telemetry telemetry;
    private final LoopGroup helpers = new LoopGroup();
    private boolean telemetryOn = false;

    /** Registers a helper (a PlanRunner, a DriveRunner) to be ticked before {@link #onLoop()}. */
    protected <T extends Loopable> T add(T helper) {
        return helpers.add(helper);
    }

    /**
     * Gives the subsystem somewhere to print and sets it up. Subclasses override
     * {@link #onInit()}.
     */
    public final void init(Telemetry telemetry) {
        this.telemetry = telemetry;
        onInit();
    }

    /**
     * Ticks registered helpers, then this subsystem's own work, then its telemetry if a driver has
     * asked for it. Subclasses override {@link #onLoop()}.
     */
    @Override
    public final void loop() {
        helpers.loop();
        onLoop();
        if (telemetryOn) {
            telemetry.addData(getClass().getSimpleName(), "telemetry on");
            onTelemetry();
        }
    }

    /** Shows or hides this subsystem's telemetry; a driver toggles it from the gamepad. */
    public final void toggleTelemetry() {
        telemetryOn = !telemetryOn;
    }

    /** What this subsystem sets up once the robot is known; nothing, for one with no setup. */
    protected abstract void onInit();

    /** This subsystem's work for one tick. */
    protected abstract void onLoop();

    /**
     * What this subsystem prints while its telemetry is on, which is only when a driver has asked
     * for it: the numbers that say what it is doing. Nothing, for one with nothing to show.
     */
    protected abstract void onTelemetry();
}
