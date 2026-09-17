package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private final Alliance alliance;
    protected Robot robot;

    private Hardware injectedHardware = null;
    /** The telemetry the robot controller gave this op mode, before it was mirrored to the dashboard. */
    private Telemetry driverStationTelemetry = null;

    protected OpMode(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance alliance() {
        return alliance;
    }

    /**
     * Drive these devices instead of the ones in the robot configuration. A simulation calls this
     * before {@link #init()}; on the robot nothing does.
     */
    public void useHardware(Hardware hardware) {
        this.injectedHardware = hardware;
    }

    /**
     * The devices this op mode drives: the injected ones if any, else the robot configuration's.
     */
    protected Hardware hardware() {
        return injectedHardware != null ? injectedHardware : Hardware.fromHardwareMap(hardwareMap);
    }

    /**
     * Builds the robot afresh. The robot controller keeps one instance of an op mode registered by
     * instance and calls this for every run, so nothing from an earlier init survives.
     */
    @Override
    public void init() {
        Hardware hardware = hardware();
        // Mirror all telemetry to the FTC Dashboard as well as the Driver Station.
        // Must happen before the robot is built, since its subsystems keep the telemetry reference.
        if (driverStationTelemetry == null) {
            driverStationTelemetry = telemetry;
        }
        telemetry = new MultipleTelemetry(driverStationTelemetry, hardware.dashboard.telemetry());
        robot = new Robot(hardware, alliance, telemetry, gamepad1, gamepad2);
    }

    /** Where a person finds this op mode's code: its class, unless a subclass knows better. */
    public String where() {
        return getClass().getName();
    }

    /** Everything this op mode ticks, in order. */
    public List<Loopable> loopOrder() {
        return robot.loopOrder();
    }

    /** Ticks the robot, then {@link #onLoop()}. Subclasses override {@link #onLoop()}. */
    @Override
    public final void loop() {
        handleTelemetryToggles();
        robot.loop();
        onLoop();
    }

    /** Per-op-mode work that runs after every subsystem has ticked. */
    protected void onLoop() {}

    /**
     * A driver turns a channel on to read it and off to get the screen back. Whose numbers each
     * button reaches is said here, once; no subsystem knows it can be turned off.
     */
    private void handleTelemetryToggles() {
        if (gamepad2.crossWasPressed()) {
            robot.channels.toggle("Drive", "Localizer");
        }
        if (gamepad2.squareWasPressed()) {
            robot.channels.toggle("Turntable", "Launcher");
        }
        if (gamepad2.circleWasPressed()) {
            robot.channels.toggle("Camera", "Brain");
        }
    }
}
