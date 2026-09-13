package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Turntable;

import java.util.List;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private final Alliance alliance;
    protected Robot robot;
    protected Launcher launcher;
    protected Drive drive;
    protected Camera camera;
    protected Nav nav;
    protected Turntable turntable;
    protected Brain brain;

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
        robot = new Robot(hardware, alliance, telemetry);
        launcher = robot.launcher;
        drive = robot.drive;
        camera = robot.camera;
        nav = robot.nav;
        turntable = robot.turntable;
        brain = robot.brain;
    }

    /** Registers something to tick after everything registered so far; see {@link Robot#add}. */
    protected <T extends Loopable> T add(T loopable) {
        return robot.add(loopable);
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
    protected void onLoop() {
    }

    private void handleTelemetryToggles() {
        if (gamepad2.crossWasPressed()) {
            drive.toggleTelemetry();
        }
        if (gamepad2.squareWasPressed()) {
            turntable.toggleTelemetry();
            launcher.toggleTelemetry();
        }
        if (gamepad2.circleWasPressed()) {
            camera.toggleTelemetry();
        }
    }
}
