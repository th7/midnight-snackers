package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Turntable;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    protected ElapsedTime runtime;
    protected Launcher launcher;
    protected Drive drive;
    protected Camera camera;

    protected Nav nav;
    protected Turntable turntable;
    protected Brain brain;

    private LoopGroup subsystems = new LoopGroup();
    private Hardware injectedHardware = null;
    /** The telemetry the robot controller gave this op mode, before it was mirrored to the dashboard. */
    private Telemetry driverStationTelemetry = null;

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
     * Builds every subsystem afresh. The robot controller keeps one instance of an op mode
     * registered by instance and calls this for every run, so nothing from an earlier init survives.
     */
    @Override
    public void init() {
        Hardware hardware = hardware();
        subsystems = new LoopGroup();
        // Mirror all telemetry to the FTC Dashboard as well as the Driver Station.
        // Must happen before subsystems are built, since they capture the telemetry reference.
        if (driverStationTelemetry == null) {
            driverStationTelemetry = telemetry;
        }
        telemetry = new MultipleTelemetry(driverStationTelemetry, hardware.dashboard.telemetry());
        runtime = new ElapsedTime();
        // Registration order is loop order. Brain goes last because it reads Camera and Nav
        // from this tick and sets the Turntable target.
        launcher = subsystems.add(new Launcher(hardware.launcher, hardware.topGate, hardware.bottomGate, runtime, telemetry));
        launcher.init();
        drive = subsystems.add(new Drive(
                hardware.leftFront, hardware.rightFront, hardware.leftBack, hardware.rightBack,
                hardware.dashboard, runtime, telemetry));
        drive.init();
        camera = subsystems.add(new Camera(hardware.aprilTags, runtime, telemetry));
        camera.init();
        MecanumDrive mecanumDrive = new MecanumDrive(
                hardware.leftFront, hardware.leftBack, hardware.rightBack, hardware.rightFront,
                hardware.imu, hardware.voltageSensor, new Pose2d(0, 0, 0));
        nav = subsystems.add(getNav(mecanumDrive));
        nav.init();
        drive.setPoseSupplier(() -> nav.currentPose().pose2d);
        turntable = subsystems.add(new Turntable(hardware.turnTable, runtime, telemetry));
        turntable.init();
        brain = subsystems.add(new Brain(runtime, telemetry, launcher, drive, camera, nav, turntable));
        brain.init();
        telemetry.addData("base.OpMode.init()", true);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    /** Registers something to tick after the subsystems, in registration order. */
    protected <T extends Loopable> T add(T loopable) {
        return subsystems.add(loopable);
    }

    /** Everything this op mode ticks, in order. */
    public List<Loopable> loopOrder() {
        return subsystems.members();
    }

    /** Ticks every registered subsystem, then {@link #onLoop()}. Subclasses override {@link #onLoop()}. */
    @Override
    public final void loop() {
        handleTelemetryToggles();
        subsystems.loop();
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

    protected abstract Nav getNav(MecanumDrive mecanumDrive);
}
