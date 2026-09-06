package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Turntable;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    protected ElapsedTime runtime;
    protected Launcher launcher;
    protected Drive drive;
    protected Camera camera;

    protected Nav nav;
    protected Turntable turntable;
    protected Brain brain;
//    private long tickCount = 0;
//    private double lastTickAt = 0;
//    private double maxTickSeconds = 0;

    /**
     * The devices this op mode drives. The robot resolves them from the configuration;
     * a simulation overrides this to supply fakes.
     */
    protected Hardware hardware() {
        return Hardware.fromHardwareMap(hardwareMap);
    }

    @Override
    public void init() {
        Hardware hardware = hardware();
        // Mirror all telemetry to the FTC Dashboard as well as the Driver Station.
        // Must happen before subsystems are built, since they capture the telemetry reference.
        telemetry = new MultipleTelemetry(telemetry, hardware.dashboard.telemetry());
        runtime = new ElapsedTime();
        launcher = new Launcher(hardware.launcher, hardware.topGate, hardware.bottomGate, runtime, telemetry);
        launcher.init();
        drive = new Drive(
                hardware.leftFront, hardware.rightFront, hardware.leftBack, hardware.rightBack,
                hardware.dashboard, runtime, telemetry);
        drive.init();
        camera = new Camera(hardware.aprilTags, runtime, telemetry);
        camera.init();
        MecanumDrive mecanumDrive = new MecanumDrive(
                hardware.leftFront, hardware.leftBack, hardware.rightBack, hardware.rightFront,
                hardware.imu, hardware.voltageSensor, new Pose2d(0, 0, 0));
        nav = getNav(mecanumDrive);
        nav.init();
        drive.setPoseSupplier(() -> nav.currentPose().pose2d);
        turntable = new Turntable(hardware.turnTable, runtime, telemetry);
        turntable.init();
        brain = new Brain(runtime, telemetry, launcher, drive, camera, nav, turntable);
        telemetry.addData("base.OpMode.init()", true);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    public void loop() {
//        profileTicks();

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

        launcher.loop();
        drive.loop();
        camera.loop();
        nav.loop();
        turntable.loop();
        brain.loop();
    }

//    private void profileTicks() {
//        tickCount += 1;
//        double currentTickAt = runtime.time();
//        double lastTickSeconds = currentTickAt - lastTickAt;
//        lastTickAt = currentTickAt;
//        if (lastTickSeconds > maxTickSeconds) {
//            maxTickSeconds = lastTickSeconds;
//        }
//        telemetry.addData("Tick (last, avg, max)", "%.03f, %.03f, %.03f", lastTickSeconds, runtime.milliseconds() / tickCount / 1000, maxTickSeconds);
//    }

    protected abstract Nav getNav(MecanumDrive mecanumDrive);
}
