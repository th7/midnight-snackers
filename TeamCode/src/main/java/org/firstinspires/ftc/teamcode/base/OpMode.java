package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Turntable;

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

    @Override
    public void init() {
        runtime = new ElapsedTime();
        launcher = new Launcher(hardwareMap, runtime, telemetry);
        launcher.init();
        drive = new Drive(hardwareMap, runtime, telemetry);
        drive.init();
        camera = new Camera(hardwareMap, runtime, telemetry);
        camera.init();
        nav = getNav();
        nav.init();
        turntable = new Turntable(hardwareMap, runtime, telemetry);
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

        if (gamepad2.crossWasPressed()) { drive.toggleTelemetry(); }
        if (gamepad2.squareWasPressed()) {
            turntable.toggleTelemetry();
            launcher.toggleTelemetry();
        }
        if (gamepad2.circleWasPressed()) { camera.toggleTelemetry(); }

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

    protected abstract Nav getNav();
}
