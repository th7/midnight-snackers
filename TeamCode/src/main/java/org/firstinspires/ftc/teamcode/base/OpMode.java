package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.ExampleSubsystem;
import org.firstinspires.ftc.teamcode.Launcher;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    protected Brain brain;
    protected Launcher launcher;
    protected Drive drive;
    protected Camera camera;
    protected ExampleSubsystem exampleSubsystem;
    protected ElapsedTime runtime;
    private long tickCount = 0;
    private double lastTickAt = 0;
    private double maxTickSeconds = 0;

    @Override
    public void init() {
        brain = new Brain(hardwareMap, new ElapsedTime(), telemetry);
        brain.init();
        runtime = brain.runtime;
        drive = brain.drive;
        launcher = brain.launcher;
        camera = brain.camera;
        exampleSubsystem = new ExampleSubsystem(runtime, telemetry);
        Trigger exampleTrigger = new Trigger(gamepad2::triangleWasPressed);
        exampleTrigger.onTrue(exampleSubsystem.outputTwice());
        telemetry.addData("base.OpMode.init()", true);
    }

    @Override
    public void start() {
        brain.runtime.reset();
    }

    public void loop() {
        tickCount += 1;
        double currentTickAt = runtime.time();
        double lastTickSeconds = currentTickAt - lastTickAt;
        lastTickAt = currentTickAt;
        if (lastTickSeconds > maxTickSeconds) {
            maxTickSeconds = lastTickSeconds;
        }
        telemetry.addData("Tick (last, avg, max)", "%.03f, %.03f, %.03f", lastTickSeconds, runtime.milliseconds() / tickCount / 1000, maxTickSeconds);

        if (gamepad2.crossWasPressed()) { drive.toggleTelemetry(); }
        if (gamepad2.squareWasPressed()) { launcher.toggleTelemetry(); }
        if (gamepad2.circleWasPressed()) {camera.toggleTelemetry(); }

        brain.loop();

//        if (gamepad2.triangleWasPressed()) { CommandScheduler.getInstance().schedule(exampleSubsystem.outputTime()); }
        CommandScheduler.getInstance().run();
    }
}
