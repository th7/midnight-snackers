package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    protected Brain brain;
    protected Launcher launcher;
    protected Drive drive;
    protected Camera camera;
    protected ElapsedTime runtime;

    @Override
    public void init() {
        brain = new Brain(hardwareMap, new ElapsedTime(), telemetry);
        brain.init();
        runtime = brain.runtime;
        drive = brain.drive;
        launcher = brain.launcher;
        camera = brain.camera;
        telemetry.addData("base.OpMode.init()", true);
    }

    @Override
    public void start() {
        brain.runtime.reset();
    }

    public void loop() {
        if (gamepad2.crossWasPressed()) { drive.toggleTelemetry(); }
        if (gamepad2.squareWasPressed()) { launcher.toggleTelemetry(); }
        if (gamepad2.circleWasPressed()) {camera.toggleTelemetry(); }
        brain.loop();
    }
}
