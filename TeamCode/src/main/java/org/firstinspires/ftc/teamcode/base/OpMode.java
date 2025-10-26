package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    protected Launcher launcher;
    Drive drive;

    @Override
    public void init() {
        drive = new Drive(hardwareMap, runtime, telemetry);
        drive.init();
        launcher = new Launcher(hardwareMap, runtime, telemetry);
        launcher.init();
        telemetry.addData("base.OpMode.init()", true);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    public void loop() {
        if (gamepad2.crossWasPressed()) { drive.toggleTelemetry(); }
        if (gamepad2.squareWasPressed()) { launcher.toggleTelemetry(); }
        drive.loop();
        launcher.loop();
    }
}
