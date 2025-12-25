package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;

public abstract class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private Brain brain;
    protected Launcher launcher;
    protected Drive drive;
    protected ElapsedTime runtime;

    @Override
    public void init() {
        brain = new Brain(hardwareMap, new ElapsedTime(), telemetry);
        brain.init();
        runtime = brain.runtime;
        drive = brain.drive;
        launcher = brain.launcher;
        telemetry.addData("base.OpMode.init()", true);
    }

    @Override
    public void start() {
        brain.runtime.reset();
    }

    public void loop() {
        if (gamepad2.crossWasPressed()) { drive.toggleTelemetry(); }
        if (gamepad2.squareWasPressed()) { launcher.toggleTelemetry(); }
        brain.loop();
    }
}
