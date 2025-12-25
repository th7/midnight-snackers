package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;

public class SuperSystem {
    private final HardwareMap hardwareMap;
    public final ElapsedTime runtime;
    public final Telemetry telemetry;

    protected Launcher launcher;
    protected Drive drive;

    public SuperSystem(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    public void init() {
        drive = new Drive(hardwareMap, runtime, telemetry);
        drive.init();
        launcher = new Launcher(hardwareMap, runtime, telemetry);
        launcher.init();
        telemetry.addData("base.SuperSystem.init()", true);
    }

    public void loop() {
        drive.loop();
        launcher.loop();
    }
}
