package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Turntable;

public class SuperSystem {
    protected final ElapsedTime runtime;
    protected final Telemetry telemetry;
    protected final Turntable turntable;
    protected Launcher launcher;
    protected Drive drive;
    protected Camera camera;
    protected Nav nav;

    public SuperSystem(ElapsedTime runtime, Telemetry telemetry, Launcher launcher, Drive drive, Camera camera, Nav nav, Turntable turntable) {
        this.runtime = runtime;
        this.telemetry = telemetry;
        this.launcher = launcher;
        this.drive = drive;
        this.camera = camera;
        this.nav = nav;
        this.turntable = turntable;
    }
}
