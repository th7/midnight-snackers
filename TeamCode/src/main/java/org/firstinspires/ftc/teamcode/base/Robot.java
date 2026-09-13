package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Turntable;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

/**
 * Every subsystem, built afresh for one run of an op mode, plus what they all share. A subsystem
 * reaches the others, the telemetry and the dashboard through the robot it was {@link #add}ed to
 * instead of being handed each one.
 */
public final class Robot implements Loopable {
    public final Telemetry telemetry;
    public final Dashboard dashboard;
    public final Launcher launcher;
    public final Drive drive;
    public final Camera camera;
    public final Nav nav;
    public final Turntable turntable;
    public final Brain brain;
    private final LoopGroup loop = new LoopGroup();

    public Robot(Hardware hardware, Alliance alliance, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.dashboard = hardware.dashboard;
        launcher = new Launcher(hardware.launcher, hardware.topGate, hardware.bottomGate);
        drive = new Drive(hardware.leftFront, hardware.rightFront, hardware.leftBack, hardware.rightBack);
        camera = new Camera(hardware.aprilTags);
        nav = new Nav(new MecanumDrive(
                hardware.leftFront, hardware.leftBack, hardware.rightBack, hardware.rightFront,
                hardware.imu, hardware.voltageSensor, new Pose2d(0, 0, 0)), alliance);
        turntable = new Turntable(hardware.turnTable);
        brain = new Brain();
        // Registration order is loop order. Brain goes last because it reads Camera and Nav
        // from this tick and sets the Turntable target.
        for (SubSystem subSystem : List.of(launcher, drive, camera, nav, turntable, brain)) {
            add(subSystem);
        }
    }

    /**
     * Registers something to tick after everything registered so far. A subsystem also learns
     * this robot and is initialised.
     */
    public <T extends Loopable> T add(T loopable) {
        if (loopable instanceof SubSystem) {
            ((SubSystem) loopable).attach(this);
        }
        loop.add(loopable);
        if (loopable instanceof SubSystem) {
            ((SubSystem) loopable).init();
        }
        return loopable;
    }

    /** Everything this robot ticks, in order. */
    public List<Loopable> loopOrder() {
        return loop.members();
    }

    @Override
    public void loop() {
        loop.loop();
    }
}
