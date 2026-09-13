package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Turntable;

/**
 * A subsystem that coordinates the others, so it sees each of them by name. It does nothing per
 * tick unless it overrides {@link #onLoop()}.
 */
public abstract class SuperSystem extends SubSystem {
    protected Launcher launcher;
    protected Drive drive;
    protected Camera camera;
    protected Nav nav;
    protected Turntable turntable;
    protected Brain brain;

    @Override
    void attach(Robot robot) {
        super.attach(robot);
        launcher = robot.launcher;
        drive = robot.drive;
        camera = robot.camera;
        nav = robot.nav;
        turntable = robot.turntable;
        brain = robot.brain;
    }

    @Override
    protected void onLoop() {
    }
}
