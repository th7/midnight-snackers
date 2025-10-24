package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.planrunner.VoidCallable;

public class Plans {
    public Arm arm;
    public Drive drive;
    public Launcher launcher;
    public OtherSubSystem otherSubSystem;
    public ElapsedTime runtime;
    private double startedWaitAt;

    public Plans(Arm arm, Drive drive, Launcher launcher, OtherSubSystem otherSubSystem, ElapsedTime runtime) {
        this.arm = arm;
        this.drive = drive;
        this.launcher = launcher;
        this.otherSubSystem = otherSubSystem;
        this.runtime = runtime;
    }

    public Plan splineBasic() {
        return new Plan(
                setZeroPosition(),
                splineBasicStep()
        );
    }

    public Plan redFarScoreAThing() {
        return farScoreAThing(Alliance.red);
    }

    public Plan redCloseScoreAThing() {
        return nearScoreAThing(Alliance.red);
    }

    public Plan blueFarScoreAThing() {
        return farScoreAThing(Alliance.blue);
    }

    public Plan blueCloseScoreAThing() {
        return nearScoreAThing(Alliance.blue);
    }

    public Plan scoreAThing() {
        return new Plan(
                setZeroPosition(),
                setCloseLaunchPower(),
                waitFor(4),
                waitForFlywheel(),
                launchStep(),
                backFromZeroALittle()

        );
    }

    private Plan farScoreAThing(Alliance alliance) {
        return new Plan(
                setBackPosition(alliance),
                setFarLaunchPowerStep(),
                moveToFarScorePosition(alliance),
                waitForFlywheel(),
                launchStep(),
                getOffLaunchLine(alliance)

        );
    }

    public Plan nearScoreAThing(Alliance alliance) {
        return new Plan(
                setBackPosition(alliance),
                setCloseLaunchPower(),
                moveFromBackToCloseGoal(alliance),
                waitForFlywheel(),
                launchStep(),
                getOffLaunchLine(alliance)
        );
    }

    public Plan splineOneStep() {
        return new Plan(
                setZeroPosition(),
                splineOneStepStep()
        );
    }

    public Plan splineSquiggleSquare() {
        return new Plan(
                setZeroPosition(),
                splineSquiggleSquareStep(),
                toZeroPosition()
        );
    }

    public Plan splineUsingPoses() {
        return new Plan(
                setZeroPosition(),
                splineUsingPosesStep()
        );
    }

    public Plan splineUsingCoords() {
        return new Plan(
                setZeroPosition(),
                splineUsingCoordsStep()
        );
    }

    private Step splineBasicStep() {
        return new Step(
                "splineBasic",
                drive::splineBasic,
                drive::done
        );
    }

    private Step setFarLaunchPowerStep() {
        return new Step(
                "setFarLaunchPower",
                launcher::setFarLaunchPower,
                () -> true
        );
    }

    private Step setCloseLaunchPower() {
        return new Step(
                "setCloseLaunchPower",
                launcher::setCloseLaunchPower,
                () -> true
        );
    }

    private Step waitForFlywheel() {
        return new Step(
                "waitForFlywheel",
                () -> {
                },
                launcher::flywheelReady
        );
    }

    private Step waitFor(double seconds) {
        return new Step(
                "waitFor " + seconds,
                () -> { startedWaitAt = runtime.time(); },
                () -> { return runtime.time() >= startedWaitAt + seconds; }
        );
    }
    private Step launchStep() {
        return new Step(
                "launch",
                launcher::launch,
                launcher::done
        );
    }

    private Step splineOneStepStep() {
        return new Step(
                "splineOneStep",
                drive::splineOneStep,
                drive::done
        );
    }

    private Step splineSquiggleSquareStep() {
        return new Step(
                "splineSquiggleSquare",
                drive::splineSquiggleSquare,
                drive::done
        );
    }

    private Step toZeroPosition() {
        return new Step(
                "toZeroPosition",
                drive::toZeroPosition,
                drive::done
        );
    }

    private Step splineUsingPosesStep() {
        return new Step(
                "splineUsingPoses",
                drive::splineUsingPoses,
                drive::done
        );
    }

    private Step splineUsingCoordsStep() {
        return new Step(
                "splineUsingCoords",
                drive::splineUsingCoords,
                drive::done
        );
    }

    private Step setZeroPosition() {
        return new Step(
                "setZeroPosition",
                () -> drive.setPose(new Pose2d(0, 0, 0)),
                () -> true
        );
    }

    //should be placed against the left side of the tile with the small launch line and against the wall
    private Step setBackPosition(Alliance alliance) {
        return new Step(
                "setBackPosition",
                () -> drive.setPose(alliance.pose(-63.5, 15.375, 0)),
                () -> true
        );
    }

    private Step setGoalPosition(Alliance alliance) {
        return new Step(
                "setBackPosition",
                () -> drive.setPose(alliance.pose(-63.5, 15.375, 0)), //not set
                () -> true
        );
    }

    private Step moveToFarScorePosition(Alliance alliance) {
        return new Step(
                "moveToFarScorePosition",
                () -> drive.drivePath(alliance.pose(33, 33, Math.PI / 4)),
                drive::done
        );
    }

    private Step moveFromBackToCloseGoal(Alliance alliance) {
        return new Step(
                "moveFromBackToCloseGoal",
                () -> drive.drivePath(
                        alliance.pose(12, 12, Math.PI / 4),
                        alliance.pose(48, 48, Math.PI / 4)
                ),
                drive::done
        );
    }

    private Step getOffLaunchLine(Alliance alliance) {
        return new Step(
                "getOffLaunchLine",
                () -> drive.drivePath(
                        alliance.pose(24, 24, Math.PI / 4),
                        alliance.pose(0, 24, Math.PI / 4)
                ),
                drive::done
        );
    }

    private Step backFromZeroALittle() {
        return new Step(
                "backFromZeroALittle",
                () -> drive.drivePath(
                        new Pose2d(-12, 0, 0)
                ),
                drive::done
        );
    }

    private Pose2d poseFacingGoal(Alliance alliance, int x, int y) {
        return alliance.poseFacingGoal(x, y);
    }
}
