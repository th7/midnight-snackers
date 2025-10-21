package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Plans {
    public Arm arm;
    public Drive drive;
    public Launcher launcher;
    public OtherSubSystem otherSubSystem;

    public Plans(Arm arm, Drive drive, Launcher launcher, OtherSubSystem otherSubSystem) {
        this.arm = arm;
        this.drive = drive;
        this.launcher = launcher;
        this.otherSubSystem = otherSubSystem;
    }

    public Plan splineBasic() {
        return new Plan(
                setZeroPosition(),
                splineBasicStep()
        );
    }

    public Plan blueFarScoreAThing() {
        return new Plan(
                setZeroPosition(),
                setFarLaunchPowerStep(),
                blueFarScoreAThingStep(),
                waitforFlywheel(),
                launchStep()

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

    private Step blueFarScoreAThingStep() {
        return new Step(
                "blueFarScoreAThing",
                drive::blueFarScoreAThing,
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

    private Step waitforFlywheel() {
        return new Step(
                "waitForFlywheel",
                () -> {},
                launcher::flywheelReady
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
}
