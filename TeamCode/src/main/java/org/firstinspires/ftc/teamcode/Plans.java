package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Plans {
    public Arm arm;
    public Drive drive;
    public OtherSubSystem otherSubSystem;

    public Plans(Arm arm, Drive drive, OtherSubSystem otherSubSystem) {
        this.arm = arm;
        this.drive = drive;
        this.otherSubSystem = otherSubSystem;
    }

    public Plan splineBasic() {
        return new Plan(
                setZeroPosition(),
                splineBasicStep()
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
