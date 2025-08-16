package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name="SplineOneStep", group="Autonomous")
public class SplineOneStep extends AutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.splineOneStep();
    }
}
