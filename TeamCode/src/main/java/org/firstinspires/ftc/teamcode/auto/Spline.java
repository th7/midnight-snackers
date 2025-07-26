package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name="Spline", group="Autonomous")
public class Spline extends AutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.splineBasic();
    }
}
