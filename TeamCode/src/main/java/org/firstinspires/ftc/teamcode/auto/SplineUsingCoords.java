package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name="SplineUsingCoords", group="Autonomous")
public class SplineUsingCoords extends AutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.splineUsingCoords();
    }
}
