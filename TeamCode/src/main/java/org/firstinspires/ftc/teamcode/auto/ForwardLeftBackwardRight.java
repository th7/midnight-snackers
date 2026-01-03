package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name = "ForwardLeftBackwardRight", group = "Autonomous")
public class ForwardLeftBackwardRight extends AutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.forwardLeftBackwardRight();
    }
}
