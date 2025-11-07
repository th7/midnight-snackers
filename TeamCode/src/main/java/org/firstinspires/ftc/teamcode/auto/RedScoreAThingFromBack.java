package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name = "RedScoreAThingFromBack", group = "Autonomous")
public class RedScoreAThingFromBack extends AutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.redScoreAThingFromBack();
    }
}
