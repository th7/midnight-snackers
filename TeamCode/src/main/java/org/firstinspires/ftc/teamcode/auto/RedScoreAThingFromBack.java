package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.RedAutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name = "RedScoreAThingFromBack", group = "Autonomous")
public class RedScoreAThingFromBack extends RedAutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.scoreAThingFromBack();
    }
}
