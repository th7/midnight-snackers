package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.BlueAutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name = "BlueScoreAThingFromBack", group = "Autonomous")
public class BlueScoreAThingFromBack extends BlueAutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.scoreAThingFromBack();
    }
}
