package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name = "BlueFarScoreAThing", group = "Autonomous")
public class BlueFarScoreAThing extends AutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.blueFarScoreAThing();
    }
}
