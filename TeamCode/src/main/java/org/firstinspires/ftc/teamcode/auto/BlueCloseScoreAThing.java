package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name="BlueCloseScoreAThing", group="Autonomous")
public class BlueCloseScoreAThing extends AutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.blueCloseScoreAThing();
    }
}
