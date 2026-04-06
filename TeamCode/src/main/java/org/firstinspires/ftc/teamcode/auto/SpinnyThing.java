package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.RelativeAutoOp;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

@Autonomous(name = "SpinnyThing", group = "Autonomous")
public class SpinnyThing extends RelativeAutoOp {
    @Override
    public PlanPart getPlan() {
        return plans.spinnyThing();
    }
}
