package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.OtherSubSystem;
import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

import java.lang.reflect.Field;

public abstract class AutoOp extends OpMode {
    public Plans plans;
    private PlanPart plan;

    @Override
    public void init() {
        super.init();
        arm.initAuto();
        drive.initAuto();
        otherSubSystem.initAuto();
        plans = new Plans(arm, drive, otherSubSystem);
        plan = getPlan();
        telemetry.addData("AutoOp.init()", true);
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Current Step:", plan.currentStep());
        plan.done();
    }

    public abstract PlanPart getPlan();
}
