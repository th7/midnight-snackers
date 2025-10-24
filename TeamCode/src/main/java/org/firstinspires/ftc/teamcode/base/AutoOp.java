package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;

public abstract class AutoOp extends OpMode {
    public Plans plans;
    private PlanPart plan;

    @Override
    public void init() {
        super.init();
        drive.initAuto();
        plans = new Plans(drive, launcher, runtime);
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
