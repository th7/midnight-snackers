package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;

public abstract class AutoOp extends OpMode {
    public Plans plans;
    private PlanRunner planRunner;

    @Override
    public void init() {
        super.init();
        plans = new Plans(runtime, telemetry, launcher, drive, camera, nav, turntable, brain);
        planRunner = add(new PlanRunner());
        planRunner.run(getPlan());
        telemetry.addData("AutoOp.init()", true);
    }

    @Override
    protected void onLoop() {
        telemetry.addData("Current Step:", planRunner.currentStep());
    }

    public abstract PlanPart getPlan();

    public boolean done() {
        return planRunner.done();
    }

    public String currentStep() {
        return planRunner.currentStep();
    }
}
