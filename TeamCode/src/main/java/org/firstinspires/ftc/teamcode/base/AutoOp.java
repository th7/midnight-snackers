package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;

/** An op mode that runs a plan for one alliance from start until the plan is done. */
public abstract class AutoOp extends OpMode {
    private PlanRunner planRunner;

    protected AutoOp(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init() {
        super.init();
        planRunner = add(new PlanRunner());
        planRunner.run(getPlan());
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
