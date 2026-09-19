package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;

public abstract class AutoOp extends OpMode {
    private PlanRunner planRunner;

    protected AutoOp(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init() {
        super.init();
        planRunner = new PlanRunner();
        planRunner.run(getPlan());
    }

    @Override
    protected void onLoop() {
        planRunner.loop();
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
