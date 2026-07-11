package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;

public abstract class AutoOp extends OpMode {
    public Plans plans;
    private final PlanRunner planRunner = new PlanRunner();

    @Override
    public void init() {
        super.init();
        plans = new Plans(runtime, telemetry, launcher, drive, camera, nav, turntable, brain);
        planRunner.run(getPlan());
        telemetry.addData("AutoOp.init()", true);
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Current Step:", planRunner.currentStep());
        planRunner.loop();
    }

    public abstract PlanPart getPlan();
}
