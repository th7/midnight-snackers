package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.Plans;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/** An op mode that runs a plan for one alliance from start until the plan is done. */
public abstract class AutoOp extends OpMode {
    private final Alliance alliance;
    public Plans plans;
    private PlanRunner planRunner;

    protected AutoOp(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance alliance() {
        return alliance;
    }

    @Override
    protected Nav getNav(MecanumDrive mecanumDrive) {
        return alliance.nav(mecanumDrive, runtime, telemetry);
    }

    @Override
    public void init() {
        super.init();
        if (!alliance.usesCameraLocalization()) {
            brain.disableCameraLocalization();
        }
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
