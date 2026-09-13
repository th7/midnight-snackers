package org.firstinspires.ftc.teamcode.planrunner;

import org.firstinspires.ftc.teamcode.base.Loopable;

public class PlanRunner implements Loopable {
    private PlanPart currentPlan = null;

    public void run(PlanPart plan) {
        if (currentPlan == null) {
            currentPlan = plan;
        }
    }

    @Override
    public void loop() {
        if (currentPlan != null && currentPlan.done()) {
            currentPlan = null;
        }
    }

    public boolean done() {
        return currentPlan == null;
    }

    public void cancel() {
        currentPlan = null;
    }

    public String currentStep() {
        if (currentPlan == null) {
            return "no plan";
        }
        return currentPlan.currentStep();
    }
}
