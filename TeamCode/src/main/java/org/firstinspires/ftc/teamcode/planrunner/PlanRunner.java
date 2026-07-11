package org.firstinspires.ftc.teamcode.planrunner;

public class PlanRunner {
    private PlanPart currentPlan = null;

    public void run(PlanPart plan) {
        if (currentPlan == null) {
            currentPlan = plan;
        }
    }

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
