package org.firstinspires.ftc.teamcode.planrunner;

import java.util.ArrayList;

public class Plan implements PlanPart {
    private final ArrayList<PlanPart> planParts = new ArrayList<>();
    private int currentPlanPartIndex = 0;

    public Plan(PlanPart... all) {
        addAll(all);
    }

    public String currentStep() {
        PlanPart currentPlanPart = currentPlanPart();
        if (currentPlanPart == null) {
            return "null";
        } else {
            return currentPlanPartIndex + ". " + currentPlanPart.currentStep();
        }
    }

    @Override
    public boolean tick() {
        PlanPart currentPlanPart = currentPlanPart();
        if (currentPlanPart == null) {
            return true;
        }
        if (currentPlanPart.tick()) {
            currentPlanPartIndex += 1;
        }
        return currentPlanPart() == null;
    }

    private PlanPart currentPlanPart() {
        try {
            return planParts.get(currentPlanPartIndex);
        } catch (IndexOutOfBoundsException e) {
            return null;
        }
    }

    private void add(PlanPart planPart) {
        planParts.add(planPart);
    }

    private void addAll(PlanPart... all) {
        for (PlanPart planPart : all) {
            add(planPart);
        }
    }
}
