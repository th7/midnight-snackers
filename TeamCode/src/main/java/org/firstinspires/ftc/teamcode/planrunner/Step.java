package org.firstinspires.ftc.teamcode.planrunner;

public class Step implements PlanPart {
    private final String name;
    private long nanoStartedAt;
    private final VoidCallable start;
    private final Callable<Boolean> done;
    private final StepDataCallable<Boolean> stepDataDone;
    private boolean started = false;

    public Step(String name, VoidCallable start, Callable<Boolean> done) {
        this.name = name;
        this.start = start;
        this.done = done;
        this.stepDataDone = null;
    }

    public Step(String name, VoidCallable start, StepDataCallable<Boolean> stepDataDone) {
        this.name = name;
        this.start = start;
        this.done = null;
        this.stepDataDone = stepDataDone;
    }

    public boolean done() {
        if (!started) {
            try {
                start.call();
            } catch (IllegalAccessException | InstantiationException e) {
                throw new RuntimeException(e);
            }
            started = true;
            nanoStartedAt = System.nanoTime();
        }

        if (this.stepDataDone != null) {
            return this.stepDataDone.call(new StepData(System.nanoTime()));
        }
        if (this.done != null) {
            return this.done.call();
        }

    }

    public String currentStep() {
        return this.name;
    }
}
