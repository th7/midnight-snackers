package org.firstinspires.ftc.teamcode.planrunner;

public class Step implements PlanPart {
    private final String name;
    private long nanoStartedAt;
    private final VoidCallable start;
    private final Callable<Boolean> done;
    private boolean started = false;

    public Step(String name, VoidCallable start, Callable<Boolean> done) {
        this.name = name;
        this.start = start;
        this.done = done;
    }

    public Step(String name, VoidCallable start, StepDataCallable<Boolean> stepDataDone) {
        this.name = name;
        this.start = start;
        this.done = () -> stepDataDone.call(new StepData(nanoStartedAt));
    }

    public boolean done() {
        if (!started) {
            start.call();
            started = true;
            nanoStartedAt = System.nanoTime();
        }

        return this.done.call();
    }

    public String currentStep() {
        return this.name;
    }
}
