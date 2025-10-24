package org.firstinspires.ftc.teamcode.planrunner;

public class Step implements PlanPart {
    private final String name;
    private final VoidCallable start;
    private final Callable<Boolean> done;
    private boolean started = false;

    public Step(VoidCallable start, Callable<Boolean> done) {
        this.name = "";
        this.start = start;
        this.done = done;
    }

    public Step(String name, VoidCallable start, Callable<Boolean> done) {
        this.name = name;
        this.start = start;
        this.done = done;
    }

    public boolean done() {
        if (!started) {
            try {
                start.call();
            } catch (IllegalAccessException | InstantiationException e) {
                throw new RuntimeException(e);
            }
            started = true;
        }

        return this.done.call();
    }

    public String currentStep() {
        return this.name;
    }
}
