package org.firstinspires.ftc.teamcode.planrunner;

public class Step {
    private boolean started = false;
    private final String name;
    private final VoidCallable start;
    private final Callable<Boolean> done;

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
        if (!started) { start.call(); started = true; }

        return this.done.call();
    }

    public String currentStep() {
        return this.name;
    }
}
