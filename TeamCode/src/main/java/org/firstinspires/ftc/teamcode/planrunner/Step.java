package org.firstinspires.ftc.teamcode.planrunner;

import java.util.function.LongPredicate;
import java.util.function.Supplier;

public class Step implements PlanPart {
    private final String name;
    private long nanoStartedAt;
    private final Runnable start;
    private final Supplier<Boolean> done;
    private boolean started = false;

    public Step(String name, Runnable start, Supplier<Boolean> done) {
        this.name = name;
        this.start = start;
        this.done = done;
    }

    public Step(String name, Runnable start, LongPredicate nanoStartedAtChecker) {
        this.name = name;
        this.start = start;
        this.done = () -> nanoStartedAtChecker.test(nanoStartedAt);

    }

    public static LongPredicate secondsElapsed(double seconds) {
        return (nanoStartedAt) -> (System.nanoTime() - nanoStartedAt) / 1_000_000_000d > seconds;
    }

    public static Step waitFor(String label, double seconds) {
        return new Step(
                label + " waitFor " + seconds,
                () -> {},
                Step.secondsElapsed(seconds)
        );
    }

    public boolean done() {
        if (!started) {
            start.run();
            started = true;
            nanoStartedAt = System.nanoTime();
        }

        return this.done.get();
    }

    public String currentStep() {
        return this.name;
    }
}
