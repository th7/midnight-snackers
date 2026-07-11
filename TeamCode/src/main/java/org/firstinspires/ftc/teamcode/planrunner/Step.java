package org.firstinspires.ftc.teamcode.planrunner;

import java.util.function.LongPredicate;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

public class Step implements PlanPart {
    private final String name;
    private final Runnable start;
    private final Supplier<Boolean> done;
    private final LongSupplier clock;
    private long nanoStartedAt;
    private boolean started = false;

    public Step(String name, Runnable start, Supplier<Boolean> done) {
        this(name, start, done, System::nanoTime);
    }

    Step(String name, Runnable start, Supplier<Boolean> done, LongSupplier clock) {
        this.name = name;
        this.start = start;
        this.done = done;
        this.clock = clock;
    }

    public Step(String name, Runnable start, LongPredicate elapsedNanosChecker) {
        this(name, start, elapsedNanosChecker, System::nanoTime);
    }

    Step(String name, Runnable start, LongPredicate elapsedNanosChecker, LongSupplier clock) {
        this.name = name;
        this.start = start;
        this.done = () -> elapsedNanosChecker.test(clock.getAsLong() - nanoStartedAt);
        this.clock = clock;
    }

    public static LongPredicate secondsElapsed(double seconds) {
        return (elapsedNanos) -> elapsedNanos / 1_000_000_000d > seconds;
    }

    public static Step waitFor(String label, double seconds) {
        return new Step(
                label + " waitFor " + seconds,
                () -> {
                },
                Step.secondsElapsed(seconds)
        );
    }

    public boolean done() {
        if (!started) {
            start.run();
            started = true;
            nanoStartedAt = clock.getAsLong();
        }

        return this.done.get();
    }

    public String currentStep() {
        return this.name;
    }
}
