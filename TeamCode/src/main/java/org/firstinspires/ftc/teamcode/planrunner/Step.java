package org.firstinspires.ftc.teamcode.planrunner;

import java.util.function.LongPredicate;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

public class Step implements PlanPart {
    private final String name;
    private final Runnable start;
    private final Supplier<Boolean> done;
    private final LongSupplier nanoClock;
    private long nanoStartedAt;
    private boolean started = false;

    public Step(String name, Runnable start, Supplier<Boolean> done) {
        this(name, start, done, System::nanoTime);
    }

    Step(String name, Runnable start, Supplier<Boolean> done, LongSupplier nanoClock) {
        this.name = name;
        this.start = start;
        this.done = done;
        this.nanoClock = nanoClock;
    }

    public Step(String name, Runnable start, LongPredicate elapsedNanosChecker, LongSupplier nanoClock) {
        this.name = name;
        this.start = start;
        this.done = () -> elapsedNanosChecker.test(nanoClock.getAsLong() - nanoStartedAt);
        this.nanoClock = nanoClock;
    }

    public static LongPredicate secondsElapsed(double seconds) {
        return (elapsedNanos) -> elapsedNanos / 1_000_000_000d > seconds;
    }

    public static Step waitFor(String label, double seconds, LongSupplier nanoClock) {
        return new Step(label + " waitFor " + seconds, () -> {}, Step.secondsElapsed(seconds), nanoClock);
    }

    @Override
    public boolean tick() {
        if (!started) {
            start.run();
            started = true;
            nanoStartedAt = nanoClock.getAsLong();
        }

        return this.done.get();
    }

    public String currentStep() {
        return this.name;
    }
}
