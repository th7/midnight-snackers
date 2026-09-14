package org.firstinspires.ftc.teamcode.planrunner;

import java.util.function.LongPredicate;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/**
 * One part of a plan: something to start, and a test of whether it is done. A <b>timed</b> step is
 * done once enough time has passed on the clock it is given, which is the robot's
 * ({@code robot.clock}), never the system's, so a simulation can own time.
 */
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

    /**
     * A timed step: done when {@code elapsedNanosChecker} accepts the nanoseconds since it started,
     * measured on {@code clock}.
     */
    public Step(String name, Runnable start, LongPredicate elapsedNanosChecker, LongSupplier clock) {
        this.name = name;
        this.start = start;
        this.done = () -> elapsedNanosChecker.test(clock.getAsLong() - nanoStartedAt);
        this.clock = clock;
    }

    public static LongPredicate secondsElapsed(double seconds) {
        return (elapsedNanos) -> elapsedNanos / 1_000_000_000d > seconds;
    }

    /** A step that does nothing but wait {@code seconds} on {@code clock}. */
    public static Step waitFor(String label, double seconds, LongSupplier clock) {
        return new Step(label + " waitFor " + seconds, () -> {}, Step.secondsElapsed(seconds), clock);
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
