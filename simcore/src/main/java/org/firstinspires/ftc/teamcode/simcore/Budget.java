package org.firstinspires.ftc.teamcode.simcore;

/** How long a run may go on the simulation's clock: a period, or no limit at all. */
public sealed interface Budget permits Budget.Period, Budget.NoLimit {
    /** A period in seconds, or infinity for no limit, which is how Java writes and reads no limit. */
    static Checked<Budget> of(double seconds) {
        if (seconds == Double.POSITIVE_INFINITY) {
            return Checked.ok(new NoLimit());
        }
        return Seconds.of(seconds)
                .fold(
                        period -> Checked.ok(new Period(period)),
                        rule -> Checked.rejected(
                                "a run's budget is a time in seconds, or Infinity for no limit, not " + seconds));
    }

    /** Whether a run that has been going for elapsed has gone past it. */
    boolean spentBy(Seconds elapsed);

    record Period(Seconds seconds) implements Budget {
        @Override
        public boolean spentBy(Seconds elapsed) {
            return elapsed.value() > seconds.value();
        }
    }

    record NoLimit() implements Budget {
        @Override
        public boolean spentBy(Seconds elapsed) {
            return false;
        }
    }
}
