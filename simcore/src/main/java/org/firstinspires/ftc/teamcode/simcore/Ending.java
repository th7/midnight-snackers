package org.firstinspires.ftc.teamcode.simcore;

import java.util.Optional;

/** How a run ends, which is asked before every loop of its op mode. */
public enum Ending {
    /** The driver pressed Stop. */
    STOPPED,

    /** An auto's plan is done, or a TeleOp's period is over. */
    DONE,

    /** An auto's plan was not done within its period. */
    TIMED_OUT;

    /** Where an op mode's plan has got: a TeleOp has none, and an auto's is under way or done. */
    public enum Plan {
        NONE,
        UNDER_WAY,
        DONE
    }

    /** How the run ends before its next loop, or nothing when it loops again. */
    public static Optional<Ending> before(boolean stopRequested, Plan plan, Seconds elapsed, Budget budget) {
        if (stopRequested) {
            return Optional.of(Ending.STOPPED);
        }
        boolean timeIsUp = budget.spentBy(elapsed);
        return switch (plan) {
            case DONE -> Optional.of(Ending.DONE);
            case NONE -> timeIsUp ? Optional.of(Ending.DONE) : Optional.empty();
            case UNDER_WAY -> timeIsUp ? Optional.of(Ending.TIMED_OUT) : Optional.empty();
        };
    }
}
