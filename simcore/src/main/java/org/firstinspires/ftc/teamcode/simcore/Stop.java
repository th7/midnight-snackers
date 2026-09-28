package org.firstinspires.ftc.teamcode.simcore;

/** What pressing Stop does to a run, by how far the run has got: see {@link RunState#onStop}. */
public enum Stop {
    /** Nothing: the run is over, or its child has been told already and its grace runs from then. */
    NOTHING,

    /** There is no child to tell yet, so the run ends stopped where it is. */
    END_UNBUILT,

    /**
     * The child is told to stop. One that hears has its grace to end in, which the watchdog holds it
     * to; one that cannot be told is killed.
     */
    TELL
}
