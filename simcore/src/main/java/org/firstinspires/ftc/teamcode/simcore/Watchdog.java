package org.firstinspires.ftc.teamcode.simcore;

/**
 * What a bench holds a run's child to, and when it stops waiting for it: one function of how far the
 * run has got, the time on the bench's clock, and the waits.
 */
public final class Watchdog {
    private Watchdog() {}

    /** How long the watchdog waits for each thing it waits for. */
    public record Waits(Seconds startup, Seconds silence, Seconds unwatched, Seconds killGrace, Seconds view) {}

    public enum Verdict {
        /** Nothing has run out yet: look again later. */
        WATCHING,

        /** The run is over, or its child has ended: there is nothing left to watch. */
        OVER,

        /** The op mode's time did not begin within the startup: the child is killed. */
        NEVER_STARTED,

        /** The child said nothing for longer than the silence, so its loop never came back: it is killed. */
        SILENT,

        /** Nobody asked about the run for longer than it may go unwatched: it is stopped, and the child killed. */
        UNWATCHED,

        /** The child did not end within its grace after Stop: it is killed. */
        IGNORED_STOP,

        VIEW_LATE;

        public boolean keepsWatching() {
            return this == WATCHING || this == VIEW_LATE;
        }
    }

    public static Verdict verdict(RunState run, Moment now, Waits waits) {
        return run.watched(now, waits);
    }
}
