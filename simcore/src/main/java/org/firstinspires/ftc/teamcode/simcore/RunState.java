package org.firstinspires.ftc.teamcode.simcore;

/** How far a run has got, as its bench sees it at one moment: what the watchdog and Stop decide from. */
public sealed interface RunState
        permits RunState.Building, RunState.Starting, RunState.Running, RunState.Stopping, RunState.Over {
    /** What the watchdog makes of the run at now. */
    Watchdog.Verdict watched(Moment now, Watchdog.Waits waits);

    /** What pressing Stop does to it. */
    Stop onStop();

    /** Whether it takes the child just built for it: one Stop ended while it built has no use for one. */
    boolean takesAChild();

    /** Its sources are being built, which takes as long as it takes: there is no child yet. */
    record Building() implements RunState {
        @Override
        public Watchdog.Verdict watched(Moment now, Watchdog.Waits waits) {
            return Watchdog.Verdict.WATCHING;
        }

        @Override
        public Stop onStop() {
            return Stop.END_UNBUILT;
        }

        @Override
        public boolean takesAChild() {
            return true;
        }
    }

    /** Its child was launched, and the op mode's time has not begun: a JVM loading says nothing. */
    record Starting(Moment launched) implements RunState {
        @Override
        public Watchdog.Verdict watched(Moment now, Watchdog.Waits waits) {
            return now.secondsSince(launched) > waits.startup().value()
                    ? Watchdog.Verdict.NEVER_STARTED
                    : Watchdog.Verdict.WATCHING;
        }

        @Override
        public Stop onStop() {
            return Stop.TELL;
        }

        @Override
        public boolean takesAChild() {
            return false;
        }
    }

    /** The op mode's time has begun: its child was last heard at heard, and the run last asked about at looked. */
    record Running(Moment heard, Moment looked) implements RunState {
        @Override
        public Watchdog.Verdict watched(Moment now, Watchdog.Waits waits) {
            if (now.secondsSince(heard) > waits.silence().value()) {
                return Watchdog.Verdict.SILENT;
            }
            if (now.secondsSince(looked) > waits.unwatched().value()) {
                return Watchdog.Verdict.UNWATCHED;
            }
            return Watchdog.Verdict.WATCHING;
        }

        @Override
        public Stop onStop() {
            return Stop.TELL;
        }

        @Override
        public boolean takesAChild() {
            return false;
        }
    }

    /** Its child was told to stop at told: Stop, or its grace, is what ends it now. */
    record Stopping(Moment told) implements RunState {
        @Override
        public Watchdog.Verdict watched(Moment now, Watchdog.Waits waits) {
            return now.secondsSince(told) > waits.killGrace().value()
                    ? Watchdog.Verdict.IGNORED_STOP
                    : Watchdog.Verdict.WATCHING;
        }

        @Override
        public Stop onStop() {
            return Stop.NOTHING;
        }

        @Override
        public boolean takesAChild() {
            return false;
        }
    }

    /** It has its outcome, or its child has ended. */
    record Over() implements RunState {
        @Override
        public Watchdog.Verdict watched(Moment now, Watchdog.Waits waits) {
            return Watchdog.Verdict.OVER;
        }

        @Override
        public Stop onStop() {
            return Stop.NOTHING;
        }

        @Override
        public boolean takesAChild() {
            return false;
        }
    }
}
