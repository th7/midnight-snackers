package org.firstinspires.ftc.teamcode.simcore;

public sealed interface View permits View.NotAwaited, View.Awaited, View.Ready {
    boolean holdsTheStart(Moment now, Seconds wait);

    default boolean waitedFor() {
        return false;
    }

    record NotAwaited() implements View {
        @Override
        public boolean holdsTheStart(Moment now, Seconds wait) {
            return false;
        }
    }

    record Awaited(Moment since) implements View {
        @Override
        public boolean holdsTheStart(Moment now, Seconds wait) {
            return now.secondsSince(since) <= wait.value();
        }

        @Override
        public boolean waitedFor() {
            return true;
        }
    }

    record Ready() implements View {
        @Override
        public boolean holdsTheStart(Moment now, Seconds wait) {
            return false;
        }
    }
}
