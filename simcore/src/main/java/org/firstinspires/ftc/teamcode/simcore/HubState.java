package org.firstinspires.ftc.teamcode.simcore;

import java.util.OptionalLong;

public sealed interface HubState permits HubState.Untried, HubState.Reached, HubState.Unreached {
    long PROBE_EVERY_MILLIS = 10_000;

    static HubState untried() {
        return new Untried();
    }

    static HubState reached(long atMillis) {
        return new Reached(atMillis);
    }

    static HubState unreached(long atMillis, String why) {
        return new Unreached(atMillis, why);
    }

    String said();

    String why();

    OptionalLong checkedAtMillis();

    boolean probeDue(long nowMillis);

    private static boolean dueSince(long atMillis, long nowMillis) {
        return nowMillis < atMillis || nowMillis - atMillis >= PROBE_EVERY_MILLIS;
    }

    record Untried() implements HubState {
        @Override
        public String said() {
            return "untried";
        }

        @Override
        public String why() {
            return "";
        }

        @Override
        public OptionalLong checkedAtMillis() {
            return OptionalLong.empty();
        }

        @Override
        public boolean probeDue(long nowMillis) {
            return true;
        }
    }

    record Reached(long atMillis) implements HubState {
        @Override
        public String said() {
            return "reachable";
        }

        @Override
        public String why() {
            return "";
        }

        @Override
        public OptionalLong checkedAtMillis() {
            return OptionalLong.of(atMillis);
        }

        @Override
        public boolean probeDue(long nowMillis) {
            return dueSince(atMillis, nowMillis);
        }
    }

    record Unreached(long atMillis, String why) implements HubState {
        @Override
        public String said() {
            return "unreachable";
        }

        @Override
        public OptionalLong checkedAtMillis() {
            return OptionalLong.of(atMillis);
        }

        @Override
        public boolean probeDue(long nowMillis) {
            return dueSince(atMillis, nowMillis);
        }
    }
}
