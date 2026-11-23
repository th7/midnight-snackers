package org.firstinspires.ftc.teamcode.simcore;

public final class Handover {
    public enum Why {
        HANDING,
        NOTHING_HELD,
        SPINNER_NOT_CLOCKWISE,
        RAMP_DOWN,
        LAUNCHER_FULL
    }

    public record Stepped(double turns, int handed, Why why) {}

    static final double ARMS_PER_TURN = 2;

    private Handover() {}

    public static Stepped step(
            double turns,
            Spinner.Turning turning,
            double fullSpeedRpm,
            boolean rampUp,
            int held,
            int room,
            Seconds dt) {
        double next = turns + turnsPerSecond(turning, fullSpeedRpm) * dt.value();
        Why why = why(turning, rampUp, held, room);
        if (why != Why.HANDING) {
            return new Stepped(next, 0, why);
        }
        double passed = StrictMath.floor(next * ARMS_PER_TURN) - StrictMath.floor(turns * ARMS_PER_TURN);
        return new Stepped(next, (int) StrictMath.max(0, StrictMath.min(StrictMath.min(held, room), passed)), why);
    }

    private static double turnsPerSecond(Spinner.Turning turning, double fullSpeedRpm) {
        double most = fullSpeedRpm / 60 * turning.ofFull();
        return switch (turning.way()) {
            case CLOCKWISE -> most;
            case COUNTERCLOCKWISE -> -most;
            case STILL -> 0;
        };
    }

    private static Why why(Spinner.Turning turning, boolean rampUp, int held, int room) {
        if (turning.way() != Spinner.Way.CLOCKWISE) {
            return Why.SPINNER_NOT_CLOCKWISE;
        }
        if (!rampUp) {
            return Why.RAMP_DOWN;
        }
        if (held <= 0) {
            return Why.NOTHING_HELD;
        }
        if (room <= 0) {
            return Why.LAUNCHER_FULL;
        }
        return Why.HANDING;
    }
}
