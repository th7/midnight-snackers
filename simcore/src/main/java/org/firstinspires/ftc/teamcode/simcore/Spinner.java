package org.firstinspires.ftc.teamcode.simcore;

public final class Spinner {
    public enum Way {
        CLOCKWISE,
        COUNTERCLOCKWISE,
        STILL
    }

    public record Turning(Way way, double ofFull) {
        public static Turning still() {
            return new Turning(Way.STILL, 0);
        }
    }

    static final double STOPPED_US = 1500;

    static final double FULL_SPEED_SPAN_US = 1000;

    private Spinner() {}

    public static Turning turning(ServoSignal signal, boolean reversed) {
        if (!(signal instanceof ServoSignal.Pulse pulse)) {
            return Turning.still();
        }
        double fromStopped = pulse.microseconds() - STOPPED_US;
        if (fromStopped == 0) {
            return Turning.still();
        }
        double ofFull = StrictMath.min(1, StrictMath.abs(fromStopped) / FULL_SPEED_SPAN_US);
        boolean clockwise = (fromStopped > 0) != reversed;
        return new Turning(clockwise ? Way.CLOCKWISE : Way.COUNTERCLOCKWISE, ofFull);
    }
}
