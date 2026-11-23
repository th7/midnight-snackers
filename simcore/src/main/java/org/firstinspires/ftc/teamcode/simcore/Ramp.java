package org.firstinspires.ftc.teamcode.simcore;

public record Ramp(double degrees) {
    static final double LEAST_US = 500;

    static final double MOST_US = 2500;

    static final double STOWED_US = 600;

    static final double DEGREES_PER_US = 270.0 / (MOST_US - LEAST_US);

    public static Ramp stowed() {
        return new Ramp(0);
    }

    public Ramp next(ServoSignal signal, double secondsPerSixtyDegrees, Seconds dt) {
        if (!(signal instanceof ServoSignal.Pulse pulse)) {
            return this;
        }
        double heard = StrictMath.max(LEAST_US, StrictMath.min(MOST_US, pulse.microseconds()));
        double target = (heard - STOWED_US) * DEGREES_PER_US;
        double most = 60 / secondsPerSixtyDegrees * dt.value();
        double toGo = target - degrees;
        return new Ramp(StrictMath.abs(toGo) <= most ? target : degrees + StrictMath.signum(toGo) * most);
    }

    public boolean up(double upDegrees) {
        return degrees >= upDegrees;
    }
}
