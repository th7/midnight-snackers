package org.firstinspires.ftc.teamcode.simcore;

public final class Feedforward {
    private final double kSVolts;
    private final double kVVoltSecondsPerTick;
    private final double kAVoltSecondsSquaredPerTick;

    private Feedforward(double kSVolts, double kVVoltSecondsPerTick, double kAVoltSecondsSquaredPerTick) {
        this.kSVolts = kSVolts;
        this.kVVoltSecondsPerTick = kVVoltSecondsPerTick;
        this.kAVoltSecondsSquaredPerTick = kAVoltSecondsSquaredPerTick;
    }

    public static Checked<Feedforward> of(
            double kSVolts, double kVVoltSecondsPerTick, double kAVoltSecondsSquaredPerTick) {
        if (!(kSVolts >= 0
                && Double.isFinite(kSVolts)
                && kVVoltSecondsPerTick >= 0
                && Double.isFinite(kVVoltSecondsPerTick))) {
            return Checked.rejected("a motor's kS and kV are finite and not below zero, not " + kSVolts + " and "
                    + kVVoltSecondsPerTick);
        }
        if (!(kAVoltSecondsSquaredPerTick > 0 && Double.isFinite(kAVoltSecondsSquaredPerTick))) {
            return Checked.rejected("a motor's kA is positive and finite, not " + kAVoltSecondsSquaredPerTick);
        }
        return Checked.ok(new Feedforward(kSVolts, kVVoltSecondsPerTick, kAVoltSecondsSquaredPerTick));
    }

    public Checked<Feedforward> scaledBy(Noise.Motor noise) {
        return of(
                kSVolts * noise.kSVolts(),
                kVVoltSecondsPerTick * noise.kVVoltSecondsPerTick(),
                kAVoltSecondsSquaredPerTick * noise.kAVoltSecondsSquaredPerTick());
    }

    public double kSVolts() {
        return kSVolts;
    }

    public double kVVoltSecondsPerTick() {
        return kVVoltSecondsPerTick;
    }

    public double kAVoltSecondsSquaredPerTick() {
        return kAVoltSecondsSquaredPerTick;
    }

    @Override
    public String toString() {
        return "kS " + kSVolts + " V, kV " + kVVoltSecondsPerTick + " V s/tick, kA " + kAVoltSecondsSquaredPerTick
                + " V s^2/tick";
    }
}
