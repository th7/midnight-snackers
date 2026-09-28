package org.firstinspires.ftc.teamcode.simcore;

public final class Power {
    private final double value;

    private Power(double value) {
        this.value = value;
    }

    public static Power none() {
        return new Power(0);
    }

    public static Power clamped(double power) {
        if (Double.isNaN(power)) {
            return none();
        }
        return new Power(Math.max(-1, Math.min(1, power)));
    }

    public double value() {
        return value;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Power that && Double.compare(value, that.value) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(value);
    }

    @Override
    public String toString() {
        return "power " + value;
    }
}
