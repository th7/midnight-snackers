package org.firstinspires.ftc.teamcode.simcore;

public enum Sense {
    FORWARD,
    REVERSE;

    public double of(double value) {
        return this == FORWARD ? value : -value;
    }
}
