package org.firstinspires.ftc.teamcode.simcore;

public enum Wheel {
    LEFT_FRONT("left front"),
    RIGHT_FRONT("right front"),
    LEFT_BACK("left back"),
    RIGHT_BACK("right back");

    private final String label;

    Wheel(String label) {
        this.label = label;
    }

    @Override
    public String toString() {
        return label;
    }
}
