package org.firstinspires.ftc.teamcode.simcore;

public enum Wheel {
    LEFT_FRONT("left front", Sense.FORWARD),
    RIGHT_FRONT("right front", Sense.FORWARD),
    LEFT_BACK("left back", Sense.REVERSE),
    RIGHT_BACK("right back", Sense.REVERSE);

    private final String label;
    private final Sense mounted;

    Wheel(String label, Sense mounted) {
        this.label = label;
        this.mounted = mounted;
    }

    public Sense mounted() {
        return mounted;
    }

    @Override
    public String toString() {
        return label;
    }
}
