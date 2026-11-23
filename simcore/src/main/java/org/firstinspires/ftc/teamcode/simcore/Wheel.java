package org.firstinspires.ftc.teamcode.simcore;

public enum Wheel {
    LEFT_FRONT("left front", Mechanism.LEFT_FRONT),
    RIGHT_FRONT("right front", Mechanism.RIGHT_FRONT),
    LEFT_BACK("left back", Mechanism.LEFT_BACK),
    RIGHT_BACK("right back", Mechanism.RIGHT_BACK);

    private static final String DRIVES_FORWARD = "drives_forward";

    private final String label;
    private final Mechanism mechanism;

    Wheel(String label, Mechanism mechanism) {
        this.label = label;
        this.mechanism = mechanism;
    }

    public Mechanism mechanism() {
        return mechanism;
    }

    public Sense drives(Directions directions) {
        return mechanism.way(directions.reversed(mechanism)).equals(DRIVES_FORWARD) ? Sense.FORWARD : Sense.REVERSE;
    }

    @Override
    public String toString() {
        return label;
    }
}
