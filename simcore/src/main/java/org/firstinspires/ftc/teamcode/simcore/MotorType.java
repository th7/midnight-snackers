package org.firstinspires.ftc.teamcode.simcore;

public enum MotorType {
    ULTRAPLANETARY_HD_HEX("REV Robotics UltraPlanetary HD Hex Motor", Turning.CLOCKWISE);

    private final String label;
    private final Turning declared;

    MotorType(String label, Turning declared) {
        this.label = label;
        this.declared = declared;
    }

    public String label() {
        return label;
    }

    public Turning declared() {
        return declared;
    }
}
