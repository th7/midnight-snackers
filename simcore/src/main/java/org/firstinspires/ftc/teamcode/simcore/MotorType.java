package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;

public enum MotorType {
    ULTRAPLANETARY_HD_HEX("REV Robotics UltraPlanetary HD Hex Motor", Turning.CLOCKWISE, 560, 300);

    private static final double ACHIEVABLE_SHARE = 0.85;
    private static final double SECONDS_A_MINUTE = 60;

    private final String label;
    private final Turning declared;
    private final double ticksPerRev;
    private final double maxRpm;

    MotorType(String label, Turning declared, double ticksPerRev, double maxRpm) {
        this.label = label;
        this.declared = declared;
        this.ticksPerRev = ticksPerRev;
        this.maxRpm = maxRpm;
    }

    public static Checked<MotorType> named(String asked) {
        List<String> labels = new ArrayList<>();
        for (MotorType type : values()) {
            if (type.label.equals(asked)) {
                return Checked.ok(type);
            }
            labels.add(type.label);
        }
        return Checked.rejected("a motor type is " + String.join(", ", labels) + ", not '" + asked + "'");
    }

    public String label() {
        return label;
    }

    public Turning declared() {
        return declared;
    }

    public double freeTicksPerSecond() {
        return ticksPerRev * maxRpm / SECONDS_A_MINUTE;
    }

    public double achievableTicksPerSecond() {
        return StrictMath.floor(freeTicksPerSecond() * ACHIEVABLE_SHARE + 0.5);
    }
}
