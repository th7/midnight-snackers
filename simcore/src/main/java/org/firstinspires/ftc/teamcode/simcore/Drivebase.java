package org.firstinspires.ftc.teamcode.simcore;

import java.util.List;

public enum Drivebase {
    MECANUM,
    TANK;

    public PerWheel<Sense> mounted() {
        return switch (this) {
            case MECANUM -> new PerWheel<>(Sense.FORWARD, Sense.FORWARD, Sense.REVERSE, Sense.REVERSE);
            case TANK -> new Sides<>(Sense.REVERSE, Sense.FORWARD).wheels();
        };
    }

    public <T> List<T> perMotor(PerWheel<T> drawnPerWheel) {
        return switch (this) {
            case MECANUM -> drawnPerWheel.inOrder();
            case TANK -> List.of(drawnPerWheel.leftFront(), drawnPerWheel.rightFront());
        };
    }

    public <T> PerWheel<T> turning(PerWheel<T> drawnPerWheel) {
        return switch (this) {
            case MECANUM -> drawnPerWheel;
            case TANK -> new Sides<>(drawnPerWheel.leftFront(), drawnPerWheel.rightFront()).wheels();
        };
    }

    public double sideways(double pushed, double sliding, Seconds dt, Traction traction) {
        return switch (this) {
            case MECANUM -> pushed;
            case TANK -> traction.gives(dt.value() > 0 ? -sliding / dt.value() : 0);
        };
    }

    public List<String> motors() {
        return switch (this) {
            case MECANUM -> List.of("LF", "RF", "LB", "RB");
            case TANK -> List.of("L", "R");
        };
    }
}
