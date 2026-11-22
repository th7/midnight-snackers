package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;

public enum Facing {
    UP,
    DOWN,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT;

    public static Checked<Facing> named(String asked) {
        for (Facing facing : values()) {
            if (facing.name().equals(asked)) {
                return Checked.ok(facing);
            }
        }
        List<String> names = new ArrayList<>();
        for (Facing facing : values()) {
            names.add(facing.name());
        }
        return Checked.rejected("a face is " + String.join(", ", names) + ", not '" + asked + "'");
    }

    Facing opposite() {
        return switch (this) {
            case UP -> DOWN;
            case DOWN -> UP;
            case FORWARD -> BACKWARD;
            case BACKWARD -> FORWARD;
            case LEFT -> RIGHT;
            case RIGHT -> LEFT;
        };
    }
}
