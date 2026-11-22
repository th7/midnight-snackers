package org.firstinspires.ftc.teamcode.simcore;

public enum Facing {
    UP,
    DOWN,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT;

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
