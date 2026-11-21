package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.simcore.Checked;

public final class Valid {
    private Valid() {}

    public static <T> T value(Checked<T> checked) {
        return checked.fold(value -> value, rule -> {
            throw new IllegalArgumentException(rule);
        });
    }
}
