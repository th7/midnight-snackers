package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.simcore.Checked;

final class Valid {
    private Valid() {}

    static <T> T value(Checked<T> checked) {
        return checked.fold(value -> value, rule -> {
            throw new IllegalArgumentException(rule);
        });
    }
}
