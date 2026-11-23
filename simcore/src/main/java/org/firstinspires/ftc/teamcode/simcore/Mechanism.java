package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public enum Mechanism {
    INTAKE("Intake", "the intake", "takes_in", "pushes_out", "power takes pollen in", "power pushes pollen out"),
    SPINNER(
            "Spinner",
            "the spinner",
            "turns_clockwise",
            "turns_counterclockwise",
            "power turns it clockwise from above",
            "power turns it counterclockwise from above"),
    FLYWHEEL_FRONT(
            "Front flywheel",
            "the front flywheel",
            "turns_counterclockwise",
            "turns_clockwise",
            "power turns its motor counterclockwise, seen from its shaft",
            "power turns its motor clockwise, seen from its shaft"),
    FLYWHEEL_BACK(
            "Back flywheel",
            "the back flywheel",
            "turns_counterclockwise",
            "turns_clockwise",
            "power turns its motor counterclockwise, seen from its shaft",
            "power turns its motor clockwise, seen from its shaft");

    private final String label;
    private final String called;
    private final String asBuilt;
    private final String reversed;
    private final String asBuiltDoes;
    private final String reversedDoes;

    Mechanism(String label, String called, String asBuilt, String reversed, String asBuiltDoes, String reversedDoes) {
        this.label = label;
        this.called = called;
        this.asBuilt = asBuilt;
        this.reversed = reversed;
        this.asBuiltDoes = asBuiltDoes;
        this.reversedDoes = reversedDoes;
    }

    public static Checked<Mechanism> named(String asked) {
        List<String> names = new ArrayList<>();
        for (Mechanism mechanism : values()) {
            if (mechanism.asked().equals(asked)) {
                return Checked.ok(mechanism);
            }
            names.add(mechanism.asked());
        }
        return Checked.rejected("a mechanism is " + String.join(", ", names) + ", not '" + asked + "'");
    }

    public String asked() {
        return name().toLowerCase(Locale.ROOT);
    }

    public String label() {
        return label;
    }

    public String way(boolean reversed) {
        return reversed ? this.reversed : asBuilt;
    }

    public Checked<Boolean> reversedIf(String way) {
        if (way.equals(asBuilt)) {
            return Checked.ok(false);
        }
        if (way.equals(reversed)) {
            return Checked.ok(true);
        }
        return Checked.rejected(called + " " + asBuilt + " or " + reversed + " on positive power, not '" + way + "'");
    }

    public String says(boolean reversed) {
        return "Positive " + (reversed ? reversedDoes : asBuiltDoes) + ".";
    }

    String said(boolean reversed) {
        return asked() + " " + way(reversed).replace('_', ' ') + " on positive power";
    }
}
