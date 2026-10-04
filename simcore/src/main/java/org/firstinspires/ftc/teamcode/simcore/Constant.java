package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public enum Constant {
    MOTOR_SPREAD(
            Group.NOISE,
            "Motor spread",
            "of the tuned value",
            "How far each drive motor's kS, kV and kA may be drawn from the tuned values, each its own way:"
                    + " 0.1 is within a tenth.",
            Noise.MOTOR_SPREAD,
            0,
            0.5),
    FLATTEST_VOLTS(
            Group.NOISE,
            "Flattest battery",
            "V",
            "The lowest voltage a run's battery may start at.",
            Noise.FLATTEST_VOLTS,
            1,
            20),
    FRESHEST_VOLTS(
            Group.NOISE,
            "Freshest battery",
            "V",
            "The highest voltage a run's battery may start at.",
            Noise.FRESHEST_VOLTS,
            1,
            20),
    SAG_VOLTS_PER_POWER(
            Group.NOISE,
            "Sag",
            "V per unit of drive power",
            "How far the battery's voltage drops under load, for each unit of power commanded of each drive motor.",
            Noise.SAG_VOLTS_PER_POWER,
            0,
            2),
    DRAIN_VOLTS_PER_SECOND(
            Group.NOISE,
            "Drain",
            "V/s",
            "How fast the battery runs down as the run goes on.",
            Noise.DRAIN_VOLTS_PER_SECOND,
            0,
            0.1),
    LEAST_TRACTION_G(
            Group.NOISE,
            "Least traction",
            "g",
            "The least grip a run's floor may give a wheel before it slips, driving or braking.",
            Noise.LEAST_TRACTION_G,
            0.05,
            2),
    MOST_TRACTION_G(
            Group.NOISE,
            "Most traction",
            "g",
            "The most grip a run's floor may give a wheel before it slips, driving or braking.",
            Noise.MOST_TRACTION_G,
            0.05,
            2),
    SET_DOWN_INCHES(
            Group.NOISE,
            "Set down, off by",
            "in",
            "How far from its start pose a hand typically sets the robot down.",
            Noise.SET_DOWN_INCHES,
            0,
            6),
    SET_DOWN_DEGREES(
            Group.NOISE,
            "Set down, turned by",
            "deg",
            "How far from its start heading a hand typically sets the robot down.",
            Noise.SET_DOWN_DEGREES,
            0,
            45),
    LOOP_SECONDS(
            Group.NOISE,
            "Loop period",
            "s",
            "The typical time from one loop of the op mode to the next.",
            Noise.LOOP_SECONDS,
            0.001,
            0.5),
    LOOP_SPREAD(
            Group.NOISE,
            "Loop spread",
            "of the period",
            "How much one loop's period varies about the typical one.",
            Noise.LOOP_SPREAD,
            0,
            1),
    LEAST_LOOP_SECONDS(
            Group.NOISE, "Shortest loop", "s", "No loop is quicker than this.", Noise.LEAST_LOOP_SECONDS, 0.001, 0.5),
    HICCUP_CHANCE(
            Group.NOISE,
            "Hiccup chance",
            "per loop",
            "The chance that a loop is a hiccup, a pause far longer than a loop.",
            Noise.HICCUP_CHANCE,
            0,
            1),
    SHORTEST_HICCUP_SECONDS(
            Group.NOISE,
            "Shortest hiccup",
            "s",
            "The shortest a hiccup lasts, and the longest an ordinary loop may.",
            Noise.SHORTEST_HICCUP_SECONDS,
            0.001,
            2),
    LONGEST_HICCUP_SECONDS(
            Group.NOISE, "Longest hiccup", "s", "The longest a hiccup lasts.", Noise.LONGEST_HICCUP_SECONDS, 0.001, 2),
    LAUNCH_THROW(
            Group.MECHANISMS,
            "Launcher throw",
            "in/s per tick/s",
            "How fast a ball leaves Reginald's launcher for each tick a second its flywheel turns.",
            Launch.IN_PER_S_PER_TICK_PER_S,
            0.01,
            1),
    TURNTABLE_SPEED(
            Group.MECHANISMS,
            "Turntable speed",
            "ticks/s at full power",
            "How fast Reginald's turntable turns when its motor is given full power.",
            Turntable.TICKS_PER_SECOND_AT_FULL_POWER,
            1,
            10000);

    public enum Group {
        NOISE("Noise", "What a seed draws a run's robot from: how it differs from the tuned model."),
        MECHANISMS("Mechanisms", "Guesses until measured on the robot, which every run uses, seeded or exact.");

        private final String label;
        private final String says;

        Group(String label, String says) {
            this.label = label;
            this.says = says;
        }

        public String asked() {
            return name().toLowerCase(Locale.ROOT);
        }

        public String label() {
            return label;
        }

        public String says() {
            return says;
        }
    }

    private final Group group;
    private final String label;
    private final String unit;
    private final String says;
    private final double byDefault;
    private final double least;
    private final double most;

    Constant(Group group, String label, String unit, String says, double byDefault, double least, double most) {
        this.group = group;
        this.label = label;
        this.unit = unit;
        this.says = says;
        this.byDefault = byDefault;
        this.least = least;
        this.most = most;
    }

    public static Checked<Constant> named(String asked) {
        for (Constant constant : values()) {
            if (constant.asked().equals(asked)) {
                return Checked.ok(constant);
            }
        }
        return Checked.rejected("a simulation constant is " + everyName() + ", not '" + asked + "'");
    }

    private static String everyName() {
        List<String> names = new ArrayList<>();
        for (Constant constant : values()) {
            names.add(constant.asked());
        }
        return String.join(", ", names);
    }

    Checked<Double> admits(double value) {
        if (!(value >= least && value <= most)) {
            return Checked.rejected(label + " is from " + least + " to " + most + " " + unit + ", not " + value);
        }
        return Checked.ok(value);
    }

    public String asked() {
        return name().toLowerCase(Locale.ROOT);
    }

    public Group group() {
        return group;
    }

    public String label() {
        return label;
    }

    public String unit() {
        return unit;
    }

    public String says() {
        return says;
    }

    public double byDefault() {
        return byDefault;
    }

    public double least() {
        return least;
    }

    public double most() {
        return most;
    }
}
