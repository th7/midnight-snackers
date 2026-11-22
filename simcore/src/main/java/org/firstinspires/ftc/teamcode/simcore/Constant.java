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
    DRIVE_KS_VOLTS(
            Group.DRIVE,
            "Drive kS",
            "V",
            "The voltage a drive motor spends overcoming friction before its wheel moves at all.",
            0.9891921306123841,
            0,
            6),
    DRIVE_KV_VOLT_SECONDS_PER_IN(
            Group.DRIVE,
            "Drive kV",
            "V per in/s",
            "The voltage a drive motor spends on each inch a second its wheel rolls.",
            0.2358092252159611,
            0.01,
            2),
    DRIVE_KA_VOLT_SECONDS_SQUARED_PER_IN(
            Group.DRIVE,
            "Drive kA",
            "V per in/s^2",
            "The voltage a drive motor spends on each inch a second its wheel gains every second.",
            0.09202969466940081,
            0.001,
            1),
    TRACK_WIDTH_IN(
            Group.DRIVE,
            "Track width",
            "in",
            "How far apart the left and right wheels act when the robot turns.",
            13.751919701544644,
            4,
            36),
    LATERAL_MULTIPLIER(
            Group.DRIVE,
            "Lateral multiplier",
            "times",
            "How many times further the wheels roll to strafe an inch than to drive one ahead.",
            1.1829409108696243,
            1,
            3),
    MASS_KG(Group.DRIVE, "Mass", "kg", "What the whole robot weighs.", 15, 1, 30),
    DEAD_WHEEL_IN_PER_TICK(
            Group.DEAD_WHEELS,
            "Dead wheel tick",
            "in",
            "How far a dead wheel rolls for each tick its encoder counts.",
            0.0005433029,
            0.00001,
            0.01),
    PARALLEL_DEAD_WHEEL_Y_IN(
            Group.DEAD_WHEELS,
            "Parallel dead wheel, left of centre",
            "in",
            "How far left of the robot's centre the dead wheel that rolls ahead sits; right is negative.",
            -0.25,
            -9,
            9),
    PERPENDICULAR_DEAD_WHEEL_X_IN(
            Group.DEAD_WHEELS,
            "Perpendicular dead wheel, ahead of centre",
            "in",
            "How far ahead of the robot's centre the dead wheel that rolls sideways sits; behind is negative.",
            -3.5,
            -9,
            9);

    public enum Group {
        NOISE("Noise", "What a seed draws a run's robot from: how it differs from the tuned model."),
        DRIVE("Drive", "How the drivetrain moves the robot: its motors' feedforward, its wheels' spacing and its weight."),
        DEAD_WHEELS("Dead wheels", "Where the odometry wheels sit on the robot and how far each rolls a tick.");

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
