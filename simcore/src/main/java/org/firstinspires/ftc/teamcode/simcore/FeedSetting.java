package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public enum FeedSetting {
    POLLEN(
            Group.POLLEN,
            "Pollen",
            "",
            "How many pollen start in the feeder, one each side of the arms against the wall.",
            2,
            0,
            2,
            true),
    FIRST_POLLEN(
            Group.POLLEN,
            "First pollen",
            "deg",
            "Where the first pollen starts against the wall, in degrees clockwise seen from above, from the back.",
            Feed.FIRST_POLLEN_DEGREES,
            0,
            360,
            false),
    SECOND_POLLEN(
            Group.POLLEN,
            "Second pollen",
            "deg",
            "Where the second pollen starts against the wall, in degrees clockwise seen from above, from the back.",
            Feed.SECOND_POLLEN_DEGREES,
            0,
            360,
            false),
    SPINNER_POWER(
            Group.SPINNER,
            "Spinner power",
            "",
            "The power the spinner's continuous servo is set to: 1 turns it clockwise seen from above at full speed,"
                    + " -1 the other way and 0 stops it.",
            1,
            -1,
            1,
            false),
    SPINNER_SPEED(
            Group.SPINNER,
            "Spinner speed",
            "rev/s",
            "How fast the spinner turns at full power with nothing holding it back.",
            Feeder.Spinner.FULL_SPEED_REV_PER_S,
            Feeder.Spinner.LEAST_FULL_SPEED_REV_PER_S,
            Feeder.Spinner.MOST_FULL_SPEED_REV_PER_S,
            false),
    SPINNER_STALL(
            Group.SPINNER,
            "Spinner stall torque",
            "N m",
            "The load that stops the spinner; it slows in proportion as its load nears this.",
            Feeder.Spinner.STALL_NEWTON_METRES,
            Feeder.Spinner.LEAST_STALL_NEWTON_METRES,
            Feeder.Spinner.MOST_STALL_NEWTON_METRES,
            false),
    RAMP_DOWN(
            Group.RAMP,
            "Ramp down",
            "deg",
            "The ramp bar's angle when it is down: 0 is as drawn, below the floor.",
            Feed.DOWN_DEGREES,
            0,
            Feeder.RampAngle.MOST_DEGREES,
            false),
    RAMP_UP(
            Group.RAMP,
            "Ramp up",
            "deg",
            "The ramp bar's angle when it is up, lifting a pollen at the back toward the wheel.",
            Feed.UP_DEGREES,
            0,
            Feeder.RampAngle.MOST_DEGREES,
            false),
    RAMP_RAISED_AT(
            Group.RAMP,
            "Raised at",
            "s",
            "When the ramp's servo is commanded up.",
            Feed.RAISED_AT_SECONDS,
            0,
            Feed.LONGEST_RUN_SECONDS,
            false),
    RAMP_LOWERED_AT(
            Group.RAMP,
            "Lowered at",
            "s",
            "When the ramp's servo is commanded back down.",
            Feed.LOWERED_AT_SECONDS,
            0,
            Feed.LONGEST_RUN_SECONDS,
            false),
    RAMP_SERVO_SPEED(
            Group.RAMP,
            "Ramp servo speed",
            "s per 60 deg",
            "How long the ramp's servo takes to turn 60 degrees.",
            Feeder.Ramp.SECONDS_PER_SIXTY_DEGREES,
            Feeder.Ramp.LEAST_SECONDS_PER_SIXTY_DEGREES,
            Feeder.Ramp.MOST_SECONDS_PER_SIXTY_DEGREES,
            false),
    BACK_SILL(
            Group.FEEDER,
            "Back sill",
            "mm",
            "How high the back window's sill is above the floor slab's underside. The floor's top is at 15, and the"
                    + " CAD draws the sill at 43.12, where it holds a lifted pollen against the wheel.",
            Feed.BACK_SILL_MM,
            Feed.LOWEST_BACK_SILL_MM,
            Feed.HIGHEST_BACK_SILL_MM,
            false),
    FLYWHEEL(
            Group.FEEDER,
            "Flywheel speed",
            "ticks/s",
            "How fast the flywheel's motor is commanded to turn, as the robot's code commands it.",
            Feed.FLYWHEEL_TICKS_PER_SECOND,
            0,
            Flywheel.TOP_TICKS_PER_SECOND,
            false),
    RUN(
            Group.RUN,
            "Run length",
            "s",
            "The longest a feed is followed; it ends sooner once every pollen has left or come to rest.",
            Feed.RUN_SECONDS,
            Feed.SHORTEST_RUN_SECONDS,
            Feed.LONGEST_RUN_SECONDS,
            false);

    public enum Group {
        POLLEN("Pollen", "The pollen in the feeder as the feed starts."),
        SPINNER("Spinner", "The spinner's continuous servo: its power, and how fast and how hard it turns."),
        RAMP("Ramp", "The ramp bar's positional servo: where it is down and up, and when it moves."),
        FEEDER("Feeder and flywheel", "The back window's sill, and the flywheel that takes a lifted pollen."),
        RUN("Run", "How long the feed is followed.");

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
    private final boolean whole;

    FeedSetting(
            Group group,
            String label,
            String unit,
            String says,
            double byDefault,
            double least,
            double most,
            boolean whole) {
        this.group = group;
        this.label = label;
        this.unit = unit;
        this.says = says;
        this.byDefault = byDefault;
        this.least = least;
        this.most = most;
        this.whole = whole;
    }

    public static Checked<FeedSetting> named(String asked) {
        List<String> names = new ArrayList<>();
        for (FeedSetting setting : values()) {
            if (setting.asked().equals(asked)) {
                return Checked.ok(setting);
            }
            names.add(setting.asked());
        }
        return Checked.rejected("a feed setting is " + String.join(", ", names) + ", not '" + asked + "'");
    }

    public Checked<Double> admits(double value) {
        if (whole && !(value >= least && value <= most && value == StrictMath.rint(value))) {
            return Checked.rejected(label + " is " + wholeNumbers() + ", not " + value);
        }
        if (!(value >= least && value <= most)) {
            return Checked.rejected(label + " is from " + least + " to " + most + inUnit() + ", not " + value);
        }
        return Checked.ok(value + 0.0);
    }

    private String wholeNumbers() {
        long first = StrictMath.round(least);
        long last = StrictMath.round(most);
        StringBuilder each = new StringBuilder();
        for (long n = first; n <= last; n++) {
            if (n > first) {
                each.append(n == last ? " or " : ", ");
            }
            each.append(n);
        }
        return each.toString();
    }

    private String inUnit() {
        return unit.isEmpty() ? "" : " " + unit;
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

    public boolean whole() {
        return whole;
    }
}
