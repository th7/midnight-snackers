package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

public enum TeamRobot {
    REGINALD("Reginald", "develop", Drivebase.MECANUM, "reginald"),
    NUGGET("Nugget", "nugget-develop", Drivebase.TANK, "nugget");

    private static final String PACKAGES = "org.firstinspires.ftc.";

    private final String displayName;
    private final String develop;
    private final Drivebase drivebase;
    private final String packageName;

    TeamRobot(String displayName, String develop, Drivebase drivebase, String packageName) {
        this.displayName = displayName;
        this.develop = develop;
        this.drivebase = drivebase;
        this.packageName = packageName;
    }

    public static Checked<TeamRobot> named(String asked) {
        for (TeamRobot robot : values()) {
            if (robot.asked().equals(asked)) {
                return Checked.ok(robot);
            }
        }
        return Checked.rejected("a robot is " + everyName() + ", not '" + asked + "'");
    }

    public static Checked<TeamRobot> stored(Optional<String> asked) {
        return asked.map(TeamRobot::named).orElse(Checked.ok(REGINALD));
    }

    private static String everyName() {
        List<String> names = new ArrayList<>();
        for (TeamRobot robot : values()) {
            names.add(robot.asked());
        }
        return String.join(" or ", names);
    }

    public String asked() {
        return name().toLowerCase(Locale.ROOT);
    }

    public String displayName() {
        return displayName;
    }

    public Drivebase drivebase() {
        return drivebase;
    }

    public String develop() {
        return develop;
    }

    public String javaPackage() {
        return PACKAGES + packageName;
    }
}
