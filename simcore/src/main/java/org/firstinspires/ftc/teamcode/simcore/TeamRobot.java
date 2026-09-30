package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

public enum TeamRobot {
    REGINALD("Reginald", "develop", "coding/", "", Drivebase.MECANUM),
    NUGGET("Nugget", "nugget-develop", "nugget/", "-nugget", Drivebase.TANK);

    private final String displayName;
    private final String develop;
    private final String branchPrefix;
    private final String nameSuffix;
    private final Drivebase drivebase;

    TeamRobot(String displayName, String develop, String branchPrefix, String nameSuffix, Drivebase drivebase) {
        this.displayName = displayName;
        this.develop = develop;
        this.branchPrefix = branchPrefix;
        this.nameSuffix = nameSuffix;
        this.drivebase = drivebase;
    }

    public sealed interface WhenMissing permits WhenMissing.Required, WhenMissing.StartsFrom {
        record Required() implements WhenMissing {}

        record StartsFrom(TeamRobot robot) implements WhenMissing {}
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

    public String userBranch(String slug) {
        return branchPrefix + slug;
    }

    public String ownName(String stem, String extension) {
        return stem + nameSuffix + extension;
    }

    public WhenMissing whenMissing() {
        return this == REGINALD ? new WhenMissing.Required() : new WhenMissing.StartsFrom(REGINALD);
    }
}
