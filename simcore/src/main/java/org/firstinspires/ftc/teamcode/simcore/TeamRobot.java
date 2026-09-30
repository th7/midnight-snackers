package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

public enum TeamRobot {
    REGINALD("Reginald", "develop", "coding/", "", Drivebase.MECANUM, "reginald"),
    NUGGET("Nugget", "nugget-develop", "nugget/", "-nugget", Drivebase.TANK, "nugget");

    private static final String PACKAGES = "org.firstinspires.ftc.";
    private static final String MAIN_SOURCES = "TeamCode/src/main/java/";
    private static final String TEST_SOURCES = "TeamCode/src/test/java/";

    private final String displayName;
    private final String develop;
    private final String branchPrefix;
    private final String nameSuffix;
    private final Drivebase drivebase;
    private final String packageName;

    TeamRobot(
            String displayName,
            String develop,
            String branchPrefix,
            String nameSuffix,
            Drivebase drivebase,
            String packageName) {
        this.displayName = displayName;
        this.develop = develop;
        this.branchPrefix = branchPrefix;
        this.nameSuffix = nameSuffix;
        this.drivebase = drivebase;
        this.packageName = packageName;
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

    public String javaPackage() {
        return PACKAGES + packageName;
    }

    public List<String> ownDirectories() {
        String directory = javaPackage().replace('.', '/') + "/";
        return List.of(MAIN_SOURCES + directory, TEST_SOURCES + directory);
    }

    public boolean owns(String key) {
        for (String directory : ownDirectories()) {
            if (key.startsWith(directory) && key.length() > directory.length() && isNamedByNames(key)) {
                return true;
            }
        }
        return false;
    }

    private static boolean isNamedByNames(String key) {
        String closed = key + "/";
        return !key.endsWith("/") && !key.contains("//") && !closed.contains("/./") && !closed.contains("/../");
    }

    public static Optional<TeamRobot> ownerOf(String key) {
        for (TeamRobot robot : values()) {
            if (robot.owns(key)) {
                return Optional.of(robot);
            }
        }
        return Optional.empty();
    }

    public WhenMissing whenMissing() {
        return this == REGINALD ? new WhenMissing.Required() : new WhenMissing.StartsFrom(REGINALD);
    }
}
