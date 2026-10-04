package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

public final class Robots {
    private final TeamRobot first;
    private final Set<TeamRobot> robots;

    private Robots(TeamRobot one, Set<TeamRobot> others) {
        EnumSet<TeamRobot> all = EnumSet.of(one);
        all.addAll(others);
        this.robots = Collections.unmodifiableSet(all);
        this.first = earliestOf(all, one);
    }

    private static TeamRobot earliestOf(Set<TeamRobot> robots, TeamRobot otherwise) {
        for (TeamRobot robot : robots) {
            return robot;
        }
        return otherwise;
    }

    public static Robots only(TeamRobot robot) {
        return new Robots(robot, EnumSet.noneOf(TeamRobot.class));
    }

    public static Robots letOnto(Optional<Robots> before, TeamRobot robot) {
        return before.map(robots -> robots.with(robot)).orElse(only(robot));
    }

    public static Checked<Robots> named(List<String> asked) {
        Checked<Optional<Robots>> read = Checked.ok(Optional.empty());
        for (String name : asked) {
            read = read.then(sofar -> TeamRobot.named(name).map(robot -> Optional.of(letOnto(sofar, robot))));
        }
        return read.then(robots -> robots.map(Checked::ok)
                .orElse(Checked.rejected("a user works on at least one robot, and none was named")));
    }

    public Robots with(TeamRobot robot) {
        return new Robots(robot, robots);
    }

    public Checked<Robots> without(TeamRobot robot) {
        EnumSet<TeamRobot> left = EnumSet.noneOf(TeamRobot.class);
        left.addAll(robots);
        left.remove(robot);
        return left.isEmpty()
                ? Checked.rejected("a user works on at least one robot, and " + robot.displayName()
                        + " is the only one left to them")
                : Checked.ok(new Robots(earliestOf(left, first), left));
    }

    public boolean has(TeamRobot robot) {
        return robots.contains(robot);
    }

    public TeamRobot workingOn(TeamRobot asked) {
        return has(asked) ? asked : first;
    }

    public Checked<TeamRobot> switchTo(TeamRobot robot) {
        return has(robot)
                ? Checked.ok(robot)
                : Checked.rejected("the admin has not let you work on " + robot.displayName());
    }

    public List<TeamRobot> all() {
        return Collections.unmodifiableList(new ArrayList<>(robots));
    }

    public List<String> asked() {
        List<String> names = new ArrayList<>();
        for (TeamRobot robot : robots) {
            names.add(robot.asked());
        }
        return Collections.unmodifiableList(names);
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Robots that && robots.equals(that.robots);
    }

    @Override
    public int hashCode() {
        return robots.hashCode();
    }

    @Override
    public String toString() {
        return asked().toString();
    }
}
