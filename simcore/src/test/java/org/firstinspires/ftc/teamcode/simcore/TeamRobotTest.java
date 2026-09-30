package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class TeamRobotTest {
    private static String rejection(Checked<TeamRobot> checked) {
        return checked.fold(robot -> "accepted " + robot, rule -> rule);
    }

    @Test
    public void theTeamHasReginaldAndNugget() {
        assertEquals(List.of(TeamRobot.REGINALD, TeamRobot.NUGGET), List.of(TeamRobot.values()));
        assertEquals("Reginald", TeamRobot.REGINALD.displayName());
        assertEquals("Nugget", TeamRobot.NUGGET.displayName());
    }

    @Test
    public void aRobotIsAskedForByItsNameInLowerCase() {
        assertEquals("reginald", TeamRobot.REGINALD.asked());
        assertEquals("nugget", TeamRobot.NUGGET.asked());
        for (TeamRobot robot : TeamRobot.values()) {
            assertEquals(robot, Valid.value(TeamRobot.named(robot.asked())));
        }
    }

    @Test
    public void aNameThatIsNoRobotIsRejectedNamingTheRobotsThereAre() {
        for (String wrong : List.of("", "Nugget", "robot", " nugget", "reginald\n")) {
            String said = rejection(TeamRobot.named(wrong));
            assertTrue(said, said.startsWith("a robot is"));
            assertTrue(said, said.contains("reginald") && said.contains("nugget"));
        }
    }

    @Test
    public void eachRobotWorksItsOwnDevelopBranch() {
        assertEquals("develop", TeamRobot.REGINALD.develop());
        assertEquals("nugget-develop", TeamRobot.NUGGET.develop());
    }

    @Test
    public void eachRobotsUserBranchesAreItsOwnAndReginaldsAreWhereTheyAlwaysWere() {
        assertEquals("coding/ada", TeamRobot.REGINALD.userBranch("ada"));
        assertEquals("nugget/ada", TeamRobot.NUGGET.userBranch("ada"));
    }

    @Test
    public void reginaldKeepsTheNamesFromBeforeThereWereTwoRobots() {
        assertEquals("worktrees.json", TeamRobot.REGINALD.ownName("worktrees", ".json"));
        assertEquals("worktrees-nugget.json", TeamRobot.NUGGET.ownName("worktrees", ".json"));
        assertEquals("root-0a1b2c3d", TeamRobot.REGINALD.ownName("root-0a1b2c3d", ""));
        assertEquals("root-0a1b2c3d-nugget", TeamRobot.NUGGET.ownName("root-0a1b2c3d", ""));
    }

    @Test
    public void aLoginStoredBeforeThereWereTwoRobotsIsReginalds() {
        assertEquals(TeamRobot.REGINALD, Valid.value(TeamRobot.stored(Optional.empty())));
        assertEquals(TeamRobot.NUGGET, Valid.value(TeamRobot.stored(Optional.of("nugget"))));
        assertTrue(rejection(TeamRobot.stored(Optional.of("bender"))).contains("bender"));
    }

    @Test
    public void reginaldsLineMustBeThereAndNuggetsStartsFromItWhenItIsNot() {
        assertEquals(new TeamRobot.WhenMissing.Required(), TeamRobot.REGINALD.whenMissing());
        assertEquals(new TeamRobot.WhenMissing.StartsFrom(TeamRobot.REGINALD), TeamRobot.NUGGET.whenMissing());
    }

    @Test
    public void reginaldDrivesOnMecanumWheelsAndNuggetOnATank() {
        assertEquals(Drivebase.MECANUM, TeamRobot.REGINALD.drivebase());
        assertEquals(Drivebase.TANK, TeamRobot.NUGGET.drivebase());
    }

    @Test
    public void eachDrivebasesMotorsAreDrawnAsTheWheelsTheyTurnAndATanksAsItsFrontWheels() {
        PerWheel<String> drawn = new PerWheel<>("lf", "rf", "lb", "rb");

        assertEquals(List.of("lf", "rf", "lb", "rb"), Drivebase.MECANUM.perMotor(drawn));
        assertEquals(List.of("lf", "rf"), Drivebase.TANK.perMotor(drawn));
        for (Drivebase drivebase : Drivebase.values()) {
            assertEquals(drivebase.motors().size(), drivebase.perMotor(drawn).size());
        }
    }

    @Test
    public void eachDrivebaseNamesItsMotorsInTheOrderItsPowersAreRecorded() {
        assertEquals(List.of("LF", "RF", "LB", "RB"), Drivebase.MECANUM.motors());
        assertEquals(List.of("L", "R"), Drivebase.TANK.motors());
    }
}
