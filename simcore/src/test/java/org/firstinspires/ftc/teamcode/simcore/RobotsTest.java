package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class RobotsTest {
    private static final Robots BOTH = Robots.only(TeamRobot.NUGGET).with(TeamRobot.REGINALD);

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    @Test
    public void aUserLetOntoOneRobotWorksOnThatOneAlone() {
        Robots nugget = Robots.only(TeamRobot.NUGGET);

        assertEquals(List.of(TeamRobot.NUGGET), nugget.all());
        assertTrue(nugget.has(TeamRobot.NUGGET));
        assertFalse(nugget.has(TeamRobot.REGINALD));
    }

    @Test
    public void bothRobotsAreListedInTheTeamsOrderWhicheverCameFirst() {
        assertEquals(List.of(TeamRobot.REGINALD, TeamRobot.NUGGET), BOTH.all());
        assertEquals(List.of("reginald", "nugget"), BOTH.asked());
        assertEquals(BOTH, Robots.only(TeamRobot.REGINALD).with(TeamRobot.NUGGET));
        assertEquals(
                BOTH.hashCode(),
                Robots.only(TeamRobot.REGINALD).with(TeamRobot.NUGGET).hashCode());
        assertEquals(BOTH, BOTH.with(TeamRobot.NUGGET));
        assertNotEquals(BOTH, Robots.only(TeamRobot.NUGGET));
    }

    @Test
    public void takingOneOfTwoAwayLeavesTheOther() {
        assertEquals(Robots.only(TeamRobot.REGINALD), Valid.value(BOTH.without(TeamRobot.NUGGET)));
        assertEquals(Robots.only(TeamRobot.NUGGET), Valid.value(BOTH.without(TeamRobot.REGINALD)));
    }

    @Test
    public void theLastRobotCannotBeTakenAwaySayingWhy() {
        String said = rejection(Robots.only(TeamRobot.NUGGET).without(TeamRobot.NUGGET));

        assertTrue(said, said.contains("at least one robot"));
        assertTrue(said, said.contains("Nugget"));
    }

    @Test
    public void takingAwayARobotTheyDoNotHaveLeavesThemAsTheyWere() {
        Robots reginald = Robots.only(TeamRobot.REGINALD);

        assertEquals(reginald, Valid.value(reginald.without(TeamRobot.NUGGET)));
    }

    @Test
    public void aLoginWorksOnTheRobotItAskedForWhileTheUserMay() {
        assertEquals(TeamRobot.NUGGET, BOTH.workingOn(TeamRobot.NUGGET));
        assertEquals(TeamRobot.REGINALD, BOTH.workingOn(TeamRobot.REGINALD));
        assertEquals(TeamRobot.NUGGET, Robots.only(TeamRobot.NUGGET).workingOn(TeamRobot.NUGGET));
    }

    @Test
    public void aLoginOnARobotTheUserMayNotWorkOnIsMovedToOneTheyMay() {
        assertEquals(TeamRobot.REGINALD, Robots.only(TeamRobot.REGINALD).workingOn(TeamRobot.NUGGET));
        assertEquals(TeamRobot.NUGGET, Robots.only(TeamRobot.NUGGET).workingOn(TeamRobot.REGINALD));
    }

    @Test
    public void aUserSwitchesOnlyToARobotTheyMayWorkOn() {
        assertEquals(TeamRobot.NUGGET, Valid.value(BOTH.switchTo(TeamRobot.NUGGET)));
        assertEquals(TeamRobot.REGINALD, Valid.value(BOTH.switchTo(TeamRobot.REGINALD)));

        String said = rejection(Robots.only(TeamRobot.REGINALD).switchTo(TeamRobot.NUGGET));

        assertTrue(said, said.contains("Nugget"));
        assertTrue(said, said.contains("admin"));
    }

    @Test
    public void lettingAUserOntoARobotAddsItToWhateverTheyHad() {
        assertEquals(Robots.only(TeamRobot.NUGGET), Robots.letOnto(Optional.empty(), TeamRobot.NUGGET));
        assertEquals(BOTH, Robots.letOnto(Optional.of(Robots.only(TeamRobot.REGINALD)), TeamRobot.NUGGET));
        assertEquals(BOTH, Robots.letOnto(Optional.of(BOTH), TeamRobot.REGINALD));
    }

    @Test
    public void robotsAreReadBackByTheNamesTheyAreAskedFor() {
        assertEquals(BOTH, Valid.value(Robots.named(BOTH.asked())));
        assertEquals(BOTH, Valid.value(Robots.named(List.of("nugget", "reginald", "nugget"))));
        assertEquals(Robots.only(TeamRobot.NUGGET), Valid.value(Robots.named(List.of("nugget"))));
    }

    @Test
    public void namesThatAreNoRobotOrNoneAtAllAreRejected() {
        String none = rejection(Robots.named(List.of()));
        String wrong = rejection(Robots.named(List.of("reginald", "bender")));

        assertTrue(none, none.contains("at least one robot"));
        assertTrue(wrong, wrong.contains("bender"));
    }
}
