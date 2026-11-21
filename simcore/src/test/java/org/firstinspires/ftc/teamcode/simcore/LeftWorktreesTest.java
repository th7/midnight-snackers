package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import java.util.List;
import org.junit.Test;

public class LeftWorktreesTest {
    @Test
    public void noWorktreesLeftIsNothingToSay() {
        assertEquals(List.of(), LeftWorktrees.notice(List.of()));
    }

    @Test
    public void eachDirectoryLeftIsNamedAndTheRescueIsSaidOnce() {
        assertEquals(
                List.of(
                        "  trees  /state/worktrees/midnight-snackers-1a2b3c4d",
                        "  trees  /state/worktrees/midnight-snackers-1a2b3c4d-nugget",
                        "         left by the coding server: the robot gateway no longer manages them and never"
                                + " changes or deletes them;",
                        "         to rescue work nobody pushed, push the branch checked out in one (coding/<name>,"
                                + " or nugget/<name> for Nugget) with git by hand"),
                LeftWorktrees.notice(List.of(
                        "/state/worktrees/midnight-snackers-1a2b3c4d",
                        "/state/worktrees/midnight-snackers-1a2b3c4d-nugget")));
    }
}
