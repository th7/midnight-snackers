package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class LeftWorktrees {
    private LeftWorktrees() {}

    public static List<String> notice(List<String> directories) {
        if (directories.isEmpty()) {
            return List.of();
        }
        List<String> lines = new ArrayList<>();
        for (String directory : directories) {
            lines.add("  trees  " + directory);
        }
        lines.add("         left by the coding server: the robot gateway no longer manages them and never changes or"
                + " deletes them;");
        lines.add("         to rescue work nobody pushed, push the branch checked out in one with git by hand"
                + " (git -C <tree> branch --show-current names it)");
        return Collections.unmodifiableList(lines);
    }
}
