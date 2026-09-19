package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;

public class RealGitTest extends GitContract {
    @Override
    protected Git gitFor(Path root) {
        return new RealGit("git", root);
    }
}
