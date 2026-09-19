package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;

public class FakeGitTest extends GitContract {
    private FakeGit fake;

    @Override
    protected Git gitFor(Path root) {
        fake = new FakeGit(root, Worktrees.DEVELOP);
        return fake;
    }

    @Override
    protected void giveItARemoteItCanReach(String name) {
        fake.addRemote(name);
    }

    @Override
    protected void giveItARemoteItCannotReach(String name) {
        fake.addRemoteItCannotReach(name);
    }
}
