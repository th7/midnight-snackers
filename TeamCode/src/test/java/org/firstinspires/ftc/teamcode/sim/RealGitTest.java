package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.nio.file.Path;

public class RealGitTest extends GitContract {
    @Override
    protected Git gitFor(Path root) {
        return new RealGit("git", root);
    }

    @Override
    protected void giveItARemoteItCanReach(String name) throws IOException {
        GitFixture.withOrigin(root, folder.getRoot().toPath().resolve(name + ".git"));
    }

    @Override
    protected void giveItARemoteItCannotReach(String name) throws IOException {
        GitFixture.git(
                root,
                "remote",
                "add",
                name,
                root.resolve("no-such-" + name + ".git").toString());
    }
}
