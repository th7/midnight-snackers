package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.Test;

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

    @Test
    public void aFakeMadeOfWhatIsOnDiskStartsWithItCommitted() throws IOException {
        Path where = folder.newFolder("already-there").toPath();
        Files.createDirectories(where.resolve("TeamCode/src"));
        Files.write(where.resolve("TeamCode/src/Plans.java"), "class Plans {}\n".getBytes(StandardCharsets.UTF_8));
        Files.write(where.resolve("README"), "a project\n".getBytes(StandardCharsets.UTF_8));

        FakeGit made = FakeGit.ofWhatIsOnDisk(where, Worktrees.DEVELOP);

        assertEquals(
                "nothing is uncommitted: what was there is the first commit", List.of(), made.uncommittedFiles(where));
        Path worktree = folder.getRoot().toPath().resolve("worktrees/ada");
        made.createWorktree(worktree, Git.Branch.of("coding/ada"), Git.Branch.of(Worktrees.DEVELOP));

        assertEquals(
                "and a worktree of it carries the files, which is what a bench builds from",
                "class Plans {}\n",
                new String(Files.readAllBytes(worktree.resolve("TeamCode/src/Plans.java")), StandardCharsets.UTF_8));
        assertEquals("a project\n", new String(Files.readAllBytes(worktree.resolve("README")), StandardCharsets.UTF_8));
    }

    @Test
    public void aFileThisGitCannotHoldAsTextIsRefusedByNameRatherThanMangled() throws IOException {
        Path where = folder.newFolder("with-a-model").toPath();
        Files.write(where.resolve("README"), "a project\n".getBytes(StandardCharsets.UTF_8));
        Files.write(where.resolve("field.glb"), new byte[] {(byte) 0xff, (byte) 0xfe, 0x00, 0x01});

        IllegalStateException refused = org.junit.Assert.assertThrows(
                IllegalStateException.class, () -> FakeGit.ofWhatIsOnDisk(where, Worktrees.DEVELOP));

        assertTrue(refused.getMessage(), refused.getMessage().contains("field.glb"));
        assertTrue(
                "it says what to do instead, since a mangled model is a test passing for a wrong reason",
                refused.getMessage().contains("real git"));
    }

    @Test
    public void aFakeMadeOfWhatIsOnDiskLeavesARealRepositoryThereAlone() throws IOException {
        Path where = folder.newFolder("with-a-dot-git").toPath();
        Files.createDirectories(where.resolve(".git/objects"));
        Files.write(where.resolve(".git/HEAD"), "ref: refs/heads/develop\n".getBytes(StandardCharsets.UTF_8));
        Files.write(where.resolve("README"), "a project\n".getBytes(StandardCharsets.UTF_8));

        FakeGit made = FakeGit.ofWhatIsOnDisk(where, Worktrees.DEVELOP);
        Path worktree = folder.getRoot().toPath().resolve("worktrees/bob");
        made.createWorktree(worktree, Git.Branch.of("coding/bob"), Git.Branch.of(Worktrees.DEVELOP));

        assertTrue("what git keeps for itself is not a file of the project", !Files.exists(worktree.resolve(".git")));
        assertEquals(List.of(), made.uncommittedFiles(where));
    }
}
