package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public abstract class GitContract {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final Git.Branch DEVELOP = Git.Branch.of(Worktrees.DEVELOP);
    private static final Git.Author ADA = Git.Author.of("ada", "ada@coding-server.invalid");

    protected Path root;
    protected Git git;

    protected abstract Git gitFor(Path root) throws IOException;

    private void started() throws IOException {
        if (git != null) {
            return;
        }
        root = folder.newFolder("root").toPath();
        GitFixture.init(root);
        git = gitFor(root);
    }

    private Path worktreeFor(String slug) throws IOException {
        started();
        Path at = folder.getRoot().toPath().resolve("worktrees").resolve(slug);
        Files.createDirectories(at.getParent());
        git.createWorktree(at, Git.Branch.of("coding/" + slug), DEVELOP);
        return at;
    }

    private void write(Path worktree, String name, String text) throws IOException {
        Files.write(worktree.resolve(name), text.getBytes(StandardCharsets.UTF_8));
    }

    @Test
    public void aNameGitWouldReadAsAnOptionIsNotABranch() {
        for (String refused : List.of("-q", "--force", "", "a b", "a..b", "a~1", "head:ref")) {
            try {
                Git.Branch.of(refused);
                fail("accepted " + refused + " as a branch");
            } catch (Git.Refused expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains("branch"));
            }
        }
    }

    @Test
    public void anAuthorNeedsANameAndAnEmailOnOneLine() {
        for (String[] refused :
                List.of(new String[] {"", "a@b"}, new String[] {"a", ""}, new String[] {"a\nb", "a@b"})) {
            try {
                Git.Author.of(refused[0], refused[1]);
                fail("accepted " + List.of(refused) + " as an author");
            } catch (Git.Refused expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains("author"));
            }
        }
    }

    @Test
    public void itSaysWhichGitItIs() throws IOException {
        started();

        assertTrue(git.version(), git.version().startsWith("git version"));
    }

    @Test
    public void aWorktreeIsMadeOnItsOwnBranchAndListedAsACheckout() throws IOException {
        Path at = worktreeFor("ada");

        assertTrue(Files.isDirectory(at));
        assertTrue(git.checkouts().stream().anyMatch(c -> c.branch.name().equals("coding/ada") && c.path.equals(at)));
        assertEquals(git.commitAt(DEVELOP.tip()), git.head(at));
    }

    @Test
    public void removingAWorktreeLeavesItsBranchAndItsCommits() throws IOException {
        Path at = worktreeFor("ada");
        write(at, "a.txt", "one");
        git.stageEverything(at);
        git.commitStaged(at, ADA, "ada's work");
        Git.Revision committed = git.head(at);

        git.removeWorktree(at);

        assertFalse(Files.isDirectory(at));
        assertEquals(committed, git.commitAt(Git.Branch.of("coding/ada").tip()));
    }

    @Test
    public void whatIsStagedIsWhatIsCommittedAndThenNothingIsUncommitted() throws IOException {
        Path at = worktreeFor("ada");
        write(at, "a.txt", "one");
        write(at, "b.txt", "two");

        assertEquals(List.of("a.txt", "b.txt"), git.uncommittedFiles(at));
        git.stageEverything(at);
        assertEquals(List.of("a.txt", "b.txt"), git.stagedFiles(at));

        Git.Revision before = git.head(at);
        git.commitStaged(at, ADA, "two files");

        assertNotEquals(before, git.head(at));
        assertEquals(List.of(), git.uncommittedFiles(at));
        assertEquals(List.of(), git.stagedFiles(at));
    }

    @Test
    public void commitsAheadAndBehindAreCounted() throws IOException {
        Path at = worktreeFor("ada");
        write(at, "a.txt", "one");
        git.stageEverything(at);
        git.commitStaged(at, ADA, "one");

        Git.Branch branch = Git.Branch.of("coding/ada");
        assertEquals(1, git.commitsBetween(DEVELOP.tip(), branch.tip()));
        assertEquals(0, git.commitsBetween(branch.tip(), DEVELOP.tip()));
        assertTrue(git.isAncestor(git.commitAt(DEVELOP.tip()), git.commitAt(branch.tip())));
        assertFalse(git.isAncestor(git.commitAt(branch.tip()), git.commitAt(DEVELOP.tip())));
    }

    @Test
    public void aRevisionNobodyHasIsAbsentRatherThanAFailure() throws IOException {
        started();

        assertTrue(git.commitIfThere(DEVELOP.tip()).isPresent());
        assertTrue(
                git.commitIfThere(Git.Revision.of("refs/heads/nobody-has-this")).isEmpty());
    }

    @Test
    public void mergeTreeSaysWhichFilesWouldConflictAndTouchesNoWorkingTree() throws IOException {
        Path ada = worktreeFor("ada");
        Path bob = worktreeFor("bob");
        write(ada, "shared.txt", "ada's line");
        git.stageEverything(ada);
        git.commitStaged(ada, ADA, "ada");
        write(bob, "shared.txt", "bob's line");
        git.stageEverything(bob);
        git.commitStaged(bob, Git.Author.of("bob", "bob@coding-server.invalid"), "bob");

        Git.MergeTree clean = git.mergeTree(git.commitAt(DEVELOP.tip()), git.head(ada));
        Git.MergeTree clashing = git.mergeTree(git.head(ada), git.head(bob));

        assertFalse(clean.conflicted());
        assertTrue(clashing.conflicted());
        assertEquals(List.of("shared.txt"), clashing.conflicts);
        assertEquals("a working tree is untouched by asking", List.of(), git.uncommittedFiles(ada));
    }

    @Test
    public void aMergeThatConflictsIsRefusedAndSaysSoRatherThanThrowing() throws IOException {
        Path ada = worktreeFor("ada");
        Path bob = worktreeFor("bob");
        write(ada, "shared.txt", "ada's line");
        git.stageEverything(ada);
        git.commitStaged(ada, ADA, "ada");
        write(bob, "shared.txt", "bob's line");
        git.stageEverything(bob);
        git.commitStaged(bob, Git.Author.of("bob", "bob@coding-server.invalid"), "bob");
        Git.Outcome moved = git.moveBranch(DEVELOP, git.head(ada), git.commitAt(DEVELOP.tip()));
        assertTrue(moved.said, moved.ok);

        Git.Outcome merged = git.merge(bob, ADA, "Pull develop", DEVELOP, Git.History.FAST_FORWARD_WHEN_IT_CAN);

        assertFalse(merged.ok);
        assertFalse(merged.said.isEmpty());
        git.abortMerge(bob);
        assertEquals(List.of(), git.uncommittedFiles(bob));
    }

    @Test
    public void aCleanMergeLandsAndFilesChangedBetweenNamesWhatItBrought() throws IOException {
        Path ada = worktreeFor("ada");
        Path bob = worktreeFor("bob");
        write(ada, "ada.txt", "ada's file");
        git.stageEverything(ada);
        git.commitStaged(ada, ADA, "ada");
        Git.Revision was = git.commitAt(DEVELOP.tip());
        assertTrue(git.moveBranch(DEVELOP, git.head(ada), was).ok);

        Git.Outcome merged = git.merge(bob, ADA, "Pull develop", DEVELOP, Git.History.FAST_FORWARD_WHEN_IT_CAN);

        assertTrue(merged.said, merged.ok);
        assertEquals(List.of("ada.txt"), git.filesChangedBetween(was, git.head(bob)));
    }

    @Test
    public void moveBranchRefusesWhenTheBranchHasMovedUnderIt() throws IOException {
        Path ada = worktreeFor("ada");
        write(ada, "a.txt", "one");
        git.stageEverything(ada);
        git.commitStaged(ada, ADA, "one");
        Git.Revision stale = git.commitAt(DEVELOP.tip());
        assertTrue(git.moveBranch(DEVELOP, git.head(ada), stale).ok);

        Git.Outcome again = git.moveBranch(DEVELOP, git.head(ada), stale);

        assertFalse("develop is no longer where the caller last saw it", again.ok);
    }

    @Test
    public void aFastForwardOnlyMergeCatchesAWorktreeUpOrRefuses() throws IOException {
        Path ada = worktreeFor("ada");
        Path bob = worktreeFor("bob");
        write(ada, "ada.txt", "ada's file");
        git.stageEverything(ada);
        git.commitStaged(ada, ADA, "ada");
        assertTrue(git.moveBranch(DEVELOP, git.head(ada), git.commitAt(DEVELOP.tip())).ok);

        assertTrue(git.fastForwardOnly(bob, DEVELOP).ok);
        assertEquals(git.commitAt(DEVELOP.tip()), git.head(bob));

        write(bob, "bob.txt", "bob's own");
        git.stageEverything(bob);
        git.commitStaged(bob, Git.Author.of("bob", "bob@coding-server.invalid"), "bob");
        write(ada, "ada2.txt", "more");
        git.stageEverything(ada);
        git.commitStaged(ada, ADA, "ada again");
        assertTrue(git.moveBranch(DEVELOP, git.head(ada), git.commitAt(DEVELOP.tip())).ok);

        assertFalse("bob has a commit develop does not", git.fastForwardOnly(bob, DEVELOP).ok);
    }

    @Test
    public void aTreeCanBeCommittedWithTwoParentsWithoutAWorkingTree() throws IOException {
        Path ada = worktreeFor("ada");
        write(ada, "ada.txt", "ada's file");
        git.stageEverything(ada);
        git.commitStaged(ada, ADA, "ada");
        Git.Revision develop = git.commitAt(DEVELOP.tip());
        Git.MergeTree tree = git.mergeTree(develop, git.head(ada));

        Git.Revision merge = git.commitTree(ADA, tree.tree, "Push ada's work", develop, git.head(ada));

        assertTrue(git.moveBranch(DEVELOP, merge, develop).ok);
        assertTrue(git.isAncestor(git.head(ada), git.commitAt(DEVELOP.tip())));
        assertTrue(git.isAncestor(develop, git.commitAt(DEVELOP.tip())));
    }

    @Test
    public void aRepositoryWithNoRemoteHasNothingToPushTo() throws IOException {
        started();

        assertFalse(git.hasRemote("origin"));
        assertTrue(git.remoteCommit("origin", DEVELOP).isEmpty());
    }

    @Test
    public void pushingSendsDevelopToTheRemoteAndThenHasNothingLeftToSend() throws IOException {
        started();
        GitFixture.withOrigin(root, folder.getRoot().toPath().resolve("origin.git"));
        Path ada = worktreeFor("ada");
        write(ada, "ada.txt", "ada's file");
        git.stageEverything(ada);
        git.commitStaged(ada, ADA, "ada");
        assertTrue(git.moveBranch(DEVELOP, git.head(ada), git.commitAt(DEVELOP.tip())).ok);

        assertTrue(git.hasRemote("origin"));
        Git.Outcome pushed = git.push("origin", DEVELOP);

        assertTrue(pushed.said, pushed.ok);
        assertEquals(
                git.commitAt(DEVELOP.tip()), git.remoteCommit("origin", DEVELOP).orElseThrow());
    }

    @Test
    public void aRemoteThatCannotBeReachedIsRefusedRatherThanThrown() throws IOException {
        started();
        GitFixture.git(
                root,
                "remote",
                "add",
                "origin",
                root.resolve("no-such-origin.git").toString());

        Git.Outcome pushed = git.push("origin", DEVELOP);

        assertFalse(pushed.ok);
        assertFalse(pushed.said.isEmpty());
    }

    @Test
    public void pruningForgetsAWorktreeDirectoryThatHasGone() throws IOException {
        Path at = worktreeFor("ada");
        deleteTree(at);

        git.pruneWorktrees();

        assertFalse(git.checkouts().stream().anyMatch(c -> c.path.equals(at)));
        git.addWorktree(at, Git.Branch.of("coding/ada"));
        assertTrue(Files.isDirectory(at));
    }

    @Test
    public void deletingABranchTakesItAway() throws IOException {
        Path at = worktreeFor("ada");
        git.removeWorktree(at);
        Git.Branch branch = Git.Branch.of("coding/ada");
        assertTrue(git.commitIfThere(branch.tip()).isPresent());

        git.deleteBranch(branch);

        assertTrue(git.commitIfThere(branch.tip()).isEmpty());
    }

    private static void deleteTree(Path root) throws IOException {
        try (var walk = Files.walk(root)) {
            for (Path path : walk.sorted(java.util.Comparator.reverseOrder()).toList()) {
                Files.deleteIfExists(path);
            }
        }
    }
}
