package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import static org.junit.Assume.assumeTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.Comparator;
import java.util.stream.Stream;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class WorktreesTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private Path root;
    private Path stateDir;

    @Before
    public void aRepositoryWithADevelopBranch() throws IOException {
        root = folder.getRoot().toPath().resolve("project");
        stateDir = folder.getRoot().toPath().resolve("state").resolve("coding-server");
        GitFixture.init(root);
    }

    private Worktrees worktrees() {
        return new Worktrees(root, stateDir, "git");
    }

    private static String read(Path file) throws IOException {
        return new String(Files.readAllBytes(file), StandardCharsets.UTF_8);
    }

    private static void deleteTree(Path dir) throws IOException {
        try (Stream<Path> walk = Files.walk(dir)) {
            walk.sorted(Comparator.reverseOrder()).forEach(p -> p.toFile().delete());
        }
    }

    @Test
    public void ensureMakesAWorktreeOnItsOwnBranchAtDevelopNotAtTheHostsHead() throws IOException {
        String develop = GitFixture.head(root);
        GitFixture.git(root, "checkout", "-q", "-b", "main");
        Files.write(root.resolve("README"), "on main\n".getBytes(StandardCharsets.UTF_8));
        GitFixture.commitAll(root, "a commit on main");
        Worktrees worktrees = worktrees();

        Worktrees.Worktree ada = worktrees.ensure("Ada Lovelace");

        assertEquals("ada-lovelace", ada.slug);
        assertEquals("coding/ada-lovelace", ada.branch);
        assertEquals(worktrees.directory().resolve("ada-lovelace"), ada.path);
        assertTrue(ada.path.startsWith(stateDir));
        assertEquals(develop, GitFixture.head(ada.path));
        assertEquals(
                "coding/ada-lovelace",
                GitFixture.git(ada.path, "rev-parse", "--abbrev-ref", "HEAD").trim());
        assertEquals("hello\n", read(ada.path.resolve("README")));
        String listed = GitFixture.git(root, "worktree", "list", "--porcelain");
        assertTrue(listed, listed.contains("worktree " + ada.path.toRealPath()));
        assertTrue(listed, listed.contains("branch refs/heads/coding/ada-lovelace"));
    }

    @Test
    public void ensureTwiceIsTheSameWorktreeAndACollidingNameGetsItsOwn() throws IOException {
        Worktrees worktrees = worktrees();

        Worktrees.Worktree first = worktrees.ensure("Ada Lovelace");
        Worktrees.Worktree again = worktrees.ensure("Ada Lovelace");
        Worktrees.Worktree second = worktrees.ensure("ada  LOVELACE");
        Worktrees.Worktree third = worktrees.ensure("ada-lovelace");

        assertEquals(first.path, again.path);
        assertEquals(first.branch, again.branch);
        assertEquals("ada-lovelace-2", second.slug);
        assertEquals("coding/ada-lovelace-2", second.branch);
        assertEquals("ada-lovelace-3", third.slug);
        assertNotEquals(first.path, second.path);
        assertTrue(Files.isDirectory(second.path.resolve(".git").getParent()));
        assertTrue(Files.exists(third.path.resolve("README")));
    }

    @Test
    public void nastyUsernamesGiveValidRefsInsideTheWorktreesDirectory() throws IOException {
        Worktrees worktrees = worktrees();
        String[] names = {
            "..",
            "a/b",
            "über",
            "!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~",
            "-leading",
            "trailing-",
            ".git",
            "refs/heads/develop"
        };

        for (String name : names) {
            Worktrees.Worktree worktree = worktrees.ensure(name);
            assertTrue(
                    name + " -> " + worktree.path,
                    worktree.path.toRealPath().startsWith(worktrees.directory().toRealPath()));
            assertEquals(name, worktrees.directory(), worktree.path.getParent());
            assertTrue(name + " -> " + worktree.slug, worktree.slug.matches("[a-z0-9]([a-z0-9-]*[a-z0-9])?"));
            assertTrue(name + " -> " + worktree.branch, worktree.branch.startsWith("coding/"));
            GitFixture.git(root, "check-ref-format", "--branch", worktree.branch);
            assertEquals(name, "hello\n", read(worktree.path.resolve("README")));
        }
        assertEquals(names.length, worktrees.directory().toFile().list().length);
        assertEquals(
                "develop is still the only branch outside coding/",
                "develop\n",
                GitFixture.git(
                        root,
                        "for-each-ref",
                        "--format=%(refname:short)",
                        "refs/heads/",
                        "--exclude=refs/heads/coding/*"));
    }

    @Test
    public void theStoreOutlivesTheProcessAndIsOwnerOnly() throws IOException {
        Worktrees.Worktree ada = worktrees().ensure("Ada Lovelace");
        worktrees().ensure("ada-lovelace");

        Worktrees later = worktrees();

        assertEquals(ada.path, later.find("Ada Lovelace").path);
        assertEquals(ada.branch, later.find("Ada Lovelace").branch);
        assertEquals("ada-lovelace-2", later.find("ada-lovelace").slug);
        assertNull(later.find("nobody"));
        assertEquals(ada.path, later.ensure("Ada Lovelace").path);
        Path store = stateDir.resolve(Worktrees.STORE_FILE);
        assertTrue(Files.exists(store));
        if (FileSystems.getDefault().supportedFileAttributeViews().contains("posix")) {
            assertEquals("rw-------", PosixFilePermissions.toString(Files.getPosixFilePermissions(store)));
            assertEquals("rwx------", PosixFilePermissions.toString(Files.getPosixFilePermissions(stateDir)));
        }
    }

    @Test
    public void theStoreIsKeptPerProjectRoot() throws IOException {
        Path other = folder.getRoot().toPath().resolve("other");
        GitFixture.init(other);
        Worktrees.Worktree here = worktrees().ensure("ada");

        Worktrees.Worktree there = new Worktrees(other, stateDir, "git").ensure("ada");

        assertNotEquals(here.path, there.path);
        assertEquals("ada", there.slug);
        assertEquals(here.path, worktrees().find("ada").path);
        assertNotEquals(here.path.getParent(), there.path.getParent());
        assertTrue(here.path.getParent().getFileName().toString().startsWith("project-"));
        assertTrue(there.path.getParent().getFileName().toString().startsWith("other-"));
    }

    @Test
    public void anUnreadableStoreStopsStartupNamingTheFile() throws IOException {
        worktrees().ensure("ada");
        Path store = stateDir.resolve(Worktrees.STORE_FILE);
        Files.write(store, "{not json".getBytes(StandardCharsets.UTF_8));

        try {
            worktrees();
            fail("a store that cannot be read must not be silently started over");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains(store.toString()));
        }
    }

    @Test
    public void aDeletedDirectoryIsRecreatedOnTheSameBranchWithItsCommittedContent() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("README"), "ada's work\n".getBytes(StandardCharsets.UTF_8));
        String committed = GitFixture.commitAll(ada.path, "ada's commit");
        deleteTree(ada.path);
        assertFalse(Files.exists(ada.path));

        Worktrees.Worktree back = worktrees.ensure("ada");

        assertEquals(ada.path, back.path);
        assertEquals(ada.branch, back.branch);
        assertEquals(committed, GitFixture.head(back.path));
        assertEquals("ada's work\n", read(back.path.resolve("README")));
        assertEquals("the branch was not moved", committed, GitFixture.commitOf(root, "coding/ada"));
    }

    @Test
    public void nothingIsWrittenUnderTheProjectRootOutsideDotGit() throws IOException {
        worktrees().ensure("ada");

        assertEquals(
                "[.git, README]", java.util.Arrays.toString(sorted(root.toFile().list())));
    }

    private static String[] sorted(String[] names) {
        java.util.Arrays.sort(names);
        return names;
    }

    @Test
    public void removeTakesTheWorktreeDirectoryAndLeavesTheBranchAndTheMapping() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("Plans.java"), "ada's work\n".getBytes(StandardCharsets.UTF_8));
        worktrees.commit("ada", "ada's work");
        worktrees.push("ada");
        String landed = GitFixture.commitOf(root, "coding/ada");

        Worktrees.Removal removal = worktrees.remove("ada", false);

        assertTrue(removal.removed);
        assertNull(removal.refused);
        assertFalse(Files.exists(ada.path));
        String listed = GitFixture.git(root, "worktree", "list", "--porcelain");
        assertFalse(listed, listed.contains("coding/ada"));
        assertEquals("the branch was not moved", landed, GitFixture.commitOf(root, "coding/ada"));
        assertEquals("the mapping is still ada's", ada.path, worktrees.find("ada").path);
        assertEquals("ada", worktrees.find("ada").slug);
        assertEquals(
                "and in the store, for the next process", ada.path, worktrees().find("ada").path);
    }

    @Test
    public void ensureAfterARemoveRebuildsTheWorktreeOnTheSameBranchWithItsCommits() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("README"), "ada's work\n".getBytes(StandardCharsets.UTF_8));
        String committed = GitFixture.commitAll(ada.path, "ada's commit");
        worktrees.remove("ada", true);
        assertFalse(Files.exists(ada.path));

        Worktrees.Worktree back = worktrees.ensure("ada");

        assertEquals(ada.path, back.path);
        assertEquals(ada.branch, back.branch);
        assertEquals("ada", back.slug);
        assertEquals(committed, GitFixture.head(back.path));
        assertEquals("ada's work\n", read(back.path.resolve("README")));
    }

    @Test
    public void removeRefusesWhenTheUserHasUncommittedFiles() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("Plans.java"), "half-typed\n".getBytes(StandardCharsets.UTF_8));

        Worktrees.Removal removal = worktrees.remove("ada", false);

        assertFalse(removal.removed);
        assertEquals("[Plans.java]", removal.refused.changed.toString());
        assertEquals(0, removal.refused.ahead);
        assertFalse(removal.refused.none());
        assertEquals("nothing was touched", "half-typed\n", read(ada.path.resolve("Plans.java")));
    }

    @Test
    public void removeRefusesWhenTheUsersBranchIsAheadOfDevelop() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("Plans.java"), "ada's work\n".getBytes(StandardCharsets.UTF_8));
        String committed = GitFixture.commitAll(ada.path, "ada's commit");

        Worktrees.Removal removal = worktrees.remove("ada", false);

        assertFalse(removal.removed);
        assertEquals("[]", removal.refused.changed.toString());
        assertEquals(1, removal.refused.ahead);
        assertEquals("nothing was touched", committed, GitFixture.head(ada.path));
    }

    @Test
    public void aForcedRemoveTakesTheWorktreeAndKeepsWhatWasCommitted() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("Plans.java"), "ada's work\n".getBytes(StandardCharsets.UTF_8));
        String committed = GitFixture.commitAll(ada.path, "ada's commit");

        Worktrees.Removal removal = worktrees.remove("ada", true);

        assertTrue(removal.removed);
        assertNull(removal.refused);
        assertFalse(Files.exists(ada.path));
        assertEquals("her commit is still on her branch", committed, GitFixture.commitOf(root, "coding/ada"));
        assertEquals("ada's work\n", GitFixture.git(root, "show", "coding/ada:Plans.java"));
        assertNotEquals(GitFixture.commitOf(root, "develop"), committed);
    }

    @Test
    public void removeMakesNoWorktreeForAUserWhoHasNone() throws IOException {
        Worktrees worktrees = worktrees();

        Worktrees.Removal removal = worktrees.remove("nobody", false);

        assertFalse(removal.removed);
        assertNull(removal.refused);
        assertNull(worktrees.find("nobody"));
        assertFalse(Files.exists(worktrees.directory().resolve("nobody")));
    }

    @Test
    public void removeSurvivesADirectoryThatHasAlreadyGoneAndLetsItBeMadeAgain() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        deleteTree(ada.path);

        Worktrees.Removal removal = worktrees.remove("ada", false);

        assertTrue(removal.removed);
        assertFalse(Files.exists(ada.path));
        assertEquals("git's administrative files went too", ada.path, worktrees.ensure("ada").path);
        assertEquals("hello\n", read(ada.path.resolve("README")));
    }

    @Test
    public void aRemovedUsersSlugIsStillTheirsAndStillTakenForEveryoneElse() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        worktrees.remove("ada", true);

        Worktrees.Worktree other = worktrees.ensure("Ada");
        Worktrees.Worktree back = worktrees.ensure("ada");

        assertEquals("ada-2", other.slug);
        assertEquals("ada", back.slug);
        assertEquals(ada.path, back.path);
        assertEquals(ada.branch, back.branch);
    }

    @Test
    public void unsavedIsWhatADeleteWouldThrowAwayAndNothingForAUserWithNoWorktree() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        assertTrue(worktrees.unsaved("ada").none());
        Files.write(ada.path.resolve("Plans.java"), "ada's work\n".getBytes(StandardCharsets.UTF_8));
        worktrees.commit("ada", "ada's work");
        Files.write(ada.path.resolve("Notes.md"), "and more\n".getBytes(StandardCharsets.UTF_8));

        Worktrees.Unsaved unsaved = worktrees.unsaved("ada");

        assertEquals("[Notes.md]", unsaved.changed.toString());
        assertEquals(1, unsaved.ahead);
        assertFalse(unsaved.none());
        assertTrue(worktrees.unsaved("nobody").none());
        assertNull("asking must not make a worktree", worktrees.find("nobody"));
    }

    @Test
    public void statusListsTheChangedFilesAndCommitMakesOneCommitAuthoredByTheUsername() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("Ada Lovelace");
        assertEquals("[]", worktrees.status("Ada Lovelace").changed.toString());
        Files.write(ada.path.resolve("README"), "ada's\n".getBytes(StandardCharsets.UTF_8));
        Files.write(ada.path.resolve("New.java"), "class New {}\n".getBytes(StandardCharsets.UTF_8));

        Worktrees.Status before = worktrees.status("Ada Lovelace");
        Worktrees.Commit commit = worktrees.commit("Ada Lovelace", "my change");
        Worktrees.Status after = worktrees.status("Ada Lovelace");

        assertEquals("[New.java, README]", before.changed.toString());
        assertEquals(0, before.ahead);
        assertEquals(0, before.behind);
        assertEquals("the tip before is develop's", GitFixture.commitOf(root, "develop"), before.head);
        assertEquals("the tip after is the commit", commit.commit, after.head);
        assertTrue(commit.made);
        assertEquals("[New.java, README]", commit.files.toString());
        assertEquals(commit.commit, GitFixture.head(ada.path));
        assertEquals(
                "Ada Lovelace|ada-lovelace@coding-server.invalid|my change\n",
                GitFixture.git(ada.path, "log", "-1", "--format=%an|%ae|%s"));
        assertEquals("ada's\n", GitFixture.git(ada.path, "show", "HEAD:README"));
        assertEquals("", GitFixture.git(ada.path, "status", "--porcelain"));
        assertEquals("[]", after.changed.toString());
        assertEquals(1, after.ahead);
        assertEquals(0, after.behind);
        assertEquals(
                "develop did not move",
                GitFixture.commitOf(root, "develop"),
                GitFixture.git(root, "rev-parse", "develop").trim());
        assertNotEquals(commit.commit, GitFixture.commitOf(root, "develop"));
    }

    @Test
    public void commitWithNothingChangedMakesNoCommitAndSaysSo() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        String head = GitFixture.head(ada.path);

        Worktrees.Commit commit = worktrees.commit("ada", "nothing");

        assertFalse(commit.made);
        assertEquals("[]", commit.files.toString());
        assertEquals(head, GitFixture.head(ada.path));
    }

    @Test
    public void statusCountsTheCommitsOnDevelopTheUserLacks() throws IOException {
        Worktrees worktrees = worktrees();
        worktrees.ensure("ada");
        Files.write(root.resolve("README"), "on develop\n".getBytes(StandardCharsets.UTF_8));
        GitFixture.commitAll(root, "a commit on develop");

        Worktrees.Status status = worktrees.status("ada");

        assertEquals(0, status.ahead);
        assertEquals(1, status.behind);
        assertEquals("[]", status.changed.toString());
    }

    private void commitOnDevelop(String file, String content) throws IOException {
        Files.write(root.resolve(file), content.getBytes(StandardCharsets.UTF_8));
        GitFixture.commitAll(root, "a commit on develop: " + file);
    }

    @Test
    public void pullFastForwardsWhenTheUserHasNoCommitsAndMergesWhenTheyDo() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        commitOnDevelop("README", "first on develop\n");

        Worktrees.Merge fastForward = worktrees.pull("ada");

        assertEquals(Worktrees.Outcome.MERGED, fastForward.outcome);
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.head(ada.path));
        assertEquals("first on develop\n", read(ada.path.resolve("README")));

        Files.write(ada.path.resolve("Mine.java"), "class Mine {}\n".getBytes(StandardCharsets.UTF_8));
        worktrees.commit("ada", "mine");
        commitOnDevelop("README", "second on develop\n");
        Worktrees.Merge merge = worktrees.pull("ada");

        assertEquals(Worktrees.Outcome.MERGED, merge.outcome);
        assertEquals("second on develop\n", read(ada.path.resolve("README")));
        assertEquals(
                "a merge commit with both parents",
                3,
                GitFixture.git(ada.path, "rev-list", "--parents", "-1", "HEAD")
                        .trim()
                        .split(" ")
                        .length);
        assertEquals(
                "ada", GitFixture.git(ada.path, "log", "-1", "--format=%an").trim());
        assertEquals(0, worktrees.status("ada").behind);
        assertEquals(2, worktrees.status("ada").ahead);
        assertEquals("", GitFixture.git(ada.path, "status", "--porcelain"));
    }

    @Test
    public void pullWithNothingNewDoesNothing() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        String head = GitFixture.head(ada.path);
        Files.write(ada.path.resolve("Mine.java"), "class Mine {}\n".getBytes(StandardCharsets.UTF_8));

        Worktrees.Merge nothing = worktrees.pull("ada");

        assertEquals(Worktrees.Outcome.NOTHING, nothing.outcome);
        assertEquals(head, GitFixture.head(ada.path));
        assertEquals("[Mine.java]", worktrees.status("ada").changed.toString());
    }

    @Test
    public void pullKeepsUncommittedEditsInFilesDevelopDidNotTouch() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("Mine.java"), "class Mine {}\n".getBytes(StandardCharsets.UTF_8));
        commitOnDevelop("README", "first on develop\n");

        Worktrees.Merge fastForward = worktrees.pull("ada");

        assertEquals(Worktrees.Outcome.MERGED, fastForward.outcome);
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.head(ada.path));
        assertEquals("first on develop\n", read(ada.path.resolve("README")));
        assertEquals("class Mine {}\n", read(ada.path.resolve("Mine.java")));
        assertEquals(
                "the edit is still uncommitted",
                "[Mine.java]",
                worktrees.status("ada").changed.toString());

        worktrees.commit("ada", "mine");
        Files.write(ada.path.resolve("Mine.java"), "class Mine { int edited; }\n".getBytes(StandardCharsets.UTF_8));
        commitOnDevelop("README", "second on develop\n");
        Worktrees.Merge merge = worktrees.pull("ada");

        assertEquals(Worktrees.Outcome.MERGED, merge.outcome);
        assertEquals(
                "a merge commit with both parents",
                3,
                GitFixture.git(ada.path, "rev-list", "--parents", "-1", "HEAD")
                        .trim()
                        .split(" ")
                        .length);
        assertEquals("second on develop\n", read(ada.path.resolve("README")));
        assertEquals("class Mine { int edited; }\n", read(ada.path.resolve("Mine.java")));
        assertEquals("[Mine.java]", worktrees.status("ada").changed.toString());
        assertEquals(
                "the merge commit took only the merge",
                "class Mine {}\n",
                GitFixture.git(ada.path, "show", "HEAD:Mine.java"));
    }

    @Test
    public void pullWithUncommittedEditsInFilesDevelopChangedIsRefusedNamingOnlyThose() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        String head = GitFixture.head(ada.path);
        commitOnDevelop("README", "on develop\n");
        Files.write(ada.path.resolve("README"), "ada's uncommitted line\n".getBytes(StandardCharsets.UTF_8));
        Files.write(ada.path.resolve("Mine.java"), "class Mine {}\n".getBytes(StandardCharsets.UTF_8));

        Worktrees.Merge refused = worktrees.pull("ada");

        assertEquals(Worktrees.Outcome.UNCOMMITTED, refused.outcome);
        assertEquals("[README]", refused.files.toString());
        assertEquals(head, GitFixture.head(ada.path));
        assertEquals("ada's uncommitted line\n", read(ada.path.resolve("README")));
        assertEquals("class Mine {}\n", read(ada.path.resolve("Mine.java")));
        assertFalse("no merge in progress", Files.exists(gitDir(ada.path).resolve("MERGE_HEAD")));

        Files.write(ada.path.resolve("README"), "hello\n".getBytes(StandardCharsets.UTF_8));
        commitOnDevelop("New.java", "class New {}\n");
        Files.write(ada.path.resolve("New.java"), "class New { int ada; }\n".getBytes(StandardCharsets.UTF_8));
        Worktrees.Merge untracked = worktrees.pull("ada");

        assertEquals("a new file of ada's that develop also adds", Worktrees.Outcome.UNCOMMITTED, untracked.outcome);
        assertEquals("[New.java]", untracked.files.toString());
        assertEquals(head, GitFixture.head(ada.path));
        assertEquals("class New { int ada; }\n", read(ada.path.resolve("New.java")));
    }

    @Test
    public void aPullThatConflictsChangesNothingAndNamesTheFiles() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("README"), "ada's line\n".getBytes(StandardCharsets.UTF_8));
        worktrees.commit("ada", "ada's");
        String head = GitFixture.head(ada.path);
        commitOnDevelop("README", "develop's line\n");
        String develop = GitFixture.commitOf(root, "develop");

        Worktrees.Merge conflicted = worktrees.pull("ada");

        assertEquals(Worktrees.Outcome.CONFLICTS, conflicted.outcome);
        assertEquals("[README]", conflicted.files.toString());
        assertEquals(head, GitFixture.head(ada.path));
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
        assertEquals("ada's line\n", read(ada.path.resolve("README")));
        assertEquals("", GitFixture.git(ada.path, "status", "--porcelain"));
        assertFalse(
                "no merge in progress",
                Files.exists(ada.path.resolve(".git"))
                        && Files.exists(gitDir(ada.path).resolve("MERGE_HEAD")));
    }

    private static Path gitDir(Path worktree) throws IOException {
        return Path.of(GitFixture.git(worktree, "rev-parse", "--git-dir").trim());
    }

    private static int parentsOf(Path cwd, String ref) throws IOException {
        return GitFixture.git(cwd, "rev-list", "--parents", "-1", ref).trim().split(" ").length - 1;
    }

    @Test
    public void pushLandsTheUsersCommitsOnDevelopWhereItIsCheckedOutAndBringsTheWorktreeUpToDate() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        String oldDevelop = GitFixture.commitOf(root, "develop");
        Files.write(ada.path.resolve("Mine.java"), "class Mine {}\n".getBytes(StandardCharsets.UTF_8));
        String mine = worktrees.commit("ada", "mine").commit;

        Worktrees.Merge pushed = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.MERGED, pushed.outcome);
        assertNull(pushed.detail);
        String develop = GitFixture.commitOf(root, "develop");
        assertEquals(2, parentsOf(root, "develop"));
        assertEquals(
                oldDevelop + " " + mine,
                GitFixture.git(root, "rev-list", "--parents", "-1", "develop")
                        .trim()
                        .substring(develop.length() + 1));
        assertEquals(
                "the host checkout, on develop, shows the pushed file",
                "class Mine {}\n",
                read(root.resolve("Mine.java")));
        assertEquals(develop, GitFixture.head(root));
        assertEquals("", GitFixture.git(root, "status", "--porcelain"));
        assertEquals("the user branch was fast-forwarded to develop", develop, GitFixture.head(ada.path));
        assertEquals("", GitFixture.git(ada.path, "status", "--porcelain"));
        assertEquals(0, worktrees.status("ada").ahead);
        assertEquals(0, worktrees.status("ada").behind);
    }

    @Test
    public void pushWhenDevelopIsCheckedOutNowhereMovesOnlyTheBranch() throws IOException {
        GitFixture.git(root, "checkout", "-q", "-b", "main");
        String mainHead = GitFixture.head(root);
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("Mine.java"), "class Mine {}\n".getBytes(StandardCharsets.UTF_8));
        String mine = worktrees.commit("ada", "mine").commit;

        Worktrees.Merge pushed = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.MERGED, pushed.outcome);
        assertEquals(2, parentsOf(root, "develop"));
        assertEquals("class Mine {}\n", GitFixture.git(root, "show", "develop:Mine.java"));
        assertEquals(
                "ada",
                GitFixture.git(root, "log", "-1", "--format=%an", "develop").trim());
        assertFalse("the host checkout, on main, is untouched", Files.exists(root.resolve("Mine.java")));
        assertEquals(mainHead, GitFixture.head(root));
        assertEquals("", GitFixture.git(root, "status", "--porcelain"));
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.head(ada.path));
        assertNotEquals(mine, GitFixture.head(ada.path));
    }

    @Test
    public void pushIsRefusedWhenTheHostHasUncommittedChangesInAFileItTouches() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("README"), "ada's\n".getBytes(StandardCharsets.UTF_8));
        String mine = worktrees.commit("ada", "mine").commit;
        String develop = GitFixture.commitOf(root, "develop");
        Files.write(root.resolve("README"), "the coach's unsaved work\n".getBytes(StandardCharsets.UTF_8));

        Worktrees.Merge refused = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.REFUSED, refused.outcome);
        assertTrue(refused.detail, refused.detail.contains("README"));
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
        assertEquals("the coach's unsaved work\n", read(root.resolve("README")));
        assertEquals(" M README\n", GitFixture.git(root, "status", "--porcelain"));
        assertFalse(Files.exists(root.resolve(".git/MERGE_HEAD")));
        assertEquals(mine, GitFixture.head(ada.path));
    }

    @Test
    public void aPushThatConflictsChangesNothingAndNamesTheFiles() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Files.write(ada.path.resolve("README"), "ada's line\n".getBytes(StandardCharsets.UTF_8));
        String mine = worktrees.commit("ada", "mine").commit;
        commitOnDevelop("README", "develop's line\n");
        String develop = GitFixture.commitOf(root, "develop");

        Worktrees.Merge conflicted = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.CONFLICTS, conflicted.outcome);
        assertEquals("[README]", conflicted.files.toString());
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
        assertEquals("develop's line\n", read(root.resolve("README")));
        assertEquals("", GitFixture.git(root, "status", "--porcelain"));
        assertEquals(mine, GitFixture.head(ada.path));
        assertEquals("", GitFixture.git(ada.path, "status", "--porcelain"));
        assertTrue(
                "a conflict is worth pressing Push for: pressing it is how the user is sent to their coach",
                worktrees.status("ada").pushable());
    }

    @Test
    public void pushWithNothingNewOrWithUncommittedChangesDoesNothing() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        String develop = GitFixture.commitOf(root, "develop");

        assertEquals(Worktrees.Outcome.NOTHING, worktrees.push("ada").outcome);

        Files.write(ada.path.resolve("Mine.java"), "class Mine {}\n".getBytes(StandardCharsets.UTF_8));
        Worktrees.Merge refused = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.UNCOMMITTED, refused.outcome);
        assertEquals("[Mine.java]", refused.files.toString());
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
    }

    private void pushDoesWhatTheStatusOffered(Worktrees worktrees, String username, Worktrees.Outcome expected)
            throws IOException {
        boolean offered = worktrees.status(username).pushable();
        String develop = GitFixture.commitOf(root, "develop");

        Worktrees.Merge merge = worktrees.push(username);

        assertEquals(expected, merge.outcome);
        boolean worthPressing =
                merge.outcome != Worktrees.Outcome.NOTHING && merge.outcome != Worktrees.Outcome.UNCOMMITTED;
        assertEquals("pushable said " + offered + " and the push was " + merge.outcome, worthPressing, offered);
        if (!worthPressing) {
            assertEquals("a push that was only refused moved develop", develop, GitFixture.commitOf(root, "develop"));
        }
    }

    @Test
    public void pushableIsTrueExactlyWhenPressingPushWouldNotJustBeRefused() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");

        pushDoesWhatTheStatusOffered(worktrees, "ada", Worktrees.Outcome.NOTHING);

        Files.write(ada.path.resolve("Mine.java"), "class Mine {}\n".getBytes(StandardCharsets.UTF_8));
        pushDoesWhatTheStatusOffered(worktrees, "ada", Worktrees.Outcome.UNCOMMITTED);

        worktrees.commit("ada", "mine");
        Files.write(ada.path.resolve("Other.java"), "class Other {}\n".getBytes(StandardCharsets.UTF_8));

        pushDoesWhatTheStatusOffered(worktrees, "ada", Worktrees.Outcome.UNCOMMITTED);

        worktrees.commit("ada", "other");
        pushDoesWhatTheStatusOffered(worktrees, "ada", Worktrees.Outcome.MERGED);
        pushDoesWhatTheStatusOffered(worktrees, "ada", Worktrees.Outcome.NOTHING);
    }

    @Test
    public void twoUsersPushInTurnAndTheSecondCarriesTheFirstsWork() throws IOException {
        Worktrees worktrees = worktrees();
        Worktrees.Worktree ada = worktrees.ensure("ada");
        Worktrees.Worktree bob = worktrees.ensure("bob");
        Files.write(ada.path.resolve("A.java"), "class A {}\n".getBytes(StandardCharsets.UTF_8));
        worktrees.commit("ada", "a");
        Files.write(bob.path.resolve("B.java"), "class B {}\n".getBytes(StandardCharsets.UTF_8));
        worktrees.commit("bob", "b");

        assertEquals(Worktrees.Outcome.MERGED, worktrees.push("ada").outcome);
        assertEquals(Worktrees.Outcome.MERGED, worktrees.push("bob").outcome);

        assertEquals("class A {}\n", GitFixture.git(root, "show", "develop:A.java"));
        assertEquals("class B {}\n", GitFixture.git(root, "show", "develop:B.java"));
        assertTrue("bob's worktree carries ada's work", Files.exists(bob.path.resolve("A.java")));
        assertFalse("ada's worktree waits for a pull", Files.exists(ada.path.resolve("B.java")));
        assertEquals("bob's commit and the merge that landed it", 2, worktrees.status("ada").behind);
        assertEquals(0, worktrees.status("bob").behind);
    }

    private Path origin() throws IOException {
        Path bare = folder.getRoot().toPath().resolve("origin.git");
        GitFixture.withOrigin(root, bare);
        return bare;
    }

    private Worktrees.Worktree committedWorker(Worktrees worktrees, String username, String file) throws IOException {
        Worktrees.Worktree worktree = worktrees.ensure(username);
        Files.write(
                worktree.path.resolve(file),
                ("class " + file.replace(".java", "") + " {}\n").getBytes(StandardCharsets.UTF_8));
        worktrees.commit(username, file);
        return worktree;
    }

    @Test
    public void pushAlsoPushesDevelopToOriginAndSaysSo() throws IOException {
        Path origin = origin();
        Worktrees worktrees = worktrees();
        committedWorker(worktrees, "ada", "Mine.java");

        Worktrees.Merge pushed = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.MERGED, pushed.outcome);
        assertEquals("origin", pushed.remote.name);
        assertEquals(Worktrees.Remote.Outcome.PUSHED, pushed.remote.outcome);
        assertNull(pushed.remote.detail);
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.commitOf(origin, "develop"));
        assertEquals(
                "only develop went to origin",
                "refs/heads/develop\n",
                GitFixture.git(origin, "for-each-ref", "--format=%(refname)"));

        Worktrees.Merge again = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.NOTHING, again.outcome);
        assertEquals(Worktrees.Remote.Outcome.UP_TO_DATE, again.remote.outcome);
    }

    @Test
    public void nothingToMergeStillPushesWhatDevelopHasThatOriginLacks() throws IOException {
        Path origin = origin();
        Worktrees worktrees = worktrees();
        worktrees.ensure("ada");
        commitOnDevelop("README", "the coach committed on develop without pushing\n");
        assertNotEquals(GitFixture.commitOf(root, "develop"), GitFixture.commitOf(origin, "develop"));

        Worktrees.Merge pushed = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.NOTHING, pushed.outcome);
        assertEquals(Worktrees.Remote.Outcome.PUSHED, pushed.remote.outcome);
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.commitOf(origin, "develop"));
    }

    @Test
    public void pushWithoutARemoteLandsOnDevelopAndSaysThereWasNowhereElse() throws IOException {
        Worktrees worktrees = worktrees();
        committedWorker(worktrees, "ada", "Mine.java");

        Worktrees.Merge pushed = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.MERGED, pushed.outcome);
        assertNull(pushed.remote);
        assertEquals(2, parentsOf(root, "develop"));
    }

    @Test
    public void whenOriginCannotBeReachedThePushStillLandsOnDevelopAndTheProblemIsReported() throws IOException {
        GitFixture.git(
                root,
                "remote",
                "add",
                "origin",
                folder.getRoot().toPath().resolve("no-such-origin.git").toString());
        Worktrees worktrees = worktrees();
        committedWorker(worktrees, "ada", "Mine.java");
        String oldDevelop = GitFixture.commitOf(root, "develop");

        Worktrees.Merge pushed = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.MERGED, pushed.outcome);
        assertNotEquals(oldDevelop, GitFixture.commitOf(root, "develop"));
        assertEquals("class Mine {}\n", read(root.resolve("Mine.java")));
        assertEquals("origin", pushed.remote.name);
        assertEquals(Worktrees.Remote.Outcome.FAILED, pushed.remote.outcome);
        assertTrue(pushed.remote.detail, pushed.remote.detail.contains("no-such-origin"));
    }

    @Test
    public void whenOriginHasMovedOnThePushIsNotForcedAndSaysSo() throws IOException {
        Path origin = origin();
        Path elsewhere = folder.getRoot().toPath().resolve("elsewhere");
        GitFixture.git(
                folder.getRoot().toPath(), "clone", "-q", "-b", "develop", origin.toString(), elsewhere.toString());
        Files.write(elsewhere.resolve("Theirs.java"), "class Theirs {}\n".getBytes(StandardCharsets.UTF_8));
        GitFixture.commitAll(elsewhere, "pushed from another machine");
        GitFixture.git(elsewhere, "push", "-q", "origin", "develop");
        String theirs = GitFixture.commitOf(origin, "develop");
        Worktrees worktrees = worktrees();
        committedWorker(worktrees, "ada", "Mine.java");

        Worktrees.Merge pushed = worktrees.push("ada");

        assertEquals(Worktrees.Outcome.MERGED, pushed.outcome);
        assertEquals(Worktrees.Remote.Outcome.FAILED, pushed.remote.outcome);
        assertTrue(
                pushed.remote.detail,
                pushed.remote.detail.contains("rejected") || pushed.remote.detail.contains("fetch first"));
        assertEquals("origin was not forced", theirs, GitFixture.commitOf(origin, "develop"));
        assertEquals("class Mine {}\n", GitFixture.git(root, "show", "develop:Mine.java"));
    }

    @Test
    public void aRootThatIsNotARepositoryStopsStartupNamingIt() throws IOException {
        Path plain = folder.newFolder("plain").toPath();

        try {
            new Worktrees(plain, stateDir, "git");
            fail("no repository, no worktrees");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains(plain.toString()));
        }
    }

    @Test
    public void aSubdirectoryOfARepositoryIsNotARootEither() throws IOException {
        Path sub = Files.createDirectories(root.resolve("TeamCode"));

        try {
            new Worktrees(sub, stateDir, "git");
            fail("the root must be the top of the working tree");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains(sub.toString()));
            assertTrue(e.getMessage(), e.getMessage().contains(root.toString()));
        }
    }

    @Test
    public void aRepositoryWithoutDevelopStopsStartupSayingHowToMakeIt() throws IOException {
        GitFixture.git(root, "checkout", "-q", "-b", "main");
        GitFixture.git(root, "branch", "-D", "develop");

        try {
            worktrees();
            fail("no develop branch, nowhere to start from");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("develop"));
            assertTrue(e.getMessage(), e.getMessage().contains("git branch develop"));
        }
    }

    @Test
    public void aMissingGitStopsStartupNamingIt() {
        String missing = folder.getRoot().toPath().resolve("no-such-git").toString();

        try {
            new Worktrees(root, stateDir, missing);
            fail("no git, no worktrees");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains(missing));
        }
    }

    @Test
    public void aGitOlderThanWhatMergeTreeNeedsStopsStartupNamingBothVersions() throws IOException {
        assumeTrue(FileSystems.getDefault().supportedFileAttributeViews().contains("posix"));
        Path stub = folder.getRoot().toPath().resolve("old-git");
        Files.write(stub, "#!/bin/sh\necho 'git version 2.30.0'\n".getBytes(StandardCharsets.UTF_8));
        Files.setPosixFilePermissions(stub, PosixFilePermissions.fromString("rwx------"));

        try {
            new Worktrees(root, stateDir, stub.toString());
            fail("merge-tree --write-tree needs 2.38");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("2.38"));
            assertTrue(e.getMessage(), e.getMessage().contains("2.30.0"));
        }
    }
}
