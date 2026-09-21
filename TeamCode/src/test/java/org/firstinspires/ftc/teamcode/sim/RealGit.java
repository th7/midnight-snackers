package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

public final class RealGit implements Git {
    private static final long SECONDS = 20;
    private static final long REMOTE_SECONDS = 30;

    private final String executable;
    private final Path root;

    public RealGit(String executable, Path root) {
        this.executable = executable;
        this.root = root;
    }

    private static final class Ran {
        final int exit;
        final String out;
        final String err;

        Ran(int exit, String out, String err) {
            this.exit = exit;
            this.out = out;
            this.err = err;
        }

        String said() {
            return (err + out).trim();
        }
    }

    private Ran must(Path cwd, String... args) {
        Ran ran = ran(cwd, SECONDS, args);
        if (ran.exit != 0) {
            throw new Git.Failed("git " + String.join(" ", args) + " failed in " + cwd + ": " + ran.said());
        }
        return ran;
    }

    private Ran ran(Path cwd, long timeoutSeconds, String... args) {
        try (Cost.Spent spent = Cost.start(Cost.Kind.GIT)) {
            return running(cwd, timeoutSeconds, args);
        }
    }

    private Ran running(Path cwd, long timeoutSeconds, String... args) {
        List<String> command = new ArrayList<>();
        command.add(executable);
        command.addAll(List.of(args));
        ProcessBuilder builder = new ProcessBuilder(command);
        if (cwd != null) {
            builder.directory(cwd.toFile());
        }
        builder.environment().put("GIT_TERMINAL_PROMPT", "0");
        Process process;
        try {
            process = builder.start();
        } catch (IOException e) {
            throw new Git.Failed("could not start " + executable + ": " + e.getMessage());
        }
        StringBuilder err = new StringBuilder();
        Thread stderr = new Thread(
                () -> {
                    try {
                        err.append(new String(process.getErrorStream().readAllBytes(), StandardCharsets.UTF_8));
                    } catch (IOException ignored) {
                    }
                },
                "git-stderr");
        stderr.start();
        try {
            String out = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            if (!process.waitFor(timeoutSeconds, TimeUnit.SECONDS)) {
                process.destroyForcibly();
                throw new Git.Failed(
                        "git " + String.join(" ", args) + " did not finish within " + timeoutSeconds + " seconds");
            }
            stderr.join();
            return new Ran(process.exitValue(), out, err.toString());
        } catch (IOException e) {
            process.destroyForcibly();
            throw new Git.Failed("git " + String.join(" ", args) + " could not be read: " + e.getMessage());
        } catch (InterruptedException e) {
            process.destroyForcibly();
            Thread.currentThread().interrupt();
            throw new Git.Failed("git " + String.join(" ", args) + " was interrupted");
        }
    }

    private String[] as(Author author, String... rest) {
        List<String> args =
                new ArrayList<>(List.of("-c", "user.name=" + author.name, "-c", "user.email=" + author.email));
        args.addAll(List.of(rest));
        return args.toArray(new String[0]);
    }

    private static List<String> nulSeparated(String out) {
        List<String> names = new ArrayList<>();
        for (String name : out.split("\0")) {
            if (!name.isEmpty()) {
                names.add(name);
            }
        }
        return names;
    }

    @Override
    public String version() {
        return ran(null, SECONDS, "--version").out.trim();
    }

    @Override
    public Optional<Path> topLevel() {
        Ran ran = ran(root, SECONDS, "rev-parse", "--show-toplevel");
        return ran.exit == 0 ? Optional.of(Path.of(ran.out.trim())) : Optional.empty();
    }

    @Override
    public Revision commitAt(Revision revision) {
        return Revision.of(must(root, "rev-parse", revision.text()).out.trim());
    }

    @Override
    public Optional<Revision> commitIfThere(Revision revision) {
        Ran ran = ran(root, SECONDS, "rev-parse", "--verify", "--quiet", revision.text());
        return ran.exit == 0 ? Optional.of(Revision.of(ran.out.trim())) : Optional.empty();
    }

    @Override
    public boolean isAncestor(Revision ancestor, Revision descendant) {
        Ran ran = ran(root, SECONDS, "merge-base", "--is-ancestor", ancestor.text(), descendant.text());
        if (ran.exit > 1) {
            throw new Git.Failed(
                    "git merge-base --is-ancestor " + ancestor + " " + descendant + " failed: " + ran.said());
        }
        return ran.exit == 0;
    }

    @Override
    public int commitsBetween(Revision from, Revision to) {
        return Integer.parseInt(must(root, "rev-list", "--count", from.text() + ".." + to.text())
                .out
                .trim());
    }

    @Override
    public Outcome moveBranch(Branch branch, Revision to, Revision from) {
        Ran ran = ran(root, SECONDS, "update-ref", branch.tip().text(), to.text(), from.text());
        return ran.exit == 0 ? Outcome.done() : Outcome.refused(ran.said());
    }

    @Override
    public boolean stillAWorktree(Path at) {
        // git's own marker for one, which is what it leaves in a worktree directory and nowhere
        // else. Reading it is this adapter's business, which is why the question lives here.
        return Files.isDirectory(at) && Files.exists(at.resolve(".git"));
    }

    @Override
    public void pruneWorktrees() {
        must(root, "worktree", "prune");
    }

    @Override
    public void addWorktree(Path at, Branch existing) {
        must(root, "worktree", "add", "-q", at.toString(), existing.name());
    }

    @Override
    public void createWorktree(Path at, Branch created, Branch from) {
        must(root, "worktree", "add", "-q", "-b", created.name(), at.toString(), from.name());
    }

    @Override
    public void removeWorktree(Path at) {
        must(root, "worktree", "remove", "--force", at.toString());
    }

    @Override
    public void deleteBranch(Branch branch) {
        must(root, "branch", "-D", branch.name());
    }

    @Override
    public List<Checkout> checkouts() {
        List<Checkout> found = new ArrayList<>();
        Path at = null;
        for (String line : must(root, "worktree", "list", "--porcelain").out.split("\n")) {
            if (line.startsWith("worktree ")) {
                at = Path.of(line.substring("worktree ".length()));
            } else if (line.startsWith("branch refs/heads/") && at != null) {
                found.add(new Checkout(at, Branch.of(line.substring("branch refs/heads/".length()))));
                at = null;
            }
        }
        return found;
    }

    @Override
    public void stageEverything(Path worktree) {
        must(worktree, "add", "-A");
    }

    @Override
    public List<String> stagedFiles(Path worktree) {
        return nulSeparated(must(worktree, "diff", "--cached", "--name-only", "-z").out);
    }

    @Override
    public void commitStaged(Path worktree, Author author, String message) {
        must(worktree, as(author, "commit", "-q", "-m", message));
    }

    @Override
    public List<String> uncommittedFiles(Path worktree) {
        String[] entries = must(worktree, "status", "--porcelain", "-z", "--untracked-files=all")
                .out
                .split("\0");
        List<String> files = new ArrayList<>();
        for (int i = 0; i < entries.length; i++) {
            if (entries[i].length() < 4) {
                continue;
            }
            files.add(entries[i].substring(3));
            char x = entries[i].charAt(0);
            if (x == 'R' || x == 'C') {
                i++;
            }
        }
        files.sort(null);
        return files;
    }

    @Override
    public Revision head(Path worktree) {
        return Revision.of(must(worktree, "rev-parse", "HEAD").out.trim());
    }

    @Override
    public List<String> filesChangedBetween(Revision from, Revision to) {
        return nulSeparated(must(root, "diff", "--name-only", "-z", from.text(), to.text()).out);
    }

    @Override
    public MergeTree mergeTree(Revision ours, Revision theirs) {
        Ran ran = ran(root, SECONDS, "merge-tree", "--write-tree", "--name-only", ours.text(), theirs.text());
        if (ran.exit > 1) {
            throw new Git.Failed("git merge-tree " + ours + " " + theirs + " failed: " + ran.said());
        }
        String[] lines = ran.out.split("\n");
        List<String> conflicts = new ArrayList<>();
        if (ran.exit == 1) {
            for (int i = 1; i < lines.length && !lines[i].isEmpty(); i++) {
                if (!conflicts.contains(lines[i])) {
                    conflicts.add(lines[i]);
                }
            }
            conflicts.sort(null);
        }
        return new MergeTree(Revision.of(lines[0].trim()), conflicts);
    }

    @Override
    public Outcome merge(Path worktree, Author author, String message, Branch from, History history) {
        List<String> args = new ArrayList<>(List.of("merge"));
        if (history == History.ALWAYS_A_MERGE_COMMIT) {
            args.add("--no-ff");
        }
        args.addAll(List.of("-q", "-m", message, from.name()));
        Ran ran = ran(worktree, SECONDS, as(author, args.toArray(new String[0])));
        return ran.exit == 0 ? Outcome.done() : Outcome.refused(ran.said());
    }

    @Override
    public void abortMerge(Path worktree) {
        ran(worktree, SECONDS, "merge", "--abort");
    }

    @Override
    public Outcome fastForwardOnly(Path worktree, Branch to) {
        Ran ran = ran(worktree, SECONDS, "merge", "--ff-only", "-q", to.name());
        return ran.exit == 0 ? Outcome.done() : Outcome.refused(ran.said());
    }

    @Override
    public Revision commitTree(Author author, Revision tree, String message, Revision first, Revision second) {
        return Revision.of(must(
                        root,
                        as(author, "commit-tree", tree.text(), "-p", first.text(), "-p", second.text(), "-m", message))
                .out
                .trim());
    }

    @Override
    public boolean hasRemote(String remote) {
        return must(root, "remote").out.lines().anyMatch(remote::equals);
    }

    @Override
    public Optional<Revision> remoteCommit(String remote, Branch branch) {
        return commitIfThere(Revision.of("refs/remotes/" + remote + "/" + branch.name()));
    }

    @Override
    public Outcome push(String remote, Branch branch) {
        Ran ran;
        try {
            ran = ran(
                    root,
                    REMOTE_SECONDS,
                    "push",
                    "--quiet",
                    remote,
                    branch.tip().text() + ":" + branch.tip().text());
        } catch (Git.Failed e) {
            return Outcome.refused(e.getMessage());
        }
        return ran.exit == 0 ? Outcome.done() : Outcome.refused(ran.said());
    }
}
