package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * One git worktree per username, on its own branch off {@code develop}, kept under the coding
 * server's state directory so nothing is written under the project root except what git itself
 * records under {@code .git}. Owned by the username, not the session, so logging in again finds
 * the same work.
 * <p>
 * Every git call goes through here. The constructor checks what the server needs before it
 * starts: a git of at least {@link #MIN_GIT_MAJOR}.{@link #MIN_GIT_MINOR}, a root that is the top
 * of a working tree, and a {@code develop} branch; each failure is an {@link IllegalStateException}
 * naming what is missing rather than a server that starts and finds out later.
 */
public final class Worktrees {
    public static final String DEVELOP = "develop";
    public static final String BRANCH_PREFIX = "coding/";
    public static final String STORE_FILE = "worktrees.json";
    /** {@code git merge-tree --write-tree}, which pushes and pulls use to find conflicts without touching a working tree. */
    public static final int MIN_GIT_MAJOR = 2;
    public static final int MIN_GIT_MINOR = 38;
    private static final int MAX_SLUG_LENGTH = 32;
    private static final long GIT_TIMEOUT_SECONDS = 120;
    private static final Pattern VERSION = Pattern.compile("git version (\\d+)\\.(\\d+)");

    /** A user's worktree: where it is and which branch it is on. */
    public static final class Worktree {
        public final String username;
        public final String slug;
        public final Path path;
        public final String branch;

        Worktree(String username, String slug, Path path, String branch) {
            this.username = username;
            this.slug = slug;
            this.path = path;
            this.branch = branch;
        }

        JsonObject toJson() {
            JsonObject item = new JsonObject();
            item.addProperty("slug", slug);
            item.addProperty("path", path.toString());
            item.addProperty("branch", branch);
            return item;
        }

        static Worktree fromJson(String username, JsonObject item) {
            return new Worktree(username, item.get("slug").getAsString(), Path.of(item.get("path").getAsString()),
                    item.get("branch").getAsString());
        }
    }

    /** A user's worktree against its branch and against {@code develop}. */
    public static final class Status {
        public final String branch;
        /** Root-relative paths with '/' separators, sorted: what a commit would take. */
        public final List<String> changed;
        /** Commits on the user branch that {@code develop} lacks. */
        public final int ahead;
        /** Commits on {@code develop} that the user branch lacks: what a pull would bring. */
        public final int behind;

        Status(String branch, List<String> changed, int ahead, int behind) {
            this.branch = branch;
            this.changed = changed;
            this.ahead = ahead;
            this.behind = behind;
        }
    }

    /** What a commit did: nothing when there was nothing to commit. */
    public static final class Commit {
        public final boolean made;
        /** The branch's tip afterwards, made or not. */
        public final String commit;
        public final List<String> files;

        Commit(boolean made, String commit, List<String> files) {
            this.made = made;
            this.commit = commit;
            this.files = files;
        }
    }

    /** How a pull or push ended. */
    public enum Outcome {
        /** The merge was made (a fast-forward counts). */
        MERGED,
        /** There was nothing to merge. */
        NOTHING,
        /** The worktree has uncommitted changes; {@code files} names them. Commit first. */
        UNCOMMITTED,
        /** The two branches conflict; {@code files} names where. Nothing was changed. */
        CONFLICTS,
        /** Git refused to apply a merge known to be clean; {@code detail} is what it said. Nothing was changed. */
        REFUSED
    }

    /** What a pull or push did. */
    public static final class Merge {
        public final Outcome outcome;
        /** Root-relative paths, sorted: the uncommitted files, or the conflicting ones. */
        public final List<String> files;
        /** What git said, when it refused. */
        public final String detail;

        Merge(Outcome outcome, List<String> files, String detail) {
            this.outcome = outcome;
            this.files = files;
            this.detail = detail;
        }
    }

    /** A git command that did not succeed; the message carries the command and what git said. */
    public static final class GitFailed extends RuntimeException {
        GitFailed(String message) {
            super(message);
        }
    }

    /** What a git command produced. */
    private static final class Result {
        final int exit;
        final String out;
        final String err;

        Result(int exit, String out, String err) {
            this.exit = exit;
            this.out = out;
            this.err = err;
        }
    }

    private final Path root;
    private final Path stateDir;
    private final String git;
    private final Path directory;
    /** By username, in creation order. */
    private final Map<String, Worktree> byUsername = new LinkedHashMap<>();

    /**
     * @param root     the project checkout; must be the top of its working tree
     * @param stateDir the coding server's state directory; the worktrees and their store go under it
     * @param git      the git executable, normally {@code "git"}
     * @throws IllegalStateException when git is missing or too old, the root is not the top of a
     *                               working tree, there is no {@code develop} branch, or the store
     *                               exists but cannot be read
     */
    public Worktrees(Path root, Path stateDir, String git) {
        this.root = root.toAbsolutePath().normalize();
        this.stateDir = stateDir.toAbsolutePath().normalize();
        this.git = git;
        checkVersion();
        checkRoot();
        checkDevelop();
        String name = this.root.getFileName() == null ? "root" : this.root.getFileName().toString();
        this.directory = this.stateDir.resolve("worktrees").resolve(name + "-" + shortHash(this.root.toString()));
        load();
    }

    /** Where this root's worktrees live: one subdirectory per slug. */
    public Path directory() {
        return directory;
    }

    public Path root() {
        return root;
    }

    /** The worktree already made for a username, or null. */
    public synchronized Worktree find(String username) {
        return byUsername.get(username);
    }

    /**
     * The username's worktree, made on the first call ({@code git worktree add -b coding/<slug>
     * <path> develop}) and recreated on its branch if its directory has gone missing since.
     *
     * @throws GitFailed when git refuses
     */
    public synchronized Worktree ensure(String username) {
        Worktree existing = byUsername.get(username);
        if (existing != null) {
            if (Files.isDirectory(existing.path) && Files.exists(existing.path.resolve(".git"))) {
                return existing;
            }
            git(root, "worktree", "prune");
            directoryReady();
            git(root, "worktree", "add", "-q", existing.path.toString(), existing.branch);
            System.out.println("recreated the worktree for " + username + " at " + existing.path + " on " + existing.branch);
            return existing;
        }
        String base = slugOf(username);
        String slug = base;
        for (int n = 2; taken(slug); n++) {
            slug = base + "-" + n;
        }
        Path path = directory.resolve(slug);
        String branch = BRANCH_PREFIX + slug;
        directoryReady();
        git(root, "worktree", "add", "-q", "-b", branch, path.toString(), DEVELOP);
        Worktree made = new Worktree(username, slug, path, branch);
        byUsername.put(username, made);
        try {
            save();
        } catch (RuntimeException e) {
            byUsername.remove(username);
            git(root, "worktree", "remove", "--force", path.toString());
            git(root, "branch", "-D", branch);
            throw e;
        }
        return made;
    }

    // --- status and commit ---

    /** The user's uncommitted changes and how their branch stands against {@code develop}. */
    public synchronized Status status(String username) {
        Worktree worktree = ensure(username);
        return new Status(worktree.branch, changedFiles(worktree),
                count(DEVELOP + ".." + worktree.branch), count(worktree.branch + ".." + DEVELOP));
    }

    /**
     * Every uncommitted change in the user's worktree, new files included, as one commit on the
     * user branch authored by the username.
     *
     * @return what was committed, or a commit that was not made when there was nothing to commit
     */
    public synchronized Commit commit(String username, String message) {
        Worktree worktree = ensure(username);
        git(worktree.path, "add", "-A");
        List<String> files = nulSeparated(git(worktree.path, "diff", "--cached", "--name-only", "-z").out);
        if (files.isEmpty()) {
            return new Commit(false, head(worktree), files);
        }
        git(worktree.path, "-c", "user.name=" + username, "-c", "user.email=" + worktree.slug + "@coding-server.invalid",
                "commit", "-q", "-m", message);
        return new Commit(true, head(worktree), files);
    }

    // --- pull: develop into the user's branch ---

    /**
     * Merges {@code develop} into the user branch, in the user's worktree: a fast-forward when
     * the branch has no commits of its own, a merge commit otherwise. Conflicts are found first
     * with {@code merge-tree}, which touches no working tree, so a conflicting pull changes
     * nothing at all.
     */
    public synchronized Merge pull(String username) {
        Worktree worktree = ensure(username);
        List<String> changed = changedFiles(worktree);
        if (!changed.isEmpty()) {
            return new Merge(Outcome.UNCOMMITTED, changed, null);
        }
        if (isAncestor(DEVELOP, worktree.branch)) {
            return new Merge(Outcome.NOTHING, List.of(), null);
        }
        MergeTree tree = mergeTree(worktree.branch, DEVELOP);
        if (!tree.conflicts.isEmpty()) {
            return new Merge(Outcome.CONFLICTS, tree.conflicts, null);
        }
        Result merged = run(worktree.path, "-c", "user.name=" + username, "-c", "user.email=" + email(worktree),
                "merge", "-q", "-m", "Pull " + DEVELOP, DEVELOP);
        if (merged.exit != 0) {
            // known clean, so this is a refusal before anything was written; make sure of it
            run(worktree.path, "merge", "--abort");
            return new Merge(Outcome.REFUSED, List.of(), (merged.err + merged.out).trim());
        }
        return new Merge(Outcome.MERGED, List.of(), null);
    }

    // --- push: the user's branch into develop ---

    /**
     * Merges the user branch into {@code develop} with a merge commit, then fast-forwards the
     * user branch and worktree to the new {@code develop}. Conflicts are found first with
     * {@code merge-tree}, so a conflicting push changes nothing at all. Where {@code develop}
     * is checked out (normally the host checkout) the merge runs there, so that working tree
     * shows the pushed work; git refuses, changing nothing, if uncommitted changes there would
     * be overwritten. Checked out nowhere, only the branch moves.
     *
     * @return {@link Outcome#MERGED} with a {@code detail} when the push landed but the
     *         worktree could not be fast-forwarded, which a commit and a pull will heal
     */
    public synchronized Merge push(String username) {
        Worktree worktree = ensure(username);
        List<String> changed = changedFiles(worktree);
        if (!changed.isEmpty()) {
            return new Merge(Outcome.UNCOMMITTED, changed, null);
        }
        if (isAncestor(worktree.branch, DEVELOP)) {
            return new Merge(Outcome.NOTHING, List.of(), null);
        }
        MergeTree tree = mergeTree(DEVELOP, worktree.branch);
        if (!tree.conflicts.isEmpty()) {
            return new Merge(Outcome.CONFLICTS, tree.conflicts, null);
        }
        String message = "Push " + username + "'s work";
        Path checkedOut = checkedOutAt(DEVELOP);
        if (checkedOut != null) {
            Result merged = run(checkedOut, "-c", "user.name=" + username, "-c", "user.email=" + email(worktree),
                    "merge", "--no-ff", "-q", "-m", message, worktree.branch);
            if (merged.exit != 0) {
                run(checkedOut, "merge", "--abort");
                return new Merge(Outcome.REFUSED, List.of(), (merged.err + merged.out).trim());
            }
        } else {
            String old = git(root, "rev-parse", "refs/heads/" + DEVELOP).out.trim();
            String tip = git(root, "-c", "user.name=" + username, "-c", "user.email=" + email(worktree),
                    "commit-tree", tree.tree, "-p", old, "-p", worktree.branch, "-m", message).out.trim();
            Result moved = run(root, "update-ref", "refs/heads/" + DEVELOP, tip, old);
            if (moved.exit != 0) {
                return new Merge(Outcome.REFUSED, List.of(), (moved.err + moved.out).trim());
            }
        }
        Result caughtUp = run(worktree.path, "merge", "--ff-only", "-q", DEVELOP);
        if (caughtUp.exit != 0) {
            return new Merge(Outcome.MERGED, List.of(), (caughtUp.err + caughtUp.out).trim());
        }
        return new Merge(Outcome.MERGED, List.of(), null);
    }

    /** The worktree where a branch is checked out, or null when it is checked out nowhere. */
    private Path checkedOutAt(String branch) {
        String[] lines = git(root, "worktree", "list", "--porcelain").out.split("\n");
        Path current = null;
        for (String line : lines) {
            if (line.startsWith("worktree ")) {
                current = Path.of(line.substring("worktree ".length()));
            } else if (line.equals("branch refs/heads/" + branch)) {
                return current;
            }
        }
        return null;
    }

    private static String email(Worktree worktree) {
        return worktree.slug + "@coding-server.invalid";
    }

    private boolean isAncestor(String maybeAncestor, String of) {
        Result result = run(root, "merge-base", "--is-ancestor", maybeAncestor, of);
        if (result.exit > 1) {
            throw new GitFailed("git merge-base --is-ancestor " + maybeAncestor + " " + of + " failed: " + result.err.trim());
        }
        return result.exit == 0;
    }

    /** What {@code merge-tree} made of two branches: the merged tree, and where it conflicts. */
    private static final class MergeTree {
        final String tree;
        final List<String> conflicts;

        MergeTree(String tree, List<String> conflicts) {
            this.tree = tree;
            this.conflicts = conflicts;
        }
    }

    /** Merges {@code theirs} into {@code ours} without touching any working tree. */
    private MergeTree mergeTree(String ours, String theirs) {
        Result result = run(root, "merge-tree", "--write-tree", "--name-only", ours, theirs);
        if (result.exit > 1) {
            throw new GitFailed("git merge-tree " + ours + " " + theirs + " failed: " + result.err.trim());
        }
        // the tree, then one conflicting file per line, then a blank line and the messages
        String[] lines = result.out.split("\n");
        List<String> files = new ArrayList<>();
        if (result.exit == 1) {
            for (int i = 1; i < lines.length && !lines[i].isEmpty(); i++) {
                if (!files.contains(lines[i])) {
                    files.add(lines[i]);
                }
            }
            files.sort(null);
        }
        return new MergeTree(lines[0].trim(), files);
    }

    private List<String> changedFiles(Worktree worktree) {
        // -z: one NUL after each entry, and a renamed entry is followed by its old path as one more
        String[] entries = git(worktree.path, "status", "--porcelain", "-z", "--untracked-files=all").out.split("\0");
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

    private int count(String range) {
        return Integer.parseInt(git(root, "rev-list", "--count", range).out.trim());
    }

    private String head(Worktree worktree) {
        return git(worktree.path, "rev-parse", "HEAD").out.trim();
    }

    private static List<String> nulSeparated(String out) {
        List<String> items = new ArrayList<>();
        for (String item : out.split("\0")) {
            if (!item.isEmpty()) {
                items.add(item);
            }
        }
        items.sort(null);
        return items;
    }

    /**
     * The username lowercased, runs of anything outside {@code [a-z0-9]} made one {@code -}, no
     * leading or trailing {@code -}, at most {@link #MAX_SLUG_LENGTH} long, never empty: a name
     * that is valid as a branch name and as a directory name.
     */
    static String slugOf(String username) {
        String slug = username.toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9]+", "-").replaceAll("^-+|-+$", "");
        if (slug.length() > MAX_SLUG_LENGTH) {
            slug = slug.substring(0, MAX_SLUG_LENGTH).replaceAll("-+$", "");
        }
        return slug.isEmpty() ? "user" : slug;
    }

    private boolean taken(String slug) {
        for (Worktree worktree : byUsername.values()) {
            if (worktree.slug.equals(slug)) {
                return true;
            }
        }
        return Files.exists(directory.resolve(slug)) || branchExists(BRANCH_PREFIX + slug);
    }

    private boolean branchExists(String branch) {
        return run(root, "rev-parse", "--verify", "--quiet", "refs/heads/" + branch).exit == 0;
    }

    private void directoryReady() {
        try {
            StateStore.ownerOnlyDirectory(directory);
        } catch (IOException e) {
            throw new IllegalStateException("could not make the worktrees directory " + directory + ": " + e.getMessage(), e);
        }
    }

    // --- what must be there before the server starts ---

    private void checkVersion() {
        Result result;
        try {
            result = run(null, "--version");
        } catch (GitFailed e) {
            throw new IllegalStateException("could not run git at " + git + ": " + e.getMessage(), e);
        }
        Matcher matcher = VERSION.matcher(result.out);
        if (result.exit != 0 || !matcher.find()) {
            throw new IllegalStateException(git + " --version did not say which git it is: " + result.out + result.err);
        }
        int major = Integer.parseInt(matcher.group(1));
        int minor = Integer.parseInt(matcher.group(2));
        if (major < MIN_GIT_MAJOR || (major == MIN_GIT_MAJOR && minor < MIN_GIT_MINOR)) {
            throw new IllegalStateException("the coding server needs git " + MIN_GIT_MAJOR + "." + MIN_GIT_MINOR
                    + " or later (for merge-tree --write-tree); " + git + " is " + result.out.trim());
        }
    }

    private void checkRoot() {
        if (!Files.isDirectory(root)) {
            throw new IllegalStateException("not a directory: " + root);
        }
        Result result = run(root, "rev-parse", "--show-toplevel");
        if (result.exit != 0) {
            throw new IllegalStateException("not a git repository: " + root + " (" + result.err.trim() + ")");
        }
        Path top = Path.of(result.out.trim());
        try {
            if (!top.toRealPath().equals(root.toRealPath())) {
                throw new IllegalStateException(root + " is inside the repository at " + top + "; run the coding server from " + top);
            }
        } catch (IOException e) {
            throw new IllegalStateException("could not resolve " + root + ": " + e.getMessage(), e);
        }
    }

    private void checkDevelop() {
        if (!branchExists(DEVELOP)) {
            throw new IllegalStateException("no " + DEVELOP + " branch in " + root + "; create it with: git branch " + DEVELOP);
        }
    }

    // --- the store ---

    private void load() {
        JsonObject stored = StateStore.load(stateDir.resolve(STORE_FILE));
        if (stored == null) {
            return;
        }
        try {
            JsonElement ours = stored.getAsJsonObject("roots").get(root.toString());
            if (ours != null) {
                for (Map.Entry<String, JsonElement> entry : ours.getAsJsonObject().entrySet()) {
                    byUsername.put(entry.getKey(), Worktree.fromJson(entry.getKey(), entry.getValue().getAsJsonObject()));
                }
            }
        } catch (RuntimeException e) {
            throw new IllegalStateException("could not read the worktrees in " + stateDir.resolve(STORE_FILE) + ": " + e, e);
        }
    }

    private void save() {
        Path file = stateDir.resolve(STORE_FILE);
        JsonObject stored = StateStore.load(file);
        JsonObject roots = stored == null || !stored.has("roots") ? new JsonObject() : stored.getAsJsonObject("roots");
        JsonObject ours = new JsonObject();
        for (Worktree worktree : byUsername.values()) {
            ours.add(worktree.username, worktree.toJson());
        }
        roots.add(root.toString(), ours);
        JsonObject body = new JsonObject();
        body.add("roots", roots);
        StateStore.save(file, body);
    }

    private static String shortHash(String text) {
        try {
            byte[] digest = MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8));
            StringBuilder hex = new StringBuilder();
            for (int i = 0; i < 4; i++) {
                hex.append(String.format("%02x", digest[i]));
            }
            return hex.toString();
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        }
    }

    // --- running git ---

    /** Runs git and requires it to succeed. */
    private Result git(Path cwd, String... args) {
        Result result = run(cwd, args);
        if (result.exit != 0) {
            throw new GitFailed("git " + String.join(" ", args) + " failed in " + cwd + ": " + (result.err + result.out).trim());
        }
        return result;
    }

    /** Runs git, in {@code cwd} when given, with a timeout, capturing what it wrote. */
    private Result run(Path cwd, String... args) {
        List<String> command = new ArrayList<>();
        command.add(git);
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
            throw new GitFailed("could not start " + git + ": " + e.getMessage());
        }
        process.getOutputStream();
        StringBuilder err = new StringBuilder();
        Thread stderr = new Thread(() -> {
            try {
                err.append(new String(process.getErrorStream().readAllBytes(), StandardCharsets.UTF_8));
            } catch (IOException ignored) {
                // the process is gone; whatever it said is lost with it
            }
        }, "git-stderr");
        stderr.start();
        try {
            String out = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            if (!process.waitFor(GIT_TIMEOUT_SECONDS, TimeUnit.SECONDS)) {
                process.destroyForcibly();
                throw new GitFailed("git " + String.join(" ", args) + " did not finish within " + GIT_TIMEOUT_SECONDS + " seconds");
            }
            stderr.join();
            return new Result(process.exitValue(), out, err.toString());
        } catch (IOException e) {
            process.destroyForcibly();
            throw new GitFailed("git " + String.join(" ", args) + ": " + e.getMessage());
        } catch (InterruptedException e) {
            process.destroyForcibly();
            Thread.currentThread().interrupt();
            throw new GitFailed("interrupted while running git " + String.join(" ", args));
        }
    }
}
