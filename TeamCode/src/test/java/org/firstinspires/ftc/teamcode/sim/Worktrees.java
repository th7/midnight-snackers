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
