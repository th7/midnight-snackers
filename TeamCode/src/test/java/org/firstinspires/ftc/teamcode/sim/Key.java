package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.util.Optional;

/**
 * A file the server has vouched for, named the way users name one: a <b>root-relative path</b> with
 * {@code /} separators, like {@code TeamCode/src/main/java/.../Plans.java}. The key means the same
 * path in every worktree, which is what lets the admin pick from the host checkout and every user
 * edit their own copy.
 *
 * <p>It exists to be a type a request string cannot become. Resolving a key against a worktree is
 * {@link #under}, and nothing else in the server resolves a path for a file a user named; the only
 * ways to make a key are {@link #of}, which relativises a real path the server itself found, and
 * {@link #under(Path, String)}, which refuses anything absolute or climbing out of the root. So
 * <em>"nothing a user sends is resolved against the filesystem"</em> stops being a sentence in a
 * document that one {@code if} happens to keep true, and becomes something javac holds: a user's
 * string reaches a key only by exact match against a set the server built.
 */
public final class Key implements Comparable<Key> {
    private final String path;

    private Key(String path) {
        this.path = path;
    }

    /** The key for a real path the server found itself: the admin's pick, or an enumerated source. */
    static Key of(Path root, Path file) {
        return new Key(root.relativize(file).toString().replace('\\', '/'));
    }

    /**
     * The key for a root-relative path that did not come from a request: one the server wrote to
     * its own state file, or one git named in a diff. Empty when it is absolute, names a drive, or
     * climbs out of the root -- checked here rather than trusted, because a state file an editor
     * has been at is no more trustworthy than anything else off disk.
     */
    static Optional<Key> under(Path root, String relative) {
        if (relative == null
                || relative.isEmpty()
                || relative.startsWith("/")
                || relative.startsWith("\\")
                || relative.contains(":")) {
            return Optional.empty();
        }
        Path resolved = root.resolve(relative).normalize();
        if (!resolved.startsWith(root) || resolved.equals(root)) {
            return Optional.empty();
        }
        return Optional.of(of(root, resolved));
    }

    /** Where this file is inside {@code worktree}: the one place a key becomes a path. */
    Path under(Path worktree) {
        return worktree.resolve(path);
    }

    /** The key as users and the pages spell it. */
    public String path() {
        return path;
    }

    public boolean isJava() {
        return path.endsWith(".java");
    }

    @Override
    public int compareTo(Key other) {
        return path.compareTo(other.path);
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Key && ((Key) other).path.equals(path);
    }

    @Override
    public int hashCode() {
        return path.hashCode();
    }

    @Override
    public String toString() {
        return path;
    }
}
