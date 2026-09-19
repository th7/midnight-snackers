package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.util.Optional;

public final class Key implements Comparable<Key> {
    private final String path;

    private Key(String path) {
        this.path = path;
    }

    static Key of(Path root, Path file) {
        return new Key(root.relativize(file).toString().replace('\\', '/'));
    }

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

    Path under(Path worktree) {
        return worktree.resolve(path);
    }

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
