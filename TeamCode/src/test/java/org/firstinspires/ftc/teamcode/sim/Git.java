package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

public interface Git {

    final class Failed extends RuntimeException {
        public Failed(String message) {
            super(message);
        }
    }

    final class Refused extends RuntimeException {
        public Refused(String message) {
            super(message);
        }
    }

    final class Branch {
        private final String name;

        private Branch(String name) {
            this.name = name;
        }

        public static Branch of(String name) {
            return new Branch(Names.checked(name, "branch"));
        }

        public String name() {
            return name;
        }

        public Revision tip() {
            return Revision.of("refs/heads/" + name);
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Branch && ((Branch) other).name.equals(name);
        }

        @Override
        public int hashCode() {
            return name.hashCode();
        }

        @Override
        public String toString() {
            return name;
        }
    }

    final class Revision {
        private final String text;

        private Revision(String text) {
            this.text = text;
        }

        public static Revision of(String text) {
            return new Revision(Names.checked(text, "revision"));
        }

        public String text() {
            return text;
        }

        @Override
        public boolean equals(Object other) {
            return other instanceof Revision && ((Revision) other).text.equals(text);
        }

        @Override
        public int hashCode() {
            return text.hashCode();
        }

        @Override
        public String toString() {
            return text;
        }
    }

    final class Names {
        private Names() {}

        static String checked(String text, String kind) {
            if (text == null || text.isEmpty()) {
                throw new Refused("a " + kind + " cannot be empty");
            }
            if (text.startsWith("-")) {
                throw new Refused("a " + kind + " cannot begin with '-', which git would read as an option: " + text);
            }
            for (int i = 0; i < text.length(); i++) {
                char c = text.charAt(i);
                if (c <= ' '
                        || c == '\u007f'
                        || c == '\\'
                        || c == '~'
                        || c == '^'
                        || c == ':'
                        || c == '?'
                        || c == '*'
                        || c == '[') {
                    throw new Refused("a " + kind + " cannot contain " + (int) c + ": " + text);
                }
            }
            if (text.contains("..") || text.endsWith(".lock") || text.endsWith("/") || text.endsWith(".")) {
                throw new Refused("git does not accept this as a " + kind + ": " + text);
            }
            return text;
        }
    }

    final class Author {
        public final String name;
        public final String email;

        private Author(String name, String email) {
            this.name = name;
            this.email = email;
        }

        public static Author of(String name, String email) {
            if (name == null || name.isBlank() || name.indexOf('\n') >= 0) {
                throw new Refused("an author needs a name on one line: " + name);
            }
            if (email == null || email.isBlank() || email.indexOf('\n') >= 0) {
                throw new Refused("an author needs an email on one line: " + email);
            }
            return new Author(name, email);
        }
    }

    final class Checkout {
        public final Path path;
        public final Branch branch;

        public Checkout(Path path, Branch branch) {
            this.path = Objects.requireNonNull(path);
            this.branch = branch;
        }
    }

    final class MergeTree {
        public final Revision tree;
        public final List<String> conflicts;

        public MergeTree(Revision tree, List<String> conflicts) {
            this.tree = tree;
            this.conflicts = List.copyOf(conflicts);
        }

        public boolean conflicted() {
            return !conflicts.isEmpty();
        }
    }

    final class Outcome {
        public final boolean ok;
        public final String said;

        private Outcome(boolean ok, String said) {
            this.ok = ok;
            this.said = said;
        }

        public static Outcome done() {
            return new Outcome(true, "");
        }

        public static Outcome refused(String said) {
            return new Outcome(false, said);
        }
    }

    enum History {
        FAST_FORWARD_WHEN_IT_CAN,
        ALWAYS_A_MERGE_COMMIT
    }

    String version();

    Optional<Path> topLevel();

    Revision commitAt(Revision revision);

    Optional<Revision> commitIfThere(Revision revision);

    boolean isAncestor(Revision ancestor, Revision descendant);

    int commitsBetween(Revision from, Revision to);

    Outcome moveBranch(Branch branch, Revision to, Revision from);

    void pruneWorktrees();

    void addWorktree(Path at, Branch existing);

    void createWorktree(Path at, Branch created, Branch from);

    void removeWorktree(Path at);

    void deleteBranch(Branch branch);

    List<Checkout> checkouts();

    void stageEverything(Path worktree);

    List<String> stagedFiles(Path worktree);

    void commitStaged(Path worktree, Author author, String message);

    List<String> uncommittedFiles(Path worktree);

    Revision head(Path worktree);

    List<String> filesChangedBetween(Revision from, Revision to);

    MergeTree mergeTree(Revision ours, Revision theirs);

    Outcome merge(Path worktree, Author author, String message, Branch from, History history);

    void abortMerge(Path worktree);

    Outcome fastForwardOnly(Path worktree, Branch to);

    Revision commitTree(Author author, Revision tree, String message, Revision first, Revision second);

    boolean hasRemote(String remote);

    Optional<Revision> remoteCommit(String remote, Branch branch);

    Outcome push(String remote, Branch branch);
}
