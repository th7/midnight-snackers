package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeMap;

public final class FakeGit implements Git {
    private static final class Snapshot {
        final Map<String, String> files;

        Snapshot(Map<String, String> files) {
            this.files = new TreeMap<>(files);
        }
    }

    private static final class Commit {
        final String id;
        final Snapshot snapshot;
        final List<String> parents;

        Commit(String id, Snapshot snapshot, List<String> parents) {
            this.id = id;
            this.snapshot = snapshot;
            this.parents = List.copyOf(parents);
        }
    }

    private static final class Tree {
        final String id;
        final Snapshot snapshot;

        Tree(String id, Snapshot snapshot) {
            this.id = id;
            this.snapshot = snapshot;
        }
    }

    private static final class Checkout1 {
        final Path path;
        String branch;
        Map<String, String> staged = new TreeMap<>();

        Checkout1(Path path, String branch) {
            this.path = path;
            this.branch = branch;
        }
    }

    private final Path root;
    private final Map<String, Commit> commits = new LinkedHashMap<>();
    private final Map<String, Tree> trees = new LinkedHashMap<>();
    private final Map<String, String> branches = new LinkedHashMap<>();
    private final Map<String, Map<String, String>> remotes = new LinkedHashMap<>();
    private final Map<Path, Checkout1> checkouts = new LinkedHashMap<>();
    private final Set<String> unreachable = new LinkedHashSet<>();
    private int next = 1;

    public FakeGit(Path root, String firstBranch) {
        this(root, firstBranch, null);
    }

    /**
     * A git whose first commit is whatever is already in the directory -- so a fixture that lays a
     * project out and then makes one of these has it committed, and a worktree of it carries the
     * files. What git keeps for itself under {@code .git} is not a file of the project, so a real
     * repository left in the same directory is neither read nor carried anywhere.
     */
    public static FakeGit ofWhatIsOnDisk(Path root, String firstBranch) {
        Path at = root.toAbsolutePath().normalize();
        return new FakeGit(root, firstBranch, onDiskUnder(at));
    }

    private FakeGit(Path root, String firstBranch, Map<String, String> start) {
        this.root = root.toAbsolutePath().normalize();
        Commit first = commit(new Snapshot(start == null ? Map.of("README", "hello\n") : start), List.of());
        branches.put(firstBranch, first.id);
        checkouts.put(this.root, new Checkout1(this.root, firstBranch));
        if (start == null) {
            write(this.root, first.snapshot.files);
        }
    }

    public void addRemote(String name) {
        remotes.put(name, new LinkedHashMap<>());
    }

    public void addRemoteItCannotReach(String name) {
        remotes.put(name, null);
        unreachable.add(name);
    }

    private String id(String kind) {
        return String.format("%s%039x", kind, next++);
    }

    private Commit commit(Snapshot snapshot, List<String> parents) {
        Commit made = new Commit(id("c"), snapshot, parents);
        commits.put(made.id, made);
        return made;
    }

    private Tree tree(Snapshot snapshot) {
        Tree made = new Tree(id("t"), snapshot);
        trees.put(made.id, made);
        return made;
    }

    private String resolve(Revision revision) {
        String text = revision.text();
        if (text.startsWith("refs/heads/")) {
            return branches.get(text.substring("refs/heads/".length()));
        }
        if (text.startsWith("refs/remotes/")) {
            String rest = text.substring("refs/remotes/".length());
            int slash = rest.indexOf('/');
            Map<String, String> remote = remotes.get(rest.substring(0, slash));
            return remote == null ? null : remote.get(rest.substring(slash + 1));
        }
        if (branches.containsKey(text)) {
            return branches.get(text);
        }
        return commits.containsKey(text) ? text : null;
    }

    private String must(Revision revision) {
        String id = resolve(revision);
        if (id == null) {
            throw new Git.Failed("no such revision: " + revision);
        }
        return id;
    }

    private Checkout1 checkout(Path worktree) {
        Checkout1 found = checkouts.get(worktree.toAbsolutePath().normalize());
        if (found == null) {
            throw new Git.Failed("not a worktree: " + worktree);
        }
        return found;
    }

    private Map<String, String> onDisk(Path worktree) {
        return onDiskUnder(worktree);
    }

    private static Map<String, String> onDiskUnder(Path worktree) {
        Map<String, String> files = new TreeMap<>();
        try (var walk = Files.walk(worktree)) {
            for (Path path : walk.filter(Files::isRegularFile).toList()) {
                String name = worktree.relativize(path).toString().replace('\\', '/');
                if (name.equals(".git") || name.startsWith(".git/")) {
                    continue;
                }
                files.put(name, asText(path, name));
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return files;
    }

    /**
     * This git keeps a file as text, which is all a test of the server ever puts in one. A file it
     * cannot hold is refused by name rather than carried through a decode that would change it:
     * a model or an image mangled in a worktree is a test passing for a reason nobody meant.
     */
    private static String asText(Path path, String name) throws IOException {
        try {
            return java.nio.charset.StandardCharsets.UTF_8
                    .newDecoder()
                    .onMalformedInput(java.nio.charset.CodingErrorAction.REPORT)
                    .onUnmappableCharacter(java.nio.charset.CodingErrorAction.REPORT)
                    .decode(java.nio.ByteBuffer.wrap(Files.readAllBytes(path)))
                    .toString();
        } catch (java.nio.charset.CharacterCodingException notText) {
            throw new IllegalStateException(name
                    + " is not text, and this git keeps a file as text. Give the test a real git"
                    + " instead, which carries the bytes: " + path);
        }
    }

    private void write(Path worktree, Map<String, String> files) {
        try {
            Files.createDirectories(worktree);
            for (Path path :
                    onDisk(worktree).keySet().stream().map(worktree::resolve).toList()) {
                Files.deleteIfExists(path);
            }
            for (Map.Entry<String, String> file : files.entrySet()) {
                Path path = worktree.resolve(file.getKey());
                Files.createDirectories(path.getParent());
                Files.write(path, file.getValue().getBytes(StandardCharsets.UTF_8));
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private Snapshot snapshotOf(String commit) {
        return commits.get(commit).snapshot;
    }

    /**
     * What is in a revision, whether it names a commit or a tree. A pull asks what a merge would
     * bring before it makes it, and the only name it has for that is the tree {@code mergeTree}
     * gave it -- which is not a commit and never becomes one when the pull is refused.
     */
    private Map<String, String> filesAt(Revision revision) {
        Tree tree = trees.get(revision.text());
        return tree != null ? tree.snapshot.files : snapshotOf(must(revision)).files;
    }

    private Set<String> ancestry(String commit) {
        Set<String> seen = new LinkedHashSet<>();
        List<String> todo = new ArrayList<>(List.of(commit));
        while (!todo.isEmpty()) {
            String at = todo.remove(todo.size() - 1);
            if (at == null || !seen.add(at)) {
                continue;
            }
            todo.addAll(commits.get(at).parents);
        }
        return seen;
    }

    private String base(String ours, String theirs) {
        Set<String> mine = ancestry(ours);
        for (String at : ancestry(theirs)) {
            if (mine.contains(at)) {
                return at;
            }
        }
        return null;
    }

    private Snapshot merged(String ours, String theirs, List<String> conflicts) {
        Snapshot base = snapshotOf(base(ours, theirs));
        Snapshot mine = snapshotOf(ours);
        Snapshot yours = snapshotOf(theirs);
        Map<String, String> out = new TreeMap<>(mine.files);
        Set<String> names = new LinkedHashSet<>(mine.files.keySet());
        names.addAll(yours.files.keySet());
        for (String name : names) {
            String was = base.files.get(name);
            String a = mine.files.get(name);
            String b = yours.files.get(name);
            if (java.util.Objects.equals(a, b)) {
                continue;
            }
            if (java.util.Objects.equals(a, was)) {
                if (b == null) {
                    out.remove(name);
                } else {
                    out.put(name, b);
                }
            } else if (java.util.Objects.equals(b, was)) {
                continue;
            } else {
                conflicts.add(name);
            }
        }
        conflicts.sort(null);
        return new Snapshot(out);
    }

    @Override
    public String version() {
        return "git version 2.99.0 (fake)";
    }

    @Override
    public Optional<Path> topLevel() {
        return Optional.of(root);
    }

    @Override
    public Revision commitAt(Revision revision) {
        return Revision.of(must(revision));
    }

    @Override
    public Optional<Revision> commitIfThere(Revision revision) {
        String id = resolve(revision);
        return id == null ? Optional.empty() : Optional.of(Revision.of(id));
    }

    @Override
    public boolean isAncestor(Revision ancestor, Revision descendant) {
        return ancestry(must(descendant)).contains(must(ancestor));
    }

    @Override
    public int commitsBetween(Revision from, Revision to) {
        Set<String> theirs = ancestry(must(to));
        theirs.removeAll(ancestry(must(from)));
        return theirs.size();
    }

    @Override
    public Outcome moveBranch(Branch branch, Revision to, Revision from) {
        String at = branches.get(branch.name());
        if (at == null || !at.equals(must(from))) {
            return Outcome.refused("the branch " + branch + " has moved");
        }
        branches.put(branch.name(), must(to));
        return Outcome.done();
    }

    @Override
    public boolean stillAWorktree(Path at) {
        Path where = at.toAbsolutePath().normalize();
        return !where.equals(root) && checkouts.containsKey(where) && Files.isDirectory(where);
    }

    @Override
    public void pruneWorktrees() {
        checkouts.entrySet().removeIf(entry -> !entry.getKey().equals(root) && !Files.isDirectory(entry.getKey()));
    }

    @Override
    public void addWorktree(Path at, Branch existing) {
        Path where = at.toAbsolutePath().normalize();
        if (!branches.containsKey(existing.name())) {
            throw new Git.Failed("no such branch: " + existing);
        }
        checkouts.put(where, new Checkout1(where, existing.name()));
        write(where, snapshotOf(branches.get(existing.name())).files);
    }

    @Override
    public void createWorktree(Path at, Branch created, Branch from) {
        if (branches.containsKey(created.name())) {
            throw new Git.Failed("branch already exists: " + created);
        }
        branches.put(created.name(), must(from.tip()));
        addWorktree(at, created);
    }

    @Override
    public void removeWorktree(Path at) {
        Path where = at.toAbsolutePath().normalize();
        checkout(where);
        checkouts.remove(where);
        try (var walk = Files.walk(where)) {
            for (Path path : walk.sorted(Comparator.reverseOrder()).toList()) {
                Files.deleteIfExists(path);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override
    public void deleteBranch(Branch branch) {
        if (branches.remove(branch.name()) == null) {
            throw new Git.Failed("no such branch: " + branch);
        }
    }

    @Override
    public List<Checkout> checkouts() {
        List<Checkout> out = new ArrayList<>();
        for (Checkout1 at : checkouts.values()) {
            out.add(new Checkout(at.path, Branch.of(at.branch)));
        }
        return out;
    }

    @Override
    public void stageEverything(Path worktree) {
        Checkout1 at = checkout(worktree);
        at.staged = onDisk(at.path);
    }

    @Override
    public List<String> stagedFiles(Path worktree) {
        Checkout1 at = checkout(worktree);
        Map<String, String> committed = snapshotOf(branches.get(at.branch)).files;
        List<String> changed = new ArrayList<>();
        Set<String> names = new LinkedHashSet<>(at.staged.keySet());
        names.addAll(committed.keySet());
        for (String name : names) {
            if (!java.util.Objects.equals(at.staged.get(name), committed.get(name))) {
                changed.add(name);
            }
        }
        changed.sort(null);
        return changed;
    }

    @Override
    public void commitStaged(Path worktree, Author author, String message) {
        Checkout1 at = checkout(worktree);
        Commit made = commit(new Snapshot(at.staged), List.of(branches.get(at.branch)));
        branches.put(at.branch, made.id);
    }

    @Override
    public List<String> uncommittedFiles(Path worktree) {
        Checkout1 at = checkout(worktree);
        Map<String, String> committed = snapshotOf(branches.get(at.branch)).files;
        Map<String, String> now = onDisk(at.path);
        List<String> changed = new ArrayList<>();
        Set<String> names = new LinkedHashSet<>(now.keySet());
        names.addAll(committed.keySet());
        for (String name : names) {
            if (!java.util.Objects.equals(now.get(name), committed.get(name))) {
                changed.add(name);
            }
        }
        changed.sort(null);
        return changed;
    }

    @Override
    public Revision head(Path worktree) {
        return Revision.of(branches.get(checkout(worktree).branch));
    }

    @Override
    public List<String> filesChangedBetween(Revision from, Revision to) {
        Map<String, String> a = filesAt(from);
        Map<String, String> b = filesAt(to);
        List<String> changed = new ArrayList<>();
        Set<String> names = new LinkedHashSet<>(a.keySet());
        names.addAll(b.keySet());
        for (String name : names) {
            if (!java.util.Objects.equals(a.get(name), b.get(name))) {
                changed.add(name);
            }
        }
        changed.sort(null);
        return changed;
    }

    @Override
    public MergeTree mergeTree(Revision ours, Revision theirs) {
        List<String> conflicts = new ArrayList<>();
        Snapshot snapshot = merged(must(ours), must(theirs), conflicts);
        return new MergeTree(Revision.of(tree(snapshot).id), conflicts);
    }

    @Override
    public Outcome merge(Path worktree, Author author, String message, Branch from, History history) {
        Checkout1 at = checkout(worktree);
        String ours = branches.get(at.branch);
        String theirs = must(from.tip());
        if (ancestry(ours).contains(theirs)) {
            return Outcome.done();
        }
        List<String> conflicts = new ArrayList<>();
        Snapshot snapshot = merged(ours, theirs, conflicts);
        if (!conflicts.isEmpty()) {
            return Outcome.refused("CONFLICT: " + String.join(", ", conflicts));
        }
        Commit made;
        if (history == History.FAST_FORWARD_WHEN_IT_CAN && ancestry(theirs).contains(ours)) {
            made = commits.get(theirs);
        } else {
            made = commit(snapshot, List.of(ours, theirs));
        }
        branches.put(at.branch, made.id);
        write(at.path, made.snapshot.files);
        at.staged = onDisk(at.path);
        return Outcome.done();
    }

    @Override
    public void abortMerge(Path worktree) {
        Checkout1 at = checkout(worktree);
        write(at.path, snapshotOf(branches.get(at.branch)).files);
        at.staged = onDisk(at.path);
    }

    @Override
    public Outcome fastForwardOnly(Path worktree, Branch to) {
        Checkout1 at = checkout(worktree);
        String ours = branches.get(at.branch);
        String theirs = must(to.tip());
        if (ancestry(ours).contains(theirs)) {
            return Outcome.done();
        }
        if (!ancestry(theirs).contains(ours)) {
            return Outcome.refused("Not possible to fast-forward, aborting.");
        }
        branches.put(at.branch, theirs);
        write(at.path, snapshotOf(theirs).files);
        at.staged = onDisk(at.path);
        return Outcome.done();
    }

    @Override
    public Revision commitTree(Author author, Revision tree, String message, Revision first, Revision second) {
        Tree found = trees.get(tree.text());
        if (found == null) {
            throw new Git.Failed("no such tree: " + tree);
        }
        return Revision.of(commit(found.snapshot, List.of(must(first), must(second))).id);
    }

    @Override
    public boolean hasRemote(String remote) {
        return remotes.containsKey(remote);
    }

    @Override
    public Optional<Revision> remoteCommit(String remote, Branch branch) {
        Map<String, String> there = remotes.get(remote);
        if (there == null || there.get(branch.name()) == null) {
            return Optional.empty();
        }
        return Optional.of(Revision.of(there.get(branch.name())));
    }

    @Override
    public Outcome push(String remote, Branch branch) {
        if (unreachable.contains(remote)) {
            return Outcome.refused("could not read from remote repository: " + remote);
        }
        Map<String, String> there = remotes.get(remote);
        if (there == null) {
            return Outcome.refused("could not read from remote repository: " + remote);
        }
        there.put(branch.name(), must(branch.tip()));
        return Outcome.done();
    }
}
