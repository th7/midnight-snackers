package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;

final class EditableSet {
    private final TeamRobot robot;
    private final Path root;
    private final Path storeFile;
    private final TreeSet<Key> picked = new TreeSet<>();

    EditableSet(TeamRobot robot, Path root, Path storeFile) {
        this.robot = robot;
        this.root = root;
        this.storeFile = storeFile;
        load();
    }

    synchronized List<Key> in(Path worktree) {
        TreeSet<Key> editable = new TreeSet<>(ownIn(worktree));
        editable.addAll(picked);
        return new ArrayList<>(editable);
    }

    synchronized Optional<Key> lookUpIn(Path worktree, String userSuppliedPath) {
        for (Key key : in(worktree)) {
            if (key.path().equals(userSuppliedPath)) {
                return Optional.of(key);
            }
        }
        return Optional.empty();
    }

    synchronized boolean containsIn(Path worktree, Key key) {
        return in(worktree).contains(key);
    }

    synchronized List<Key> picked() {
        return new ArrayList<>(picked);
    }

    synchronized TeamRobot.Picking add(Key key) {
        TeamRobot.Picking picking = robot.picking(key.path());
        if (picking instanceof TeamRobot.Picking.Pickable) {
            picked.add(key);
            save();
        }
        return picking;
    }

    synchronized boolean remove(String userSuppliedPath) {
        for (Key key : picked) {
            if (key.path().equals(userSuppliedPath)) {
                picked.remove(key);
                save();
                return true;
            }
        }
        return false;
    }

    private List<Key> ownIn(Path worktree) {
        List<Key> own = new ArrayList<>();
        for (String directory : robot.ownDirectories()) {
            Path packageDir = worktree.resolve(directory);
            if (!Files.isDirectory(packageDir, LinkOption.NOFOLLOW_LINKS) || !stillIn(worktree, packageDir)) {
                continue;
            }
            for (Path file : regularFilesUnder(packageDir)) {
                Key key = Key.of(worktree, file);
                if (robot.owns(key.path())) {
                    own.add(key);
                }
            }
        }
        return own;
    }

    private static boolean stillIn(Path worktree, Path directory) {
        try {
            return directory.toRealPath().startsWith(worktree.toRealPath());
        } catch (IOException e) {
            throw new UncheckedIOException("could not tell where " + directory + " is", e);
        }
    }

    private static List<Path> regularFilesUnder(Path directory) {
        try (Stream<Path> files = Files.walk(directory)) {
            return files.filter(file -> Files.isRegularFile(file, LinkOption.NOFOLLOW_LINKS))
                    .collect(Collectors.toList());
        } catch (IOException e) {
            throw new UncheckedIOException("could not list the files under " + directory, e);
        }
    }

    private void load() {
        JsonObject stored = StateStore.load(storeFile);
        if (stored == null) {
            return;
        }
        try {
            JsonElement ours = stored.getAsJsonObject("roots").get(root.toString());
            if (ours != null) {
                for (JsonElement element : ours.getAsJsonArray()) {
                    Key.under(root, element.getAsString()).ifPresent(picked::add);
                }
            }
        } catch (RuntimeException e) {
            throw new IllegalStateException("could not read the editable files in " + storeFile + ": " + e, e);
        }
    }

    private void save() {
        JsonObject stored = StateStore.load(storeFile);
        JsonObject roots = stored == null || !stored.has("roots") ? new JsonObject() : stored.getAsJsonObject("roots");
        JsonArray ours = new JsonArray();
        for (Key key : picked) {
            ours.add(key.path());
        }
        roots.add(root.toString(), ours);
        JsonObject body = new JsonObject();
        body.add("roots", roots);
        StateStore.save(storeFile, body);
    }
}
