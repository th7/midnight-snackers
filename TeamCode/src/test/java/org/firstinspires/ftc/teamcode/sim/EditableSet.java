package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;

/**
 * The files the admin has picked for users to edit, and the only thing that decides whether a name
 * a user sent is one of them. A user's string is matched here, exactly, against what the admin
 * picked; what comes back is a {@link Key}, which is the only kind of thing the rest of the server
 * will resolve against a worktree.
 *
 * <p>It is stored per **project root**, so two checkouts on one machine keep their own picks, and
 * it outlives the process. A stored path is checked on the way back in exactly as a fresh pick is,
 * since a state file an editor has been at is no more trustworthy than anything else off disk.
 */
final class EditableSet {
    private final Path root;
    private final Path storeFile;
    private final TreeSet<Key> keys = new TreeSet<>();

    /**
     * @param root the project root the picks are relative to and stored under
     * @param storeFile the state file holding every root's picks
     */
    EditableSet(Path root, Path storeFile) {
        this.root = root;
        this.storeFile = storeFile;
        load();
    }

    /**
     * The key for a path a user named, or empty: an exact match against what the admin picked, and
     * never a resolution against the filesystem. A name that is not in the set is not a file here,
     * whatever it is on disk.
     */
    synchronized Optional<Key> lookUp(String userSuppliedPath) {
        for (Key key : keys) {
            if (key.path().equals(userSuppliedPath)) {
                return Optional.of(key);
            }
        }
        return Optional.empty();
    }

    synchronized boolean contains(Key key) {
        return keys.contains(key);
    }

    /** The picks, in path order. */
    synchronized List<Key> list() {
        return new ArrayList<>(keys);
    }

    /** Picks a file the server has already found under the root. */
    synchronized void add(Key key) {
        keys.add(key);
        save();
    }

    /** Unpicks a file a user or the admin named; false when it was not picked in the first place. */
    synchronized boolean remove(String userSuppliedPath) {
        Optional<Key> key = lookUp(userSuppliedPath);
        if (key.isEmpty()) {
            return false;
        }
        keys.remove(key.get());
        save();
        return true;
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
                    // checked on the way in, like any other path that did not come from here
                    Key.under(root, element.getAsString()).ifPresent(keys::add);
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
        for (Key key : keys) {
            ours.add(key.path());
        }
        roots.add(root.toString(), ours);
        JsonObject body = new JsonObject();
        body.add("roots", roots);
        StateStore.save(storeFile, body);
    }
}
