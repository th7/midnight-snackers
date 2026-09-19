package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;

final class EditableSet {
    private final Path root;
    private final Path storeFile;
    private final TreeSet<Key> keys = new TreeSet<>();

    EditableSet(Path root, Path storeFile) {
        this.root = root;
        this.storeFile = storeFile;
        load();
    }

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

    synchronized List<Key> list() {
        return new ArrayList<>(keys);
    }

    synchronized void add(Key key) {
        keys.add(key);
        save();
    }

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
