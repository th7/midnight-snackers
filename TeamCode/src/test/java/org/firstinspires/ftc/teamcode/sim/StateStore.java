package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;

final class StateStore {
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final Store STORE = new OnDiskStore();

    private StateStore() {}

    static JsonObject load(Path file) {
        return load(STORE, file);
    }

    static JsonObject load(Store store, Path file) {
        byte[] bytes = store.readIfThere(file).orElse(null);
        if (bytes == null) {
            return null;
        }
        try {
            JsonObject object = GSON.fromJson(new String(bytes, StandardCharsets.UTF_8), JsonObject.class);
            if (object == null) {
                throw new IllegalStateException("empty");
            }
            return object;
        } catch (RuntimeException e) {
            throw new IllegalStateException("could not read " + file + ": " + e, e);
        }
    }

    static void save(Path file, JsonObject body) {
        save(STORE, file, body);
    }

    static void save(Store store, Path file, JsonObject body) {
        store.writeWhole(file, GSON.toJson(body).getBytes(StandardCharsets.UTF_8));
    }

    static void ownerOnlyDirectory(Path dir) throws IOException {
        STORE.ownerOnlyDirectory(dir);
    }
}
