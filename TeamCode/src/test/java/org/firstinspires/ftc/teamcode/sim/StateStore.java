package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermission;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.Set;

final class StateStore {
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();

    private StateStore() {}

    static JsonObject load(Path file) {
        if (!Files.exists(file)) {
            return null;
        }
        try {
            JsonObject object =
                    GSON.fromJson(new String(Files.readAllBytes(file), StandardCharsets.UTF_8), JsonObject.class);
            if (object == null) {
                throw new IllegalStateException("empty");
            }
            return object;
        } catch (IOException | RuntimeException e) {
            throw new IllegalStateException("could not read " + file + ": " + e, e);
        }
    }

    static void save(Path file, JsonObject body) {
        boolean posix = posix();
        Set<PosixFilePermission> ownerOnlyFile = PosixFilePermissions.fromString("rw-------");
        Path dir = file.getParent();
        try {
            ownerOnlyDirectory(dir);
            Path temp = posix
                    ? Files.createTempFile(
                            dir,
                            "." + file.getFileName(),
                            ".saving",
                            PosixFilePermissions.asFileAttribute(ownerOnlyFile))
                    : Files.createTempFile(dir, "." + file.getFileName(), ".saving");
            try {
                Files.write(temp, GSON.toJson(body).getBytes(StandardCharsets.UTF_8));
                Files.move(temp, file, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
            } finally {
                Files.deleteIfExists(temp);
            }
        } catch (IOException e) {
            throw new UncheckedIOException("could not save " + file + ": " + e.getMessage(), e);
        }
    }

    static void ownerOnlyDirectory(Path dir) throws IOException {
        Files.createDirectories(dir);
        if (posix()) {
            Files.setPosixFilePermissions(dir, PosixFilePermissions.fromString("rwx------"));
        }
    }

    private static boolean posix() {
        return FileSystems.getDefault().supportedFileAttributeViews().contains("posix");
    }
}
