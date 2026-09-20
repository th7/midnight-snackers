package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public final class InMemoryStore implements Store {
    private final Map<Path, byte[]> files = new LinkedHashMap<>();
    private final Set<Path> directories = new LinkedHashSet<>();
    private final Set<Path> unreadable = new LinkedHashSet<>();
    private final Set<Path> unwritable = new LinkedHashSet<>();

    private static Path key(Path path) {
        return path.toAbsolutePath().normalize();
    }

    public InMemoryStore with(Path file, byte[] bytes) {
        files.put(key(file), bytes.clone());
        return this;
    }

    public InMemoryStore thatCannotRead(Path file) {
        unreadable.add(key(file));
        return this;
    }

    public InMemoryStore thatCannotWrite(Path file) {
        unwritable.add(key(file));
        return this;
    }

    public Set<Path> directories() {
        return Set.copyOf(directories);
    }

    @Override
    public Optional<byte[]> readIfThere(Path file) {
        Path at = key(file);
        if (unreadable.contains(at)) {
            throw new Failed("could not read " + file, new IllegalStateException("unreadable"));
        }
        byte[] bytes = files.get(at);
        return bytes == null ? Optional.empty() : Optional.of(bytes.clone());
    }

    @Override
    public void writeWhole(Path file, byte[] bytes) {
        Path at = key(file);
        if (unwritable.contains(at)) {
            throw new Failed("could not save " + file, new IllegalStateException("unwritable"));
        }
        ownerOnlyDirectory(at.getParent());
        files.put(at, bytes.clone());
    }

    @Override
    public boolean isFile(Path file) {
        return files.containsKey(key(file));
    }

    @Override
    public void ownerOnlyDirectory(Path directory) {
        directories.add(key(directory));
    }
}
