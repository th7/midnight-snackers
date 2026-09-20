package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.NoSuchFileException;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.Optional;

public final class OnDiskStore implements Store {

    private static boolean posix() {
        return FileSystems.getDefault().supportedFileAttributeViews().contains("posix");
    }

    @Override
    public Optional<byte[]> readIfThere(Path file) {
        try {
            return Optional.of(Files.readAllBytes(file));
        } catch (NoSuchFileException absent) {
            return Optional.empty();
        } catch (IOException e) {
            throw new Failed("could not read " + file + ": " + e.getMessage(), e);
        }
    }

    @Override
    public void writeWhole(Path file, byte[] bytes) {
        Path target = file.toAbsolutePath();
        Path directory = target.getParent();
        try {
            ownerOnlyDirectory(directory);
            Path temp = posix
                    ? Files.createTempFile(
                            directory,
                            "." + target.getFileName(),
                            ".saving",
                            PosixFilePermissions.asFileAttribute(PosixFilePermissions.fromString("rw-------")))
                    : Files.createTempFile(directory, "." + target.getFileName(), ".saving");
            try {
                Files.write(temp, bytes);
                Files.move(temp, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
            } finally {
                Files.deleteIfExists(temp);
            }
        } catch (IOException e) {
            throw new Failed("could not save " + file + ": " + e.getMessage(), e);
        }
    }

    @Override
    public boolean isFile(Path file) {
        return Files.isRegularFile(file);
    }

    @Override
    public void ownerOnlyDirectory(Path directory) {
        try {
            Files.createDirectories(directory);
            if (posix) {
                Files.setPosixFilePermissions(directory, PosixFilePermissions.fromString("rwx------"));
            }
        } catch (IOException e) {
            throw new Failed("could not make the directory " + directory + ": " + e.getMessage(), e);
        }
    }

    private static final boolean posix = posix();
}
