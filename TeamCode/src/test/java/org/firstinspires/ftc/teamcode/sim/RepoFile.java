package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * A file of the repository, written as one. The **state directory** holds session secrets, so
 * writing anything through its {@link Store} makes the directory it lands in owner-only -- which is
 * right there and wrong on a source tree, where it turns a checked-out directory into one only the
 * person who last regenerated something can read. Nothing written here is secret: a golden trace,
 * a budget, an answer somebody reads in a diff.
 */
public final class RepoFile {

    private RepoFile() {}

    public static void write(Path file, byte[] bytes) {
        try {
            Path directory = file.toAbsolutePath().getParent();
            if (directory != null) {
                Files.createDirectories(directory);
            }
            Files.write(file, bytes);
        } catch (IOException e) {
            throw new UncheckedIOException("could not write " + file, e);
        }
    }
}
