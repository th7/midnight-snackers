package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.util.Optional;

public interface Store {

    final class Failed extends RuntimeException {
        public Failed(String message, Throwable cause) {
            super(message, cause);
        }
    }

    Optional<byte[]> readIfThere(Path file);

    void writeWhole(Path file, byte[] bytes);

    boolean isFile(Path file);

    void ownerOnlyDirectory(Path directory);
}
