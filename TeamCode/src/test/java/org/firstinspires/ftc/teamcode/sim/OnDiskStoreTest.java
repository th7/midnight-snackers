package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFilePermissions;

public class OnDiskStoreTest extends StoreContract {
    private final Store store = new OnDiskStore();

    @Override
    protected Store store() {
        return store;
    }

    @Override
    protected Path somewhere() throws IOException {
        return folder.getRoot().toPath();
    }

    @Override
    protected Path unreadable() throws IOException {
        if (!FileSystems.getDefault().supportedFileAttributeViews().contains("posix")) {
            return null;
        }
        Path file = somewhere().resolve("locked.json");
        Files.write(file, "secret".getBytes(java.nio.charset.StandardCharsets.UTF_8));
        Files.setPosixFilePermissions(file, PosixFilePermissions.fromString("-w-------"));
        return file;
    }
}
