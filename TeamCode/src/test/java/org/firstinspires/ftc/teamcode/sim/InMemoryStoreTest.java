package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.nio.file.Path;

public class InMemoryStoreTest extends StoreContract {
    private final InMemoryStore store = new InMemoryStore();

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
        Path file = somewhere().resolve("locked.json");
        store.with(file, "secret".getBytes(java.nio.charset.StandardCharsets.UTF_8))
                .thatCannotRead(file);
        return file;
    }
}
