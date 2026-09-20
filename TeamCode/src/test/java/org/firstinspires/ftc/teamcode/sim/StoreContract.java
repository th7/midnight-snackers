package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public abstract class StoreContract {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    protected abstract Store store();

    protected abstract Path somewhere() throws IOException;

    private static byte[] bytes(String text) {
        return text.getBytes(StandardCharsets.UTF_8);
    }

    @Test
    public void aFileNobodyWroteIsAbsentRatherThanAFailure() throws IOException {
        Path file = somewhere().resolve("nobody-wrote-this.json");

        assertTrue(store().readIfThere(file).isEmpty());
        assertFalse(store().isFile(file));
    }

    @Test
    public void whatIsWrittenWholeComesBackWhole() throws IOException {
        Path file = somewhere().resolve("state.json");

        store().writeWhole(file, bytes("{\"a\":1}"));

        assertArrayEquals(bytes("{\"a\":1}"), store().readIfThere(file).orElseThrow());
        assertTrue(store().isFile(file));
    }

    @Test
    public void writingAgainReplacesWhatWasThereRatherThanAppending() throws IOException {
        Path file = somewhere().resolve("state.json");
        store().writeWhole(file, bytes("the first thing, which is longer"));

        store().writeWhole(file, bytes("short"));

        assertArrayEquals(bytes("short"), store().readIfThere(file).orElseThrow());
    }

    @Test
    public void aFileIsWrittenIntoADirectoryThatDidNotExistYet() throws IOException {
        Path file = somewhere().resolve("not").resolve("there").resolve("yet.json");

        store().writeWhole(file, bytes("made"));

        assertArrayEquals(bytes("made"), store().readIfThere(file).orElseThrow());
    }

    @Test
    public void aFileThatCannotBeReadIsAFailureAndNotAnEmptyOne() throws IOException {
        Path file = unreadable();
        if (file == null) {
            return;
        }

        try {
            store().readIfThere(file);
            fail("reading " + file + " should have failed rather than answering");
        } catch (Store.Failed expected) {
            assertTrue(
                    expected.getMessage(),
                    expected.getMessage().contains(file.getFileName().toString()));
        }
    }

    @Test
    public void nothingHalfWrittenIsEverLeftBehind() throws IOException {
        Path file = somewhere().resolve("state.json");
        store().writeWhole(file, bytes("{\"whole\":true}"));

        assertEquals(
                "a reader only ever sees a whole file",
                "{\"whole\":true}",
                new String(store().readIfThere(file).orElseThrow(), StandardCharsets.UTF_8));
    }

    protected abstract Path unreadable() throws IOException;
}
