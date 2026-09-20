package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assume.assumeTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFilePermissions;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class RepoFileTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static boolean posix() {
        return FileSystems.getDefault().supportedFileAttributeViews().contains("posix");
    }

    @Test
    public void writingAFileOfTheRepositoryLeavesItsDirectoryAsItFoundIt() throws IOException {
        assumeTrue(posix());
        Path directory = folder.newFolder("sources").toPath();
        Files.setPosixFilePermissions(directory, PosixFilePermissions.fromString("rwxr-xr-x"));

        RepoFile.write(directory.resolve("golden.txt"), "a trace\n".getBytes(StandardCharsets.UTF_8));

        assertEquals(
                "a source tree is not the state directory: nothing written here is secret, and a"
                        + " directory somebody else can read is how a checkout is meant to be",
                "rwxr-xr-x",
                PosixFilePermissions.toString(Files.getPosixFilePermissions(directory)));
    }

    @Test
    public void whatWasWrittenIsWhatIsRead() throws IOException {
        Path file = folder.getRoot().toPath().resolve("made/up/the/way/down/answer.json");

        RepoFile.write(file, "{\"child-jvm\": 44}\n".getBytes(StandardCharsets.UTF_8));

        assertTrue("a directory on the way is made", Files.isRegularFile(file));
        assertArrayEquals("{\"child-jvm\": 44}\n".getBytes(StandardCharsets.UTF_8), Files.readAllBytes(file));
    }

    @Test
    public void writingOverAFileReplacesIt() throws IOException {
        Path file = folder.newFile("again.txt").toPath();

        RepoFile.write(file, "first\n".getBytes(StandardCharsets.UTF_8));
        RepoFile.write(file, "second\n".getBytes(StandardCharsets.UTF_8));

        assertEquals("second\n", new String(Files.readAllBytes(file), StandardCharsets.UTF_8));
    }

    @Test
    public void theStateStoreIsStillTheOneThatLocksADirectoryDown() throws IOException {
        assumeTrue(posix());
        Path directory = folder.newFolder("state").toPath();
        Files.setPosixFilePermissions(directory, PosixFilePermissions.fromString("rwxr-xr-x"));

        new OnDiskStore().writeWhole(directory.resolve("sessions.json"), "{}".getBytes(StandardCharsets.UTF_8));

        assertEquals(
                "session secrets live there, so writing one makes the directory owner-only; that is"
                        + " the habit a file of the repository must not be written with",
                "rwx------",
                PosixFilePermissions.toString(Files.getPosixFilePermissions(directory)));
    }
}
