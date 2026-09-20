package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.acmerobotics.roadrunner.Pose2d;
import java.nio.file.Path;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class StartPosesTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private Path file() {
        return folder.getRoot().toPath().resolve("start-poses.json");
    }

    @Test
    public void aStoreThatCannotBeReadStopsTheBenchRatherThanStartingOver() {
        Path file = folder.getRoot().toPath().resolve("start-poses.json");
        InMemoryStore store = new InMemoryStore()
                .with(file, "{}".getBytes(java.nio.charset.StandardCharsets.UTF_8))
                .thatCannotRead(file);

        try {
            new StartPoses(file, store);
            fail("a start-poses file that cannot be read must not be silently started over");
        } catch (Store.Failed expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("start-poses.json"));
        }
    }

    @Test
    public void aStoreWithNothingInItStartsFresh() {
        Path file = folder.getRoot().toPath().resolve("start-poses.json");

        StartPoses poses = new StartPoses(file, new InMemoryStore());

        assertEquals(Long.valueOf(1), poses.seed("anything"));
    }

    @Test
    public void everyOpModeRunsOnSeedOneUntilSomeoneSaysOtherwise() {
        StartPoses starts = new StartPoses(file());

        assertEquals(Long.valueOf(StartPoses.DEFAULT_SEED), starts.seed("Any"));
        assertEquals(1, StartPoses.DEFAULT_SEED);
    }

    @Test
    public void aSeedIsRememberedPerOpModeAndOutlivesTheStore() {
        StartPoses starts = new StartPoses(file());

        assertEquals(Long.valueOf(5), starts.putSeed("Square", 5L));

        assertEquals(Long.valueOf(5), starts.seed("Square"));
        assertEquals(Long.valueOf(1), starts.seed("Other"));
        assertEquals(Long.valueOf(5), new StartPoses(file()).seed("Square"));
    }

    @Test
    public void noSeedIsTheExactRobotAndIsRememberedAsSuchNotAsUnset() {
        StartPoses starts = new StartPoses(file());

        assertNull(starts.putSeed("Square", null));

        assertNull(starts.seed("Square"));
        assertNull(new StartPoses(file()).seed("Square"));
    }

    @Test
    public void placingKeepsTheSeedAndSeedingKeepsThePlace() {
        StartPoses starts = new StartPoses(file());
        starts.putSeed("Square", 5L);
        starts.put("Square", new Pose2d(-56, -12, 0.5));
        starts.putSeed("Square", 6L);

        StartPoses reloaded = new StartPoses(file());
        assertEquals(Long.valueOf(6), reloaded.seed("Square"));
        assertEquals(-56, reloaded.get("Square").position.x, 0);
        assertEquals(-12, reloaded.get("Square").position.y, 0);
        assertEquals(0.5, reloaded.get("Square").heading.toDouble(), 1e-9);
    }
}
