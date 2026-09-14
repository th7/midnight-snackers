package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

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

    // --- the seed: which robot an op mode's runs are made on, kept beside where they start ---

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
