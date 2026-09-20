package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Map;
import org.junit.Test;

public class EnvironmentTest {
    @Test
    public void aPortNobodySetIsTheDefault() {
        assertEquals(21987, CodingServer.port(Map.of(), "CODING_ADMIN_PORT", 21987));
        assertEquals(21987, CodingServer.port(Map.of("CODING_ADMIN_PORT", "   "), "CODING_ADMIN_PORT", 21987));
    }

    @Test
    public void aPortSomebodySetIsUsedEvenWithSpaceAroundIt() {
        assertEquals(9000, CodingServer.port(Map.of("CODING_ADMIN_PORT", " 9000 "), "CODING_ADMIN_PORT", 21987));
    }

    @Test
    public void aPortThatIsNotOneIsRefusedNamingTheVariableAndTheValue() {
        for (String wrong : new String[] {"nine thousand", "-1", "65536", "9000x"}) {
            try {
                CodingServer.port(Map.of("CODING_ADMIN_PORT", wrong), "CODING_ADMIN_PORT", 21987);
                fail("accepted " + wrong + " as a port");
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains("CODING_ADMIN_PORT"));
                assertTrue(expected.getMessage(), expected.getMessage().contains(wrong));
            }
        }
    }

    @Test
    public void theStateDirectoryFollowsXdgWhenItIsAnAbsolutePath() {
        assertEquals(
                java.nio.file.Path.of("/state/midnight-snackers/coding-server"),
                CodingServer.stateDir(Map.of("XDG_STATE_HOME", "/state")));
        assertEquals(
                java.nio.file.Path.of("/home/ada/.local/state/midnight-snackers/coding-server"),
                CodingServer.stateDir(Map.of("XDG_STATE_HOME", "not/absolute", "HOME", "/home/ada")));
    }

    @Test
    public void noLivePortIsNoLiveServerAndABadOneIsRefused() {
        assertNull(SimRunner.livePortIn(Map.of()));
        assertNull(SimRunner.livePortIn(Map.of("SIM_LIVE", " ")));
        assertEquals(Integer.valueOf(8000), SimRunner.livePortIn(Map.of("SIM_LIVE", " 8000 ")));

        try {
            SimRunner.livePortIn(Map.of("SIM_LIVE", "yes please"));
            fail("accepted a live port that is not one");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("SIM_LIVE"));
        }
    }
}
