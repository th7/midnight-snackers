package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Map;
import org.junit.Test;

public class EnvironmentTest {
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
