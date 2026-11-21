package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.OptionalLong;
import org.junit.Test;

public class HubStateTest {
    @Test
    public void aHubNeverTriedIsSaidSoAndIsDueAProbe() {
        HubState untried = HubState.untried();

        assertEquals("untried", untried.said());
        assertEquals("", untried.why());
        assertEquals(OptionalLong.empty(), untried.checkedAtMillis());
        assertTrue(untried.probeDue(0));
    }

    @Test
    public void aHubReachedIsSaidSoWithWhen() {
        HubState reached = HubState.reached(1_000);

        assertEquals("reachable", reached.said());
        assertEquals("", reached.why());
        assertEquals(OptionalLong.of(1_000), reached.checkedAtMillis());
    }

    @Test
    public void aHubNotReachedIsSaidSoWithWhenAndWhy() {
        HubState unreached = HubState.unreached(1_000, "Connection refused");

        assertEquals("unreachable", unreached.said());
        assertEquals("Connection refused", unreached.why());
        assertEquals(OptionalLong.of(1_000), unreached.checkedAtMillis());
    }

    @Test
    public void aHubCheckedTenSecondsAgoIsDueAnotherProbeAndNotBefore() {
        assertFalse(HubState.reached(1_000).probeDue(10_999));
        assertTrue(HubState.reached(1_000).probeDue(11_000));
        assertFalse(HubState.unreached(1_000, "timed out").probeDue(10_999));
        assertTrue(HubState.unreached(1_000, "timed out").probeDue(11_000));
    }

    @Test
    public void aClockThatWentBackMakesAProbeDue() {
        assertTrue(HubState.reached(5_000).probeDue(4_000));
    }
}
