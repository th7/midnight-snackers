package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HubMountingTest {
    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    @Test
    public void reginaldsHubSitsWithItsLogoToTheRightAndItsUsbPortsUp() {
        HubMounting reginalds = HubMounting.reginalds();

        assertEquals(Facing.RIGHT, reginalds.logo());
        assertEquals(Facing.UP, reginalds.usb());
    }

    @Test
    public void theLogoAndTheUsbPortsFaceAcrossEachOther() {
        assertEquals(Facing.DOWN, Valid.value(HubMounting.of(Facing.DOWN, Facing.RIGHT)).logo());
        assertRejected("the hub's logo and its USB ports face along different axes", HubMounting.of(Facing.UP, Facing.UP));
        assertRejected("not UP and DOWN", HubMounting.of(Facing.UP, Facing.DOWN));
        assertRejected("not LEFT and RIGHT", HubMounting.of(Facing.LEFT, Facing.RIGHT));
        assertRejected("not FORWARD and BACKWARD", HubMounting.of(Facing.FORWARD, Facing.BACKWARD));
    }
}
