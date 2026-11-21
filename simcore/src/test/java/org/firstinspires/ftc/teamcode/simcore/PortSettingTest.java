package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;

import java.util.Map;
import org.junit.Test;

public class PortSettingTest {
    private static String said(Checked<Integer> port) {
        return port.fold(String::valueOf, rule -> rule);
    }

    @Test
    public void aPortNobodySetIsTheFallback() {
        assertEquals("21988", said(PortSetting.of(Map.of(), "SIMULATOR_PORT", 21988)));
        assertEquals("21988", said(PortSetting.of(Map.of("SIMULATOR_PORT", "   "), "SIMULATOR_PORT", 21988)));
    }

    @Test
    public void aPortSomebodySetIsUsedEvenWithSpaceAroundIt() {
        assertEquals("9000", said(PortSetting.of(Map.of("SIMULATOR_PORT", " 9000 "), "SIMULATOR_PORT", 21988)));
    }

    @Test
    public void theWholeRangeIsAPortAndZeroAsksForAnyFreeOne() {
        assertEquals("0", said(PortSetting.of(Map.of("SIMULATOR_PORT", "0"), "SIMULATOR_PORT", 21988)));
        assertEquals("65535", said(PortSetting.of(Map.of("SIMULATOR_PORT", "65535"), "SIMULATOR_PORT", 21988)));
    }

    @Test
    public void aPortThatIsNotOneIsRefusedNamingTheSettingAndTheValue() {
        for (String wrong : new String[] {"nine thousand", "-1", "65536", "9000x", "99999999999"}) {
            assertEquals(
                    "SIMULATOR_PORT must be a port number from 0 to 65535, not '" + wrong + "'",
                    said(PortSetting.of(Map.of("SIMULATOR_PORT", wrong), "SIMULATOR_PORT", 21988)));
        }
    }
}
