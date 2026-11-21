package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Map;
import org.junit.Test;

public class WhereTheHostIsTest {
    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    private static WhereTheHostIs from(String value) {
        return Valid.value(WhereTheHostIs.from(Map.of(WhereTheHostIs.ENV, value)));
    }

    @Test
    public void unsetOrBlankIsUnsaid() {
        assertTrue(Valid.value(WhereTheHostIs.from(Map.of())) instanceof WhereTheHostIs.Unsaid);
        assertTrue(from("  ") instanceof WhereTheHostIs.Unsaid);
    }

    @Test
    public void anAddressAloneIsTheGatewaysUserPort() {
        WhereTheHostIs.At at = (WhereTheHostIs.At) from("192.168.1.20");

        assertEquals("http://192.168.1.20:21986/", at.url());
    }

    @Test
    public void anAddressWithAPortIsThatPort() {
        assertEquals("http://host-mac.local:8080/", ((WhereTheHostIs.At) from(" host-mac.local:8080 ")).url());
    }

    @Test
    public void anythingButAnAddressAndAPortIsRefusedNamingTheSettingAndAnExample() {
        for (String wrong : new String[] {"http://192.168.1.20", "192.168.1.20:", "192.168.1.20:99999", "a b", ":80"}) {
            String said = rejection(WhereTheHostIs.from(Map.of(WhereTheHostIs.ENV, wrong)));

            assertTrue(wrong + ": " + said, said.contains("ROBOT_GATEWAY"));
            assertTrue(wrong + ": " + said, said.contains("192.168.1.20"));
            assertTrue(wrong + ": " + said, said.contains("'" + wrong + "'"));
        }
    }
}
