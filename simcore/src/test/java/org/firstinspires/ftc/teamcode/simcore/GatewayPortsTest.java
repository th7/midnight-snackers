package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Map;
import org.junit.Test;

public class GatewayPortsTest {
    private static String said(Checked<GatewayPorts> ports) {
        return ports.fold(found -> "admin " + found.admin() + ", users " + found.users(), rule -> rule);
    }

    @Test
    public void withNothingSetTheAdminListensOn21987AndTheUsersOn21986() {
        assertEquals("admin 21987, users 21986", said(GatewayPorts.from(Map.of())));
    }

    @Test
    public void eachPortIsSetByItsOwnName() {
        assertEquals(
                "admin 8001, users 8002",
                said(GatewayPorts.from(
                        Map.of("ROBOT_GATEWAY_ADMIN_PORT", "8001", "ROBOT_GATEWAY_USER_PORT", " 8002 "))));
    }

    @Test
    public void aBlankPortIsNoneSet() {
        assertEquals("admin 21987, users 21986", said(GatewayPorts.from(Map.of("ROBOT_GATEWAY_USER_PORT", " "))));
    }

    @Test
    public void zeroAsksForAnyFreePort() {
        assertEquals(
                "admin 0, users 0",
                said(GatewayPorts.from(Map.of("ROBOT_GATEWAY_ADMIN_PORT", "0", "ROBOT_GATEWAY_USER_PORT", "0"))));
    }

    @Test
    public void aPortThatIsNotANumberOrOutOfRangeIsRefusedNamingTheSettingAndWhatItHeld() {
        String words = said(GatewayPorts.from(Map.of("ROBOT_GATEWAY_USER_PORT", "eighty")));
        String negative = said(GatewayPorts.from(Map.of("ROBOT_GATEWAY_ADMIN_PORT", "-1")));
        String tooHigh = said(GatewayPorts.from(Map.of("ROBOT_GATEWAY_ADMIN_PORT", "65536")));

        assertEquals("ROBOT_GATEWAY_USER_PORT must be a port number from 0 to 65535, not 'eighty'", words);
        assertEquals("ROBOT_GATEWAY_ADMIN_PORT must be a port number from 0 to 65535, not '-1'", negative);
        assertEquals("ROBOT_GATEWAY_ADMIN_PORT must be a port number from 0 to 65535, not '65536'", tooHigh);
        assertEquals("admin 65535, users 21986", said(GatewayPorts.from(Map.of("ROBOT_GATEWAY_ADMIN_PORT", "65535"))));
    }

    @Test
    public void aSettingTheCodingServerReadIsRefusedNamingWhatReplacesIt() {
        String admin = said(GatewayPorts.from(Map.of("CODING_ADMIN_PORT", "8001")));
        String users = said(GatewayPorts.from(Map.of("CODING_USER_PORT", "8002")));

        assertEquals(
                "CODING_ADMIN_PORT is no longer read: the coding server is now the robot gateway,"
                        + " so set ROBOT_GATEWAY_ADMIN_PORT=8001 instead and unset CODING_ADMIN_PORT",
                admin);
        assertEquals(
                "CODING_USER_PORT is no longer read: the coding server is now the robot gateway,"
                        + " so set ROBOT_GATEWAY_USER_PORT=8002 instead and unset CODING_USER_PORT",
                users);
    }

    @Test
    public void aSettingTheCodingServerReadIsRefusedEvenBesideItsReplacement() {
        String said = said(GatewayPorts.from(Map.of("CODING_USER_PORT", "8002", "ROBOT_GATEWAY_USER_PORT", "8002")));

        assertTrue(said, said.startsWith("CODING_USER_PORT is no longer read"));
    }

    @Test
    public void bothOldSettingsAreNamedAtOnce() {
        String said = said(GatewayPorts.from(Map.of("CODING_ADMIN_PORT", "1", "CODING_USER_PORT", "2")));

        assertTrue(said, said.contains("ROBOT_GATEWAY_ADMIN_PORT=1"));
        assertTrue(said, said.contains("ROBOT_GATEWAY_USER_PORT=2"));
    }

    @Test
    public void aBlankOldSettingIsNoneSet() {
        assertEquals("admin 21987, users 21986", said(GatewayPorts.from(Map.of("CODING_ADMIN_PORT", ""))));
    }
}
