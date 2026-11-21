package org.firstinspires.ftc.teamcode.gateway;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.simcore.Peer;
import org.firstinspires.ftc.teamcode.simcore.RelayAdmission;
import org.junit.After;
import org.junit.Test;

public class RobotRelayTest {
    private static final String LOOPBACK = InetAddress.getLoopbackAddress().getHostAddress();

    private final FakeHub hub = FakeHub.start();
    private final AtomicReference<RelayAdmission> admission = new AtomicReference<>(RelayAdmission.of(List.of()));
    private RobotRelay relay;

    @After
    public void stop() {
        if (relay != null) {
            relay.stop();
        }
        hub.close();
    }

    private static Peer peer(String address) {
        return Peer.of(address).fold(found -> found, rule -> {
            throw new AssertionError(rule);
        });
    }

    private void approvedFrom(String address) {
        admission.set(RelayAdmission.of(List.of(new RelayAdmission.Login("ada", peer(address), true))));
    }

    private void noLongerApproved() {
        admission.set(RelayAdmission.of(List.of(new RelayAdmission.Login("ada", peer(LOOPBACK), false))));
    }

    private RobotRelay relayTo(InetSocketAddress to) {
        relay = RobotRelay.start(InetAddress.getLoopbackAddress(), 0, to, admission::get);
        return relay;
    }

    private Socket client() throws IOException {
        Socket client = new Socket(InetAddress.getLoopbackAddress(), relay.port());
        client.setSoTimeout(10_000);
        return client;
    }

    private static void send(Socket client, String text) throws IOException {
        OutputStream out = client.getOutputStream();
        out.write(text.getBytes(StandardCharsets.UTF_8));
        out.flush();
    }

    private static String read(Socket client, int length) throws IOException {
        byte[] bytes = client.getInputStream().readNBytes(length);
        return new String(bytes, StandardCharsets.UTF_8);
    }

    private static void assertClosedByTheRelay(Socket client) throws IOException {
        InputStream in = client.getInputStream();
        try {
            assertEquals("the relay closed the connection", -1, in.read());
        } catch (java.net.SocketException reset) {
            assertTrue(
                    reset.getMessage(),
                    reset.getMessage().toLowerCase(java.util.Locale.ROOT).contains("reset"));
        } catch (SocketTimeoutException e) {
            throw new AssertionError("the relay left the connection hanging", e);
        }
    }

    private static void await(String what, BooleanSupplier condition) throws InterruptedException {
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (!condition.getAsBoolean()) {
            if (System.nanoTime() > deadline) {
                throw new AssertionError("never: " + what);
            }
            Thread.sleep(10);
        }
    }

    @Test
    public void anAdmittedClientsBytesReachTheHubAndItsAnswerComesBack() throws Exception {
        approvedFrom(LOOPBACK);
        relayTo(hub.address());

        try (Socket client = client()) {
            send(client, "CNXN host::");

            assertEquals("CNXN host::", read(client, "CNXN host::".length()));
        }
        assertEquals("CNXN host::", hub.received());
        assertEquals(1, hub.accepted());
    }

    @Test
    public void aClientFromAnAddressWithNoApprovedLoginIsClosedWithoutTheHubHearingOfIt() throws Exception {
        approvedFrom("10.0.0.7");
        relayTo(hub.address());

        try (Socket client = client()) {
            send(client, "CNXN host::");

            assertClosedByTheRelay(client);
        }
        assertEquals(0, hub.accepted());
        assertEquals("", hub.received());
    }

    @Test
    public void aHubThatCannotBeReachedClosesTheClientAndIsSaid() throws Exception {
        approvedFrom(LOOPBACK);
        InetSocketAddress nobody = FakeHub.nobodyListening();
        relayTo(nobody);

        try (Socket client = client()) {
            assertClosedByTheRelay(client);
        }
        assertEquals("unreachable", relay.hubState().said());
        assertTrue(relay.hubState().why(), !relay.hubState().why().isEmpty());
    }

    @Test
    public void aHubReachedForAClientIsSaidReachable() throws Exception {
        approvedFrom(LOOPBACK);
        relayTo(hub.address());
        assertEquals("untried", relay.hubState().said());

        try (Socket client = client()) {
            send(client, "x");
            read(client, 1);
        }
        assertEquals("reachable", relay.hubState().said());
    }

    @Test
    public void aProbeSaysWhetherTheHubCanBeReachedBeforeAnyClientTries() throws Exception {
        relayTo(hub.address());

        relay.probeIfDue();

        await("the probe reached the hub", () -> relay.hubState().said().equals("reachable"));
        assertEquals(List.of(), relay.openPeers());
    }

    @Test
    public void aProbeOfAHubNobodyAnswersSaysItIsUnreachable() throws Exception {
        relayTo(FakeHub.nobodyListening());

        relay.probeIfDue();

        await("the probe failed", () -> relay.hubState().said().equals("unreachable"));
    }

    @Test
    public void anOpenConnectionIsListedByTheAddressItCameFrom() throws Exception {
        approvedFrom(LOOPBACK);
        relayTo(hub.address());

        try (Socket client = client()) {
            send(client, "x");
            read(client, 1);

            assertEquals(List.of(peer(LOOPBACK)), relay.openPeers());
        }
        await(
                "the closed connection is no longer listed",
                () -> relay.openPeers().isEmpty());
    }

    @Test
    public void aSweepClosesTheConnectionsNoLongerAdmittedAtBothEnds() throws Exception {
        approvedFrom(LOOPBACK);
        relayTo(hub.address());

        try (Socket client = client()) {
            send(client, "x");
            read(client, 1);
            noLongerApproved();

            relay.sweep();

            assertClosedByTheRelay(client);
            await("the hub's end closed", () -> hub.open() == 0);
            assertEquals(List.of(), relay.openPeers());
        }
    }

    @Test
    public void aSweepLeavesAConnectionStillAdmittedOpen() throws Exception {
        approvedFrom(LOOPBACK);
        relayTo(hub.address());

        try (Socket client = client()) {
            send(client, "x");
            read(client, 1);

            relay.sweep();

            send(client, "y");
            assertEquals("y", read(client, 1));
        }
    }

    @Test
    public void aHubThatDropsTheConnectionClosesTheClient() throws Exception {
        approvedFrom(LOOPBACK);
        relayTo(hub.address());

        try (Socket client = client()) {
            send(client, "x");
            read(client, 1);

            hub.dropEveryConnection();

            assertClosedByTheRelay(client);
        }
    }

    @Test
    public void theRelayListensOnAdbsOwnPortAndReachesTheHubsAdbPortUnlessTheEnvironmentSaysOtherwise() {
        assertEquals(5555, RobotRelay.port(Map.of()));
        assertEquals(new InetSocketAddress("192.168.43.1", 5555), RobotRelay.hub(Map.of()));
        assertEquals(5600, RobotRelay.port(Map.of(RobotRelay.PORT_ENV, "5600")));
        assertEquals(
                new InetSocketAddress("10.0.0.2", 5601), RobotRelay.hub(Map.of(RobotRelay.HUB_ENV, "10.0.0.2:5601")));
    }

    @Test
    public void aHubAddressWithNoPortIsRefusedNamingTheVariable() {
        IllegalArgumentException refused = assertThrows(
                IllegalArgumentException.class, () -> RobotRelay.hub(Map.of(RobotRelay.HUB_ENV, "10.0.0.2")));

        assertTrue(refused.getMessage(), refused.getMessage().contains(RobotRelay.HUB_ENV));
        assertTrue(refused.getMessage(), refused.getMessage().contains("host:port"));
    }

    @Test
    public void whetherAClientIsAdmittedIsAskedAsItArrives() throws Exception {
        relayTo(hub.address());
        try (Socket refused = client()) {
            assertClosedByTheRelay(refused);
        }
        approvedFrom(LOOPBACK);

        try (Socket client = client()) {
            send(client, "x");
            assertEquals("x", read(client, 1));
        }
    }

    @Test
    public void theRelayPortIsReadFromTheEnvironmentAndABadOneIsRefusedNamingIt() {
        assertEquals(5555, RobotRelay.port(Map.of()));
        assertEquals(6000, RobotRelay.port(Map.of("ROBOT_RELAY_PORT", " 6000 ")));

        try {
            RobotRelay.port(Map.of("ROBOT_RELAY_PORT", "65536"));
            fail("accepted a relay port that is not one");
        } catch (IllegalArgumentException expected) {
            assertEquals("ROBOT_RELAY_PORT must be a port number from 0 to 65535, not '65536'", expected.getMessage());
        }
    }
}
