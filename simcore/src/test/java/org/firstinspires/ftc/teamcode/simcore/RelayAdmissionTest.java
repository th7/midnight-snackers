package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class RelayAdmissionTest {
    private static Peer peer(String address) {
        return Valid.value(Peer.of(address));
    }

    private static RelayAdmission.Login approved(String username, String address) {
        return new RelayAdmission.Login(username, peer(address), true);
    }

    private static RelayAdmission.Login notApproved(String username, String address) {
        return new RelayAdmission.Login(username, peer(address), false);
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    @Test
    public void anAddressWithNoRecordedTextIsNoPeer() {
        String said = rejection(Peer.of(""));

        assertTrue(said, said.contains("no address"));
        assertEquals("10.0.0.7", peer("10.0.0.7").address());
    }

    @Test
    public void anAddressAnApprovedLoginCameFromIsAdmittedNamingWhoseItIs() {
        RelayAdmission admission = RelayAdmission.of(List.of(approved("ada", "10.0.0.7"), approved("bob", "10.0.0.9")));

        assertEquals(List.of("ada"), Valid.value(admission.admit(peer("10.0.0.7"))));
    }

    @Test
    public void anAddressNoLoginCameFromIsRefusedNamingIt() {
        RelayAdmission admission = RelayAdmission.of(List.of(approved("ada", "10.0.0.7")));

        String said = rejection(admission.admit(peer("10.0.0.8")));

        assertTrue(said, said.contains("10.0.0.8"));
        assertTrue(said, said.contains("no approved login"));
    }

    @Test
    public void anAddressOnlyLoginsNotApprovedCameFromIsRefused() {
        RelayAdmission admission = RelayAdmission.of(List.of(notApproved("ada", "10.0.0.7")));

        String said = rejection(admission.admit(peer("10.0.0.7")));

        assertTrue(said, said.contains("no approved login"));
    }

    @Test
    public void anAddressTwoApprovedUsersShareIsAdmittedForBothEachNamedOnce() {
        RelayAdmission admission = RelayAdmission.of(List.of(
                approved("ada", "10.0.0.7"),
                notApproved("cy", "10.0.0.7"),
                approved("bob", "10.0.0.7"),
                approved("ada", "10.0.0.7")));

        assertEquals(List.of("ada", "bob"), Valid.value(admission.admit(peer("10.0.0.7"))));
    }

    @Test
    public void theConnectionsToCloseAreThoseFromAddressesNoApprovedLoginCameFrom() {
        RelayAdmission admission =
                RelayAdmission.of(List.of(notApproved("ada", "10.0.0.7"), approved("bob", "10.0.0.9")));

        assertEquals(
                List.of(peer("10.0.0.7"), peer("10.0.0.8")),
                admission.toClose(List.of(peer("10.0.0.7"), peer("10.0.0.9"), peer("10.0.0.8"))));
    }

    @Test
    public void aConnectionStaysOpenWhileAnotherApprovedLoginSharesItsAddress() {
        RelayAdmission admission =
                RelayAdmission.of(List.of(notApproved("ada", "10.0.0.7"), approved("bob", "10.0.0.7")));

        assertEquals(List.of(), admission.toClose(List.of(peer("10.0.0.7"))));
    }

    @Test
    public void theUsersRelayingAreThoseWithAnApprovedLoginFromAnOpenConnectionsAddress() {
        RelayAdmission admission = RelayAdmission.of(List.of(
                approved("ada", "10.0.0.7"),
                approved("bob", "10.0.0.9"),
                notApproved("cy", "10.0.0.8"),
                approved("ada", "10.0.0.7")));

        assertEquals(List.of("ada"), admission.relaying(List.of(peer("10.0.0.7"), peer("10.0.0.7"), peer("10.0.0.8"))));
    }

    @Test
    public void theConnectLineLeavesOutAdbsOwnPort() {
        assertEquals("adb connect 192.168.1.20", RelayAdmission.connectLine("192.168.1.20", 5555));
    }

    @Test
    public void theConnectLineNamesAnyOtherPort() {
        assertEquals("adb connect 192.168.1.20:5600", RelayAdmission.connectLine("192.168.1.20", 5600));
    }

    @Test
    public void theConnectLineBracketsAnIpv6AddressAndNamesItsPort() {
        assertEquals("adb connect [fd00::20]:5555", RelayAdmission.connectLine("fd00::20", 5555));
    }
}
