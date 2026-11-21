package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Collections;

/** An address of this machine that another machine on the LAN could reach it by. */
final class LanAddress {
    private LanAddress() {}

    static InetAddress first() throws IOException {
        for (NetworkInterface nic : Collections.list(NetworkInterface.getNetworkInterfaces())) {
            for (InetAddress address : Collections.list(nic.getInetAddresses())) {
                if (!address.isLoopbackAddress() && !address.isLinkLocalAddress() && address.getAddress().length == 4) {
                    return address;
                }
            }
        }
        throw new AssertionError("could not judge: this machine has no non-loopback IPv4 address to connect from");
    }
}
