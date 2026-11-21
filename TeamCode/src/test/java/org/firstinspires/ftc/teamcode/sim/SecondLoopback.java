package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;

final class SecondLoopback {
    private static final String ADDRESS = "127.0.0.2";

    private SecondLoopback() {}

    static InetAddress address() throws IOException {
        InetAddress second = InetAddress.getByName(ADDRESS);
        try (Socket probe = new Socket()) {
            probe.bind(new InetSocketAddress(second, 0));
        } catch (IOException e) {
            throw new AssertionError(
                    "could not judge: this machine cannot use " + ADDRESS
                            + " as a second address of its own, as Linux and Windows can and macOS cannot without an alias",
                    e);
        }
        return second;
    }
}
