package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public final class RelayAdmission {
    public static final int ADB_PORT = 5555;

    public record Login(String username, Peer from, boolean approved) {}

    private final List<Login> logins;

    private RelayAdmission(List<Login> logins) {
        this.logins = Collections.unmodifiableList(new ArrayList<>(logins));
    }

    public static RelayAdmission of(List<Login> logins) {
        return new RelayAdmission(logins);
    }

    public Checked<List<String>> admit(Peer peer) {
        List<String> users = approvedFrom(List.of(peer));
        return users.isEmpty() ? Checked.rejected("no approved login came from " + peer.address()) : Checked.ok(users);
    }

    public List<Peer> toClose(List<Peer> open) {
        List<Peer> closing = new ArrayList<>();
        for (Peer peer : open) {
            if (approvedFrom(List.of(peer)).isEmpty() && !closing.contains(peer)) {
                closing.add(peer);
            }
        }
        return Collections.unmodifiableList(closing);
    }

    public List<String> relaying(List<Peer> open) {
        return approvedFrom(open);
    }

    private List<String> approvedFrom(List<Peer> peers) {
        Set<String> users = new LinkedHashSet<>();
        for (Login login : logins) {
            if (login.approved() && peers.contains(login.from())) {
                users.add(login.username());
            }
        }
        return Collections.unmodifiableList(new ArrayList<>(users));
    }

    public static String connectLine(String host, int port) {
        if (host.contains(":")) {
            return "adb connect [" + host + "]:" + port;
        }
        return port == ADB_PORT ? "adb connect " + host : "adb connect " + host + ":" + port;
    }
}
