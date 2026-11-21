package org.firstinspires.ftc.teamcode.simcore;

import java.util.Collections;
import java.util.Map;

public sealed interface WhereTheHostIs permits WhereTheHostIs.Unsaid, WhereTheHostIs.At {
    String ENV = "ROBOT_GATEWAY";

    static Checked<WhereTheHostIs> from(Map<String, String> env) {
        String value = env.getOrDefault(ENV, "");
        String written = value.trim();
        if (written.isEmpty()) {
            return Checked.ok(new Unsaid());
        }
        StringBuilder host = new StringBuilder();
        StringBuilder port = new StringBuilder();
        boolean pastColon = false;
        boolean wellFormed = true;
        char[] chars = written.toCharArray();
        for (char c : chars) {
            if (c == ':' && !pastColon) {
                pastColon = true;
            } else if (pastColon) {
                port.append(c);
            } else if (Character.isLetterOrDigit(c) || c == '.' || c == '-') {
                host.append(c);
            } else {
                wellFormed = false;
            }
        }
        String refused = ENV + " is the host's LAN address, with a port only where the host's user listener is not on "
                + GatewayPorts.DEFAULT_USER_PORT + ", such as 192.168.1.20 or 192.168.1.20:"
                + GatewayPorts.DEFAULT_USER_PORT
                + ", not '" + value + "'";
        if (!wellFormed || host.length() == 0 || (pastColon && port.length() == 0)) {
            return Checked.rejected(refused);
        }
        String named = host.toString();
        return PortSetting.of(Collections.singletonMap(ENV, port.toString()), ENV, GatewayPorts.DEFAULT_USER_PORT)
                .fold(n -> Checked.ok(new At(named, n)), rule -> Checked.rejected(refused));
    }

    record Unsaid() implements WhereTheHostIs {}

    final class At implements WhereTheHostIs {
        private final String host;
        private final int port;

        private At(String host, int port) {
            this.host = host;
            this.port = port;
        }

        public String url() {
            return "http://" + host + ":" + port + "/";
        }

        @Override
        public String toString() {
            return host + ":" + port;
        }
    }
}
