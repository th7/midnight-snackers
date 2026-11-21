package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public final class GatewayPorts {
    public static final String ADMIN_PORT_ENV = "ROBOT_GATEWAY_ADMIN_PORT";
    public static final String USER_PORT_ENV = "ROBOT_GATEWAY_USER_PORT";
    public static final int DEFAULT_ADMIN_PORT = 21987;
    public static final int DEFAULT_USER_PORT = 21986;

    private enum ReadByTheCodingServer {
        CODING_ADMIN_PORT(ADMIN_PORT_ENV),
        CODING_USER_PORT(USER_PORT_ENV);

        private final String now;

        ReadByTheCodingServer(String now) {
            this.now = now;
        }
    }

    private final int admin;
    private final int users;

    private GatewayPorts(int admin, int users) {
        this.admin = admin;
        this.users = users;
    }

    public static Checked<GatewayPorts> from(Map<String, String> env) {
        List<String> refused = new ArrayList<>();
        for (ReadByTheCodingServer renamed : ReadByTheCodingServer.values()) {
            String old = renamed.name();
            String now = renamed.now;
            String value = env.getOrDefault(old, "").trim();
            if (!value.isEmpty()) {
                refused.add(old + " is no longer read: the coding server is now the robot gateway, so set " + now + "="
                        + value + " instead and unset " + old);
            }
        }
        if (!refused.isEmpty()) {
            return Checked.rejected(String.join("; ", refused));
        }
        return PortSetting.of(env, ADMIN_PORT_ENV, DEFAULT_ADMIN_PORT)
                .then(admin -> PortSetting.of(env, USER_PORT_ENV, DEFAULT_USER_PORT)
                        .map(users -> new GatewayPorts(admin, users)));
    }

    public int admin() {
        return admin;
    }

    public int users() {
        return users;
    }
}
