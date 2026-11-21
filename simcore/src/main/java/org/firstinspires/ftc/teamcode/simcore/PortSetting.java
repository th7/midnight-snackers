package org.firstinspires.ftc.teamcode.simcore;

import java.util.Map;

public final class PortSetting {
    public static final int HIGHEST = 65535;

    private static final int MOST_DIGITS = 5;

    private PortSetting() {}

    public static Checked<Integer> of(Map<String, String> env, String name, int fallback) {
        String value = env.getOrDefault(name, "");
        String written = value.trim();
        if (written.isEmpty()) {
            return Checked.ok(fallback);
        }
        return parsed(written)
                .then(port -> port <= HIGHEST ? Checked.ok(port) : Checked.rejected(""))
                .fold(
                        Checked::ok,
                        unused -> Checked.rejected(
                                name + " must be a port number from 0 to " + HIGHEST + ", not '" + value + "'"));
    }

    private static Checked<Integer> parsed(String written) {
        if (written.length() > MOST_DIGITS) {
            return Checked.rejected("");
        }
        char[] digits = written.toCharArray();
        int port = 0;
        for (char c : digits) {
            int digit = Character.digit(c, 10);
            if (digit < 0) {
                return Checked.rejected("");
            }
            port = port * 10 + digit;
        }
        return Checked.ok(port);
    }
}
