package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public final class KeptModels {
    private static final String PROJECT = "midnight-snackers";
    private static final String MODELS = "field-models";

    private final String root;
    private final List<String> under;

    private KeptModels(String root, List<String> before) {
        this.root = root;
        List<String> all = new ArrayList<>(before);
        all.add(PROJECT);
        all.add(MODELS);
        this.under = Collections.unmodifiableList(all);
    }

    public static Checked<KeptModels> of(String osName, Map<String, String> env, String userHome) {
        if (osName.startsWith("Windows")) {
            String local = env.getOrDefault("LOCALAPPDATA", "").trim();
            return local.isEmpty()
                    ? Checked.rejected("Windows names where a user's programs keep their files in LOCALAPPDATA, and"
                            + " it is not set, so the field models have nowhere to be kept")
                    : Checked.ok(new KeptModels(local, List.of()));
        }
        String home = env.getOrDefault("HOME", "").trim();
        String base = home.isEmpty() ? userHome.trim() : home;
        String xdg = env.getOrDefault("XDG_DATA_HOME", "").trim();
        if (!osName.startsWith("Mac") && xdg.startsWith("/")) {
            return Checked.ok(new KeptModels(xdg, List.of()));
        }
        if (base.isEmpty()) {
            return Checked.rejected("neither HOME nor the JVM names a home directory, so the field models have"
                    + " nowhere to be kept");
        }
        return Checked.ok(new KeptModels(
                base,
                osName.startsWith("Mac") ? List.of("Library", "Application Support") : List.of(".local", "share")));
    }

    public String root() {
        return root;
    }

    public List<String> under() {
        return under;
    }
}
