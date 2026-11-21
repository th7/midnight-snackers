package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.nio.file.Paths;

public final class SimTrace {
    public static final Path DIR =
            Paths.get("src", "test", "resources", "org", "firstinspires", "ftc", "teamcode", "sim", "traces");

    public static final String REGENERATE = "simTrace.regenerate";

    private SimTrace() {}
}
