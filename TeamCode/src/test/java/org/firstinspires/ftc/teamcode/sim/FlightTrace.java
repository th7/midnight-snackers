package org.firstinspires.ftc.teamcode.sim;

import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public final class FlightTrace {
    public static final double TOLERANCE = 1e-6;

    private static final Store STORE = new OnDiskStore();

    private final List<String> lines = new ArrayList<>();

    public void sample(double seconds, List<Object> values) {
        StringBuilder line = new StringBuilder(String.format(Locale.ROOT, "%.3f", seconds));
        for (Object value : values) {
            line.append(' ').append(value instanceof Double d ? format(d) : value);
        }
        lines.add(line.toString());
    }

    public static String format(double value) {
        String text = String.format(Locale.ROOT, "%.9f", value);
        return text.equals("-0.000000000") ? "0.000000000" : text;
    }

    public void assertMatches(String name) {
        Path path = SimTrace.DIR.resolve(name + ".trace");
        if (Boolean.getBoolean(SimTrace.REGENERATE)) {
            RepoFile.write(path, (String.join("\n", lines) + "\n").getBytes(StandardCharsets.UTF_8));
            throw new AssertionError("regenerated " + path.toAbsolutePath() + "; re-run without -D"
                    + SimTrace.REGENERATE + " to check it");
        }
        if (!STORE.isFile(path)) {
            throw new AssertionError("no trace at " + path.toAbsolutePath() + "; write it with"
                    + " ./gradlew :TeamCode:testDebugUnitTest --rerun -D" + SimTrace.REGENERATE
                    + "=true --tests '*FlightTraceTest*'");
        }
        List<String> expected = List.of(new String(STORE.readIfThere(path).orElseThrow(), StandardCharsets.UTF_8)
                .strip()
                .split("\n"));
        if (expected.size() != lines.size()) {
            throw new AssertionError(path + " has " + expected.size() + " samples, the run " + lines.size());
        }
        for (int i = 0; i < expected.size(); i++) {
            if (!matches(expected.get(i), lines.get(i))) {
                throw new AssertionError(
                        path + ", sample " + i + ":\n  expected " + expected.get(i) + "\n  actual   " + lines.get(i));
            }
        }
    }

    private static boolean matches(String expected, String actual) {
        String[] want = expected.trim().split("\\s+"), got = actual.trim().split("\\s+");
        if (want.length != got.length) {
            return false;
        }
        for (int i = 0; i < want.length; i++) {
            if (!want[i].equals(got[i]) && !(numeric(want[i]) && numeric(got[i]) && close(want[i], got[i]))) {
                return false;
            }
        }
        return true;
    }

    private static boolean numeric(String token) {
        return token.matches("-?\\d+\\.\\d+");
    }

    private static boolean close(String a, String b) {
        return Math.abs(Double.parseDouble(a) - Double.parseDouble(b)) <= TOLERANCE;
    }
}
