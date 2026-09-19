package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public final class SimTrace {
    public static final int TICKS = 100;

    public static final double TOLERANCE = 2e-6;

    public static final Path DIR =
            Paths.get("src", "test", "resources", "org", "firstinspires", "ftc", "teamcode", "sim", "traces");

    public static final String REGENERATE = "simTrace.regenerate";

    private SimTrace() {}

    private static final class Line {
        final double seconds;
        final double[] powers;

        Line(double seconds, double[] powers) {
            this.seconds = seconds;
            this.powers = powers;
        }
    }

    public static void assertMatches(String name, SimRecording recording) {
        List<Line> actual = linesOf(recording);
        Path path = DIR.resolve(name + ".trace");

        if (Boolean.getBoolean(REGENERATE)) {
            write(path, name, actual);
            throw new AssertionError(String.format(
                    "regenerated %s (%d ticks). A run that wrote the trace has not checked it: "
                            + "read the diff, then re-run without -D%s to check it.",
                    path.toAbsolutePath(), actual.size(), REGENERATE));
        }

        if (!Files.isRegularFile(path)) {
            throw new AssertionError(String.format(
                    "no golden trace at %s. Nothing is being checked. Generate it with:%n"
                            + "  ./gradlew :TeamCode:testDebugUnitTest --rerun -D%s=true --tests '*WheelPowerTraceTest*'",
                    path.toAbsolutePath(), REGENERATE));
        }

        List<Line> expected = read(path);
        compare(path, expected, actual);
    }

    private static List<Line> linesOf(SimRecording recording) {
        List<SimRecording.Tick> ticks = recording.ticks();
        List<Line> lines = new ArrayList<>();
        for (int i = 0; i < Math.min(TICKS, ticks.size()); i++) {
            SimRecording.Tick tick = ticks.get(i);
            lines.add(new Line(tick.seconds, tick.wheelPowers));
        }
        return lines;
    }

    private static void compare(Path path, List<Line> expected, List<Line> actual) {
        if (expected.size() != actual.size()) {
            throw new AssertionError(String.format(
                    "%s holds %d ticks but the run gave %d. The run is a different length than when the "
                            + "trace was written, so there is nothing to compare tick for tick: find out "
                            + "why it now ends where it does.",
                    path, expected.size(), actual.size()));
        }
        for (int i = 0; i < expected.size(); i++) {
            Line want = expected.get(i);
            Line got = actual.get(i);
            for (int wheel = 0; wheel < WHEELS.length; wheel++) {
                double a = want.powers[wheel];
                double b = got.powers[wheel];
                if (Math.abs(a - b) > TOLERANCE) {
                    throw new AssertionError(String.format(
                            "%s, tick %d (%.2fs): %s was %s, now %s.%n"
                                    + "  expected %s%n"
                                    + "  actual   %s%n"
                                    + "The robot code commands the wheels differently than when this trace was "
                                    + "written. If that is what you meant, regenerate with -D%s=true.",
                            path,
                            i,
                            got.seconds,
                            WHEELS[wheel],
                            format(a),
                            format(b),
                            format(want),
                            format(got),
                            REGENERATE));
                }
            }
        }
    }

    private static void write(Path path, String name, List<Line> lines) {
        StringBuilder out = new StringBuilder();
        out.append("# ").append(name).append(": what the robot code wrote to the wheels, tick by tick.\n");
        out.append("# Written by SimTrace; regenerate with -D")
                .append(REGENERATE)
                .append("=true.\n");
        out.append("# seconds ").append(String.join(" ", WHEELS)).append('\n');
        for (Line line : lines) {
            out.append(format(line)).append('\n');
        }
        try {
            Files.createDirectories(path.getParent());
            Files.write(path, out.toString().getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new UncheckedIOException("could not write " + path.toAbsolutePath(), e);
        }
    }

    private static List<Line> read(Path path) {
        List<String> raw;
        try {
            raw = Files.readAllLines(path, StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException("could not read " + path.toAbsolutePath(), e);
        }
        List<Line> lines = new ArrayList<>();
        for (String text : raw) {
            if (text.isBlank() || text.startsWith("#")) {
                continue;
            }
            String[] fields = text.trim().split("\\s+");
            if (fields.length != WHEELS.length + 1) {
                throw new AssertionError(String.format(
                        "%s is not a trace: expected %d numbers on a line, found %d in '%s'",
                        path, WHEELS.length + 1, fields.length, text));
            }
            double[] powers = new double[WHEELS.length];
            for (int i = 0; i < powers.length; i++) {
                powers[i] = Double.parseDouble(fields[i + 1]);
            }
            lines.add(new Line(Double.parseDouble(fields[0]), powers));
        }
        return lines;
    }

    private static final String[] WHEELS = {"leftFront", "rightFront", "leftBack", "rightBack"};

    private static String format(Line line) {
        StringBuilder text = new StringBuilder(String.format(Locale.ROOT, "%.3f", line.seconds));
        for (double power : line.powers) {
            text.append(' ').append(format(power));
        }
        return text.toString();
    }

    private static String format(double power) {
        String text = String.format(Locale.ROOT, "%.6f", power);
        return text.equals("-0.000000") ? "0.000000" : text;
    }
}
