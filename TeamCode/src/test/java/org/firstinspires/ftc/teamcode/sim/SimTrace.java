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

/**
 * What the robot code wrote to the four wheel motors, tick by tick, kept in a file and compared
 * against the next run: a <b>golden trace</b>. A run of the simulator is the same tick for tick
 * every time it is made, so the powers are too, and a change in how any intent reaches the wheels
 * — the mixing, the saturation, the feedforward, the voltage — shows up here as a diff with a tick
 * number on it, rather than as a robot that drives oddly at a competition.
 *
 * <p>It says only what the robot <em>commanded</em>, never where the robot went. The pose is the
 * physics engine's answer to the powers, so pinning it would pin dyn4j as well, and a rigid-body
 * simulation compounds a difference in its last digit into a visibly different path given enough
 * ticks. The powers are what our code decides, and they are what this holds.
 *
 * <p>For the same reason a trace is only the first {@link #TICKS} ticks of a run: every intent and
 * every branch of the command path runs many times over within that, while a difference too small
 * to be ours has not yet had time to grow into one.
 *
 * <p>A trace is a change detector, not a judgement: it cannot tell a regression from a change
 * somebody meant. When a diff is intended, look at it, then regenerate:
 *
 * <pre>
 * ./gradlew :TeamCode:testDebugUnitTest --rerun -DsimTrace.regenerate=true --tests '*WheelPowerTraceTest*'
 * </pre>
 *
 * That run rewrites the files and then fails, because a run that wrote the answer down has not
 * checked it. Re-run without the flag to see it pass.
 */
public final class SimTrace {
    /** How many ticks of a run a trace holds; see the class comment for why it is a prefix. */
    public static final int TICKS = 100;

    /**
     * How far a power may drift from the file before it counts as a change. The file holds six
     * decimals, so this is a little over half of the last digit it prints: enough that a rounding
     * cliff cannot fail a run, and far below any difference our own code could make, the smallest
     * of which is the drive's minimum power of 0.1.
     */
    public static final double TOLERANCE = 2e-6;

    /** Where the trace files live, relative to the module directory the unit tests run in. */
    public static final Path DIR =
            Paths.get("src", "test", "resources", "org", "firstinspires", "ftc", "teamcode", "sim", "traces");

    /** The system property that rewrites the files instead of checking them. */
    public static final String REGENERATE = "simTrace.regenerate";

    private SimTrace() {}

    /** One tick's command: the run's time, and the power on each of the four wheels. */
    private static final class Line {
        final double seconds;
        final double[] powers;

        Line(double seconds, double[] powers) {
            this.seconds = seconds;
            this.powers = powers;
        }
    }

    /**
     * Checks {@code recording}'s wheel powers against the trace file named {@code name}, or
     * rewrites that file when {@link #REGENERATE} is set.
     *
     * @throws AssertionError if they differ, if the file is missing, or if this run wrote it
     */
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

    /**
     * The recording's first {@link #TICKS} ticks, or all of them when the run was shorter — an auto
     * that finishes sooner is traced end to end. How many there are is part of what a trace says,
     * so {@link #compare} fails on the count before it looks at any power.
     */
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

    /** The wheels a tick's powers are in, in {@link SimRecording.Tick#wheelPowers}' order. */
    private static final String[] WHEELS = {"leftFront", "rightFront", "leftBack", "rightBack"};

    private static String format(Line line) {
        StringBuilder text = new StringBuilder(String.format(Locale.ROOT, "%.3f", line.seconds));
        for (double power : line.powers) {
            text.append(' ').append(format(power));
        }
        return text.toString();
    }

    /** A power as the file holds it: six decimals, and never "-0.000000", which is just zero. */
    private static String format(double power) {
        String text = String.format(Locale.ROOT, "%.6f", power);
        return text.equals("-0.000000") ? "0.000000" : text;
    }
}
