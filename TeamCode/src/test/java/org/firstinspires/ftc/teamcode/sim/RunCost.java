package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

public final class RunCost {
    public enum Stage {
        OP_MODE("op mode"),
        TICK("tick"),
        PHYSICS("physics"),
        WRITE("write"),
        READ("read"),
        SERVE("serve");

        final String named;

        Stage(String named) {
            this.named = named;
        }

        boolean simulating() {
            return this == OP_MODE || this == TICK || this == PHYSICS;
        }
    }

    static final String A_BROWSER_TAKES = "gzip, deflate, br";

    static final double AUTO_POLL_SECONDS = 0.1;

    static final double TELEOP_POLL_SECONDS = 0.05;

    private final SimRecording recording;
    private final double simulatedSeconds;
    private final Map<Stage, Long> nanos = new EnumMap<>(Stage.class);
    int polls;

    long lineBytes;
    long pollBytes;
    long pollBytesSent;
    long runBytes;
    long runBytesSent;
    long pageBytes;

    private RunCost(SimRecording recording, double simulatedSeconds, SimRunner.Meter meter) {
        this.recording = recording;
        this.simulatedSeconds = simulatedSeconds;
        nanos.put(Stage.OP_MODE, meter.opModeNanos);
        nanos.put(Stage.TICK, meter.tickNanos);
        nanos.put(Stage.PHYSICS, meter.physicsNanos);
    }

    public static RunCost of(SimCatalog.Entry entry, double seconds, Path outputDir) {
        SimRecording recording = new SimRecording(entry.name, entry.kind);
        SimRobot sim = new SimRobot();
        SimRunner.Meter meter = new SimRunner.Meter();
        try {
            SimRunner.record(
                    recording,
                    entry.opMode(),
                    sim,
                    seconds,
                    outputDir,
                    new SimDriverStation(),
                    SimRunner.Pace.FASTEST,
                    meter);
        } catch (RuntimeException | AssertionError endedAsTheRecordingSays) {
        }
        RunCost cost = new RunCost(recording, sim.nanoTime() / 1e9, meter);
        cost.carry(entry.kind.equals(SimCatalog.TELEOP) ? TELEOP_POLL_SECONDS : AUTO_POLL_SECONDS);
        return cost;
    }

    public SimRecording recording() {
        return recording;
    }

    private void carry(double pollSeconds) {
        List<String> lines = new ArrayList<>();
        long started = System.nanoTime();
        for (SimRecording.Tick tick : recording.ticks()) {
            lines.add(SimRunStream.tick(tick));
        }
        nanos.put(Stage.WRITE, System.nanoTime() - started);
        for (String line : lines) {
            lineBytes += bytes(line);
        }

        List<SimRunStream.TickLine> kept = new ArrayList<>();
        SimRunStream.Listener bench = new SimRunStream.Listener() {
            @Override
            public void started() {}

            @Override
            public void tick(SimRunStream.TickLine tick) {
                kept.add(tick);
            }

            @Override
            public void finished(String outcome) {}
        };
        started = System.nanoTime();
        for (String line : lines) {
            SimRunStream.accept(line, bench);
        }
        nanos.put(Stage.READ, System.nanoTime() - started);

        long serving = 0;
        int from = 0;
        while (from < kept.size()) {
            long window = pollWindow(kept.get(from), pollSeconds);
            int to = from + 1;
            while (to < kept.size() && pollWindow(kept.get(to), pollSeconds) == window) {
                to++;
            }
            started = System.nanoTime();
            String body = SimReplayPage.update(keptUpTo(kept, to), from);
            byte[] sent = sentToABrowser(body);
            serving += System.nanoTime() - started;
            pollBytes += bytes(body);
            pollBytesSent += sent.length;
            polls++;
            from = to;
        }
        nanos.put(Stage.SERVE, serving);

        String whole = SimReplayPage.update(keptUpTo(kept, kept.size()), 0);
        runBytes = bytes(whole);
        runBytesSent = sentToABrowser(whole).length;
        pageBytes = bytes(SimReplayPage.written(recording));
    }

    private static long pollWindow(SimRunStream.TickLine tick, double pollSeconds) {
        return (long) Math.floor(tick.seconds() / pollSeconds);
    }

    private SimReplayPage.Source keptUpTo(List<SimRunStream.TickLine> kept, int to) {
        return new SimReplayPage.Source() {
            @Override
            public String name() {
                return recording.name();
            }

            @Override
            public String kind() {
                return recording.kind();
            }

            @Override
            public String ticksJson(int from) {
                return SimRunStream.TickLine.array(kept.subList(Math.min(from, to), to));
            }

            @Override
            public String outcome() {
                return to == kept.size() ? recording.outcome() : null;
            }

            @Override
            public JsonElement match() {
                return JsonNull.INSTANCE;
            }
        };
    }

    private static byte[] sentToABrowser(String json) {
        return TinyHttpServer.sent(Response.json(json), Map.of("accept-encoding", A_BROWSER_TAKES)).body;
    }

    private static long bytes(String text) {
        return text.getBytes(StandardCharsets.UTF_8).length;
    }

    public String report() {
        int ticks = recording.ticks().size();
        StringBuilder out = new StringBuilder(String.format(
                "what a run costs: %s, the exact robot, %d ticks over %.2f s simulated (%s)%n",
                recording.name(), ticks, simulatedSeconds, recording.outcome()));
        if (ticks == 0) {
            return out.append("  could not judge: the run made no ticks").toString();
        }
        out.append("  ms per tick\n");
        long simulating = 0;
        for (Stage stage : Stage.values()) {
            long spent = nanos.get(stage);
            if (stage.simulating()) {
                simulating += spent;
            }
            out.append(String.format("    %-8s %8.3f%n", stage.named, spent / 1e6 / ticks));
        }
        out.append(String.format(
                "  simulated %.0fx faster than real time%n", simulatedSeconds / Math.max(simulating / 1e9, 1e-9)));
        out.append("  bytes\n");
        out.append(String.format("    %-8s %10d%n", "a tick", lineBytes / ticks));
        out.append(String.format(
                "    %-8s %10d  sent %d, %d polls%n", "a poll", pollBytes / polls, pollBytesSent / polls, polls));
        out.append(String.format("    %-8s %10d  sent %d%n", "the run", runBytes, runBytesSent));
        out.append(String.format("    %-8s %10d%n", "replay", pageBytes));
        return out.append("  a time is this machine's: printed, never judged").toString();
    }

    public static void main(String[] args) {
        SimCatalog catalog = SimCatalog.discover();
        SimBench.Waits game = SimBench.Waits.ofTheBench();
        List<SimCatalog.Entry> entries = new ArrayList<>();
        if (args.length == 0) {
            entries.addAll(catalog.entries());
        } else {
            entries.add(catalog.find(args[0])
                    .orElseThrow(() -> new IllegalArgumentException("no op mode named " + args[0])));
        }
        for (SimCatalog.Entry entry : entries) {
            double seconds = args.length >= 2
                    ? Double.parseDouble(args[1])
                    : entry.kind.equals(SimCatalog.TELEOP) ? game.teleOpPeriod : game.autonomousPeriod;
            System.out.println(of(entry, seconds, SimRunner.DEFAULT_OUTPUT_DIR).report());
            System.out.println();
        }
    }
}
