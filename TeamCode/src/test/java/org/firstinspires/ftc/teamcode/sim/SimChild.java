package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import java.io.BufferedReader;
import java.io.FileDescriptor;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.opmode.OpMode;

public final class SimChild {
    private static final long STREAM_PERIOD_MILLIS = 20;
    private static final Gson GSON = new Gson();

    private SimChild() {}

    public static void main(String[] args) {
        // Whatever the op mode started ends with this JVM, however it ends short of being killed:
        // the server kills the tree then, since a killed JVM runs no hook.
        Runtime.getRuntime()
                .addShutdownHook(new Thread(ProcessTree::killDescendantsOfThisJvm, "sim-child-descendants"));
        PrintStream protocol = new PrintStream(new FileOutputStream(FileDescriptor.out), true, StandardCharsets.UTF_8);
        System.setOut(System.err);
        protocol.println(SimRunStream.hello());
        if (args.length >= 1 && args[0].equals("--list")) {
            protocol.println(GSON.toJson(catalog(args, 1).toJson()));
            System.exit(0);
        }
        if (args.length >= 4 && args[0].equals("--run")) {
            String outcome = run(catalog(args, 4), args[1], Double.parseDouble(args[2]), Paths.get(args[3]), protocol);
            protocol.println(SimRunStream.finished(outcome));
            System.exit(0);
        }
        System.err.println("usage: --list [source...] | --run <op mode name> <seconds> <replay dir> [source...]");
        System.exit(2);
    }

    private static SimCatalog catalog(String[] args, int from) {
        if (args.length <= from) {
            return SimCatalog.discover();
        }
        Class<?>[] sources = new Class<?>[args.length - from];
        for (int i = from; i < args.length; i++) {
            try {
                sources[i - from] = Class.forName(args[i]);
            } catch (ClassNotFoundException e) {
                throw new IllegalArgumentException("no such source of op modes: " + args[i], e);
            }
        }
        return SimCatalog.of(sources);
    }

    private static String run(SimCatalog catalog, String name, double seconds, Path replayDir, PrintStream protocol) {
        Optional<SimCatalog.Entry> entry = catalog.find(name);
        if (entry.isEmpty()) {
            return SimRunStream.Outcome.noOpModeNamed(name);
        }
        OpMode opMode;
        try {
            opMode = entry.get().opMode();
        } catch (RuntimeException e) {
            return SimRunStream.Outcome.couldNotBuild(name, e);
        }
        SimRecording recording = new SimRecording(entry.get().name, entry.get().kind);
        SimDriverStation driverStation = new SimDriverStation();
        Thread driver = new Thread(() -> readDriverStation(driverStation, recording), "sim-driver-station");
        driver.setDaemon(true);
        driver.start();

        Optional<Pose2d> start = driverStation.awaitPlacement();
        if (start.isEmpty()) {
            recording.finish(SimRunStream.Outcome.stopped());
            return recording.outcome();
        }
        Long seed = driverStation.seed();
        SimRobot sim = new SimRobot(seed == null ? SimNoise.NONE : SimNoise.seeded(seed));
        System.err.println("Robot: " + sim.noise());
        sim.setDown(start.get());
        Thread streamer = new Thread(() -> stream(recording, protocol), "sim-stream");
        streamer.setDaemon(true);
        streamer.start();
        protocol.println(SimRunStream.started());
        try {
            SimRunner.record(recording, opMode, sim, seconds, replayDir, driverStation);
        } catch (RuntimeException | Error e) {
        }
        streamer.interrupt();
        try {
            streamer.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        flush(recording, protocol);
        return recording.outcome();
    }

    private static void readDriverStation(SimDriverStation driverStation, SimRecording recording) {
        try (BufferedReader in = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8))) {
            for (String line = in.readLine(); line != null; line = in.readLine()) {
                if (line.isBlank()) {
                    continue;
                }
                try {
                    driverStation.accept(GSON.fromJson(line, JsonObject.class));
                } catch (RuntimeException e) {
                    recording.finish(SimRunStream.Outcome.badDriverStation(e.getMessage()));
                    driverStation.stop();
                    return;
                }
            }
        } catch (IOException e) {
            recording.finish(SimRunStream.Outcome.badDriverStation(e.toString()));
        }
        driverStation.stop();
    }

    private static int streamed = 0;

    private static void stream(SimRecording recording, PrintStream protocol) {
        while (!Thread.currentThread().isInterrupted()) {
            flush(recording, protocol);
            try {
                Thread.sleep(STREAM_PERIOD_MILLIS);
            } catch (InterruptedException e) {
                return;
            }
        }
    }

    private static synchronized void flush(SimRecording recording, PrintStream protocol) {
        for (SimRecording.Tick tick : recording.ticksFrom(streamed)) {
            protocol.println(SimRunStream.tick(tick));
            streamed++;
        }
    }
}
