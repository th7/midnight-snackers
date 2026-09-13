package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.base.OpMode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileDescriptor;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/**
 * The JVM a simulated run happens in. The server launches one per run with the freshly compiled
 * main classes first on the classpath, so the run executes the sources as last saved, every
 * class identity is consistent, static state starts clean, and a hung op mode is a process that
 * can be killed.
 * <ul>
 * <li>{@code --list} prints the catalog as one JSON line.</li>
 * <li>{@code --run <class> <seconds> <replayDir>} runs the op mode, printing each tick as one
 * JSON line as it happens and finally one {@code {"outcome": ...}} line. Standard input is the
 * driver station: one {@link SimDriverStation#accept line} at a time, and the run ends stopped
 * when the input ends.</li>
 * </ul>
 * The protocol owns the real standard output; anything the op mode prints goes to standard
 * error instead, so student output cannot corrupt the stream.
 */
public final class SimChild {
    private static final long STREAM_PERIOD_MILLIS = 20;
    private static final Gson GSON = new Gson();

    private SimChild() {
    }

    public static void main(String[] args) {
        PrintStream protocol = new PrintStream(new FileOutputStream(FileDescriptor.out), true, StandardCharsets.UTF_8);
        System.setOut(System.err);
        if (args.length >= 1 && args[0].equals("--list")) {
            protocol.println(SimReplayPage.toLine(SimCatalog.discover().toJson()));
            System.exit(0);
        }
        if (args.length == 4 && args[0].equals("--run")) {
            String outcome = run(args[1], Double.parseDouble(args[2]), Paths.get(args[3]), protocol);
            JsonObject last = new JsonObject();
            last.addProperty("outcome", outcome);
            protocol.println(SimReplayPage.toLine(last));
            System.exit(0);
        }
        System.err.println("usage: --list | --run <op mode class> <seconds> <replay dir>");
        System.exit(2);
    }

    private static String run(String className, double seconds, Path replayDir, PrintStream protocol) {
        OpMode opMode;
        try {
            opMode = Class.forName(className).asSubclass(OpMode.class).getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException | ClassCastException e) {
            return "could not build " + className + ": " + e;
        }
        SimRecording recording = new SimRecording(SimRunner.nameOf(opMode), SimCatalog.kindOf(opMode.getClass()));
        SimDriverStation driverStation = new SimDriverStation();
        Thread driver = new Thread(() -> readDriverStation(driverStation, recording), "sim-driver-station");
        driver.setDaemon(true);
        driver.start();
        Thread streamer = new Thread(() -> stream(recording, protocol), "sim-stream");
        streamer.setDaemon(true);
        streamer.start();
        try {
            SimRunner.record(recording, opMode, new SimRobot(), seconds, replayDir, driverStation);
        } catch (RuntimeException | Error e) {
            // the outcome is on the recording
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

    /**
     * Standard input, line by line, into the driver station. A line that is not a driver station
     * line is a bug in whoever is driving, so it ends the run with that as the outcome rather than
     * letting the robot carry on as if nothing had been said. The end of the input ends the run too.
     */
    private static void readDriverStation(SimDriverStation driverStation, SimRecording recording) {
        try (BufferedReader in = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8))) {
            for (String line = in.readLine(); line != null; line = in.readLine()) {
                if (line.isBlank()) {
                    continue;
                }
                try {
                    driverStation.accept(GSON.fromJson(line, JsonObject.class));
                } catch (RuntimeException e) {
                    recording.finish("could not read the driver station: " + e.getMessage());
                    driverStation.stop();
                    return;
                }
            }
        } catch (IOException e) {
            recording.finish("could not read the driver station: " + e);
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
            protocol.println(SimReplayPage.toLine(SimReplayPage.tickJson(tick)));
            streamed++;
        }
    }

    // --- the parent's side ---

    /**
     * Starts a child with {@code classpathFirst} ahead of this JVM's own classpath.
     */
    public static Process launch(List<Path> classpathFirst, String... args) {
        List<String> classpath = new ArrayList<>();
        for (Path path : classpathFirst) {
            classpath.add(path.toAbsolutePath().toString());
        }
        classpath.add(System.getProperty("java.class.path"));
        List<String> command = new ArrayList<>();
        command.add(Paths.get(System.getProperty("java.home"), "bin", "java").toString());
        command.add("-cp");
        command.add(String.join(File.pathSeparator, classpath));
        command.add(SimChild.class.getName());
        command.addAll(List.of(args));
        try {
            return new ProcessBuilder(command).start();
        } catch (IOException e) {
            throw new UncheckedIOException("could not start the simulation child: " + command.get(0), e);
        }
    }
}
