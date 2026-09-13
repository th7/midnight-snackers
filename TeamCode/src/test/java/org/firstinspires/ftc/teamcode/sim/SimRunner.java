package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.base.OpMode;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs an op mode against a {@link SimRobot} in real time, the way the robot controller would:
 * init, start, then loop. An auto loops until its plan is done, and failing that within its
 * timeout is an error; a TeleOp loops on the {@link SimDriverStation}'s gamepads until the driver
 * presses Stop or its time is up. Every run, finished or not, leaves a replay page named after
 * the op mode in the output directory.
 * <p>
 * To watch an auto run as it happens, set {@code SIM_LIVE} to a port and open that port in a browser:
 * <pre>
 * SIM_LIVE=8765 ./gradlew :TeamCode:testDebugUnitTest --rerun --tests '*ForwardLeftBackwardRightSimTest*'
 * </pre>
 * The page at http://localhost:8765/ follows the run and stays up until it has shown the end
 * (at most {@link #LIVE_HOLD_SECONDS} after the run finishes).
 * <p>
 * Real time because Road Runner actions and {@code Step} timers read the system clock.
 */
public final class SimRunner {
    public static final Path DEFAULT_OUTPUT_DIR = Paths.get("build", "sim");
    public static final String LIVE_PORT_ENV = "SIM_LIVE";
    public static final double LIVE_HOLD_SECONDS = 30;
    private static final long TICK_MILLIS = 5;
    /**
     * Poses are kept for every loop, but the dashboard drawings (which repeat the whole planned
     * path and pose history each loop) only this often, so a replay stays around a megabyte.
     */
    private static final double DRAWING_PERIOD_SECONDS = 0.05;

    private SimRunner() {
    }

    /**
     * @return what happened, loop by loop
     * @throws AssertionError if the plan is not done within {@code timeoutSeconds}
     */
    public static SimRecording run(AutoOp opMode, SimRobot sim, double timeoutSeconds) {
        return run(opMode, sim, timeoutSeconds, DEFAULT_OUTPUT_DIR, livePortFromEnvironment());
    }

    public static SimRecording run(AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir) {
        return run(opMode, sim, timeoutSeconds, outputDir, null);
    }

    /** Runs a catalog entry's auto, with the recording named as the driver station names the op mode. */
    public static SimRecording run(SimCatalog.Entry entry, SimRobot sim, double timeoutSeconds) {
        return run(entry, sim, timeoutSeconds, DEFAULT_OUTPUT_DIR, livePortFromEnvironment());
    }

    public static SimRecording run(SimCatalog.Entry entry, SimRobot sim, double timeoutSeconds, Path outputDir) {
        return run(entry, sim, timeoutSeconds, outputDir, null);
    }

    private static SimRecording run(SimCatalog.Entry entry, SimRobot sim, double timeoutSeconds, Path outputDir, Integer livePort) {
        if (!entry.kind.equals(SimCatalog.AUTO)) {
            throw new IllegalArgumentException(entry.name + " is a TeleOp; run it from a driver station with record()");
        }
        return run(new SimRecording(entry.name, entry.kind), (AutoOp) entry.opMode(), sim, timeoutSeconds, outputDir, livePort);
    }

    /**
     * @param livePort serve a live view on this port while the run is in progress, or null for none
     */
    public static SimRecording run(AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir, Integer livePort) {
        return run(new SimRecording(nameOf(opMode)), opMode, sim, timeoutSeconds, outputDir, livePort);
    }

    private static SimRecording run(SimRecording recording, AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir, Integer livePort) {
        SimLiveServer live = livePort == null ? null : SimLiveServer.start(recording, livePort);
        if (live != null) {
            System.out.println("Simulation live view: " + live.url());
        }
        try {
            return record(recording, opMode, sim, timeoutSeconds, outputDir);
        } finally {
            if (live != null) {
                live.awaitViewerSawOutcome(LIVE_HOLD_SECONDS);
                live.stop();
            }
        }
    }

    /**
     * Run an auto into a recording the caller already holds, so it can be watched while this is in
     * progress. The recording always ends with an outcome and a replay page, even when this throws.
     */
    public static SimRecording record(SimRecording recording, AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir) {
        return record(recording, opMode, sim, timeoutSeconds, outputDir, new SimDriverStation());
    }

    /**
     * Run any op mode into a recording the caller already holds, driven from {@code driverStation}.
     * An auto ends done when its plan is, or fails by timing out after {@code seconds}; a TeleOp
     * ends done when {@code seconds} are up. Either ends stopped when the driver station says so.
     * The recording always ends with an outcome and a replay page, even when this throws.
     */
    public static SimRecording record(SimRecording recording, OpMode opMode, SimRobot sim, double seconds, Path outputDir,
                                      SimDriverStation driverStation) {
        try {
            loopUntilDone(opMode, sim, seconds, recording, driverStation);
            recording.finish("done");
        } catch (RuntimeException | Error e) {
            recording.finish("failed: " + e);
            throw e;
        } finally {
            Path page = outputDir.resolve(fileName(recording.name()) + ".html");
            SimReplayPage.write(recording, page);
            System.out.println("Simulation replay: " + page.toAbsolutePath());
        }
        return recording;
    }

    private static Integer livePortFromEnvironment() {
        String value = System.getenv(LIVE_PORT_ENV);
        if (value == null || value.isBlank()) {
            return null;
        }
        try {
            return Integer.parseInt(value.trim());
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(LIVE_PORT_ENV + " must be a port number, not '" + value + "'", e);
        }
    }

    private static void loopUntilDone(OpMode opMode, SimRobot sim, double seconds, SimRecording recording,
                                      SimDriverStation driverStation) {
        AutoOp auto = opMode instanceof AutoOp ? (AutoOp) opMode : null;
        opMode.useHardware(sim.hardware());
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        opMode.start();

        int packetsSeen = sim.dashboard.packets.size();
        double lastDrawingAt = Double.NEGATIVE_INFINITY;
        long startedAt = System.nanoTime();
        long lastTickAt = startedAt;
        while (true) {
            if (driverStation.stopRequested()) {
                recording.finish("stopped");
                return;
            }
            if (auto != null && auto.done()) {
                return;
            }
            long now = System.nanoTime();
            double elapsed = seconds(now - startedAt);
            if (elapsed > seconds) {
                if (auto == null) {
                    return; // a TeleOp's time is simply up
                }
                recording.finish(String.format("timed out after %.1fs", seconds));
                throw new AssertionError(String.format(
                        "op mode still running after %.1fs; current step: %s; true pose: %s",
                        seconds, auto.currentStep(), sim.pose()));
            }
            sim.step(seconds(now - lastTickAt));
            lastTickAt = now;
            driverStation.applyTo(opMode.gamepad1, opMode.gamepad2);
            opMode.loop();

            List<TelemetryPacket> allPackets = sim.dashboard.packets;
            List<TelemetryPacket> thisLoop = new ArrayList<>();
            if (allPackets.size() > packetsSeen && elapsed - lastDrawingAt >= DRAWING_PERIOD_SECONDS) {
                thisLoop.addAll(allPackets.subList(packetsSeen, allPackets.size()));
                lastDrawingAt = elapsed;
            }
            packetsSeen = allPackets.size();
            recording.add(new SimRecording.Tick(elapsed, sim.pose(), auto != null ? auto.currentStep() : "",
                    new double[]{sim.leftFront.power, sim.rightFront.power, sim.leftBack.power, sim.rightBack.power},
                    thisLoop,
                    auto == null ? driverStation.state(1) : null,
                    auto == null ? driverStation.state(2) : null));
            sleep();
        }
    }

    /** The recording's name as a file name: anything a path could not hold becomes an underscore. */
    static String fileName(String name) {
        return name.replaceAll("[^A-Za-z0-9._ -]", "_");
    }

    /**
     * The op mode's class name, or for an anonymous subclass the nearest named class.
     */
    static String nameOf(OpMode opMode) {
        Class<?> type = opMode.getClass();
        while (type.getSimpleName().isEmpty()) {
            type = type.getSuperclass();
        }
        return type.getSimpleName();
    }

    private static double seconds(long nanos) {
        return nanos / 1_000_000_000d;
    }

    private static void sleep() {
        try {
            Thread.sleep(TICK_MILLIS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
        }
    }
}
