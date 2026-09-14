package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.base.OpMode;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;

/**
 * Runs an op mode against a {@link SimRobot} the way the robot controller would: init, start, then
 * loop. Time is the simulation's: the world moves one loop period between one loop and the next,
 * whatever the machine is doing, and the op mode's clock is the world's, so a run is the same tick
 * for tick every time it is made. The loop period is {@link #LOOP_SECONDS} exactly, unless the
 * robot's {@link SimNoise noise} varies it the way a real loop varies, and then it is the same
 * sequence of periods for the same seed. An auto loops until its plan is done, and failing that within
 * its timeout is an error; a TeleOp loops on the {@link SimDriverStation}'s gamepads until the
 * driver presses Stop or its time is up. Every run, finished or not, leaves a replay page named
 * after the op mode in the output directory.
 * <p>
 * A run's {@link Pace} says whether to hold it to real time. A run nobody watches goes as fast as
 * the machine can; one driven from a driver station or watched live keeps to real time, so it can
 * be driven and followed. To watch an auto run as it happens, set {@code SIM_LIVE} to a port and
 * open that port in a browser:
 * <pre>
 * SIM_LIVE=8765 ./gradlew :TeamCode:testDebugUnitTest --rerun --tests '*ForwardLeftBackwardRightSimTest*'
 * </pre>
 * The page at http://localhost:8765/ follows the run and stays up until it has shown the end
 * (at most {@link #LIVE_HOLD_SECONDS} after the run finishes).
 */
public final class SimRunner {
    /** Whether a run keeps to real time or goes as fast as it can; either way it is the same run. */
    public enum Pace {
        /** One loop period of wall clock per loop, at least: for a run somebody drives or watches. */
        REAL_TIME,
        /** No waiting: for a run nobody is watching, such as a test. */
        FASTEST
    }

    public static final Path DEFAULT_OUTPUT_DIR = Paths.get("build", "sim");
    public static final String LIVE_PORT_ENV = "SIM_LIVE";
    public static final double LIVE_HOLD_SECONDS = 30;
    /** How far the world moves between one op mode loop and the next: about what a loop takes on the robot. */
    public static final double LOOP_SECONDS = 0.02;
    /**
     * Poses are kept for every loop, but the dashboard drawings (which repeat the whole planned
     * path and pose history each loop) only this often, so a replay stays around a megabyte.
     */
    private static final double DRAWING_PERIOD_SECONDS = 0.05;

    private SimRunner() {}

    /**
     * Runs an auto as fast as the machine can, unless {@code SIM_LIVE} names a port to watch it on.
     *
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

    private static SimRecording run(
            SimCatalog.Entry entry, SimRobot sim, double timeoutSeconds, Path outputDir, Integer livePort) {
        if (!entry.kind.equals(SimCatalog.AUTO)) {
            throw new IllegalArgumentException(entry.name + " is a TeleOp; run it from a driver station with record()");
        }
        return run(
                new SimRecording(entry.name, entry.kind),
                (AutoOp) entry.opMode(),
                sim,
                timeoutSeconds,
                outputDir,
                livePort);
    }

    /**
     * @param livePort serve a live view on this port while the run is in progress, or null for none
     */
    public static SimRecording run(
            AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir, Integer livePort) {
        return run(new SimRecording(nameOf(opMode)), opMode, sim, timeoutSeconds, outputDir, livePort);
    }

    private static SimRecording run(
            SimRecording recording,
            AutoOp opMode,
            SimRobot sim,
            double timeoutSeconds,
            Path outputDir,
            Integer livePort) {
        SimLiveServer live = livePort == null ? null : SimLiveServer.start(recording, livePort);
        if (live != null) {
            System.out.println("Simulation live view: " + live.url());
        }
        try {
            // A run somebody is watching keeps to real time; a run nobody is watching need not.
            Pace pace = live == null ? Pace.FASTEST : Pace.REAL_TIME;
            return record(recording, opMode, sim, timeoutSeconds, outputDir, new SimDriverStation(), pace);
        } finally {
            if (live != null) {
                live.awaitViewerSawOutcome(LIVE_HOLD_SECONDS);
                live.stop();
            }
        }
    }

    /**
     * Run an auto into a recording the caller already holds, so it can be watched while this is in
     * progress, in real time. The recording always ends with an outcome and a replay page, even
     * when this throws.
     */
    public static SimRecording record(
            SimRecording recording, AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir) {
        return record(recording, opMode, sim, timeoutSeconds, outputDir, new SimDriverStation(), Pace.REAL_TIME);
    }

    /**
     * Run any op mode into a recording the caller already holds, driven from {@code driverStation}
     * in real time, so the driver can drive it. An auto ends done when its plan is, or fails by
     * timing out after {@code seconds}; a TeleOp ends done when {@code seconds} are up. Either ends
     * stopped when the driver station says so. The recording always ends with an outcome and a
     * replay page, even when this throws.
     */
    public static SimRecording record(
            SimRecording recording,
            OpMode opMode,
            SimRobot sim,
            double seconds,
            Path outputDir,
            SimDriverStation driverStation) {
        return record(recording, opMode, sim, seconds, outputDir, driverStation, Pace.REAL_TIME);
    }

    /** {@link #record(SimRecording, OpMode, SimRobot, double, Path, SimDriverStation)} at the given pace. */
    public static SimRecording record(
            SimRecording recording,
            OpMode opMode,
            SimRobot sim,
            double seconds,
            Path outputDir,
            SimDriverStation driverStation,
            Pace pace) {
        try {
            loopUntilDone(opMode, sim, seconds, recording, driverStation, pace);
            recording.finish(SimRunStream.Outcome.done());
        } catch (RuntimeException | Error e) {
            recording.finish(SimRunStream.Outcome.failed(e));
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

    /**
     * The op mode's time begins at its first loop, at the world's clock as it then stands. Each
     * loop happens, is recorded, and then the world moves one loop period (the robot's noise says
     * how long); so a tick's time is the sum of the periods before it, and the last tick is one
     * period behind the world.
     */
    private static void loopUntilDone(
            OpMode opMode,
            SimRobot sim,
            double seconds,
            SimRecording recording,
            SimDriverStation driverStation,
            Pace pace) {
        AutoOp auto = opMode instanceof AutoOp ? (AutoOp) opMode : null;
        opMode.useHardware(sim.hardware());
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        opMode.start();

        int packetsSeen = sim.dashboard.packets.size();
        double lastDrawingAt = Double.NEGATIVE_INFINITY;
        long startedAtNanos = sim.nanoTime();
        long wallStartedAt = System.nanoTime();
        while (true) {
            if (driverStation.stopRequested()) {
                recording.finish(SimRunStream.Outcome.stopped());
                return;
            }
            if (auto != null && auto.done()) {
                return;
            }
            double elapsed = (sim.nanoTime() - startedAtNanos) / 1e9;
            if (elapsed > seconds) {
                if (auto == null) {
                    return; // a TeleOp's time is simply up
                }
                recording.finish(SimRunStream.Outcome.timedOut(seconds));
                throw new AssertionError(String.format(
                        "op mode still running after %.1fs; current step: %s; true pose: %s",
                        seconds, auto.currentStep(), sim.pose()));
            }
            driverStation.applyTo(opMode.gamepad1, opMode.gamepad2);
            opMode.loop();

            List<TelemetryPacket> allPackets = sim.dashboard.packets;
            List<TelemetryPacket> thisLoop = new ArrayList<>();
            if (allPackets.size() > packetsSeen && elapsed - lastDrawingAt >= DRAWING_PERIOD_SECONDS) {
                thisLoop.addAll(allPackets.subList(packetsSeen, allPackets.size()));
                lastDrawingAt = elapsed;
            }
            packetsSeen = allPackets.size();
            recording.add(new SimRecording.Tick(
                    elapsed,
                    sim.pose(),
                    auto != null ? auto.currentStep() : "",
                    new double[] {sim.leftFront.power, sim.rightFront.power, sim.leftBack.power, sim.rightBack.power},
                    thisLoop,
                    auto == null ? driverStation.state(1) : null,
                    auto == null ? driverStation.state(2) : null,
                    sim.pieces(),
                    sim.held(),
                    sim.scored()));

            sim.step(sim.noise().nextLoopSeconds());
            if (pace == Pace.REAL_TIME) {
                holdToRealTime(wallStartedAt + (sim.nanoTime() - startedAtNanos));
            }
        }
    }

    /** Waits until the wall clock reaches {@code wallNanos}; a machine that is already late does not wait. */
    private static void holdToRealTime(long wallNanos) {
        long remaining = wallNanos - System.nanoTime();
        if (remaining <= 0) {
            return;
        }
        try {
            Thread.sleep(remaining / 1_000_000, (int) (remaining % 1_000_000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
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
}
