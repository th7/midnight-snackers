package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.opmode.AutoOp;
import org.firstinspires.ftc.teamcode.opmode.OpMode;

public final class SimRunner {
    public enum Pace {
        REAL_TIME,

        FASTEST
    }

    public static final Path DEFAULT_OUTPUT_DIR = Paths.get("build", "sim");
    public static final String LIVE_PORT_ENV = "SIM_LIVE";
    public static final double LIVE_HOLD_SECONDS = 30;

    public static final double LOOP_SECONDS = 0.02;

    private static final double DRAWING_PERIOD_SECONDS = 0.05;

    private SimRunner() {}

    public static SimRecording run(AutoOp opMode, SimRobot sim, double timeoutSeconds) {
        return run(opMode, sim, timeoutSeconds, DEFAULT_OUTPUT_DIR, livePortFromEnvironment());
    }

    public static SimRecording run(AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir) {
        return run(opMode, sim, timeoutSeconds, outputDir, null);
    }

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
            Pace pace = live == null ? Pace.FASTEST : Pace.REAL_TIME;
            return record(recording, opMode, sim, timeoutSeconds, outputDir, new SimDriverStation(), pace);
        } finally {
            if (live != null) {
                live.awaitViewerSawOutcome(LIVE_HOLD_SECONDS);
                live.stop();
            }
        }
    }

    public static SimRecording record(
            SimRecording recording, AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir) {
        return record(recording, opMode, sim, timeoutSeconds, outputDir, new SimDriverStation(), Pace.REAL_TIME);
    }

    public static SimRecording record(
            SimRecording recording,
            OpMode opMode,
            SimRobot sim,
            double seconds,
            Path outputDir,
            SimDriverStation driverStation) {
        return record(recording, opMode, sim, seconds, outputDir, driverStation, Pace.REAL_TIME);
    }

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
                    return;
                }
                recording.finish(SimRunStream.Outcome.timedOut(seconds));
                throw new AssertionError(String.format(
                        "op mode still running after %.1fs; current step: %s; true pose: %s",
                        seconds, auto.currentStep(), sim.pose()));
            }

            SimDriverStation.Applied applied = driverStation.applyTo(opMode.gamepad1, opMode.gamepad2);
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
                    auto == null ? applied.gamepad1 : null,
                    auto == null ? applied.gamepad2 : null,
                    sim.pieces(),
                    sim.held(),
                    sim.scored(),
                    sim.tilt()));

            sim.step(sim.noise().nextLoopSeconds());
            if (pace == Pace.REAL_TIME) {
                holdToRealTime(wallStartedAt + (sim.nanoTime() - startedAtNanos));
            }
        }
    }

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

    static String fileName(String name) {
        return name.replaceAll("[^A-Za-z0-9._ -]", "_");
    }

    static String nameOf(OpMode opMode) {
        Class<?> type = opMode.getClass();
        while (type.getSimpleName().isEmpty()) {
            type = type.getSuperclass();
        }
        return type.getSimpleName();
    }
}
