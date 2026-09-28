package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.opmode.AutoOp;
import org.firstinspires.ftc.teamcode.opmode.OpMode;
import org.firstinspires.ftc.teamcode.simcore.Budget;
import org.firstinspires.ftc.teamcode.simcore.Ending;
import org.firstinspires.ftc.teamcode.simcore.Seconds;

public final class SimRunner {
    public enum Pace {
        REAL_TIME,

        FASTEST
    }

    public static final Path DEFAULT_OUTPUT_DIR = Paths.get("build", "sim");
    public static final String LIVE_PORT_ENV = "SIM_LIVE";
    public static final double LIVE_HOLD_SECONDS = 30;

    public static final double LOOP_SECONDS = 0.02;

    public static final class Meter {
        long opModeNanos;
        long tickNanos;
        long physicsNanos;

        private void measured(long opMode, long tick, long physics) {
            opModeNanos += opMode;
            tickNanos += tick;
            physicsNanos += physics;
        }
    }

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
            Ending ending =
                    ended(recording, opMode, sim, timeoutSeconds, outputDir, new SimDriverStation(), pace, new Meter());
            // What a test of an auto is for: one whose plan is not done within its time fails.
            if (ending == Ending.TIMED_OUT) {
                throw new AssertionError(String.format(
                        "op mode still running after %.1fs; current step: %s; true pose: %s",
                        timeoutSeconds, opMode.currentStep(), sim.pose()));
            }
            return recording;
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
        return record(recording, opMode, sim, seconds, outputDir, driverStation, pace, new Meter());
    }

    public static SimRecording record(
            SimRecording recording,
            OpMode opMode,
            SimRobot sim,
            double seconds,
            Path outputDir,
            SimDriverStation driverStation,
            Pace pace,
            Meter meter) {
        ended(recording, opMode, sim, seconds, outputDir, driverStation, pace, meter);
        return recording;
    }

    /** Runs the op mode until something ends it, and says how in the recording and its replay. */
    private static Ending ended(
            SimRecording recording,
            OpMode opMode,
            SimRobot sim,
            double seconds,
            Path outputDir,
            SimDriverStation driverStation,
            Pace pace,
            Meter meter) {
        Budget budget = Valid.value(Budget.of(seconds));
        try {
            Ending ending = loopUntilDone(opMode, sim, budget, recording, driverStation, pace, meter);
            recording.finish(outcomeOf(ending, seconds));
            return ending;
        } catch (RuntimeException | Error e) {
            recording.finish(SimRunStream.Outcome.failed(e));
            throw e;
        } finally {
            Path page = outputDir.resolve(fileName(recording.name()) + ".html");
            SimReplayPage.write(recording, page);
            System.out.println("Simulation replay: " + page.toAbsolutePath());
        }
    }

    private static String outcomeOf(Ending ending, double seconds) {
        return switch (ending) {
            case STOPPED -> SimRunStream.Outcome.stopped();
            case DONE -> SimRunStream.Outcome.done();
            case TIMED_OUT -> SimRunStream.Outcome.timedOut(seconds);
        };
    }

    private static Integer livePortFromEnvironment() {
        return livePortIn(System.getenv());
    }

    static Integer livePortIn(java.util.Map<String, String> env) {
        String value = env.get(LIVE_PORT_ENV);
        if (value == null || value.isBlank()) {
            return null;
        }
        try {
            return Integer.parseInt(value.trim());
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(LIVE_PORT_ENV + " must be a port number, not '" + value + "'", e);
        }
    }

    private static Ending loopUntilDone(
            OpMode opMode,
            SimRobot sim,
            Budget budget,
            SimRecording recording,
            SimDriverStation driverStation,
            Pace pace,
            Meter meter) {
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
            double elapsed = (sim.nanoTime() - startedAtNanos) / 1e9;
            Optional<Ending> ending = Ending.before(
                    driverStation.stopRequested(), planOf(auto), Valid.value(Seconds.of(elapsed)), budget);
            if (ending.isPresent()) {
                return ending.get();
            }

            long looping = System.nanoTime();
            SimDriverStation.Applied applied = driverStation.applyTo(opMode.gamepad1, opMode.gamepad2);
            opMode.loop();
            long ticking = System.nanoTime();

            List<TelemetryPacket> allPackets = sim.dashboard.packets;
            List<TelemetryPacket> thisLoop = new ArrayList<>();
            if (allPackets.size() > packetsSeen && elapsed - lastDrawingAt >= DRAWING_PERIOD_SECONDS) {
                thisLoop.addAll(allPackets.subList(packetsSeen, allPackets.size()));
                lastDrawingAt = elapsed;
            }
            packetsSeen = allPackets.size();
            SimRecording.Tick.Builder tick = SimRecording.Tick.at(
                            elapsed,
                            sim.pose(),
                            auto != null ? auto.currentStep() : "",
                            new double[] {
                                sim.leftFront.power, sim.rightFront.power, sim.leftBack.power, sim.rightBack.power
                            },
                            thisLoop)
                    .withBalls(sim.pieces(), sim.held())
                    .scoring(sim.scored())
                    .tilted(sim.tilt());
            if (auto == null) {
                tick.drivenBy(applied.gamepad1, applied.gamepad2);
            }
            recording.add(tick.tick());

            long stepping = System.nanoTime();
            sim.step(sim.noise().nextLoopSeconds());
            meter.measured(ticking - looping, stepping - ticking, System.nanoTime() - stepping);
            if (pace == Pace.REAL_TIME) {
                holdToRealTime(wallStartedAt + (sim.nanoTime() - startedAtNanos));
            }
        }
    }

    private static Ending.Plan planOf(AutoOp auto) {
        if (auto == null) {
            return Ending.Plan.NONE;
        }
        return auto.done() ? Ending.Plan.DONE : Ending.Plan.UNDER_WAY;
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
