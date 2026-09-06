package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs an autonomous op mode against a {@link SimRobot} in real time, the way the robot
 * controller would: init, start, then loop until the op mode's plan is done. Every run, finished
 * or not, leaves a replay page named after the op mode in the output directory.
 * <p>
 * Real time because Road Runner actions and {@code Step} timers read the system clock.
 */
public final class SimRunner {
    public static final Path DEFAULT_OUTPUT_DIR = Paths.get("build", "sim");
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
        return run(opMode, sim, timeoutSeconds, DEFAULT_OUTPUT_DIR);
    }

    public static SimRecording run(AutoOp opMode, SimRobot sim, double timeoutSeconds, Path outputDir) {
        SimRecording recording = new SimRecording(nameOf(opMode));
        try {
            loopUntilDone(opMode, sim, timeoutSeconds, recording);
            recording.finish("done");
        } catch (RuntimeException | Error e) {
            if (!recording.finished()) {
                recording.finish("failed: " + e);
            }
            throw e;
        } finally {
            Path page = outputDir.resolve(recording.name() + ".html");
            SimReplayPage.write(recording, page);
            System.out.println("Simulation replay: " + page.toAbsolutePath());
        }
        return recording;
    }

    private static void loopUntilDone(AutoOp opMode, SimRobot sim, double timeoutSeconds, SimRecording recording) {
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        opMode.start();

        int packetsSeen = sim.dashboard.packets.size();
        double lastDrawingAt = Double.NEGATIVE_INFINITY;
        long startedAt = System.nanoTime();
        long lastTickAt = startedAt;
        while (!opMode.done()) {
            long now = System.nanoTime();
            double elapsed = seconds(now - startedAt);
            if (elapsed > timeoutSeconds) {
                recording.finish(String.format("timed out after %.1fs", timeoutSeconds));
                throw new AssertionError(String.format(
                        "op mode still running after %.1fs; current step: %s; true pose: %s",
                        timeoutSeconds, opMode.currentStep(), sim.pose()));
            }
            sim.step(seconds(now - lastTickAt));
            lastTickAt = now;
            opMode.loop();

            List<TelemetryPacket> allPackets = sim.dashboard.packets;
            List<TelemetryPacket> thisLoop = new ArrayList<>();
            if (allPackets.size() > packetsSeen && elapsed - lastDrawingAt >= DRAWING_PERIOD_SECONDS) {
                thisLoop.addAll(allPackets.subList(packetsSeen, allPackets.size()));
                lastDrawingAt = elapsed;
            }
            packetsSeen = allPackets.size();
            recording.add(new SimRecording.Tick(elapsed, sim.pose(), opMode.currentStep(),
                    new double[]{sim.leftFront.power, sim.rightFront.power, sim.leftBack.power, sim.rightBack.power},
                    thisLoop));
            sleep();
        }
    }

    /**
     * The op mode's class name, or for an anonymous subclass the nearest named class.
     */
    private static String nameOf(AutoOp opMode) {
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
