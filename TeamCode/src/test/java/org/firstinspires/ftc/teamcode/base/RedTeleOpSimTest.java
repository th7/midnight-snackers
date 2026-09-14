package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.sim.SimDriverStation;
import org.firstinspires.ftc.teamcode.sim.SimDriverStation.State;
import org.firstinspires.ftc.teamcode.sim.SimRecording;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.sim.SimRunner;
import org.junit.Test;

/**
 * Runs the real RedTeleOp against the simulated robot, driven from a simulated gamepad the way
 * the bench's on-screen controller drives it. Takes a few seconds of wall clock.
 */
public class RedTeleOpSimTest {
    private static final double TIMEOUT_SECONDS = 30;

    @Test
    public void theLeftStickDrivesTheRobotAndStopEndsTheRun() throws Exception {
        SimRobot sim = new SimRobot();
        SimDriverStation station = new SimDriverStation();
        SimRecording recording = new SimRecording("RedTeleOp", "teleop");
        Thread runner = new Thread(() -> SimRunner.record(
                recording, new RedTeleOp(), sim, TIMEOUT_SECONDS, SimRunner.DEFAULT_OUTPUT_DIR, station));
        runner.start();

        station.set(1, state("{\"left_stick_y\": -1}"));
        await("drove forward", () -> sim.pose().position.x > 12);
        assertTrue("stayed on its line", Math.abs(sim.pose().position.y) < 2);

        station.set(1, State.NEUTRAL);
        await("stopped driving", () -> {
            List<SimRecording.Tick> ticks = recording.ticks();
            if (ticks.isEmpty()) {
                return false;
            }
            for (double power : ticks.get(ticks.size() - 1).wheelPowers) {
                if (power != 0) {
                    return false;
                }
            }
            return true;
        });
        // The robot has inertia: it coasts on after the stick is released, and this waits for it to
        // rest: no movement over a tenth of a second of the simulation's time. Two wall-clock polls
        // can fall within one loop of the simulation and see the same pose while it still coasts.
        long[] lastNanos = {sim.nanoTime()};
        double[] lastSeen = {sim.pose().position.x};
        await("came to rest", () -> {
            if (sim.nanoTime() - lastNanos[0] < 100_000_000L) {
                return false;
            }
            double x = sim.pose().position.x;
            boolean resting = Math.abs(x - lastSeen[0]) < 0.01;
            lastSeen[0] = x;
            lastNanos[0] = sim.nanoTime();
            return resting;
        });
        double restingX = sim.pose().position.x;

        station.set(1, state("{\"right_stick_x\": -1}"));
        await("turned left", () -> sim.pose().heading.toDouble() > Math.toRadians(30));
        assertEquals("turning does not translate", restingX, sim.pose().position.x, 1);

        station.stop();
        runner.join(10_000);
        assertFalse("the run ended when the driver pressed Stop", runner.isAlive());
        assertEquals("stopped", recording.outcome());
        assertTrue(
                "the replay carries the driver's inputs",
                recording.ticks().stream().anyMatch(tick -> tick.gamepad1 != null && tick.gamepad1.leftStickY == -1));
    }

    private static State state(String json) {
        return State.fromJson(new Gson().fromJson(json, JsonObject.class));
    }

    private static void await(String what, BooleanSupplier condition) throws InterruptedException {
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (!condition.getAsBoolean()) {
            if (System.nanoTime() > deadline) {
                throw new AssertionError("never " + what);
            }
            Thread.sleep(20);
        }
    }
}
