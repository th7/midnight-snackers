package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs an autonomous op mode against a {@link SimRobot} in real time, the way the robot
 * controller would: init, start, then loop until the op mode's plan is done.
 * <p>
 * Real time because Road Runner actions and {@code Step} timers read the system clock.
 */
public final class SimRunner {
    private static final long TICK_MILLIS = 5;

    private SimRunner() {
    }

    /**
     * @return the true pose after every loop, in order
     * @throws AssertionError if the plan is not done within {@code timeoutSeconds}
     */
    public static List<Pose2d> run(AutoOp opMode, SimRobot sim, double timeoutSeconds) {
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        opMode.start();

        List<Pose2d> trace = new ArrayList<>();
        long startedAt = System.nanoTime();
        long lastTickAt = startedAt;
        while (!opMode.done()) {
            long now = System.nanoTime();
            if (seconds(now - startedAt) > timeoutSeconds) {
                throw new AssertionError(String.format(
                        "op mode still running after %.1fs; current step: %s; true pose: %s",
                        timeoutSeconds, opMode.currentStep(), sim.pose()));
            }
            sim.step(seconds(now - lastTickAt));
            lastTickAt = now;
            opMode.loop();
            trace.add(sim.pose());
            sleep();
        }
        return trace;
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
