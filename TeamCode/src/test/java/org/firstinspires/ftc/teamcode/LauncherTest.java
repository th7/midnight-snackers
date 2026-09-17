package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

public class LauncherTest {
    private static final double DELTA = 0.0001;
    private static final double TOP_GATE_OPEN = 1;
    private static final double TOP_GATE_CLOSED = 0.6;
    private static final double BOTTOM_GATE_CLOSED = 0.4;
    private static final double CLOSE_LAUNCH_VELOCITY = 1050;

    private final SimDevices devices = new SimDevices();
    private final Launcher launcher = new Robot(devices.hardware(), Alliance.RELATIVE, new FakeTelemetry()).launcher;

    @Test
    public void aFreshRobotParksTheGatesAndConfiguresTheFlywheel() {
        assertEquals(TOP_GATE_OPEN, devices.topGate.position, DELTA);
        assertEquals(BOTTOM_GATE_CLOSED, devices.bottomGate.position, DELTA);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, devices.launcher.getMode());
    }

    @Test
    public void loopDrivesTheCommandedVelocity() {
        launcher.setCloseLaunchPower();

        launcher.loop();

        assertEquals(CLOSE_LAUNCH_VELOCITY, devices.launcher.commandedVelocity, DELTA);
    }

    @Test
    public void launchWaitsForTheFlywheelBeforeCyclingTheGates() {
        launcher.launchyLaunch();
        assertFalse(launcher.launchDone());

        launcher.loop();
        launcher.loop();

        assertEquals(CLOSE_LAUNCH_VELOCITY, devices.launcher.commandedVelocity, DELTA);
        assertEquals(TOP_GATE_OPEN, devices.topGate.position, DELTA);

        devices.launcher.measuredVelocity = CLOSE_LAUNCH_VELOCITY;
        launcher.loop();
        launcher.loop();

        assertEquals(TOP_GATE_CLOSED, devices.topGate.position, DELTA);
    }

    /**
     * How long one launch holds the bottom gate open, on the simulated clock: the wait the driver
     * tunes from gamepad 2.
     */
    private double secondsWithTheBottomGateOpen() {
        devices.launcher.measuredVelocity = CLOSE_LAUNCH_VELOCITY;
        launcher.launchyLaunch();
        Double openedAt = null;
        Double closedAt = null;
        for (int loops = 0; !launcher.launchDone() && loops < 4000; loops++) {
            launcher.loop();
            boolean open = devices.bottomGate.position > BOTTOM_GATE_CLOSED + DELTA;
            if (open && openedAt == null) {
                openedAt = devices.nanoTime() / 1e9;
            } else if (!open && openedAt != null && closedAt == null) {
                closedAt = devices.nanoTime() / 1e9;
            }
            devices.advance(0.005);
        }
        if (openedAt == null || closedAt == null) {
            throw new AssertionError("the launch never opened and closed the bottom gate");
        }
        return closedAt - openedAt;
    }

    /** Left alone, the launch waits what it has always waited. */
    @Test
    public void theBottomGateWaitStartsAtWhatTheLaunchHasAlwaysWaited() {
        assertEquals(0.15, launcher.bottomGateWaitSeconds(), DELTA);
    }

    /**
     * The driver's gamepad-2 bumpers tune that wait: they moved a number the launch never read,
     * so a driver tuning at the field watched the telemetry change and the robot not.
     */
    @Test
    public void tuningTheBottomGateWaitChangesHowLongTheLaunchHoldsItOpen() {
        double before = secondsWithTheBottomGateOpen();

        for (int presses = 0; presses < 10; presses++) {
            launcher.increaseBottomGateWaitTime();
        }

        assertEquals(before + 10 * Launcher.BOTTOM_GATE_WAIT_STEP_SECONDS, secondsWithTheBottomGateOpen(), 0.01);
    }

    /** A wait cannot be tuned below nothing: the gate would close in the same loop it opened. */
    @Test
    public void theBottomGateWaitStopsAtZero() {
        for (int presses = 0; presses < 1000; presses++) {
            launcher.decreaseBottomGateWaitTime();
        }

        assertEquals(0, launcher.bottomGateWaitSeconds(), DELTA);
    }

    /** The launch's timed steps run on the robot's clock, which is the simulation's. */
    @Test
    public void launchRunsToCompletionOnTheSimulatedClockAndParksTheGates() {
        devices.launcher.measuredVelocity = CLOSE_LAUNCH_VELOCITY;
        launcher.launchyLaunch();

        for (int loops = 0; !launcher.launchDone() && loops < 100; loops++) {
            launcher.loop();
            devices.advance(0.02);
        }

        assertTrue(launcher.launchDone());
        assertEquals(TOP_GATE_OPEN, devices.topGate.position, DELTA);
        assertEquals(BOTTOM_GATE_CLOSED, devices.bottomGate.position, DELTA);
    }
}
