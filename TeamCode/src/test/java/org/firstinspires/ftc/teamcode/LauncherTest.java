package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

public class LauncherTest {
    private static final double DELTA = 0.0001;
    private static final double TOP_GATE_OPEN = 1;
    private static final double TOP_GATE_CLOSED = 0.6;
    private static final double BOTTOM_GATE_CLOSED = 0.4;
    private static final double CLOSE_LAUNCH_VELOCITY = 1050;

    private final SimRobot sim = new SimRobot();
    private final Launcher launcher = new Robot(sim.hardware(), Alliance.RELATIVE, new FakeTelemetry()).launcher;

    @Test
    public void aFreshRobotParksTheGatesAndConfiguresTheFlywheel() {
        assertEquals(TOP_GATE_OPEN, sim.topGate.position, DELTA);
        assertEquals(BOTTOM_GATE_CLOSED, sim.bottomGate.position, DELTA);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, sim.launcher.getMode());
    }

    @Test
    public void loopDrivesTheCommandedVelocity() {
        launcher.setCloseLaunchPower();

        launcher.loop();

        assertEquals(CLOSE_LAUNCH_VELOCITY, sim.launcher.commandedVelocity, DELTA);
    }

    @Test
    public void launchWaitsForTheFlywheelBeforeCyclingTheGates() {
        launcher.launchyLaunch();
        assertFalse(launcher.launchDone());

        launcher.loop();
        launcher.loop();

        assertEquals(CLOSE_LAUNCH_VELOCITY, sim.launcher.commandedVelocity, DELTA);
        assertEquals(TOP_GATE_OPEN, sim.topGate.position, DELTA);

        sim.launcher.measuredVelocity = CLOSE_LAUNCH_VELOCITY;
        launcher.loop();
        launcher.loop();

        assertEquals(TOP_GATE_CLOSED, sim.topGate.position, DELTA);
    }

    /** The launch's timed steps run on the robot's clock, which is the simulation's. */
    @Test
    public void launchRunsToCompletionOnTheSimulatedClockAndParksTheGates() {
        sim.launcher.measuredVelocity = CLOSE_LAUNCH_VELOCITY;
        launcher.launchyLaunch();

        for (int loops = 0; !launcher.launchDone() && loops < 100; loops++) {
            launcher.loop();
            sim.step(0.02);
        }

        assertTrue(launcher.launchDone());
        assertEquals(TOP_GATE_OPEN, sim.topGate.position, DELTA);
        assertEquals(BOTTOM_GATE_CLOSED, sim.bottomGate.position, DELTA);
    }
}
