package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.junit.Test;

public class LauncherTest {
    private static final double DELTA = 0.0001;
    private static final double TOP_GATE_OPEN = 1;
    private static final double TOP_GATE_CLOSED = 0.6;
    private static final double BOTTOM_GATE_CLOSED = 0.4;
    private static final double CLOSE_LAUNCH_VELOCITY = 1050;

    private final FakeDcMotorEx flywheel = new FakeDcMotorEx();
    private final FakeServo topGate = new FakeServo();
    private final FakeServo bottomGate = new FakeServo();
    private final Launcher launcher =
            new Launcher(flywheel, topGate, bottomGate, new ElapsedTime(), new FakeTelemetry());

    @Test
    public void initParksTheGatesAndConfiguresTheFlywheel() {
        launcher.init();

        assertEquals(TOP_GATE_OPEN, topGate.position, DELTA);
        assertEquals(BOTTOM_GATE_CLOSED, bottomGate.position, DELTA);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, flywheel.getMode());
    }

    @Test
    public void loopDrivesTheCommandedVelocity() {
        launcher.init();
        launcher.setCloseLaunchPower();

        launcher.loop();

        assertEquals(CLOSE_LAUNCH_VELOCITY, flywheel.commandedVelocity, DELTA);
    }

    @Test
    public void launchWaitsForTheFlywheelBeforeCyclingTheGates() {
        launcher.init();
        launcher.launchyLaunch();
        assertFalse(launcher.launchDone());

        launcher.loop();
        launcher.loop();

        assertEquals(CLOSE_LAUNCH_VELOCITY, flywheel.commandedVelocity, DELTA);
        assertEquals(TOP_GATE_OPEN, topGate.position, DELTA);

        flywheel.measuredVelocity = CLOSE_LAUNCH_VELOCITY;
        launcher.loop();
        launcher.loop();

        assertEquals(TOP_GATE_CLOSED, topGate.position, DELTA);
    }

    @Test
    public void launchRunsToCompletionAndParksTheGates() throws InterruptedException {
        launcher.init();
        flywheel.measuredVelocity = CLOSE_LAUNCH_VELOCITY;
        launcher.launchyLaunch();

        long deadline = System.nanoTime() + 2_000_000_000L;
        while (!launcher.launchDone() && System.nanoTime() < deadline) {
            launcher.loop();
            Thread.sleep(1);
        }

        assertTrue(launcher.launchDone());
        assertEquals(TOP_GATE_OPEN, topGate.position, DELTA);
        assertEquals(BOTTOM_GATE_CLOSED, bottomGate.position, DELTA);
    }
}
