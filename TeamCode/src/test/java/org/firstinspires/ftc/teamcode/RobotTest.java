package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

public class RobotTest {
    private final SimRobot sim = new SimRobot();
    private final FakeTelemetry telemetry = new FakeTelemetry();
    private final Robot robot = new Robot(sim.hardware(), Alliance.RELATIVE, telemetry);

    @Test
    public void buildsEverySubsystemWithBrainLastThenThePlans() {
        assertEquals(
                List.of(
                        robot.localizer,
                        robot.launcher,
                        robot.intake,
                        robot.drive,
                        robot.camera,
                        robot.nav,
                        robot.turntable,
                        robot.brain,
                        robot.plans),
                robot.loopOrder());
    }

    @Test
    public void keepsTheAllianceAndGamepadsItWasBuiltWith() {
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();

        Robot built = new Robot(sim.hardware(), Alliance.RED, telemetry, gamepad1, gamepad2);

        assertEquals(Alliance.RED, built.alliance);
        assertSame(gamepad1, built.gamepad1);
        assertSame(gamepad2, built.gamepad2);
    }

    @Test
    public void aRobotBuiltWithoutGamepadsHasIdleOnes() {
        assertNotNull(robot.gamepad1);
        assertNotNull(robot.gamepad2);
        assertEquals(0, robot.gamepad1.left_stick_y, 0);
    }

    @Test
    public void theBrainUsesTheCameraOnlyWhenPlayingForAnAlliance() {
        assertFalse(robot.brain.usingCameraLocalization());
        assertTrue(new Robot(sim.hardware(), Alliance.BLUE, telemetry).brain.usingCameraLocalization());
        assertTrue(new Robot(sim.hardware(), Alliance.RED, telemetry).brain.usingCameraLocalization());
    }

    @Test
    public void theNavPlaysForTheRobotsAlliance() {
        Robot red = new Robot(sim.hardware(), Alliance.RED, telemetry);

        assertEquals(-1, red.nav.pose(0, 1, 1).y(), 0);
        assertEquals(1, robot.nav.pose(0, 1, 1).y(), 0);
    }

    @Test
    public void theDashboardIsTheHardwaresDashboard() {
        assertSame(sim.dashboard, robot.dashboard);
    }

    @Test
    public void theRobotRunsOnItsHardwaresClock() {
        assertEquals(0, robot.clock.getAsLong());

        sim.step(1.5);

        assertEquals(1_500_000_000L, robot.clock.getAsLong());
    }
}
