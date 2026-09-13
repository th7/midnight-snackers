package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Brain;
import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * A robot is built once per run from the hardware and the alliance. Everything a subsystem used to
 * be handed, it now reaches through the robot it was added to.
 */
public class RobotTest {
    private static class Recorder extends SubSystem {
        Telemetry initialisedWith;
        int inits = 0;

        @Override
        public void init() {
            initialisedWith = telemetry;
            inits++;
        }

        @Override
        protected void onLoop() {
        }
    }

    private static class Coordinator extends SuperSystem {
        Nav navSeen;
        Brain brainSeen;

        @Override
        public void init() {
            navSeen = nav;
            brainSeen = brain;
        }
    }

    private final SimRobot sim = new SimRobot();
    private final FakeTelemetry telemetry = new FakeTelemetry();
    private final Robot robot = new Robot(sim.hardware(), Alliance.RELATIVE, telemetry);

    @Test
    public void buildsEverySubsystemWithBrainLast() {
        assertEquals(
                List.of(robot.launcher, robot.drive, robot.camera, robot.nav, robot.turntable, robot.brain),
                robot.loopOrder());
    }

    @Test
    public void addingASubsystemAttachesItThenInitialisesItThenTicksItLast() {
        Recorder recorder = robot.add(new Recorder());

        assertSame(robot, recorder.robot);
        assertSame(telemetry, recorder.initialisedWith);
        assertEquals(1, recorder.inits);
        assertSame(recorder, robot.loopOrder().get(robot.loopOrder().size() - 1));
    }

    @Test
    public void aSuperSystemSeesEverySubsystemByName() {
        Coordinator coordinator = robot.add(new Coordinator());

        assertSame(robot.nav, coordinator.navSeen);
        assertSame(robot.brain, coordinator.brainSeen);
    }

    @Test
    public void theBrainSeesTheOtherSubsystemsWhenItIsInitialised() {
        assertSame(robot, robot.brain.robot);
        assertSame(robot.nav, robot.brain.nav);
    }

    @Test
    public void aPlainHelperJoinsTheLoopAfterTheSubsystems() {
        List<String> ticks = new ArrayList<>();
        Loopable helper = robot.add(() -> ticks.add("helper"));

        robot.loop();

        assertSame(helper, robot.loopOrder().get(robot.loopOrder().size() - 1));
        assertEquals(List.of("helper"), ticks);
    }

    @Test
    public void theNavPlaysForTheRobotsAlliance() {
        Robot red = new Robot(sim.hardware(), Alliance.RED, telemetry);

        assertEquals(-1, red.nav.pose(0, 1, 1).pose2d.position.y, 0);
        assertEquals(1, robot.nav.pose(0, 1, 1).pose2d.position.y, 0);
    }

    @Test
    public void theDashboardIsTheHardwaresDashboard() {
        assertSame(sim.dashboard, robot.dashboard);
    }
}
