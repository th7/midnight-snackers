package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * A robot is built once per run from the hardware and the alliance. Everything a subsystem used to
 * be handed, it now reaches through the robot it was added to.
 */
public class RobotTest {
    private static class Recorder extends SubSystem {
        Telemetry initialisedWith;
        int inits = 0;

        @Override
        protected void onInit() {
            initialisedWith = telemetry;
            inits++;
        }

        @Override
        protected void onLoop() {}

        @Override
        protected void onTelemetry() {}
    }

    /** A subsystem that coordinates others: it reaches them through the robot it was added to. */
    /** A subsystem handed what it coordinates, the way the real ones are. */
    private static class Coordinator extends SubSystem {
        final Nav navSeen;
        final Brain brainSeen;

        Coordinator(Nav nav, Brain brain) {
            this.navSeen = nav;
            this.brainSeen = brain;
        }

        @Override
        protected void onInit() {}

        @Override
        protected void onLoop() {}

        @Override
        protected void onTelemetry() {}
    }

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

    /** The camera may place the robot only when playing for an alliance; no op mode has to say so. */
    @Test
    public void theBrainUsesTheCameraOnlyWhenPlayingForAnAlliance() {
        assertFalse(robot.brain.usingCameraLocalization());
        assertTrue(new Robot(sim.hardware(), Alliance.BLUE, telemetry).brain.usingCameraLocalization());
        assertTrue(new Robot(sim.hardware(), Alliance.RED, telemetry).brain.usingCameraLocalization());
    }

    @Test
    public void addingASubsystemGivesItTelemetryThenInitialisesItThenTicksItLast() {
        Recorder recorder = robot.add(new Recorder());

        assertSame(telemetry, recorder.initialisedWith);
        assertEquals(1, recorder.inits);
        assertSame(recorder, robot.loopOrder().get(robot.loopOrder().size() - 1));
    }

    @Test
    public void aSubsystemThatCoordinatesOthersIsHandedThem() {
        Coordinator coordinator = robot.add(new Coordinator(robot.nav, robot.brain));

        assertSame(robot.nav, coordinator.navSeen);
        assertSame(robot.brain, coordinator.brainSeen);
    }

    @Test
    public void somethingThatIsNotASubsystemJoinsTheLoopAfterTheSubsystems() {
        List<String> ticks = new ArrayList<>();
        Loopable alsoTicked = robot.alsoTick(() -> ticks.add("helper"));

        robot.loop();

        assertSame(alsoTicked, robot.loopOrder().get(robot.loopOrder().size() - 1));
        assertEquals(List.of("helper"), ticks);
    }

    /**
     * A subsystem registered through the door that does not initialise would run a whole match
     * with a null telemetry, and find out the first time a driver asked to see it. Refused at the
     * moment the robot is built instead.
     */
    @Test
    public void aSubsystemOfferedToAlsoTickIsRefusedRatherThanLeftUninitialised() {
        IllegalArgumentException refused = assertThrows(
                IllegalArgumentException.class, () -> robot.alsoTick(new Coordinator(robot.nav, robot.brain)));

        assertTrue(refused.getMessage(), refused.getMessage().contains("add()"));
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

    /** Every timer in the robot code reads the hardware's clock, so a simulation can own time. */
    @Test
    public void theRobotRunsOnItsHardwaresClock() {
        assertEquals(0, robot.clock.getAsLong());

        sim.step(1.5);

        assertEquals(1_500_000_000L, robot.clock.getAsLong());
    }
}
