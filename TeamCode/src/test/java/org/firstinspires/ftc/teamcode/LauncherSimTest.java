package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimField;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * The real {@link Launcher} on the simulated robot: its gate sequence, timed on the simulation's
 * clock, drops a preloaded ball into the spinning flywheel, and the ball flies into the mouth of
 * the cell the blue hive is holding up.
 */
public class LauncherSimTest {
    private final SimRobot sim = new SimRobot();
    private final Robot robot = new Robot(sim.hardware(), Alliance.BLUE, new FakeTelemetry());

    @Test
    public void aCloseLaunchFromTheLaunchDistanceScoresOneBallInTheBlueHive() {
        SimField.Cell cell = sim.upturnedCell("Blue");
        int already = sim.scored("Blue");
        sim.setPose(facing(cell, Nav.LAUNCH_DISTANCE));
        robot.launcher.setCloseLaunchPower();

        robot.launcher.launchyLaunch();
        int loops = 0;
        while (!robot.launcher.launchDone() && loops++ < 200) {
            robot.launcher.loop();
            sim.step(0.02);
        }
        assertTrue("the launch sequence finished", robot.launcher.launchDone());
        sim.step(2.0);

        assertEquals(already + 1, sim.scored("Blue"));
        assertEquals(SimRobot.PRELOAD - 1, sim.held());
    }

    /**
     * The preload is what a hive is short of: the three nectar the blue hive is set up with are
     * three fifths of its load and four pollen are four eighths, so launching the lot empties the
     * robot, fills the hive and tips it, which drops everything that was in it on the floor.
     */
    @Test
    public void thePreloadEmptiesTheRobotAndFillsTheHive() {
        SimField.Cell cell = sim.upturnedCell("Blue");
        double leaning = sim.tilt("Blue");
        assertEquals("three fifths full to start with", 0.6, sim.load("Blue"), 0.001);
        sim.setPose(facing(cell, Nav.LAUNCH_DISTANCE));
        robot.launcher.setCloseLaunchPower();

        for (int launch = 0; launch < SimRobot.PRELOAD; launch++) {
            robot.launcher.launchyLaunch();
            int loops = 0;
            while (!robot.launcher.launchDone() && loops++ < 200) {
                robot.launcher.loop();
                sim.step(0.02);
            }
        }
        sim.step(3.0);

        assertEquals(0, sim.held());
        assertEquals("the hive tipped", -leaning, sim.tilt("Blue"), 0.001);
        assertEquals("and the cell that went under dropped what was in it", 0, sim.scored("Blue"));
    }

    /** The pose {@code distance} inches out from the cell's mouth, facing it, as the hive leans now. */
    private Pose2d facing(SimField.Cell cell, double distance) {
        double[] centre = cell.mouthCentreAt(sim.tilt(cell.alliance));
        double[] normal = cell.mouthNormalAt(sim.tilt(cell.alliance));
        double length = Math.hypot(normal[0], normal[1]);
        double nx = normal[0] / length, ny = normal[1] / length;
        return new Pose2d(centre[0] + nx * distance, centre[1] + ny * distance, Math.atan2(-ny, -nx));
    }
}
