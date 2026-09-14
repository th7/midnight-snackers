package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimField;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * The real {@link Launcher} on the simulated robot: its gate sequence, timed on the simulation's
 * clock, drops a preloaded ball into the spinning flywheel, and the ball flies into the hive.
 */
public class LauncherSimTest {
    /** How far from the goal the robot code launches from ({@code Nav}), in inches. */
    private static final double LAUNCH_DISTANCE = 40;

    private final SimRobot sim = new SimRobot();
    private final Robot robot = new Robot(sim.hardware(), Alliance.BLUE, new FakeTelemetry());

    @Test
    public void aCloseLaunchFromTheLaunchDistanceScoresOneBallInTheBlueHive() {
        SimField.Cell cell = lowestCellOf("Blue");
        sim.setPose(facing(cell, LAUNCH_DISTANCE));
        robot.launcher.setCloseLaunchPower();

        robot.launcher.launchyLaunch();
        int loops = 0;
        while (!robot.launcher.launchDone() && loops++ < 200) {
            robot.launcher.loop();
            sim.step(0.02);
        }
        assertTrue("the launch sequence finished", robot.launcher.launchDone());
        sim.step(2.0);

        assertEquals(1, sim.scored("Blue"));
        assertEquals(SimRobot.PRELOAD - 1, sim.held());
    }

    @Test
    public void threeLaunchesEmptyTheRobot() {
        SimField.Cell cell = lowestCellOf("Blue");
        sim.setPose(facing(cell, LAUNCH_DISTANCE));
        robot.launcher.setCloseLaunchPower();

        for (int launch = 0; launch < 3; launch++) {
            robot.launcher.launchyLaunch();
            int loops = 0;
            while (!robot.launcher.launchDone() && loops++ < 200) {
                robot.launcher.loop();
                sim.step(0.02);
            }
        }
        sim.step(3.0);

        assertEquals(0, sim.held());
        assertEquals(3, sim.scored("Blue"));
    }

    private static SimField.Cell lowestCellOf(String alliance) {
        SimField.Cell lowest = null;
        for (SimField.Cell cell : SimRobot.FIELD.cells) {
            if (cell.alliance.equals(alliance) && (lowest == null || cell.mouthCentre[2] < lowest.mouthCentre[2])) {
                lowest = cell;
            }
        }
        return lowest;
    }

    /** The pose {@code distance} inches from the cell's mouth, straight out from it, facing it. */
    private static Pose2d facing(SimField.Cell cell, double distance) {
        double nx = cell.mouthNormal[0], ny = cell.mouthNormal[1];
        double length = Math.hypot(nx, ny);
        nx /= length;
        ny /= length;
        return new Pose2d(
                cell.mouthCentre[0] + nx * distance, cell.mouthCentre[1] + ny * distance, Math.atan2(-ny, -nx));
    }
}
