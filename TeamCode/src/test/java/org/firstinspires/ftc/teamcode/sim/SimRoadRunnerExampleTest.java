package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.nugget.RoadRunnerExample;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.firstinspires.ftc.teamcode.simcore.Noise;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimRoadRunnerExampleTest {
    private static final String NAME = "Nugget RoadRunner Example";

    private static final double AUTONOMOUS_SECONDS = 30;

    private static final double OUTSIDE_THE_MIDDLE_IN = 36;

    private static final List<String> LEGS = List.of(
            "along the blue side",
            "along the back wall",
            "along the red side",
            "along the audience wall",
            "spin where it started");

    @Rule
    public final TemporaryFolder replays = new TemporaryFolder();

    private static SimCatalog.Entry example() {
        return SimCatalog.discover(TeamRobot.NUGGET)
                .find(NAME)
                .orElseThrow(() -> new AssertionError(NAME + " is not on Nugget's Simulate tab"));
    }

    private SimRecording run(SimRobot nugget) {
        Path dir = replays.getRoot().toPath();
        return SimRunner.run(example(), nugget, AUTONOMOUS_SECONDS, dir);
    }

    @Test
    public void itIsOnNuggetsSimulateTabAsAnAutoUnderNuggetAndNotOnReginalds() {
        SimCatalog.Entry entry = example();

        assertEquals(SimCatalog.AUTO, entry.kind);
        assertEquals("Nugget", entry.group);
        assertEquals(RoadRunnerExample.class.getName(), entry.where);
        assertTrue(SimCatalog.discover(TeamRobot.REGINALD).find(NAME).isEmpty());
    }

    @Test
    public void setDownWhereItsPlanSaysItDrivesALapOfTheFieldAndStopsWhereItStarted() {
        SimRobot nugget = new SimRobot(TeamRobot.NUGGET, SimNoise.NONE, Constants.defaults());
        nugget.setDown(RoadRunnerExample.START);

        SimRecording run = run(nugget);

        assertEquals(SimRunStream.Outcome.done(), run.outcome());
        assertLapped(run.poses());
        assertNear(RoadRunnerExample.START, nugget.pose(), 1, Math.toRadians(2));
    }

    @Test
    public void itsPlanIsDoneWellWithinTheAutonomousPeriodAndTheRunEndsWithIt() {
        SimRobot nugget = new SimRobot(TeamRobot.NUGGET, SimNoise.NONE, Constants.defaults());
        nugget.setDown(RoadRunnerExample.START);

        SimRecording run = run(nugget);

        List<SimRecording.Tick> ticks = run.ticks();
        double lasted = ticks.get(ticks.size() - 1).seconds;
        assertTrue("it took " + lasted + "s", lasted < AUTONOMOUS_SECONDS - 5);
    }

    @Test
    public void eachLegIsAStepOfItsPlanAndTheRunSaysWhichItIsOn() {
        SimRobot nugget = new SimRobot(TeamRobot.NUGGET, SimNoise.NONE, Constants.defaults());
        nugget.setDown(RoadRunnerExample.START);

        SimRecording run = run(nugget);

        List<String> driven = new ArrayList<>();
        for (SimRecording.Tick tick : run.ticks()) {
            for (String leg : LEGS) {
                if (tick.step.contains(leg) && !driven.contains(leg)) {
                    driven.add(leg);
                }
            }
        }
        assertEquals(LEGS, driven);
    }

    @Test
    public void aSeededNuggetImperfectAsItIsStillDrivesItsLapAndComesBackToAboutWhereItWasSetDown() {
        SimRobot nugget = new SimRobot(
                TeamRobot.NUGGET, Noise.seeded(StartPoses.DEFAULT_SEED, Constants.defaults()), Constants.defaults());
        nugget.setDown(RoadRunnerExample.START);
        Pose2d setDown = nugget.pose();

        SimRecording run = run(nugget);

        assertEquals(SimRunStream.Outcome.done(), run.outcome());
        assertLapped(run.poses());
        assertNear(setDown, nugget.pose(), 3, Math.toRadians(5));
    }

    private static void assertLapped(List<Pose2d> poses) {
        double east = Double.NEGATIVE_INFINITY;
        double north = Double.NEGATIVE_INFINITY;
        double west = Double.POSITIVE_INFINITY;
        double south = Double.POSITIVE_INFINITY;
        for (Pose2d pose : poses) {
            assertTrue(
                    "kept outside the middle of the field, but was at " + pose,
                    pose.position.norm() > OUTSIDE_THE_MIDDLE_IN);
            east = Math.max(east, pose.position.x);
            north = Math.max(north, pose.position.y);
            west = Math.min(west, pose.position.x);
            south = Math.min(south, pose.position.y);
        }
        assertTrue("reached the back wall's side: " + east, east > OUTSIDE_THE_MIDDLE_IN);
        assertTrue("reached the red side: " + north, north > OUTSIDE_THE_MIDDLE_IN);
        assertTrue("reached the audience's side: " + west, west < -OUTSIDE_THE_MIDDLE_IN);
        assertTrue("reached the blue side: " + south, south < -OUTSIDE_THE_MIDDLE_IN);
    }

    private static void assertNear(Pose2d expected, Pose2d actual, double inches, double radians) {
        assertEquals(
                "ended at " + actual,
                0,
                expected.position.minus(actual.position).norm(),
                inches);
        assertEquals("ended at " + actual, 0, expected.heading.minus(actual.heading), radians);
    }
}
