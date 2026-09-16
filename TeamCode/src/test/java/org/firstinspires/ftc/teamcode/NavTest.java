package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Vector2d;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * Nav speaks in poses on the field: it mirrors them for the alliance, says where the robot is
 * and whether it is near somewhere, and knows where to launch from.
 *
 * <p>Where things are is all it knows. How the robot gets to one of them -- what path to take and
 * what to ask the wheels for along the way -- is the drive's, and Nav does not hold the drive that
 * would tell it.
 */
public class NavTest {
    private static final double DELTA = 0.001;

    private static Nav navFor(Alliance alliance) {
        return new Robot(new SimRobot().hardware(), alliance, new FakeTelemetry()).nav;
    }

    private static double distance(Nav.Pose from, Vector2d to) {
        return Math.hypot(to.x - from.x(), to.y - from.y());
    }

    /**
     * Nav says where; the drive says how. A path is built from the robot's own model of itself --
     * its track width, its wheels, what it can accelerate at -- which is the drive's to know, so
     * Road Runner's drive belongs there and not here.
     */
    @Test
    public void navDoesNotHoldTheDriveThatBuildsPaths() {
        List<String> held = new ArrayList<>();
        for (Field field : Nav.class.getDeclaredFields()) {
            if (MecanumDrive.class.isAssignableFrom(field.getType())) {
                held.add(field.getName());
            }
        }

        assertEquals("Nav should ask the drive to go somewhere, not build the path itself", List.of(), held);
    }

    @Test
    public void redMirrorsYAndHeadingAcrossTheCentreLineAndBlueDoesNot() {
        Nav.Pose red = navFor(Alliance.RED).pose(1, 2, 0.5);
        Nav.Pose blue = navFor(Alliance.BLUE).pose(1, 2, 0.5);
        Nav.Pose relative = navFor(Alliance.RELATIVE).pose(1, 2, 0.5);

        assertEquals(1, red.x(), DELTA);
        assertEquals(-2, red.y(), DELTA);
        assertEquals(-0.5, red.heading(), DELTA);
        assertEquals(2, blue.y(), DELTA);
        assertEquals(0.5, blue.heading(), DELTA);
        assertEquals(2, relative.y(), DELTA);
        assertEquals(0.5, relative.heading(), DELTA);
    }

    @Test
    public void theRobotStartsAtTheOriginAndIsWhereItWasLastPut() {
        Nav nav = navFor(Alliance.BLUE);
        assertEquals(0, nav.currentPose().x(), DELTA);
        assertEquals(0, nav.currentPose().y(), DELTA);

        nav.setPose(nav.pose(12, -6, 1));

        assertEquals(12, nav.currentPose().x(), DELTA);
        assertEquals(-6, nav.currentPose().y(), DELTA);
        assertEquals(1, nav.currentPose().heading(), DELTA);
    }

    @Test
    public void theLaunchPoseIsFortyInchesShortOfTheGoalFacingIt() {
        Nav nav = navFor(Alliance.BLUE);
        nav.setPose(nav.pose(10, 5, 2));
        Vector2d goal = Alliance.BLUE.launchTarget;

        Nav.Pose launch = nav.launchPose().get();

        assertEquals(40, distance(launch, goal), DELTA);
        assertEquals(Math.atan2(goal.y - launch.y(), goal.x - launch.x()), launch.heading(), DELTA);
        assertEquals(
                "on the line from the robot to the goal", Math.atan2(goal.y - 5, goal.x - 10), launch.heading(), DELTA);
    }

    @Test
    public void playingForNoAllianceThereIsNoGoalAndSoNoLaunchPose() {
        assertTrue(navFor(Alliance.RELATIVE).launchPose().isEmpty());
        assertTrue(navFor(Alliance.RED).launchPose().isPresent());
    }

    @Test
    public void theTurntableHeadingIsZeroUntilTheCameraHasPlacedTheRobotThenTheGoalsBearingBackwards() {
        Nav nav = navFor(Alliance.BLUE);
        Vector2d goal = Alliance.BLUE.launchTarget;
        assertEquals(0, nav.relativeHeadingToTarget(), DELTA);

        nav.setFieldPosition(nav.pose(0, 0, 0.25));

        assertEquals(-(Math.atan2(goal.y, goal.x) - 0.25), nav.relativeHeadingToTarget(), DELTA);
    }

    @Test
    public void theFirstSightingPlacesTheRobotAndLaterOnesNudgeItAnInchAtMost() {
        Nav nav = navFor(Alliance.BLUE);

        nav.setFieldPosition(nav.pose(10, 20, 0.5));
        assertEquals(10, nav.currentPose().x(), DELTA);
        assertEquals(20, nav.currentPose().y(), DELTA);
        assertEquals(0.5, nav.currentPose().heading(), DELTA);

        nav.setFieldPosition(nav.pose(20, 19.5, 0));
        assertEquals(11, nav.currentPose().x(), DELTA);
        assertEquals(19.5, nav.currentPose().y(), DELTA);
        assertEquals(
                "a later sighting never changes the heading",
                0.5,
                nav.currentPose().heading(),
                DELTA);
    }

    @Test
    public void nearIsWithinThreeInchesAndSixDegrees() {
        Nav nav = navFor(Alliance.BLUE);

        assertTrue(nav.near(nav.pose(2, -2, 0.05)));
        assertFalse(nav.near(nav.pose(4, 0, 0)));
        assertFalse(nav.near(nav.pose(0, 4, 0)));
        assertFalse(nav.near(nav.pose(0, 0, 0.2)));
    }

    @Test
    public void aPoseRotatedKeepsItsPlaceAndTurnsItsHeading() {
        Nav.Pose pose = navFor(Alliance.BLUE).pose(3, 4, 1).rotated(-0.25);

        assertEquals(3, pose.x(), DELTA);
        assertEquals(4, pose.y(), DELTA);
        assertEquals(0.75, pose.heading(), DELTA);
    }
}
