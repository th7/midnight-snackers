package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.Drive.Held;
import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * The drive takes one intent at a time, from whoever is driving, and writes the motors itself:
 * manual powers, steering toward a pose, or following a Road Runner action.
 */
public class DriveTest {
    private static final double DELTA = 0.0001;
    /** An action that never finishes and drives nothing, like a path still being planned. */
    private static final Action FOREVER = packet -> true;

    private final SimRobot sim = new SimRobot();
    private final Robot robot = new Robot(sim.hardware(), Alliance.RELATIVE, new FakeTelemetry());
    private final Drive drive = robot.drive;

    /** The four wheel powers, in {@link #assertPowers}' order. */
    private double[] powers() {
        return new double[] {sim.leftFront.power, sim.rightFront.power, sim.leftBack.power, sim.rightBack.power};
    }

    private void assertPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        assertEquals("leftFront", leftFront, sim.leftFront.power, DELTA);
        assertEquals("rightFront", rightFront, sim.rightFront.power, DELTA);
        assertEquals("leftBack", leftBack, sim.leftBack.power, DELTA);
        assertEquals("rightBack", rightBack, sim.rightBack.power, DELTA);
    }

    @Test
    public void manualStraightPowerDrivesAllWheelsTheSame() {
        drive.manual(0.5f, 0, 0);

        assertPowers(0.5, 0.5, 0.5, 0.5);
    }

    @Test
    public void manualStrafePowerCrossesTheDiagonals() {
        drive.manual(0, 0.5f, 0);

        assertPowers(-0.5, 0.5, 0.5, -0.5);
    }

    @Test
    public void manualTurnPowerSpinsTheSidesOppositeWays() {
        drive.manual(0, 0, 0.5f);

        assertPowers(-0.5, 0.5, -0.5, 0.5);
    }

    @Test
    public void manualPowersAddPerWheel() {
        drive.manual(0.4f, 0.2f, 0.1f);

        assertPowers(0.1, 0.7, 0.5, 0.3);
    }

    /**
     * A command that would ask more of a wheel than it has is scaled down whole, so the robot goes
     * slower in the direction asked for rather than somewhere else. Twice a command, once a wheel
     * has run out, is still the same command: every wheel in the same ratio.
     */
    @Test
    public void aCommandTooBigForAWheelIsScaledDownWhole() {
        drive.manual(0.25f, 0.25f, 0.25f); // the most a wheel is asked for is 0.75: nothing is clipped
        double[] gentle = powers();

        drive.manual(0.5f, 0.5f, 0.5f); // the same again, but a wheel is now asked for 1.5

        double[] hard = powers();
        double scale = 1 / 1.5 * 2;
        for (int wheel = 0; wheel < gentle.length; wheel++) {
            assertEquals("wheel " + wheel, gentle[wheel] * scale, hard[wheel], DELTA);
        }
        assertPowers(-1.0 / 3, 1, 1.0 / 3, 1.0 / 3);
    }

    @Test
    public void towardAFarPoseDrivesForwardAtFullPowerAndIsNotArrived() {
        boolean arrived = drive.toward(robot.nav.pose(48, 0, 0));

        assertFalse(arrived);
        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void towardThePoseTheRobotIsAtIsArrivedAndStill() {
        boolean arrived = drive.toward(robot.nav.pose(0, 0, 0));

        assertTrue(arrived);
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void anAxisTheDriverHoldsIsTheirsAndTheRestAreTheDrives() {
        drive.toward(robot.nav.pose(48, 0, 0), Held.NONE.strafe(0.5f));

        // full power forward from the drive, plus the driver's strafe; a wheel is asked for 1.5,
        // so the whole command is scaled to fit rather than the far wheels being clipped
        assertPowers(1.0 / 3, 1, 1, 1.0 / 3);
    }

    @Test
    public void holdingEveryAxisIsManualDriving() {
        drive.toward(robot.nav.pose(48, 0, 0), Held.NONE.straight(0).strafe(0).turn(0.5f));

        assertPowers(-0.5, 0.5, -0.5, 0.5);
    }

    @Test
    public void whileFollowingAnActionTheDriverIsIgnoredUntilItIsCancelled() {
        drive.follow(FOREVER);
        drive.loop();

        drive.manual(1, 0, 0);
        assertFalse(drive.done());
        assertPowers(0, 0, 0, 0);

        drive.cancel();
        drive.manual(1, 0, 0);
        assertTrue(drive.done());
        assertPowers(1, 1, 1, 1);
    }

    /**
     * Cancelling is a driver or a plan saying they are done with what the drive was doing, and a
     * robot nobody is driving should not still be driving. It used to keep its last power until
     * something else asked for something, so a plan step that cancelled its own path as it arrived
     * left the robot rolling into whatever came next.
     */
    @Test
    public void cancellingStopsTheWheels() {
        drive.manual(0.8f, 0, 0);
        assertPowers(0.8, 0.8, 0.8, 0.8);

        drive.follow(FOREVER);
        drive.cancel();

        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void cancellingATrajectoryPartWayThroughStopsTheWheels() {
        drive.strafeTo(robot.nav.pose(48, 0, 0));
        drive.loop();
        drive.loop();
        assertTrue("the trajectory should be driving by now", sim.leftFront.power != 0);

        drive.cancel();

        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void anActionCannotBeFollowedOverAnotherStillInProgress() {
        drive.follow(FOREVER);

        try {
            drive.follow(FOREVER);
            throw new AssertionError("a second follow() was accepted");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("in progress"));
        }
    }

    /** The dashboard field view shows the robot where Nav says it is, even when nothing is driving. */
    @Test
    public void idleLoopsDrawTheRobotAtNavsPoseOnTheDashboard() {
        drive.loop();
        drive.loop();

        assertEquals(2, sim.dashboard.packets.size());
    }
}
