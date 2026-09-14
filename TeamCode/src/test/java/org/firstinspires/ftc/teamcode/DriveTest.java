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
    public void manualPowersAddPerWheelAndSaturateAtFullPower() {
        drive.manual(0.8f, 0, 0.8f);

        assertPowers(0, 1, 0, 1);
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

        // full power forward from the drive, plus the driver's strafe, clamped per wheel
        assertPowers(0.5, 1, 1, 0.5);
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
