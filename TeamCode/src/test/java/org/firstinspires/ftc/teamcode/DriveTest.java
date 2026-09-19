package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.Drive.Held;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

public class DriveTest {
    private static final double DELTA = 0.0001;

    private static final Action FOREVER = packet -> true;

    private final SimDevices devices = new SimDevices();
    private final FakeTelemetry screen = new FakeTelemetry();
    private final Robot robot = new Robot(devices.hardware(), Alliance.RELATIVE, screen);
    private final Drive drive = robot.drive;

    private double[] powers() {
        return new double[] {
            devices.leftFront.power, devices.rightFront.power, devices.leftBack.power, devices.rightBack.power
        };
    }

    private void assertPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        assertEquals("leftFront", leftFront, devices.leftFront.power, DELTA);
        assertEquals("rightFront", rightFront, devices.rightFront.power, DELTA);
        assertEquals("leftBack", leftBack, devices.leftBack.power, DELTA);
        assertEquals("rightBack", rightBack, devices.rightBack.power, DELTA);
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

    @Test
    public void aCommandTooBigForAWheelIsScaledDownWhole() {
        drive.manual(0.25f, 0.25f, 0.25f);
        double[] gentle = powers();

        drive.manual(0.5f, 0.5f, 0.5f);

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
        assertTrue("the trajectory should be driving by now", devices.leftFront.power != 0);

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

    @Test
    public void towardWhileAnActionIsFollowedCommandsNothingAndDoesNotClaimToHaveArrived() {
        drive.follow(FOREVER);
        drive.loop();

        boolean arrived = drive.toward(robot.nav.pose(0, 0, 0));

        assertFalse(arrived);
        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void theSteeringDoesNotRunWhileAnActionOwnsTheWheelsSoItsControllersRememberNoLoopTheRobotIgnored() {
        robot.channels.toggle(Drive.CHANNEL);
        drive.follow(FOREVER);

        drive.toward(robot.nav.pose(48, 0, 0));
        drive.loop();

        assertTrue(
                "the drive should still say it has not steered: " + screen.captions,
                screen.captions.contains("Drive.steering"));
        assertFalse(
                "nothing to report about a steering that never happened",
                screen.captions.contains("Drive.steeringArrived"));
    }

    @Test
    public void idleLoopsDrawTheRobotAtNavsPoseOnTheDashboard() {
        drive.loop();
        drive.loop();

        assertEquals(2, devices.dashboard.packets.size());
    }
}
