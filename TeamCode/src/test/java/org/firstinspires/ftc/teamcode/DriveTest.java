package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

public class DriveTest {
    private static final double DELTA = 0.0001;

    private final SimRobot sim = new SimRobot();
    private final Robot robot = new Robot(sim.hardware(), Alliance.RELATIVE, new FakeTelemetry());
    private final Drive drive = robot.drive;

    @Test
    public void straightPowerDrivesAllWheelsTheSame() {
        drive.setStraightPower(0.5f);

        drive.useDirectPower();

        assertEquals(0.5, sim.leftFront.power, DELTA);
        assertEquals(0.5, sim.rightFront.power, DELTA);
        assertEquals(0.5, sim.leftBack.power, DELTA);
        assertEquals(0.5, sim.rightBack.power, DELTA);
    }

    @Test
    public void turnPowerSpinsTheSidesOppositeWays() {
        drive.setTurnPower(0.5f);

        drive.useDirectPower();

        assertEquals(-0.5, sim.leftFront.power, DELTA);
        assertEquals(0.5, sim.rightFront.power, DELTA);
        assertEquals(-0.5, sim.leftBack.power, DELTA);
        assertEquals(0.5, sim.rightBack.power, DELTA);
    }

    @Test
    public void fastDriveToAFarPoseDrivesForwardAtFullPower() {
        boolean done = drive.fastDriveTo(
                new Nav.Pose(new Pose2d(48, 0, 0)),
                new Nav.Pose(new Pose2d(0, 0, 0)));

        drive.useDirectPower();

        assertFalse(done);
        assertEquals(1, sim.leftFront.power, DELTA);
        assertEquals(1, sim.rightFront.power, DELTA);
        assertEquals(1, sim.leftBack.power, DELTA);
        assertEquals(1, sim.rightBack.power, DELTA);
    }

    /** The dashboard field view shows the robot where Nav says it is, even when nothing is driving. */
    @Test
    public void idleLoopsDrawTheRobotAtNavsPoseOnTheDashboard() {
        drive.loop();
        drive.loop();

        assertEquals(2, sim.dashboard.packets.size());
    }
}
