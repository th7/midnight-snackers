package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.junit.Test;

public class DriveTest {
    private static final double DELTA = 0.0001;

    private final FakeDcMotorEx leftFront = new FakeDcMotorEx();
    private final FakeDcMotorEx rightFront = new FakeDcMotorEx();
    private final FakeDcMotorEx leftBack = new FakeDcMotorEx();
    private final FakeDcMotorEx rightBack = new FakeDcMotorEx();
    private final Drive drive =
            new Drive(leftFront, rightFront, leftBack, rightBack, new FakeDashboard(), new ElapsedTime(), new FakeTelemetry());

    @Test
    public void straightPowerDrivesAllWheelsTheSame() {
        drive.setStraightPower(0.5f);

        drive.useDirectPower();

        assertEquals(0.5, leftFront.power, DELTA);
        assertEquals(0.5, rightFront.power, DELTA);
        assertEquals(0.5, leftBack.power, DELTA);
        assertEquals(0.5, rightBack.power, DELTA);
    }

    @Test
    public void turnPowerSpinsTheSidesOppositeWays() {
        drive.setTurnPower(0.5f);

        drive.useDirectPower();

        assertEquals(-0.5, leftFront.power, DELTA);
        assertEquals(0.5, rightFront.power, DELTA);
        assertEquals(-0.5, leftBack.power, DELTA);
        assertEquals(0.5, rightBack.power, DELTA);
    }

    @Test
    public void fastDriveToAFarPoseDrivesForwardAtFullPower() {
        boolean done = drive.fastDriveTo(
                new Nav.Pose(new Pose2d(48, 0, 0)),
                new Nav.Pose(new Pose2d(0, 0, 0)));

        drive.useDirectPower();

        assertFalse(done);
        assertEquals(1, leftFront.power, DELTA);
        assertEquals(1, rightFront.power, DELTA);
        assertEquals(1, leftBack.power, DELTA);
        assertEquals(1, rightBack.power, DELTA);
    }
}
