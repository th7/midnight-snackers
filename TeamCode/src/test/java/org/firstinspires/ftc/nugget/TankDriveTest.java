package org.firstinspires.ftc.nugget;

import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.junit.Test;

public class TankDriveTest {
    private static final double EXACTLY = 1e-9;

    private final FakeNugget nugget = new FakeNugget();
    private final FakeDcMotorEx left = nugget.left;
    private final FakeDcMotorEx right = nugget.right;
    private final TankDrive drive = new TankDrive(nugget.hardware());

    @Test
    public void oneMotorTurnsOppositeTheOther() {
        assertEquals(DcMotorSimple.Direction.REVERSE, left.getDirection());
        assertEquals(DcMotorSimple.Direction.FORWARD, right.getDirection());
    }

    @Test
    public void bothMotorsBrakeWhenAskedForNothing() {
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, left.getZeroPowerBehavior());
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, right.getZeroPowerBehavior());
    }

    @Test
    public void straightAheadAsksBothSidesForTheSame() {
        drive.drive(0.5, 0);

        assertEquals(0.5, left.power, EXACTLY);
        assertEquals(0.5, right.power, EXACTLY);
    }

    @Test
    public void turningCounterclockwiseSlowsTheLeftSideAndSpeedsTheRight() {
        drive.drive(0, 0.5);

        assertEquals(-0.5, left.power, EXACTLY);
        assertEquals(0.5, right.power, EXACTLY);
    }

    @Test
    public void aCommandAskingMoreThanASideHasIsScaledDownWholeRatherThanClipped() {
        drive.drive(1, 0.5);

        assertEquals(1, right.power, EXACTLY);
        assertEquals(
                "the left keeps the share of the right it was asked for, so the robot turns as sharply as it was pointed",
                0.5 / 1.5,
                left.power,
                EXACTLY);
    }

    @Test
    public void eachSideIsAskedForItsOwnPower() {
        drive.sides(0.25, -0.5);

        assertEquals(0.25, left.power, EXACTLY);
        assertEquals(-0.5, right.power, EXACTLY);
    }

    @Test
    public void sidesAskedForMoreThanTheyHaveAreScaledDownWholeRatherThanClipped() {
        drive.sides(-0.75, 1.5);

        assertEquals(-0.5, left.power, EXACTLY);
        assertEquals(1, right.power, EXACTLY);
    }

    @Test
    public void stoppingAsksBothSidesForNothing() {
        drive.drive(0.7, -0.2);

        drive.stop();

        assertEquals(0, left.power, EXACTLY);
        assertEquals(0, right.power, EXACTLY);
    }
}
