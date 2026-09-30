package org.firstinspires.ftc.nugget;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.junit.Test;

public class TankDriveTest {
    private static final double EXACTLY = 1e-9;

    private final FakeDcMotorEx left = new FakeDcMotorEx();
    private final FakeDcMotorEx right = new FakeDcMotorEx();
    private final TankDrive drive =
            new TankDrive(NuggetHardware.builder().left(left).right(right).build());

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
    public void stoppingAsksBothSidesForNothing() {
        drive.drive(0.7, -0.2);

        drive.stop();

        assertEquals(0, left.power, EXACTLY);
        assertEquals(0, right.power, EXACTLY);
    }

    @Test
    public void aHardwareMissingAMotorIsRefusedNamingIt() {
        IllegalStateException refused = assertThrows(
                IllegalStateException.class,
                () -> NuggetHardware.builder().left(new FakeDcMotorEx()).build());

        assertTrue(refused.getMessage(), refused.getMessage().contains("right"));
        assertFalse("not the one it was given", refused.getMessage().contains("left"));
    }
}
