package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;

import org.junit.Test;

public class FastDriveTest {
    private static final float DELTA = 0.0001f;

    private final FastDrive fastDrive = new FastDrive();

    @Test
    public void notDoneBeforeAnyUpdate() {
        assertFalse(fastDrive.doneMoving());
        assertFalse(fastDrive.nearDestination());
    }

    @Test
    public void atTheDestinationAllPowersAreZeroAndMovingIsDone() {
        fastDrive.setDestination(new Pose2d(0, 0, 0));
        fastDrive.update(new Pose2d(0, 0, 0));

        assertEquals(0f, fastDrive.straightPower(), DELTA);
        assertEquals(0f, fastDrive.strafePower(), DELTA);
        assertEquals(0f, fastDrive.turnPower(), DELTA);
        assertTrue(fastDrive.doneMoving());
    }

    @Test
    public void farAheadDrivesStraightAtFullPower() {
        fastDrive.setDestination(new Pose2d(48, 0, 0));
        fastDrive.update(new Pose2d(0, 0, 0));

        assertEquals(1f, fastDrive.straightPower(), DELTA);
        assertEquals(0f, fastDrive.strafePower(), DELTA);
        assertEquals(0f, fastDrive.turnPower(), DELTA);
        assertFalse(fastDrive.doneMoving());
        assertFalse(fastDrive.nearXDestination());
    }

    @Test
    public void headingErrorTurnsWithoutTranslating() {
        fastDrive.setDestination(new Pose2d(0, 0, Math.PI / 2));
        fastDrive.update(new Pose2d(0, 0, 0));

        assertEquals(0f, fastDrive.straightPower(), DELTA);
        assertEquals(0f, fastDrive.strafePower(), DELTA);
        assertEquals(1f, Math.abs(fastDrive.turnPower()), DELTA);
        assertFalse(fastDrive.nearHDestination());
        assertFalse(fastDrive.doneMoving());
    }

    @Test
    public void withinTolerancesCountsAsNearTheDestination() {
        fastDrive.setDestination(new Pose2d(0.5, 0.3, 0.01));
        fastDrive.update(new Pose2d(0, 0, 0));

        assertTrue(fastDrive.nearXDestination());
        assertTrue(fastDrive.nearYDestination());
        assertTrue(fastDrive.nearHDestination());
        assertTrue(fastDrive.doneMoving());
    }
}
